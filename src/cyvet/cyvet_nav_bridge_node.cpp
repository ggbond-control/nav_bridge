#include <algorithm>
#include <cctype>
#include <csignal>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_bridge/cyvet/cyvet_backend.hpp"

namespace nav_bridge {
class CyvetNavBridgeNode final : public rclcpp::Node {
public:
    CyvetNavBridgeNode() : Node("nav_bridge_node") {
        CyvetOptions options;
        options.device_id=declare_parameter<std::string>("device_id", "");
        options.service_name=declare_parameter<std::string>("robot_service_name", "robotServer");
        options.event_topic=declare_parameter<std::string>("event_topic", "/robotServer/Event");
        options.robot_domain_id=declare_parameter<int>("robot_domain_id",42);
        options.enable_observations=declare_parameter<bool>("enable_observations",true);
        options.connect_timeout_ms=declare_parameter<int>("connect_timeout_ms",5000);
        options.reconnect_interval_ms=declare_parameter<int>("reconnect_interval_ms",2000);
        options.action_timeout_ms=declare_parameter<int>("action_timeout_ms",10000);
        options.rpc_timeout_ms=declare_parameter<int>("rpc_timeout_ms",200);
        options.lease_ms=declare_parameter<int>("lease_ms",5000);
        options.cmd_vel_timeout_ms=declare_parameter<int>("cmd_vel_timeout_ms",500);
        options.telemetry_timeout_ms=declare_parameter<int>("telemetry_timeout_ms",2000);
        options.cmd_vel_rate_hz=declare_parameter<double>("cmd_vel_rate_hz",30.0);
        options.lateral_sign=declare_parameter<double>("lateral_sign",1.0);
        options.default_control_profile=declare_parameter<std::string>("default_control_profile","slow");
        imu_frame_=declare_parameter<std::string>("imu_frame_id","imu_link");
        odom_frame_=declare_parameter<std::string>("odom_frame_id","odom");
        base_frame_=declare_parameter<std::string>("base_frame_id","base_link");
        state_pub_=create_publisher<std_msgs::msg::Int32>("/robot_basic_state",10);
        gait_pub_=create_publisher<std_msgs::msg::Int32>("/robot_gait_state",10);
        battery_pub_=create_publisher<std_msgs::msg::UInt8>("/battery/level",10);
        diagnostics_pub_=create_publisher<std_msgs::msg::String>("~/backend_status",10);
        fault_pub_=create_publisher<std_msgs::msg::String>("/robot_fault",10);
        imu_pub_=create_publisher<sensor_msgs::msg::Imu>("/imu/data",rclcpp::SensorDataQoS());
        odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/leg_odom",10);
        joint_pub_=create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
        backend_=std::make_unique<CyvetBackend>(options);
        backend_->setImuCallback([this](const BackendImu &data) {
            sensor_msgs::msg::Imu msg;
            msg.header.stamp=now(); msg.header.frame_id=imu_frame_;
            msg.linear_acceleration.x=data.ax; msg.linear_acceleration.y=data.ay; msg.linear_acceleration.z=data.az;
            msg.angular_velocity.x=data.gx; msg.angular_velocity.y=data.gy; msg.angular_velocity.z=data.gz;
            msg.orientation.x=data.qx; msg.orientation.y=data.qy; msg.orientation.z=data.qz; msg.orientation.w=data.qw;
            if (data.qx*data.qx+data.qy*data.qy+data.qz*data.qz+data.qw*data.qw < 1e-6)
                msg.orientation_covariance[0]=-1;
            imu_pub_->publish(msg);
        });
        backend_->setOdometryCallback([this](const BackendOdometry &data) {
            nav_msgs::msg::Odometry msg;
            msg.header.stamp=now(); msg.header.frame_id=odom_frame_; msg.child_frame_id=base_frame_;
            msg.pose.pose.position.x=data.px; msg.pose.pose.position.y=data.py;
            msg.pose.pose.orientation.x=data.qx; msg.pose.pose.orientation.y=data.qy;
            msg.pose.pose.orientation.z=data.qz; msg.pose.pose.orientation.w=data.qw;
            msg.twist.twist.linear.x=data.vx; msg.twist.twist.linear.y=data.vy;
            msg.twist.twist.angular.z=data.wz;
            // Planar model odometry; absent dimensions have high uncertainty.
            msg.pose.covariance[0]=msg.pose.covariance[7]=0.05;
            msg.pose.covariance[14]=msg.pose.covariance[21]=msg.pose.covariance[28]=1e6;
            msg.pose.covariance[35]=0.1;
            msg.twist.covariance=msg.pose.covariance;
            odom_pub_->publish(msg);
        });
        backend_->setJointCallback([this](const BackendJointState &data) {
            sensor_msgs::msg::JointState msg;
            msg.header.stamp=now(); msg.name=data.names; msg.position=data.positions;
            msg.velocity=data.velocities; msg.effort=data.efforts;
            joint_pub_->publish(msg);
        });
        backend_->setFaultCallback([this](const BackendFault &data) {
            std_msgs::msg::String msg;
            msg.data="code="+std::to_string(data.code)+": "+data.message; fault_pub_->publish(msg);
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Cyvet fault: %s", msg.data.c_str());
        });
        velocity_sub_=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::QoS(1),
            [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
                auto result=backend_->move(msg->linear.x,msg->linear.y,msg->angular.z);
                if (!result.success) RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"%s",result.message.c_str());
            });
        // Dedicated groups keep stop services and telemetry responsive during stand confirmation.
        service_group_=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        addTrigger("stand",[this] { return backend_->stand(); });
        addTrigger("ready",[this] { return backend_->stand(); });
        addTrigger("lie",[this] { return backend_->lie(); });
        addTrigger("soft_estop",[this] { return backend_->softEstop(true); });
        addTrigger("release_control",[this] { return backend_->releaseControl(); });
        addParameters("set_gait",[this](const auto &parameter) {
            const auto &v=parameter.value;
            if (parameter.name!="gait") return BackendResult{false,"Expected parameter gait"};
            if (v.type==2 && (v.integer_value==0 || v.integer_value==3))
                return backend_->setGait(static_cast<int>(v.integer_value));
            if (v.type==4) {
                std::string name=v.string_value;
                std::transform(name.begin(),name.end(),name.begin(),
                    [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
                if (name=="WALK" || name=="SLOW") return backend_->setGait(0);
                if (name=="RUN" || name=="FAST") return backend_->setGait(3);
            }
            return BackendResult{false,"Cyvet supports only WALK/SLOW/0 and RUN/FAST/3; no medium model"};
        });
        addParameters("set_speed",[this](const auto &parameter) {
            if (parameter.name!="speed" || parameter.value.type!=2 ||
                (parameter.value.integer_value!=1 && parameter.value.integer_value!=3))
                return BackendResult{false,"Expected integer speed 1=slow or 3=fast; medium is unsupported"};
            return backend_->setSpeed(static_cast<int>(parameter.value.integer_value));
        });
        for (const std::string name : {"set_body_height","charge_command"})
            addParameters(name,[name](const auto &) { return BackendResult{false,"Cyvet "+name+" is not supported"}; });
        timer_=create_wall_timer(std::chrono::milliseconds(100),[this] {
            const auto state=backend_->state();
            std_msgs::msg::Int32 basic; basic.data=static_cast<int>(state.motion_state); state_pub_->publish(basic);
            std_msgs::msg::Int32 gait; gait.data=state.mode; gait_pub_->publish(gait);
            if (state.connected && state.battery_percent>=0) {
                std_msgs::msg::UInt8 battery;
                battery.data=static_cast<uint8_t>(std::clamp(state.battery_percent,0.0,100.0)); battery_pub_->publish(battery);
            }
            std_msgs::msg::String diagnostics; diagnostics.data=backend_->diagnostics(); diagnostics_pub_->publish(diagnostics);
        });
        auto result=backend_->connect();
        RCLCPP_INFO(get_logger(),"%s",result.message.c_str());
    }
    ~CyvetNavBridgeNode() override { backend_.reset(); }
private:
    void addTrigger(const std::string &name,std::function<BackendResult()> operation) {
        triggers_.push_back(create_service<std_srvs::srv::Trigger>("~/"+name,
            [operation](const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
                auto result=operation(); response->success=result.success; response->message=result.message;
            },rclcpp::ServicesQoS(),service_group_));
    }
    void addParameters(const std::string &name,
                       std::function<BackendResult(const rcl_interfaces::msg::Parameter &)> operation) {
        parameters_.push_back(create_service<rcl_interfaces::srv::SetParameters>("~/"+name,
            [operation](rcl_interfaces::srv::SetParameters::Request::SharedPtr request,
                        rcl_interfaces::srv::SetParameters::Response::SharedPtr response) {
                BackendResult result{false,"Expected exactly one parameter"};
                if (request->parameters.size()==1) result=operation(request->parameters.front());
                rcl_interfaces::msg::SetParametersResult item; item.successful=result.success; item.reason=result.message;
                response->results.assign(std::max<size_t>(1,request->parameters.size()),item);
            },rclcpp::ServicesQoS(),service_group_));
    }
    std::unique_ptr<CyvetBackend> backend_;
    std::string imu_frame_,odom_frame_,base_frame_;
    rclcpp::CallbackGroup::SharedPtr service_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_,gait_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr battery_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_,fault_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> triggers_;
    std::vector<rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr> parameters_;
};
}  // namespace nav_bridge
namespace {
volatile std::sig_atomic_t shutdown_requested=0;
void signalHandler(int) { shutdown_requested=1; }
}
int main(int argc,char **argv) {
    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT,signalHandler); std::signal(SIGTERM,signalHandler);
    int status=0;
    try {
        auto node=std::make_shared<nav_bridge::CyvetNavBridgeNode>();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);
        executor.add_node(node);
        std::thread spin([&executor] { executor.spin(); });
        while (rclcpp::ok() && !shutdown_requested) std::this_thread::sleep_for(std::chrono::milliseconds(20));
        executor.cancel(); spin.join(); executor.remove_node(node); node.reset();
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("cyvet_nav_bridge"),"%s",error.what()); status=1;
    }
    rclcpp::shutdown(); return status;
}
