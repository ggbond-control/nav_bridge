#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "nav_bridge/d1_max/d1_max_backend.hpp"

namespace nav_bridge {

class D1MaxNavBridgeNode final : public rclcpp::Node {
public:
    D1MaxNavBridgeNode() : Node("d1_max_nav_bridge_node") {
        host_ip_ = declare_parameter<std::string>("d1_host_ip", "192.168.234.1");
        host_port_ = declare_parameter<int>("d1_host_port", 8082);
        auto_reconnect_ = declare_parameter<bool>("auto_reconnect", true);
        connect_timeout_ms_ = declare_parameter<int>("connect_timeout_ms", 5000);
        reconnect_interval_ms_ = declare_parameter<int>("reconnect_interval_ms", 2000);
        cmd_vel_rate_hz_ = declare_parameter<int>("cmd_vel_rate_hz", 50);
        cmd_vel_timeout_ms_ = declare_parameter<int>("cmd_vel_timeout_ms", 500);

        backend_ = std::make_unique<D1MaxBackend>(host_ip_, host_port_, auto_reconnect_,
                                                  connect_timeout_ms_, reconnect_interval_ms_);
        backend_->setStateCallback([this](const BackendState &state) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_ = state;
        });
        backend_->setImuCallback([this](const BackendImu &data) {
            sensor_msgs::msg::Imu msg;
            msg.header.stamp = now(); msg.header.frame_id = "imu_link";
            msg.linear_acceleration.x = data.ax; msg.linear_acceleration.y = data.ay; msg.linear_acceleration.z = data.az;
            msg.angular_velocity.x = data.gx; msg.angular_velocity.y = data.gy; msg.angular_velocity.z = data.gz;
            msg.orientation.x = data.qx; msg.orientation.y = data.qy; msg.orientation.z = data.qz; msg.orientation.w = data.qw;
            imu_pub_->publish(msg);
        });
        backend_->setOdometryCallback([this](const BackendOdometry &data) {
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = now(); msg.header.frame_id = "odom"; msg.child_frame_id = "base_link";
            msg.pose.pose.position.x = data.px; msg.pose.pose.position.y = data.py; msg.pose.pose.position.z = data.pz;
            msg.pose.pose.orientation.x = data.qx; msg.pose.pose.orientation.y = data.qy;
            msg.pose.pose.orientation.z = data.qz; msg.pose.pose.orientation.w = data.qw;
            msg.twist.twist.linear.x = data.vx; msg.twist.twist.linear.y = data.vy; msg.twist.twist.linear.z = data.vz;
            msg.twist.twist.angular.x = data.wx; msg.twist.twist.angular.y = data.wy; msg.twist.twist.angular.z = data.wz;
            odom_pub_->publish(msg);
        });
        backend_->setJointCallback([this](const BackendJointState &data) {
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = now(); msg.name = data.names; msg.position = data.positions;
            msg.velocity = data.velocities; msg.effort = data.efforts;
            joint_pub_->publish(msg);
        });
        backend_->setFaultCallback([this](const BackendFault &fault) {
            std_msgs::msg::String msg;
            msg.data = "code=" + std::to_string(fault.code) + ", level=" + std::to_string(fault.level) + ": " + fault.message;
            fault_pub_->publish(msg);
        });

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(command_mutex_);
                vx_ = msg->linear.x;
                vy_ = msg->linear.y;
                vyaw_ = msg->angular.z;
                last_cmd_time_ = std::chrono::steady_clock::now();
                if (!state().control_owned) backend_->takeControl();
            });
        basic_state_pub_ = create_publisher<std_msgs::msg::Int32>("/robot_basic_state", 10);
        battery_pub_ = create_publisher<std_msgs::msg::UInt8>("/battery/level", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/leg_odom", 10);
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        fault_pub_ = create_publisher<std_msgs::msg::String>("/robot_fault", 10);

        stand_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/stand", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                fillResponse(response, backend_->stand());
            });
        lie_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/lie", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                fillResponse(response, backend_->lie());
            });
        estop_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/soft_estop", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                fillResponse(response, backend_->softEstop(true));
            });
        release_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/release_control", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                fillResponse(response, backend_->releaseControl());
            });
        mode_srv_ = create_service<rcl_interfaces::srv::SetParameters>(
            "~/set_mode", [this](const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
                                  std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response) {
                rcl_interfaces::msg::SetParametersResult result;
                if (request->parameters.size() != 1 || request->parameters[0].name != "mode" ||
                    request->parameters[0].value.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
                    result.successful = false; result.reason = "Expected one integer parameter named mode (1=GENERAL,2=IN_PLACE,3=STAIR).";
                } else {
                    const int mode = static_cast<int>(request->parameters[0].value.integer_value);
                    const auto backend_result = (mode == 1 || mode == 2 || mode == 3)
                                                     ? backend_->setMode(mode)
                                                     : BackendResult{false, "Mode must be 1, 2 or 3."};
                    result.successful = backend_result.success; result.reason = backend_result.message;
                }
                response->results.push_back(result);
            });
        speed_srv_ = create_service<rcl_interfaces::srv::SetParameters>(
            "~/set_speed", [this](const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
                                   std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response) {
                rcl_interfaces::msg::SetParametersResult result;
                if (request->parameters.size() != 1 || request->parameters[0].name != "speed" ||
                    request->parameters[0].value.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
                    result.successful = false; result.reason = "Expected one integer parameter named speed (1=SLOW,2=MEDIUM,3=HIGH).";
                } else {
                    const int speed = static_cast<int>(request->parameters[0].value.integer_value);
                    const auto backend_result = (speed == 1 || speed == 2 || speed == 3)
                                                     ? backend_->setSpeed(speed)
                                                     : BackendResult{false, "Speed must be 1, 2 or 3."};
                    result.successful = backend_result.success; result.reason = backend_result.message;
                }
                response->results.push_back(result);
            });

        const auto period = std::chrono::milliseconds(1000 / std::max(1, cmd_vel_rate_hz_));
        timer_ = create_wall_timer(period, [this]() { controlTick(); });
        const auto result = backend_->connect();
        if (!result.success) {
            RCLCPP_ERROR(get_logger(), "D1 Max connect failed: %s", result.message.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "D1 Max backend connected to %s:%d", host_ip_.c_str(), host_port_);
        }
    }

    ~D1MaxNavBridgeNode() override {
        if (backend_) backend_->disconnect();
    }

private:
    static void fillResponse(const std::shared_ptr<std_srvs::srv::Trigger::Response> &response,
                             const BackendResult &result) {
        response->success = result.success;
        response->message = result.message;
    }


    BackendState state() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_;
    }

    void controlTick() {
        const auto current = std::chrono::steady_clock::now();
        double vx, vy, vyaw;
        std::chrono::steady_clock::time_point last;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            vx = vx_; vy = vy_; vyaw = vyaw_; last = last_cmd_time_;
        }
        if (!state().control_owned) return;
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(current - last).count();
        if (age > cmd_vel_timeout_ms_) vx = vy = vyaw = 0.0;
        const auto result = backend_->move(vx, vy, vyaw);
        if (!result.success) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "D1 Max Move failed: %s", result.message.c_str());
        }
        const auto current_state = state();
        std_msgs::msg::Int32 state_msg;
        state_msg.data = static_cast<int32_t>(current_state.motion_state);
        basic_state_pub_->publish(state_msg);
        if (current_state.battery_percent >= 0.0) {
            std_msgs::msg::UInt8 battery_msg;
            battery_msg.data = static_cast<uint8_t>(std::clamp(current_state.battery_percent, 0.0, 100.0));
            battery_pub_->publish(battery_msg);
        }
    }

    std::unique_ptr<D1MaxBackend> backend_;
    std::string host_ip_;
    int host_port_{8082};
    bool auto_reconnect_{true};
    int connect_timeout_ms_{5000};
    int reconnect_interval_ms_{2000};
    int cmd_vel_rate_hz_{50};
    int cmd_vel_timeout_ms_{500};
    mutable std::mutex state_mutex_;
    BackendState state_;
    std::mutex command_mutex_;
    double vx_{0.0}, vy_{0.0}, vyaw_{0.0};
    std::chrono::steady_clock::time_point last_cmd_time_{std::chrono::steady_clock::time_point::min()};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr basic_state_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr battery_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_srv_, lie_srv_, estop_srv_, release_srv_;
    rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr mode_srv_, speed_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav_bridge

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav_bridge::D1MaxNavBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
