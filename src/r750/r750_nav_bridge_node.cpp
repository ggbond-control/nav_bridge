#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_bridge/r750/r750_backend.hpp"

namespace nav_bridge {
class R750NavBridgeNode final : public rclcpp::Node {
public:
    R750NavBridgeNode() : Node("nav_bridge_node") {
        rcl_interfaces::msg::ParameterDescriptor startup;
        startup.read_only = true;
        startup.description = "Applied at startup; restart the node to change";
        R750Options options;
        options.serial_port = declare_parameter<std::string>("serial_port", options.serial_port, startup);
        options.serial_baud = declare_parameter<int>("serial_baud", options.serial_baud, startup);
        options.connect_timeout_ms = declare_parameter<int>("connect_timeout_ms", options.connect_timeout_ms, startup);
        options.reconnect_interval_ms = declare_parameter<int>("reconnect_interval_ms", options.reconnect_interval_ms, startup);
        options.cmd_vel_rate_hz = declare_parameter<double>("cmd_vel_rate_hz", options.cmd_vel_rate_hz, startup);
        options.cmd_vel_timeout_ms = declare_parameter<int>("cmd_vel_timeout_ms", options.cmd_vel_timeout_ms, startup);
        options.max_linear_speed = declare_parameter<double>("max_linear_speed", options.max_linear_speed, startup);
        options.max_angular_speed = declare_parameter<double>("max_angular_speed", options.max_angular_speed, startup);
        options.telemetry_timeout_ms = declare_parameter<int>("telemetry_timeout_ms", options.telemetry_timeout_ms, startup);
        options.startup_acquire_control = declare_parameter<bool>("startup_acquire_control", options.startup_acquire_control, startup);
        options.enable_motion_output = declare_parameter<bool>("enable_motion_output", options.enable_motion_output, startup);
        options.battery_empty_voltage = declare_parameter<double>("battery_empty_voltage", options.battery_empty_voltage, startup);
        options.battery_full_voltage = declare_parameter<double>("battery_full_voltage", options.battery_full_voltage, startup);
        const auto state_rate_hz = declare_parameter<double>("state_publish_rate_hz", options.state_publish_rate_hz, startup);
        if (!std::isfinite(state_rate_hz) || state_rate_hz <= 0.0 || state_rate_hz > 100.0) {
            throw std::invalid_argument("state_publish_rate_hz must be in (0, 100]");
        }
        options.state_publish_rate_hz = state_rate_hz;

        state_pub_ = create_publisher<std_msgs::msg::Int32>("/robot_basic_state", 10);
        gait_pub_ = create_publisher<std_msgs::msg::Int32>("/robot_gait_state", 10);
        battery_pub_ = create_publisher<std_msgs::msg::UInt8>("/battery/level", 10);
        raw_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/raw_vel", 10);
        diagnostics_pub_ = create_publisher<std_msgs::msg::String>("~/backend_status", 10);
        fault_pub_ = create_publisher<std_msgs::msg::String>("/robot_fault", 10);
        backend_ = std::make_unique<R750Backend>(options);
        backend_->setRawVelocityCallback([this](double linear, double lateral, double angular) {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = linear;
            msg.linear.y = lateral;
            msg.angular.z = angular;
            raw_vel_pub_->publish(msg);
        });
        backend_->setFaultCallback([this](const BackendFault &fault) {
            std_msgs::msg::String msg;
            msg.data = fault.message;
            fault_pub_->publish(msg);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "R750: %s", msg.data.c_str());
        });
        velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1),
            [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
                const auto result = backend_->move(msg->linear.x, msg->linear.y, msg->angular.z);
                if (!result.success) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", result.message.c_str());
                }
            });
        // Service I/O must not block the ROS velocity subscriber. The estop
        // latch also has to outlive any in-flight velocity write.
        service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        addTrigger("ready", [this] { return backend_->stand(); });
        addTrigger("stand", [this] { return backend_->stand(); });
        addTrigger("lie", [this] { return backend_->lie(); });
        addTrigger("soft_estop", [this] { return backend_->softEstop(true); });
        addTrigger("release_control", [this] { return backend_->releaseControl(); });
        for (const std::string name : {"set_gait", "set_speed", "set_body_height", "charge_command"}) {
            unsupported_.push_back(create_service<rcl_interfaces::srv::SetParameters>("~/" + name,
                [name](rcl_interfaces::srv::SetParameters::Request::SharedPtr request,
                       rcl_interfaces::srv::SetParameters::Response::SharedPtr response) {
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = false;
                    result.reason = "R750 " + name + " is unsupported in this backend";
                    response->results.assign(std::max<size_t>(1, request->parameters.size()), result);
                }, rclcpp::ServicesQoS(), service_group_));
        }
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / state_rate_hz), [this] {
            const auto state = backend_->state();
            if (!output_status_logged_ || last_connected_ != state.connected ||
                last_output_enabled_ != state.control_owned) {
                RCLCPP_INFO(get_logger(),
                    "R750 connected=%s, bridge_output_enabled=%s; device control owner=UNKNOWN, "
                    "controlled by bridge=UNKNOWN (serial protocol reports no arbitration feedback)",
                    state.connected ? "true" : "false", state.control_owned ? "true" : "false");
                output_status_logged_ = true;
                last_connected_ = state.connected;
                last_output_enabled_ = state.control_owned;
            }
            std_msgs::msg::Int32 basic;
            basic.data = static_cast<int>(state.motion_state);
            state_pub_->publish(basic);
            std_msgs::msg::Int32 gait;
            gait.data = state.mode;  // 0 = the bridge's Idle mode, not a physical gait.
            gait_pub_->publish(gait);
            if (state.connected && state.battery_percent >= 0.0) {
                std_msgs::msg::UInt8 battery;
                battery.data = static_cast<uint8_t>(std::clamp(state.battery_percent, 0.0, 100.0));
                battery_pub_->publish(battery);
            }
            std_msgs::msg::String diagnostics;
            diagnostics.data = backend_->diagnostics();
            diagnostics_pub_->publish(diagnostics);
        });
        RCLCPP_INFO(get_logger(), "R750 connecting to serial port %s at %d baud",
                    options.serial_port.c_str(), options.serial_baud);
        const auto result = backend_->connect();
        RCLCPP_INFO(get_logger(), "R750 initial connection: %s; failed connections retry in background",
                    result.message.c_str());
    }

    ~R750NavBridgeNode() override {
        // Join workers while publishers used by their fault callbacks still exist.
        backend_.reset();
    }

private:
    void addTrigger(const std::string &name, std::function<BackendResult()> operation) {
        triggers_.push_back(create_service<std_srvs::srv::Trigger>("~/" + name,
            [operation](std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
                const auto result = operation();
                response->success = result.success;
                response->message = result.message;
            }, rclcpp::ServicesQoS(), service_group_));
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_, gait_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr battery_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr raw_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_, fault_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::CallbackGroup::SharedPtr service_group_;
    std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> triggers_;
    std::vector<rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr> unsupported_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<R750Backend> backend_;
    bool output_status_logged_{false}, last_connected_{false}, last_output_enabled_{false};
};
}  // namespace nav_bridge

namespace {
volatile std::sig_atomic_t shutdown_requested = 0;
void signalHandler(int) { shutdown_requested = 1; }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    int result = 0;
    try {
        auto node = std::make_shared<nav_bridge::R750NavBridgeNode>();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
        executor.add_node(node);
        std::thread spin([&executor] { executor.spin(); });
        while (rclcpp::ok() && !shutdown_requested) std::this_thread::sleep_for(std::chrono::milliseconds(20));
        executor.cancel();
        spin.join();
        executor.remove_node(node);
        node.reset();
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("r750_nav_bridge"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
