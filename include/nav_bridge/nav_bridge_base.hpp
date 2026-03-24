#pragma once

/// @file nav_bridge_base.hpp
/// @brief 导航桥接抽象基类 — 定义与协议无关的导航层通用接口

#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

namespace nav_bridge {

/// 机器人基本状态 (通用)
enum class RobotState {
    DISCONNECTED,
    LYING_DOWN,
    STANDING_UP,
    INITIAL_STAND,
    FORCE_STAND,
    STEPPING,
    GOING_DOWN,
    SOFT_ESTOP,
    RL_MODE,
};

/// @brief 导航桥接基类
/// 子类需实现具体的通信协议 (UDP / ROS Topic / CAN 等)
class NavBridgeBase : public rclcpp::Node {
public:
    explicit NavBridgeBase(const std::string &node_name,
                           const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~NavBridgeBase() = default;

    /// 初始化通信连接
    virtual bool initialize() = 0;

    /// 关闭通信连接
    virtual void shutdown() = 0;

protected:
    // ===================== 下行接口 (ROS2 → 机器人) =====================

    /// 发送速度指令 (body frame)
    /// @param vx    前进速度 (m/s, 正前负后)
    /// @param vy    侧向速度 (m/s, 正右负左)
    /// @param vyaw  偏航角速度 (rad/s, 正右转负左转)
    virtual void sendVelocityCommand(double vx, double vy, double vyaw) = 0;

    /// 发送步态切换指令
    virtual void sendGaitCommand(uint32_t gait_cmd_code) = 0;

    // ===================== 上行接口 (机器人 → ROS2) =====================

    /// 处理接收到的数据, 解析并发布为ROS2话题
    virtual void processIncomingData() = 0;

    // ===================== ROS2 话题接口 =====================

    // -- 订阅 --
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // -- 发布 --
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gait_state_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr battery_level_pub_;

    // ===================== ROS2 服务接口 =====================
    // -- 服务端 --
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lie_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_srv_;

    virtual void handleStandRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res)      = 0;
    virtual void handleLieRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> res)      = 0;
    virtual void handleReadyRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res)      = 0;

    // ===================== TF =====================
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ===================== 定时器 =====================
    rclcpp::TimerBase::SharedPtr receive_timer_;
    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;  ///< 轴指令定频发送

    // ===================== 状态 =====================
    std::atomic<RobotState> robot_state_{RobotState::DISCONNECTED};

    // ===================== cmd_vel 回调 =====================
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    virtual void onControlInputUpdated() {}

    // 缓存最新的速度指令
    std::atomic<double> target_vx_{0.0};
    std::atomic<double> target_vy_{0.0};
    std::atomic<double> target_vyaw_{0.0};

    // 控制活跃状态跟踪 (超时后停止发送轴指令及心跳, 交还遥控器)
    std::atomic<std::chrono::steady_clock::time_point> last_active_time_{
        std::chrono::steady_clock::time_point::min()};  // 初始为最小值 = 从未收到

    // ===================== 参数 =====================
    std::string imu_frame_id_{"imu_link"};
    std::string odom_frame_id_{"odom"};
    std::string base_frame_id_{"base_link"};
    bool publish_tf_{true};
};

}  // namespace nav_bridge
