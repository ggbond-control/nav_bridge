#pragma once

/// @file x30_nav_bridge.hpp
/// @brief 绝影X30导航桥接子类 — 基于UDP模拟手柄轴指令

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "nav_bridge/action_executor.hpp"
#include "nav_bridge/nav_bridge_base.hpp"
#include "nav_bridge/control_session_manager.hpp"
#include "nav_bridge/robot_state_store.hpp"
#include "nav_bridge/udp_transport.hpp"
#include "nav_bridge/x30_protocol.hpp"

namespace nav_bridge {

class X30NavBridge : public NavBridgeBase {
public:
    explicit X30NavBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~X30NavBridge() override;

    bool initialize() override;
    void shutdown() override;

protected:
    void sendVelocityCommand(double vx, double vy, double vyaw) override;
    void sendGaitCommand(uint32_t gait_cmd_code) override;
    void processIncomingData() override;
    void onControlInputUpdated() override;

    void handleStandRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) override;
    void handleLieRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) override;
    void handleReadyRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) override;
    void handleReleaseControlRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) override;

private:
    // ===================== UDP 通信 =====================
    UdpTransport motion_udp_;   ///< 与103运动主机通信
    UdpTransport percept_udp_;  ///< 与105感知主机通信 (可选)

    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    void receiveLoop();

    // ===================== 数据解析 & 发布 =====================
    void applyControlActions(const ControlActions &actions);
    void handleRcsData(const x30_protocol::RcsData &data);
    void handleMotionState(const x30_protocol::MotionStateData &data);
    void handleControllerSensor(const x30_protocol::ControllerSensorData &data);
    void handleBattery(const x30_protocol::BatterySensorData &data);

    // ===================== 定频发送轴指令 =====================
    void sendCmdVelTick();

    // ===================== 参数 =====================
    std::string motion_host_ip_;
    int motion_host_port_;
    int local_recv_port_;
    int heartbeat_interval_ms_;
    int cmd_vel_rate_hz_;
    int cmd_vel_timeout_ms_;

    // ===================== 业务模块 =====================
    RobotStateStore state_store_;
    ControlSessionManager control_session_;
    std::unique_ptr<ActionExecutor> action_executor_;
};

}  // namespace nav_bridge
