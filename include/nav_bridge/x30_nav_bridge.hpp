#pragma once

/// @file x30_nav_bridge.hpp
/// @brief 绝影X30导航桥接子类 — 基于UDP模拟手柄轴指令

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "nav_bridge/nav_bridge_base.hpp"
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
    void sendHeartbeat() override;
    void processIncomingData() override;

    void handleStandRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) override;
    void handleLieDownRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res) override;
    void handleForceStandRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) override;
    void handleReadyRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) override;
    void handleMotionRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res) override;

private:
    // ===================== UDP 通信 =====================
    UdpTransport motion_udp_;   ///< 与103运动主机通信
    UdpTransport percept_udp_;  ///< 与105感知主机通信 (可选)

    // ===================== 接收线程 =====================
    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    void receiveLoop();

    // ===================== 数据解析 & 发布 =====================
    void handleRcsData(const x30_protocol::RcsData &data);
    void handleMotionState(const x30_protocol::MotionStateData &data);
    void handleControllerSensor(const x30_protocol::ControllerSensorData &data);
    void handleBattery(const x30_protocol::BatterySensorData &data);

    // ===================== 定频发送轴指令 =====================
    void sendCmdVelTick();

    // ===================== 状态等待 =====================
    /// 确保已注册控制权 (预发心跳)
    void ensureControlTakeover();
    /// 阻塞等待机器人基本状态到达目标状态之一
    bool waitForBasicState(const std::vector<x30_protocol::BasicState> &targets, int timeout_ms);
    /// 阻塞等待机器人步态到达目标步态之一
    bool waitForGaitState(const std::vector<x30_protocol::GaitState> &targets, int timeout_ms);

    // ===================== 状态 =====================
    std::mutex state_mutex_;
    uint8_t current_gait_state_{0};   ///< 当前步态
    uint8_t current_basic_state_{0};  ///< 当前基本状态
    bool heartbeat_confirmed_{false};
    bool rcs_received_{false};  ///< 是否已收到首条 RcsData

    // ===================== 连接状态跟踪 =====================
    std::atomic<bool> connected_{false};
    std::atomic<std::chrono::steady_clock::time_point> last_recv_time_{
        std::chrono::steady_clock::now()};

    // ===================== 参数 =====================
    std::string motion_host_ip_;
    int motion_host_port_;
    int local_recv_port_;
    int heartbeat_interval_ms_;
    int cmd_vel_rate_hz_;
    int cmd_vel_timeout_ms_;
};

}  // namespace nav_bridge
