#pragma once

/// @file action_executor.hpp
/// @brief 动作执行器，封装站立/趴下/ready/运动等业务流程

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "nav_bridge/control_session_manager.hpp"
#include "nav_bridge/robot_state_store.hpp"
#include "nav_bridge/x30_protocol.hpp"

namespace nav_bridge {

struct ActionResult {
    bool success{false};
    std::string message;
};

class ActionExecutor {
public:
    using ControlApplier = std::function<void(const ControlActions &)>;
    using CommandSender  = std::function<void(uint32_t)>;

    ActionExecutor(rclcpp::Logger logger, RobotStateStore &state_store,
                   ControlSessionManager &control_session, ControlApplier control_applier,
                   CommandSender command_sender);

    ActionResult stand();
    ActionResult lieDown();
    ActionResult forceStand();
    ActionResult ready();

private:
    void ensureControlTakeover();
    void ensureControlTakeover(int warmup_ms, int pulse_ms);
    bool sendCommandDuringTakeoverWindow(uint32_t command,
                                         const std::vector<x30_protocol::BasicState> &targets,
                                         int overall_timeout_ms, int resend_interval_ms,
                                         const char *phase_name, int initial_warmup_ms);
    bool sendCommandUntilBasicState(uint32_t command,
                                    const std::vector<x30_protocol::BasicState> &targets,
                                    int timeout_ms, const char *phase_name, int warmup_ms);
    bool waitForBasicState(const std::vector<x30_protocol::BasicState> &targets, int timeout_ms);
    bool waitForGaitState(const std::vector<x30_protocol::GaitState> &targets, int timeout_ms);

    rclcpp::Logger logger_;
    RobotStateStore &state_store_;
    ControlSessionManager &control_session_;
    ControlApplier control_applier_;
    CommandSender command_sender_;
};

}  // namespace nav_bridge
