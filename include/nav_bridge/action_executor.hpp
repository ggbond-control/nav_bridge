#pragma once

/// @file action_executor.hpp
/// @brief 动作执行器，封装 stand/lie 以及内部力控站立流程

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "nav_bridge/robot_state_store.hpp"
#include "nav_bridge/x30_protocol.hpp"

namespace nav_bridge {

struct ActionResult {
    bool success{false};
    std::string message;
};

class ActionExecutor {
public:
    using ControlWarmup = std::function<void(int, int)>;
    using CommandSender  = std::function<void(uint32_t)>;

    ActionExecutor(rclcpp::Logger logger, RobotStateStore &state_store,
                   ControlWarmup control_warmup, CommandSender command_sender);

    ActionResult stand();
    ActionResult forceStand();
    ActionResult lieDown();

private:
    void ensureControlTakeover();
    void ensureControlTakeover(int warmup_ms, int pulse_ms);
    bool sendToggleCommandWithRetries(uint32_t command,
                                      const std::vector<x30_protocol::BasicState> &targets,
                                      int overall_timeout_ms, int resend_interval_ms,
                                      const char *phase_name, int initial_warmup_ms);
    bool sendSingleCommandAndWait(uint32_t command,
                                  const std::vector<x30_protocol::BasicState> &targets,
                                  int timeout_ms, const char *phase_name, int warmup_ms);
    bool waitForBasicState(const std::vector<x30_protocol::BasicState> &targets, int timeout_ms);
    bool waitForGaitState(const std::vector<x30_protocol::GaitState> &targets, int timeout_ms);

    rclcpp::Logger logger_;
    RobotStateStore &state_store_;
    ControlWarmup control_warmup_;
    CommandSender command_sender_;
};

}  // namespace nav_bridge
