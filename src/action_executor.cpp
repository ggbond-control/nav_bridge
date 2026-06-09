/// @file action_executor.cpp
/// @brief 动作执行器实现

#include "nav_bridge/action_executor.hpp"

#include <algorithm>
#include <thread>

namespace nav_bridge {

using namespace x30_protocol;

namespace {

constexpr int kControlWarmupMs          = 400;
constexpr int kControlWarmupPulseMs     = 100;
constexpr int kSingleCommandTimeoutMs   = 5000;
constexpr int kGaitSwitchTimeoutMs      = 5000;
constexpr int kStandMountainTimeoutMs   = 12000;
constexpr int kSteppingEntryTimeoutMs   = 4000;
constexpr int kStandStartMotionTimeoutMs = 8000;
constexpr int kStandStartMotionResendMs  = 3000;
constexpr int kStandTransitionMs        = 16000;
constexpr int kStandResendIntervalMs    = 4000;
constexpr int kStopMotionTimeoutMs      = 12000;
constexpr int kStopMotionResendMs       = 4000;
constexpr int kLieTransitionTimeoutMs   = 6000;
constexpr int kLieFinalWaitMs           = 7000;
constexpr int kLieResendIntervalMs      = 3000;
constexpr int kLieOverallTimeoutMs      = 12000;
constexpr int kLieStandSettleMs         = 3000;
constexpr int kLieStandSettlePollMs     = 100;
constexpr int kLieStopMotionTimeoutMs   = 8000;
constexpr int kLieStopMotionResendMs    = 3000;
constexpr int kMaxCommandAttempts       = 2;

}  // namespace

ActionExecutor::ActionExecutor(rclcpp::Logger logger, RobotStateStore &state_store,
                               ControlWarmup control_warmup, CommandSender command_sender)
    : logger_(logger),
      state_store_(state_store),
      control_warmup_(std::move(control_warmup)),
      command_sender_(std::move(command_sender)) {}

void ActionExecutor::ensureControlTakeover() {
    ensureControlTakeover(kControlWarmupMs, kControlWarmupPulseMs);
}

void ActionExecutor::ensureControlTakeover(int warmup_ms, int pulse_ms) {
    if (control_warmup_) {
        control_warmup_(warmup_ms, pulse_ms);
    }
}

bool ActionExecutor::sendSingleCommandAndWait(uint32_t command,
                                              const std::vector<BasicState> &targets,
                                              int timeout_ms, const char *phase_name,
                                              int warmup_ms) {
    for (int attempt = 1; attempt <= kMaxCommandAttempts; ++attempt) {
        ensureControlTakeover(warmup_ms, kControlWarmupPulseMs);
        command_sender_(command);

        if (waitForBasicState(targets, timeout_ms)) {
            return true;
        }

        if (attempt < kMaxCommandAttempts) {
            RCLCPP_WARN(logger_, "⚠️ %s 未在 %dms 内出现状态响应，重新接管后重试一次", phase_name,
                        timeout_ms);
        }
    }
    return false;
}

bool ActionExecutor::sendToggleCommandWithRetries(uint32_t command,
                                                  const std::vector<BasicState> &targets,
                                                  int overall_timeout_ms,
                                                  int resend_interval_ms,
                                                  const char *phase_name,
                                                  int initial_warmup_ms) {
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(overall_timeout_ms);

    ensureControlTakeover(initial_warmup_ms, kControlWarmupPulseMs);

    int attempt = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        ++attempt;
        command_sender_(command);

        auto now = std::chrono::steady_clock::now();
        auto remaining_ms =
            static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        int wait_ms = std::min(resend_interval_ms, std::max(0, remaining_ms));
        if (wait_ms <= 0) {
            break;
        }

        if (waitForBasicState(targets, wait_ms)) {
            return true;
        }

        if (std::chrono::steady_clock::now() < deadline) {
            RCLCPP_WARN(logger_,
                        "⚠️ %s 在第 %d 次尝试后仍无响应，保持接管并继续重发命令",
                        phase_name, attempt);
            ensureControlTakeover(kControlWarmupMs, kControlWarmupPulseMs);
        }
    }

    return false;
}

bool ActionExecutor::waitForBasicState(const std::vector<BasicState> &targets, int timeout_ms) {
    return state_store_.waitForBasicState(targets, timeout_ms);
}

bool ActionExecutor::waitForGaitState(const std::vector<GaitState> &targets, int timeout_ms) {
    return state_store_.waitForGaitState(targets, timeout_ms);
}

ActionResult ActionExecutor::forceStand() {
    uint8_t state = state_store_.basicState();

    if (state == static_cast<uint8_t>(BasicState::FORCE_STAND)) {
        return {true, "Robot is already in force stand mode."};
    }

    if (state == static_cast<uint8_t>(BasicState::SOFT_ESTOP)) {
        RCLCPP_INFO(logger_, "⚠️ 机器人在软急停状态, 先恢复到贴下...");
        if (!sendSingleCommandAndWait(CMD_STAND_UP_DOWN, {BasicState::LYING_DOWN},
                                      kSingleCommandTimeoutMs, "软急停恢复",
                                      kControlWarmupMs)) {
            RCLCPP_WARN(logger_, "⚠️ 从软急停恢复贴下超时");
            return {false, "Timeout recovering from soft estop to lying down."};
        }
        RCLCPP_INFO(logger_, "✅ 已恢复到贴下状态");
        state = static_cast<uint8_t>(BasicState::LYING_DOWN);
    }

    if (state != static_cast<uint8_t>(BasicState::LYING_DOWN) &&
        state != static_cast<uint8_t>(BasicState::GOING_DOWN) &&
        state != static_cast<uint8_t>(BasicState::INITIAL_STAND) &&
        state != static_cast<uint8_t>(BasicState::STEPPING) &&
        state != static_cast<uint8_t>(BasicState::RL_MODE)) {
        return {false, "Robot is not in a state that can enter force stand (current state=" +
                           std::to_string(state) + ")."};
    }

    if (state == static_cast<uint8_t>(BasicState::LYING_DOWN) ||
        state == static_cast<uint8_t>(BasicState::GOING_DOWN)) {
        RCLCPP_INFO(logger_, "⏳ 等待机器人起立...");
        if (!sendToggleCommandWithRetries(CMD_STAND_UP_DOWN,
                                          {BasicState::STANDING_UP, BasicState::INITIAL_STAND,
                                           BasicState::FORCE_STAND, BasicState::STEPPING},
                                          kStandTransitionMs, kStandResendIntervalMs, "起立",
                                          kControlWarmupMs)) {
            RCLCPP_WARN(logger_, "⚠️ 起立指令发出后未观察到早期状态变化");
            return {false, "No response after stand-up command."};
        }

        if (!waitForBasicState(
                {BasicState::INITIAL_STAND, BasicState::FORCE_STAND, BasicState::STEPPING}, 6500)) {
            RCLCPP_WARN(logger_, "⚠️ 起立超时");
            return {false, "Timeout waiting for robot to stand up."};
        }
        RCLCPP_INFO(logger_, "✅ 起立完成");
        state = state_store_.basicState();
    }

    state = state_store_.basicState();
    if (state == static_cast<uint8_t>(BasicState::RL_MODE)) {
        RCLCPP_INFO(logger_, "⏳ 先切换回行走步态...");
        ensureControlTakeover(kControlWarmupMs, kControlWarmupPulseMs);
        command_sender_(CMD_GAIT_WALK);
        if (!waitForGaitState({GaitState::WALK}, kGaitSwitchTimeoutMs)) {
            RCLCPP_WARN(logger_, "⚠️ 从RL模式切回行走步态超时");
            return {false, "Timeout switching gait back to walk before stand."};
        }
        if (!waitForBasicState({BasicState::STEPPING}, kSteppingEntryTimeoutMs)) {
            RCLCPP_WARN(logger_, "⚠️ 行走步态已切换，但未观察到踏步状态");
            return {false, "Timeout waiting for stepping state after switching to walk."};
        }
        state = state_store_.basicState();
    }

    if (state == static_cast<uint8_t>(BasicState::RL_MODE) ||
        state == static_cast<uint8_t>(BasicState::STEPPING)) {
        RCLCPP_INFO(logger_, "⏳ 先停止运动，回到站立静止态...");
        if (!sendToggleCommandWithRetries(CMD_MOTION,
                                          {BasicState::FORCE_STAND, BasicState::INITIAL_STAND},
                                          kStopMotionTimeoutMs,
                                          kStopMotionResendMs, "停止运动",
                                          kControlWarmupMs)) {
            RCLCPP_WARN(logger_, "⚠️ 从运动状态退回站立超时");
            return {false, "Timeout stopping motion before entering force stand."};
        }
        state = state_store_.basicState();
    }

    if (state == static_cast<uint8_t>(BasicState::INITIAL_STAND)) {
        RCLCPP_INFO(logger_, "⏳ 切入力控站立...");
        ensureControlTakeover(kControlWarmupMs, kControlWarmupPulseMs);
        command_sender_(CMD_FORCE_CONTROL);
        if (!waitForBasicState({BasicState::FORCE_STAND}, 5000)) {
            RCLCPP_WARN(logger_, "⚠️ 切入力控站立超时");
            return {false, "Timeout entering force stand mode."};
        }
    }

    if (state_store_.basicState() != static_cast<uint8_t>(BasicState::FORCE_STAND)) {
        return {false, "Robot failed to reach force stand state."};
    }

    RCLCPP_INFO(logger_, "✅ 已进入力控站立状态");
    return {true, "Robot is now in force stand mode."};
}

ActionResult ActionExecutor::lieDown() {
    uint8_t state = state_store_.basicState();

    if (state == static_cast<uint8_t>(BasicState::LYING_DOWN)) {
        return {true, "Robot is already lying down."};
    }

    if (state == static_cast<uint8_t>(BasicState::GOING_DOWN)) {
        bool ok = waitForBasicState({BasicState::LYING_DOWN}, 5000);
        return {ok, ok ? "Robot has lied down." : "Timeout waiting for lie down."};
    }

    if (state == static_cast<uint8_t>(BasicState::SOFT_ESTOP)) {
        RCLCPP_INFO(logger_, "⏳ 从软急停恢复趴下...");
        bool ok = sendSingleCommandAndWait(CMD_STAND_UP_DOWN, {BasicState::LYING_DOWN},
                                           kSingleCommandTimeoutMs, "软急停恢复趴下",
                                           kControlWarmupMs);
        if (ok) {
            RCLCPP_INFO(logger_, "✅ 趴下完成");
        } else {
            RCLCPP_WARN(logger_, "⚠️ 趴下超时");
        }
        return {ok, ok ? "Robot recovered and lied down." : "Timeout recovering from estop."};
    }

    if (state == static_cast<uint8_t>(BasicState::RL_MODE)) {
        RCLCPP_INFO(logger_, "⏳ 从运动态退出，进入趴下流程...");
        if (!sendSingleCommandAndWait(CMD_STAND_UP_DOWN,
                                      {BasicState::FORCE_STAND, BasicState::INITIAL_STAND,
                                       BasicState::STEPPING, BasicState::GOING_DOWN,
                                       BasicState::LYING_DOWN},
                                      kLieTransitionTimeoutMs, "退出运动态准备趴下",
                                      kControlWarmupMs)) {
            RCLCPP_WARN(logger_, "⚠️ 无法从运动态退出到趴下链");
            return {false, "Timeout exiting motion state before lie down."};
        }

        state = state_store_.basicState();
        if (state == static_cast<uint8_t>(BasicState::LYING_DOWN)) {
            RCLCPP_INFO(logger_, "✅ 趴下完成");
            return {true, "Robot has lied down."};
        }
        if (state == static_cast<uint8_t>(BasicState::GOING_DOWN)) {
            bool ok = waitForBasicState({BasicState::LYING_DOWN}, kLieFinalWaitMs);
            if (ok) {
                RCLCPP_INFO(logger_, "✅ 趴下完成");
            } else {
                RCLCPP_WARN(logger_, "⚠️ 趴下超时");
            }
            return {ok, ok ? "Robot has lied down." : "Timeout waiting for robot to lie down."};
        }
    }

    state = state_store_.basicState();
    if (state == static_cast<uint8_t>(BasicState::STEPPING)) {
        RCLCPP_INFO(logger_, "⏳ 当前仍在踏步运动，先停止运动回到站立态...");
        if (!sendToggleCommandWithRetries(CMD_MOTION,
                                          {BasicState::FORCE_STAND, BasicState::INITIAL_STAND},
                                          kLieStopMotionTimeoutMs, kLieStopMotionResendMs,
                                          "趴下前停止运动", kControlWarmupMs)) {
            RCLCPP_WARN(logger_, "⚠️ 趴下前停止运动超时");
            return {false, "Timeout stopping motion before lie down."};
        }
    }

    state = state_store_.basicState();
    if (state == static_cast<uint8_t>(BasicState::FORCE_STAND) ||
        state == static_cast<uint8_t>(BasicState::INITIAL_STAND)) {
        RCLCPP_INFO(logger_, "⏳ 已进入站立态，等待稳定后再执行最终趴下...");
        auto settle_deadline = std::chrono::steady_clock::now() +
                               std::chrono::milliseconds(kLieStandSettleMs);
        while (std::chrono::steady_clock::now() < settle_deadline) {
            state = state_store_.basicState();
            if (state == static_cast<uint8_t>(BasicState::LYING_DOWN)) {
                RCLCPP_INFO(logger_, "✅ 趴下完成");
                return {true, "Robot has lied down."};
            }
            if (state == static_cast<uint8_t>(BasicState::GOING_DOWN)) {
                bool ok = waitForBasicState({BasicState::LYING_DOWN}, kLieFinalWaitMs);
                return {ok, ok ? "Robot has lied down." :
                                  "Timeout waiting for robot to lie down."};
            }
            if (state == static_cast<uint8_t>(BasicState::STEPPING)) {
                RCLCPP_INFO(logger_, "⏳ 稳定等待期间又进入踏步，重新停止运动后再趴下");
                if (!sendToggleCommandWithRetries(CMD_MOTION,
                                                  {BasicState::FORCE_STAND,
                                                   BasicState::INITIAL_STAND},
                                                  kLieStopMotionTimeoutMs,
                                                  kLieStopMotionResendMs,
                                                  "趴下前停止运动", kControlWarmupMs)) {
                    RCLCPP_WARN(logger_, "⚠️ 趴下前停止运动超时");
                    return {false, "Timeout stopping motion before lie down."};
                }
                settle_deadline = std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(kLieStandSettleMs);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(kLieStandSettlePollMs));
        }
        RCLCPP_INFO(logger_, "✅ 已退出运动态，继续执行最终趴下");
    }

    RCLCPP_INFO(logger_, "⏳ 等待机器人趴下...");
    bool ok = sendToggleCommandWithRetries(CMD_STAND_UP_DOWN,
                                           {BasicState::GOING_DOWN, BasicState::LYING_DOWN},
                                           kLieOverallTimeoutMs, kLieResendIntervalMs, "趴下",
                                           kControlWarmupMs) &&
              waitForBasicState({BasicState::LYING_DOWN}, kLieFinalWaitMs);
    if (ok) {
        RCLCPP_INFO(logger_, "✅ 趴下完成");
    } else {
        RCLCPP_WARN(logger_, "⚠️ 趴下超时");
    }
    return {ok, ok ? "Robot has lied down." : "Timeout waiting for robot to lie down."};
}

ActionResult ActionExecutor::stand() {
    RCLCPP_INFO(logger_, "🚀 收到 stand 指令, 开始执行启动序列...");

    uint8_t state = state_store_.basicState();
    uint8_t gait  = state_store_.gaitState();

    if (state == static_cast<uint8_t>(BasicState::SOFT_ESTOP)) {
        RCLCPP_INFO(logger_, "📌 [0/5] 机器人在软急停状态, 先恢复到贴下...");
        if (!sendSingleCommandAndWait(CMD_STAND_UP_DOWN, {BasicState::LYING_DOWN},
                                      kSingleCommandTimeoutMs, "stand: 软急停恢复",
                                      kControlWarmupMs)) {
            RCLCPP_ERROR(logger_, "❌ Stand 失败: 从软急停恢复超时");
            return {false, "Stand failed: timeout recovering from soft estop."};
        }
        RCLCPP_INFO(logger_, "✅ [0/5] 已恢复到贴下状态");
        state = state_store_.basicState();
    }

    if (state == static_cast<uint8_t>(BasicState::LYING_DOWN) ||
        state == static_cast<uint8_t>(BasicState::GOING_DOWN)) {
        RCLCPP_INFO(logger_, "📌 [1/5] 起立...");
        if (!sendToggleCommandWithRetries(CMD_STAND_UP_DOWN,
                                          {BasicState::STANDING_UP, BasicState::INITIAL_STAND,
                                           BasicState::FORCE_STAND, BasicState::STEPPING},
                                          kStandTransitionMs, kStandResendIntervalMs,
                                          "stand: 起立", kControlWarmupMs)) {
            RCLCPP_ERROR(logger_, "❌ Stand 失败: 起立指令无早期响应");
            return {false, "Stand failed: no response at step 1 (stand up)."};
        }

        if (!waitForBasicState(
                {BasicState::INITIAL_STAND, BasicState::FORCE_STAND, BasicState::STEPPING}, 6500)) {
            RCLCPP_ERROR(logger_, "❌ Stand 失败: 起立超时");
            return {false, "Stand failed: timeout at step 1 (stand up)."};
        }
        RCLCPP_INFO(logger_, "✅ [1/5] 起立完成");
    } else if (state == static_cast<uint8_t>(BasicState::INITIAL_STAND) ||
               state == static_cast<uint8_t>(BasicState::FORCE_STAND) ||
               state == static_cast<uint8_t>(BasicState::STEPPING) ||
               state == static_cast<uint8_t>(BasicState::RL_MODE)) {
        RCLCPP_INFO(logger_, "✅ [1/5] 已站立, 跳过");
    } else {
        RCLCPP_ERROR(logger_, "❌ Stand 失败: 无法识别的状态=%d", state);
        return {false, "Stand failed: unexpected state=" + std::to_string(state)};
    }

    state = state_store_.basicState();
    if (state == static_cast<uint8_t>(BasicState::INITIAL_STAND)) {
        RCLCPP_INFO(logger_, "📌 [2/5] 切入力控站立...");
        ensureControlTakeover(kControlWarmupMs, kControlWarmupPulseMs);
        command_sender_(CMD_FORCE_CONTROL);

        if (!waitForBasicState({BasicState::FORCE_STAND}, 5000)) {
            RCLCPP_ERROR(logger_, "❌ Stand 失败: 力控站立超时");
            return {false, "Stand failed: timeout at step 2 (force stand)."};
        }
        RCLCPP_INFO(logger_, "✅ [2/5] 力控站立完成");
    } else if (state == static_cast<uint8_t>(BasicState::FORCE_STAND) ||
               state == static_cast<uint8_t>(BasicState::RL_MODE) ||
               state == static_cast<uint8_t>(BasicState::STEPPING)) {
        RCLCPP_INFO(logger_, "✅ [2/5] 已在力控/运动, 跳过");
    } else {
        RCLCPP_ERROR(logger_, "❌ Stand 失败: 无法切入力控(state=%d)", state);
        return {false, "Stand failed: unexpected state for force stand (state=" +
                           std::to_string(state) + ")."};
    }

    state = state_store_.basicState();
    gait  = state_store_.gaitState();
    if (state == static_cast<uint8_t>(BasicState::RL_MODE) &&
        gait == static_cast<uint8_t>(GaitState::MOUNTAIN)) {
        RCLCPP_INFO(logger_, "✅ [3/4] 已在RL模式+山地步态, 跳过");
    } else {
        if (state == static_cast<uint8_t>(BasicState::FORCE_STAND)) {
            RCLCPP_INFO(logger_, "📌 [3/4] 当前为力控站立, 先进入踏步再切换山地...");
            if (!sendToggleCommandWithRetries(CMD_MOTION, {BasicState::STEPPING},
                                              kStandStartMotionTimeoutMs,
                                              kStandStartMotionResendMs,
                                              "stand: 力控站立进入踏步",
                                              kControlWarmupMs)) {
                auto final_snap = state_store_.snapshot();
                RCLCPP_ERROR(logger_,
                             "❌ Stand 失败: 力控站立未进入踏步 (basic=%u, gait=%u)",
                             final_snap.basic_state, final_snap.gait_state);
                return {false, "Stand failed: timeout entering stepping before mountain gait."};
            }
            state = state_store_.basicState();
            gait  = state_store_.gaitState();
            RCLCPP_INFO(logger_, "✅ [3/4] 已进入踏步, 准备切换山地 (state=%d, gait=%d)",
                        state, gait);
        }

        RCLCPP_INFO(logger_, "📌 [3/4] 切换山地步态 (state=%d, gait=%d)...", state, gait);
        ensureControlTakeover(kControlWarmupMs, kControlWarmupPulseMs);
        command_sender_(CMD_GAIT_MOUNTAIN);

        if (!state_store_.waitForState(
                [](uint8_t basic_state, uint8_t gait_state) {
                    return basic_state == static_cast<uint8_t>(BasicState::RL_MODE) &&
                           gait_state == static_cast<uint8_t>(GaitState::MOUNTAIN);
                },
                kStandMountainTimeoutMs)) {
            auto final_snap = state_store_.snapshot();
            RCLCPP_ERROR(logger_,
                         "❌ Stand 失败: 未进入RL模式+山地步态 (basic=%u, gait=%u)",
                         final_snap.basic_state, final_snap.gait_state);
            return {false, "Stand failed: timeout waiting for RL_MODE + MOUNTAIN."};
        }
        RCLCPP_INFO(logger_, "✅ [3/4] 山地步态+RL模式 就绪");
    }

    RCLCPP_INFO(logger_, "🎉 Stand 序列完成! 机器人已就绪 (RL模式)");
    return {true, "Robot is standing in RL mode with mountain gait."};
}

}  // namespace nav_bridge
