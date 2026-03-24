#pragma once

/// @file robot_state_store.hpp
/// @brief 机器人状态仓库，统一管理连接、基本状态和步态状态

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <vector>

#include "nav_bridge/x30_protocol.hpp"

namespace nav_bridge {

struct StateSnapshot {
    uint8_t basic_state{0};
    uint8_t gait_state{0};
    bool connected{false};
    bool rcs_received{false};
    std::chrono::steady_clock::time_point last_receive_time{
        std::chrono::steady_clock::time_point::min()};
};

struct MotionStateTransition {
    uint8_t previous_basic_state{0};
    uint8_t previous_gait_state{0};
    uint8_t current_basic_state{0};
    uint8_t current_gait_state{0};
};

class RobotStateStore {
public:
    MotionStateTransition updateMotionState(uint8_t basic_state, uint8_t gait_state) {
        std::lock_guard<std::mutex> lock(mutex_);
        MotionStateTransition transition;
        transition.previous_basic_state = basic_state_;
        transition.previous_gait_state  = gait_state_;
        basic_state_                    = basic_state;
        gait_state_                     = gait_state;
        transition.current_basic_state  = basic_state_;
        transition.current_gait_state   = gait_state_;
        cv_.notify_all();
        return transition;
    }

    void markPacketReceived(std::chrono::steady_clock::time_point now) {
        connected_.store(true);
        last_receive_time_.store(now);
    }

    void markDisconnected() {
        connected_.store(false);
    }

    bool connected() const {
        return connected_.load();
    }

    std::chrono::steady_clock::time_point lastReceiveTime() const {
        return last_receive_time_.load();
    }

    StateSnapshot snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return StateSnapshot{
            basic_state_, gait_state_, connected_.load(), rcs_received_,
            last_receive_time_.load()};
    }

    uint8_t basicState() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return basic_state_;
    }

    uint8_t gaitState() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return gait_state_;
    }

    bool markRcsReceived() {
        std::lock_guard<std::mutex> lock(mutex_);
        bool first = !rcs_received_;
        rcs_received_ = true;
        return first;
    }

    bool waitForBasicState(const std::vector<x30_protocol::BasicState> &targets, int timeout_ms) {
        auto matches = [this, &targets]() {
            for (const auto &target : targets) {
                if (basic_state_ == static_cast<uint8_t>(target)) {
                    return true;
                }
            }
            return false;
        };

        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), matches);
    }

    bool waitForGaitState(const std::vector<x30_protocol::GaitState> &targets, int timeout_ms) {
        auto matches = [this, &targets]() {
            for (const auto &target : targets) {
                if (gait_state_ == static_cast<uint8_t>(target)) {
                    return true;
                }
            }
            return false;
        };

        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), matches);
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    uint8_t basic_state_{0};
    uint8_t gait_state_{0};
    bool rcs_received_{false};
    std::atomic<bool> connected_{false};
    std::atomic<std::chrono::steady_clock::time_point> last_receive_time_{
        std::chrono::steady_clock::now()};
};

}  // namespace nav_bridge
