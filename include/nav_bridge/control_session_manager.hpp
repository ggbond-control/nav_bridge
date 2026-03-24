#pragma once

/// @file control_session_manager.hpp
/// @brief 控制会话管理，统一处理心跳、控制接管和超时释放

#include <chrono>
#include <mutex>

namespace nav_bridge {

struct ControlActions {
    bool session_started{false};
    bool session_stopped{false};
    bool send_heartbeat{false};
    bool send_query{false};
    bool send_zero_velocity{false};
    bool active{false};
};

class ControlSessionManager {
public:
    ControlSessionManager() = default;

    ControlSessionManager(int heartbeat_interval_ms, int session_timeout_ms)
        : heartbeat_interval_ms_(heartbeat_interval_ms), session_timeout_ms_(session_timeout_ms) {}

    void configure(int heartbeat_interval_ms, int session_timeout_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        heartbeat_interval_ms_ = heartbeat_interval_ms;
        session_timeout_ms_    = session_timeout_ms;
    }

    void noteActivity(std::chrono::steady_clock::time_point now) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_activity_time_ = now;
    }

    void holdUntil(std::chrono::steady_clock::time_point deadline) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (deadline > hold_until_) {
            hold_until_ = deadline;
        }
    }

    void clearHoldAtOrBefore(std::chrono::steady_clock::time_point deadline) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (hold_until_ <= deadline) {
            hold_until_ = std::chrono::steady_clock::time_point::min();
        }
    }

    ControlActions ensureSession(std::chrono::steady_clock::time_point now) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_activity_time_ = now;
        return evaluateLocked(now, true);
    }

    ControlActions tick(std::chrono::steady_clock::time_point now) {
        std::lock_guard<std::mutex> lock(mutex_);
        return evaluateLocked(now, false);
    }

    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        active_              = false;
        query_sent_          = false;
        last_heartbeat_sent_ = std::chrono::steady_clock::time_point::min();
        hold_until_          = std::chrono::steady_clock::time_point::min();
        had_active_session_  = false;
    }

    bool hasHadActiveSession() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return had_active_session_;
    }

private:
    ControlActions evaluateLocked(std::chrono::steady_clock::time_point now, bool force_heartbeat) {
        ControlActions actions;

        bool activity_alive = last_activity_time_ != std::chrono::steady_clock::time_point::min() &&
                              now - last_activity_time_ <
                                  std::chrono::milliseconds(session_timeout_ms_);
        bool hold_alive =
            hold_until_ != std::chrono::steady_clock::time_point::min() && now < hold_until_;
        bool should_be_active = activity_alive || hold_alive;

        if (active_ && !should_be_active) {
            active_                  = false;
            query_sent_              = false;
            last_heartbeat_sent_     = std::chrono::steady_clock::time_point::min();
            actions.session_stopped  = true;
            actions.send_zero_velocity = true;
            actions.active           = false;
            return actions;
        }

        if (!active_ && !should_be_active) {
            return actions;
        }

        if (!active_ && should_be_active) {
            active_                 = true;
            had_active_session_     = true;
            actions.session_started = true;
            force_heartbeat         = true;
        }

        if (force_heartbeat ||
            last_heartbeat_sent_ == std::chrono::steady_clock::time_point::min() ||
            now - last_heartbeat_sent_ >= std::chrono::milliseconds(heartbeat_interval_ms_)) {
            last_heartbeat_sent_   = now;
            actions.send_heartbeat = true;
        }

        if (!query_sent_) {
            query_sent_         = true;
            actions.send_query  = true;
        }

        actions.active = true;
        return actions;
    }

    mutable std::mutex mutex_;
    int heartbeat_interval_ms_{200};
    int session_timeout_ms_{5000};
    bool active_{false};
    bool had_active_session_{false};
    bool query_sent_{false};
    std::chrono::steady_clock::time_point last_heartbeat_sent_{
        std::chrono::steady_clock::time_point::min()};
    std::chrono::steady_clock::time_point last_activity_time_{
        std::chrono::steady_clock::time_point::min()};
    std::chrono::steady_clock::time_point hold_until_{
        std::chrono::steady_clock::time_point::min()};
};

}  // namespace nav_bridge
