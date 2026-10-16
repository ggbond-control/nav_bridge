#include <gtest/gtest.h>

#include <future>
#include <thread>

#include "nav_bridge/d1_max/d1_max_backend.hpp"

using namespace std::chrono_literals;

namespace nav_bridge {

// Exercise the actual blocking confirmation with asynchronous telemetry.
// No SDK connection or robot commands are made by these tests.
class D1MaxUndockTest : public testing::Test {
protected:
    D1MaxBackend backend{"127.0.0.1", 8082, false, 100, 100, false};

    std::future<BackendResult> wait(int timeout_ms = 1000) {
        return std::async(std::launch::async, [this, timeout_ms] {
            return backend.waitForUndockCompleted(10, timeout_ms);
        });
    }

    void report(bool success, bool connected, bool fresh = true, bool failure = false) {
        {
            std::lock_guard<std::mutex> lock(backend.mutex_);
            backend.undock_success_sequence_ = success ? 11 : 0;
            backend.undock_failure_sequence_ = failure ? 12 : 0;
            backend.charging_pile_connected_ = connected;
            backend.last_basic_state_time_ = std::chrono::steady_clock::now() -
                (fresh ? 0s : 10s);
        }
        backend.motion_cv_.notify_all();
    }
};

TEST_F(D1MaxUndockTest, SuccessWaitsForDisconnection) {
    auto result = wait();
    std::this_thread::sleep_for(20ms);
    report(true, true);
    EXPECT_EQ(result.wait_for(60ms), std::future_status::timeout);
    report(true, false);
    ASSERT_EQ(result.wait_for(300ms), std::future_status::ready);
    EXPECT_TRUE(result.get().success);
}

TEST_F(D1MaxUndockTest, DisconnectionWaitsForSdkSuccess) {
    auto result = wait();
    std::this_thread::sleep_for(20ms);
    report(false, false);
    EXPECT_EQ(result.wait_for(60ms), std::future_status::timeout);
    report(true, false);
    ASSERT_EQ(result.wait_for(300ms), std::future_status::ready);
    EXPECT_TRUE(result.get().success);
}

TEST_F(D1MaxUndockTest, OldDisconnectedSampleCannotCompleteExit) {
    auto result = wait(150);
    std::this_thread::sleep_for(20ms);
    report(true, false, false);
    EXPECT_FALSE(result.get().success);
}

TEST_F(D1MaxUndockTest, FailureWinsOverSuccessAndDisconnection) {
    auto result = wait();
    std::this_thread::sleep_for(20ms);
    report(true, false, true, true);
    EXPECT_FALSE(result.get().success);
}

}  // namespace nav_bridge
