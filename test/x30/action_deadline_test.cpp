#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <vector>

#include "nav_bridge/x30/action_executor.hpp"

using namespace std::chrono_literals;
using namespace nav_bridge;
using namespace nav_bridge::x30_protocol;

TEST(X30ActionDeadline, ChargePreparationCannotOutliveItsServiceBudget) {
    RobotStateStore store;
    store.updateMotionState(static_cast<uint8_t>(BasicState::RL_MODE),
                            static_cast<uint8_t>(GaitState::MOUNTAIN));
    std::vector<uint32_t> commands;
    ActionExecutor executor(rclcpp::get_logger("x30_deadline_test"), store,
        [](int warmup_ms, int) { std::this_thread::sleep_for(std::chrono::milliseconds(warmup_ms)); },
        [&](uint32_t command) { commands.push_back(command); });
    const auto start = std::chrono::steady_clock::now();
    const auto result = executor.forceStand(start + 80ms);
    EXPECT_FALSE(result.success);
    EXPECT_LT(std::chrono::steady_clock::now() - start, 250ms);
    const auto count = commands.size();
    EXPECT_FALSE(executor.forceStand(start).success);
    EXPECT_EQ(commands.size(), count);  // An expired request must not send a motion command.
}

TEST(X30ActionDeadline, FreshGaitWaitSharesTheRemainingBudget) {
    RobotStateStore store;
    store.updateMotionState(static_cast<uint8_t>(BasicState::RL_MODE),
                            static_cast<uint8_t>(GaitState::MOUNTAIN));
    std::vector<uint32_t> commands;
    ActionExecutor executor(rclcpp::get_logger("x30_deadline_test"), store,
        [](int, int) {}, [&](uint32_t command) { commands.push_back(command); });
    const auto start = std::chrono::steady_clock::now();
    EXPECT_FALSE(executor.forceStand(start + 80ms).success);
    EXPECT_LT(std::chrono::steady_clock::now() - start, 250ms);
    EXPECT_EQ(commands, std::vector<uint32_t>({CMD_GAIT_WALK}));
    store.updateMotionState(static_cast<uint8_t>(BasicState::FORCE_STAND),
                            static_cast<uint8_t>(GaitState::WALK));
    EXPECT_TRUE(executor.forceStand().success);
}
