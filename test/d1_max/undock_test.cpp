#include <gtest/gtest.h>

#include <atomic>
#include <functional>
#include <future>
#include <thread>

#include "nav_bridge/d1_max/d1_max_backend.hpp"
#include "robot_sdk/sdk_client.hpp"

using namespace std::chrono_literals;

// Interpose only the SDK calls exercised here. Real backend callbacks and
// charge methods run unchanged, without a robot connection or network commands.
namespace sdk_charge_test {
std::weak_ptr<robot_sdk::IDataCallback> callback;
std::atomic<bool> connected{true};
std::atomic<int> undock_calls{0};
std::atomic<int> stop_calls{0};
std::atomic<int> control_calls{0};
std::atomic<int> move_calls{0};
std::atomic<int> recharge_calls{0};
std::atomic<bool> acknowledge_stop{true};
std::function<void()> on_stop;

void task(robot_sdk::TaskType type, robot_sdk::TaskStatus status) {
    robot_sdk::TaskStateInfo data{};
    data.task_type = type;
    data.task_status = status;
    callback.lock()->OnTaskStateData(data);
}

void basic(bool bluetooth, robot_sdk::PowerSupplyStatus power = robot_sdk::PowerSupplyStatus::DISCHARGING,
           bool present = true, robot_sdk::MachineStatus machine = robot_sdk::MachineStatus::REMOTE) {
    robot_sdk::RobotState data{};
    data.charging_pile_connected = bluetooth;
    data.battery.present1 = present;
    data.battery.power_supply_status1 = power;
    data.motion_status = robot_sdk::MotionStatus::MOTION_STATUS_GAIT;
    data.machine_status = machine;
    callback.lock()->OnRobotStateData(data);
}
}  // namespace sdk_charge_test

namespace robot_sdk {
bool SDKClient::IsConnected() const { return sdk_charge_test::connected; }
std::error_code SDKClient::Disconnect(bool, DisConnectHandler) {
    sdk_charge_test::connected = false;
    return {};
}
void SDKClient::SetDataCallback(std::shared_ptr<IDataCallback> callback) {
    sdk_charge_test::callback = callback;
}
std::error_code SDKClient::TakeControl(int, WriteHandler) {
    ++sdk_charge_test::control_calls;
    return {};
}
std::error_code SDKClient::Move(float, float, float, int, WriteHandler) {
    ++sdk_charge_test::move_calls;
    return {};
}
std::error_code SDKClient::StartRechargeTask(int, WriteHandler) {
    ++sdk_charge_test::recharge_calls;
    sdk_charge_test::task(TaskType::RECHARGING, TaskStatus::RUNNING);
    sdk_charge_test::basic(true, PowerSupplyStatus::CHARGING);
    return {};
}
std::error_code SDKClient::StopRechargeTask(int, WriteHandler) {
    ++sdk_charge_test::stop_calls;
    if (sdk_charge_test::acknowledge_stop)
        sdk_charge_test::task(TaskType::RECHARGING, TaskStatus::STOPPED);
    if (sdk_charge_test::on_stop) sdk_charge_test::on_stop();
    return {};
}
std::error_code SDKClient::StartUnDockTask(int, WriteHandler) {
    ++sdk_charge_test::undock_calls;
    sdk_charge_test::task(TaskType::UNDOCK, TaskStatus::RUNNING);
    return {};
}
}  // namespace robot_sdk

namespace nav_bridge {
using robot_sdk::PowerSupplyStatus;
using robot_sdk::TaskStatus;
using robot_sdk::TaskType;
using sdk_charge_test::basic;
using sdk_charge_test::task;

class D1MaxUndockTest : public testing::Test {
protected:
    D1MaxBackend backend{"127.0.0.1", 8082, false, 100, 100, false};

    void SetUp() override {
        sdk_charge_test::connected = true;
        sdk_charge_test::undock_calls = 0;
        sdk_charge_test::stop_calls = 0;
        sdk_charge_test::control_calls = 0;
        sdk_charge_test::move_calls = 0;
        sdk_charge_test::recharge_calls = 0;
        sdk_charge_test::acknowledge_stop = true;
        sdk_charge_test::on_stop = {};
    }

    std::future<BackendResult> exit(int timeout_ms = 500) {
        return std::async(std::launch::async, [this, timeout_ms] {
            return backend.startUndock(timeout_ms);
        });
    }

    bool waitForUndockCall() {
        const auto deadline = std::chrono::steady_clock::now() + 300ms;
        while (sdk_charge_test::undock_calls == 0 && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(1ms);
        return sdk_charge_test::undock_calls > 0;
    }

    bool required() {
        std::lock_guard<std::mutex> lock(backend.mutex_);
        return backend.undock_required_;
    }

    void stale() {
        std::lock_guard<std::mutex> lock(backend.mutex_);
        backend.last_basic_state_time_ = std::chrono::steady_clock::now() - 10s;
    }

    BackendResult confirmRecharge() { return backend.waitForRechargeStarted(0, 50); }

    void beginUndock(bool bluetooth = true) {
        basic(bluetooth);
        task(TaskType::UNDOCK, TaskStatus::RUNNING);
    }

    void succeed(bool bluetooth = true, PowerSupplyStatus power = PowerSupplyStatus::DISCHARGING) {
        task(TaskType::UNDOCK, TaskStatus::SUCCESS);
        basic(bluetooth, power);
    }
};

TEST_F(D1MaxUndockTest, RechargeRequiresPileConnectionBeforeAnySdkCommand) {
    basic(false);
    const auto result = backend.startRecharge(50, 50);
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.message.find("Timed out waiting"), std::string::npos);
    EXPECT_EQ(sdk_charge_test::control_calls, 0);
    EXPECT_EQ(sdk_charge_test::move_calls, 0);
    EXPECT_EQ(sdk_charge_test::recharge_calls, 0);
}

TEST_F(D1MaxUndockTest, MissingOrStalePileConnectionPreventsRecharge) {
    const auto missing = backend.startRecharge(50, 50);
    EXPECT_FALSE(missing.success);
    EXPECT_NE(missing.message.find("Timed out waiting"), std::string::npos);
    basic(true);
    stale();
    const auto expired = backend.startRecharge(50, 50);
    EXPECT_FALSE(expired.success);
    EXPECT_NE(expired.message.find("Timed out waiting"), std::string::npos);
    EXPECT_EQ(sdk_charge_test::control_calls, 0);
    EXPECT_EQ(sdk_charge_test::move_calls, 0);
    EXPECT_EQ(sdk_charge_test::recharge_calls, 0);
}

TEST_F(D1MaxUndockTest, ConnectedPileStartsRechargeAndConfirmsBatteryCharging) {
    basic(true);
    EXPECT_TRUE(backend.startRecharge(50, 50).success);
    EXPECT_EQ(sdk_charge_test::control_calls, 1);
    EXPECT_EQ(sdk_charge_test::move_calls, 1);
    EXPECT_EQ(sdk_charge_test::recharge_calls, 1);
}

TEST_F(D1MaxUndockTest, ConnectedPileCannotBypassDisconnectedSdk) {
    basic(true);
    sdk_charge_test::connected = false;
    EXPECT_FALSE(backend.startRecharge(50, 50).success);
    EXPECT_EQ(sdk_charge_test::control_calls, 0);
    EXPECT_EQ(sdk_charge_test::move_calls, 0);
    EXPECT_EQ(sdk_charge_test::recharge_calls, 0);
}

TEST_F(D1MaxUndockTest, RechargeWaitsForPileConnectionBeforeSendingCommands) {
    basic(false);
    auto result = std::async(std::launch::async, [this] { return backend.startRecharge(50, 1000); });
    EXPECT_EQ(result.wait_for(30ms), std::future_status::timeout);
    basic(false);
    EXPECT_EQ(result.wait_for(30ms), std::future_status::timeout);
    EXPECT_EQ(sdk_charge_test::control_calls, 0);
    EXPECT_EQ(sdk_charge_test::move_calls, 0);
    EXPECT_EQ(sdk_charge_test::recharge_calls, 0);
    basic(true);
    EXPECT_TRUE(result.get().success);
    EXPECT_EQ(sdk_charge_test::recharge_calls, 1);
}

TEST_F(D1MaxUndockTest, RechargeWaitsForFirstOrRefreshedPileState) {
    auto first = std::async(std::launch::async, [this] { return backend.startRecharge(50, 1000); });
    EXPECT_EQ(first.wait_for(30ms), std::future_status::timeout);
    basic(true);
    EXPECT_TRUE(first.get().success);

    stale();
    auto refreshed = std::async(std::launch::async, [this] { return backend.startRecharge(50, 1000); });
    EXPECT_EQ(refreshed.wait_for(30ms), std::future_status::timeout);
    EXPECT_EQ(sdk_charge_test::recharge_calls, 1);
    basic(true);
    EXPECT_TRUE(refreshed.get().success);
    EXPECT_EQ(sdk_charge_test::recharge_calls, 2);
}

TEST_F(D1MaxUndockTest, StandingOffDockWithBluetoothNeedsNoSdkCommand) {
    basic(true);
    EXPECT_TRUE(backend.startUndock(50).success);
    EXPECT_EQ(sdk_charge_test::undock_calls, 0);
    EXPECT_EQ(sdk_charge_test::stop_calls, 0);
    EXPECT_EQ(sdk_charge_test::control_calls, 0);
}

TEST_F(D1MaxUndockTest, BluetoothUnavailableConfirmsExitWithoutBatteryStatus) {
    basic(false, PowerSupplyStatus::UNKNOWN, false);
    EXPECT_TRUE(backend.startUndock(50).success);
    EXPECT_EQ(sdk_charge_test::undock_calls, 0);
}

TEST_F(D1MaxUndockTest, MissingOrStaleTelemetryCannotConfirmExit) {
    EXPECT_FALSE(backend.startUndock(50).success);
    basic(false);
    stale();
    EXPECT_FALSE(backend.startUndock(50).success);
    EXPECT_EQ(sdk_charge_test::undock_calls, 0);
}

TEST_F(D1MaxUndockTest, UnknownBatteryWithBluetoothDoesNotTriggerBlindUndock) {
    basic(true, PowerSupplyStatus::UNKNOWN);
    EXPECT_FALSE(backend.startUndock(50).success);
    EXPECT_EQ(sdk_charge_test::undock_calls, 0);
}

TEST_F(D1MaxUndockTest, RobotSdkDisconnectedCannotBeTreatedAsPileDisconnected) {
    basic(false);
    sdk_charge_test::connected = false;
    EXPECT_FALSE(backend.startUndock(50).success);
}

TEST_F(D1MaxUndockTest, StopChangingBatteryToDischargingStillRequiresUndock) {
    basic(true, PowerSupplyStatus::CHARGING);
    task(TaskType::RECHARGING, TaskStatus::RUNNING);
    sdk_charge_test::on_stop = [] { basic(true); };
    auto result = exit();
    ASSERT_TRUE(waitForUndockCall());
    EXPECT_EQ(sdk_charge_test::stop_calls, 1);
    EXPECT_TRUE(required());
    EXPECT_EQ(result.wait_for(30ms), std::future_status::timeout);
    succeed();
    EXPECT_TRUE(result.get().success);
    EXPECT_FALSE(required());
    EXPECT_TRUE(backend.startUndock(50).success);
    EXPECT_EQ(sdk_charge_test::undock_calls, 1);
}

TEST_F(D1MaxUndockTest, BluetoothDisconnectAfterStoppingRechargeNeedsNoUndock) {
    basic(true, PowerSupplyStatus::CHARGING);
    task(TaskType::RECHARGING, TaskStatus::RUNNING);
    sdk_charge_test::on_stop = [] { basic(false); };
    EXPECT_TRUE(backend.startUndock(50).success);
    EXPECT_EQ(sdk_charge_test::stop_calls, 1);
    EXPECT_EQ(sdk_charge_test::undock_calls, 0);
    EXPECT_FALSE(required());
}

TEST_F(D1MaxUndockTest, BluetoothDisconnectMustWaitForSdkRechargeStopped) {
    basic(false);
    task(TaskType::RECHARGING, TaskStatus::RUNNING);
    sdk_charge_test::acknowledge_stop = false;
    auto result = exit();
    EXPECT_EQ(result.wait_for(30ms), std::future_status::timeout);
    EXPECT_EQ(sdk_charge_test::stop_calls, 1);
    EXPECT_EQ(sdk_charge_test::undock_calls, 0);
    task(TaskType::RECHARGING, TaskStatus::STOPPED);
    basic(false);
    EXPECT_TRUE(result.get().success);
    EXPECT_EQ(sdk_charge_test::control_calls, 0);
}

TEST_F(D1MaxUndockTest, FullBatteryDoesNotClearPreviouslyObservedChargeSession) {
    basic(true, PowerSupplyStatus::CHARGING);
    basic(true, PowerSupplyStatus::FULL);
    auto result = exit();
    ASSERT_TRUE(waitForUndockCall());
    EXPECT_EQ(result.wait_for(10ms), std::future_status::timeout);
    succeed(true, PowerSupplyStatus::FULL);
    EXPECT_TRUE(result.get().success);
}

TEST_F(D1MaxUndockTest, RestartedInRechargeModeWithFullBatteryRequiresUndock) {
    basic(true, PowerSupplyStatus::FULL, true, robot_sdk::MachineStatus::RECHARGE);
    auto result = exit();
    ASSERT_TRUE(waitForUndockCall());
    EXPECT_EQ(result.wait_for(10ms), std::future_status::timeout);
    succeed(true, PowerSupplyStatus::FULL);
    EXPECT_TRUE(result.get().success);
}

TEST_F(D1MaxUndockTest, BluetoothConnectedDoesNotBlockSuccessfulUndock) {
    beginUndock();
    auto result = exit();
    std::this_thread::sleep_for(20ms);
    succeed(true);
    EXPECT_TRUE(result.get().success);
    EXPECT_EQ(sdk_charge_test::undock_calls, 0);
}

TEST_F(D1MaxUndockTest, BluetoothDisconnectCannotBypassRunningSdkTask) {
    beginUndock(false);
    auto result = exit();
    EXPECT_EQ(result.wait_for(30ms), std::future_status::timeout);
    succeed(false);
    EXPECT_TRUE(result.get().success);
    EXPECT_EQ(sdk_charge_test::undock_calls, 0);
}

TEST_F(D1MaxUndockTest, SuccessfulSdkTaskWaitsForBatteryToStopCharging) {
    beginUndock();
    auto result = exit();
    std::this_thread::sleep_for(20ms);
    succeed(true, PowerSupplyStatus::CHARGING);
    EXPECT_EQ(result.wait_for(30ms), std::future_status::timeout);
    basic(true);
    EXPECT_TRUE(result.get().success);
}

TEST_F(D1MaxUndockTest, SuccessfulSdkTaskAcceptsBluetoothDisconnect) {
    beginUndock();
    auto result = exit();
    std::this_thread::sleep_for(20ms);
    succeed(false, PowerSupplyStatus::UNKNOWN);
    EXPECT_TRUE(result.get().success);
}

TEST_F(D1MaxUndockTest, TelemetryBeforeSdkSuccessCannotCompleteExit) {
    beginUndock();
    auto result = exit();
    std::this_thread::sleep_for(20ms);
    basic(true);
    task(TaskType::UNDOCK, TaskStatus::SUCCESS);
    EXPECT_EQ(result.wait_for(30ms), std::future_status::timeout);
    basic(true);
    EXPECT_TRUE(result.get().success);
}

TEST_F(D1MaxUndockTest, FailedSdkTaskCannotBeBypassedByBluetoothDisconnect) {
    beginUndock(false);
    auto result = exit();
    std::this_thread::sleep_for(20ms);
    task(TaskType::UNDOCK, TaskStatus::FAILURE);
    basic(false);
    EXPECT_FALSE(result.get().success);
    EXPECT_TRUE(required());
}

TEST_F(D1MaxUndockTest, RetryAfterTimeoutJoinsSdkTaskWithoutNewCommand) {
    basic(true, PowerSupplyStatus::CHARGING);
    EXPECT_FALSE(backend.startUndock(30).success);
    auto result = exit();
    EXPECT_EQ(result.wait_for(30ms), std::future_status::timeout);
    EXPECT_EQ(sdk_charge_test::undock_calls, 1);
    succeed();
    EXPECT_TRUE(result.get().success);
}

TEST_F(D1MaxUndockTest, ChargingStateAndConfirmationDoNotRequireBluetooth) {
    basic(false, PowerSupplyStatus::CHARGING);
    EXPECT_EQ(backend.chargeState(), 2);
    task(TaskType::RECHARGING, TaskStatus::RUNNING);
    EXPECT_TRUE(confirmRecharge().success);
    basic(true);
    EXPECT_EQ(backend.chargeState(), 1);
    EXPECT_FALSE(confirmRecharge().success);
}

TEST_F(D1MaxUndockTest, ChargingTaskAloneIsNotBatteryCharging) {
    basic(true);
    task(TaskType::RECHARGING, TaskStatus::RUNNING);
    EXPECT_EQ(backend.chargeState(), 1);
    EXPECT_FALSE(confirmRecharge().success);
}

TEST_F(D1MaxUndockTest, ConfirmedExitClearsHistoricalFailureForExecutorIdleCheck) {
    basic(true);
    task(TaskType::RECHARGING, TaskStatus::RUNNING);
    task(TaskType::RECHARGING, TaskStatus::FAILURE);
    EXPECT_EQ(backend.chargeState(), 4);
    basic(false);
    EXPECT_TRUE(backend.startUndock(50).success);
    EXPECT_EQ(backend.chargeState(), 0);
}

TEST_F(D1MaxUndockTest, CompletedUndockCannotSatisfyLaterFullBatteryChargeSession) {
    beginUndock();
    auto first = exit();
    EXPECT_EQ(first.wait_for(20ms), std::future_status::timeout);
    succeed();
    ASSERT_TRUE(first.get().success);
    basic(true, PowerSupplyStatus::CHARGING);
    basic(true, PowerSupplyStatus::FULL);
    auto second = exit();
    ASSERT_TRUE(waitForUndockCall());
    EXPECT_EQ(second.wait_for(20ms), std::future_status::timeout);
    succeed(true, PowerSupplyStatus::FULL);
    EXPECT_TRUE(second.get().success);
    EXPECT_EQ(sdk_charge_test::undock_calls, 1);
    EXPECT_EQ(backend.chargeState(), 0);
}
}  // namespace nav_bridge
