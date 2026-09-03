#include "nav_bridge/d1_max/d1_max_backend.hpp"

#include <algorithm>
#include <cmath>
#include <system_error>
#include <thread>
#include <chrono>

#include "robot_sdk/sdk_callback.hpp"
#include "robot_sdk/sdk_client.hpp"

namespace nav_bridge {
namespace {

BackendMotionState mapMotionState(robot_sdk::MotionStatus status) {
    using robot_sdk::MotionStatus;
    switch (status) {
    case MotionStatus::MOTION_STATUS_LIE_DOWN: return BackendMotionState::LYING_DOWN;
    case MotionStatus::MOTION_STATUS_STAND_UP: return BackendMotionState::STANDING_UP;
    case MotionStatus::MOTION_STATUS_LOCKED: return BackendMotionState::LOCKED;
    case MotionStatus::MOTION_STATUS_WALK:
    case MotionStatus::MOTION_STATUS_BALANCE_STAND:
    case MotionStatus::MOTION_STATUS_STAIR:
    case MotionStatus::MOTION_STATUS_CRAWL:
    case MotionStatus::MOTION_STATUS_CRAWL_WALK:
    case MotionStatus::MOTION_STATUS_CLIMB:
    case MotionStatus::MOTION_STATUS_SLIM:
    case MotionStatus::MOTION_STATUS_GAIT:
    case MotionStatus::MOTION_STATUS_DSB:
    case MotionStatus::MOTION_STATUS_POS_CONTROL:
    case MotionStatus::MOTION_STATUS_SK_WALK:
    case MotionStatus::MOTION_STATUS_SAND: return BackendMotionState::MOVING;
    default: return BackendMotionState::UNKNOWN;
    }
}

}  // namespace

class D1MaxBackend::DataCallback final : public robot_sdk::IDataCallback {
public:
    explicit DataCallback(D1MaxBackend &owner) : owner_(owner) {}

    void OnRobotStateData(const robot_sdk::RobotState &data) override {
        BackendState state;
        state.connected = true;
        state.motion_state = mapMotionState(data.motion_status);
        {
            std::lock_guard<std::mutex> lock(owner_.mutex_);
            state.mode = owner_.navigation_gait_;
            owner_.motion_status_ = static_cast<int>(data.motion_status);
            owner_.machine_status_ = static_cast<int>(data.machine_status);
            owner_.charging_pile_connected_ = data.charging_pile_connected;
            owner_.battery1_present_ = data.battery.present1;
            owner_.battery2_present_ = data.battery.present2;
            owner_.battery1_power_supply_status_ =
                static_cast<int>(data.battery.power_supply_status1);
            owner_.battery2_power_supply_status_ =
                static_cast<int>(data.battery.power_supply_status2);
        }
        state.control_owned = data.control_source == robot_sdk::CtrlSource::CTRL_SOURCE_SDK;
        state.vx = data.speed.line;
        state.vy = data.speed.translation;
        state.vyaw = data.speed.angle;
        owner_.motion_cv_.notify_all();
        const bool p1 = data.battery.present1;
        const bool p2 = data.battery.present2;
        if (p1 && p2) state.battery_percent = (data.battery.power1 + data.battery.power2) * 0.5;
        else if (p1) state.battery_percent = data.battery.power1;
        else if (p2) state.battery_percent = data.battery.power2;
        owner_.updateState(state);
    }

    void OnTaskStateData(const robot_sdk::TaskStateInfo &data) override {
        {
            std::lock_guard<std::mutex> lock(owner_.mutex_);
            owner_.task_type_ = static_cast<int>(data.task_type);
            owner_.task_status_ = static_cast<int>(data.task_status);
            owner_.task_error_code_ = data.error_code;
            ++owner_.task_state_sequence_;
        }
        owner_.motion_cv_.notify_all();
    }

    void OnImuData(const robot_sdk::ImuData &data) override {
        BackendImu sample{data.acc_x, data.acc_y, data.acc_z, data.gyro_x, data.gyro_y,
                          data.gyro_z, data.quat_x, data.quat_y, data.quat_z, data.quat_w};
        ImuCallback callback;
        { std::lock_guard<std::mutex> lock(owner_.mutex_); callback = owner_.imu_callback_; }
        if (callback) callback(sample);
    }

    void OnMcData(const robot_sdk::MotionData &data) override {
        BackendOdometry sample;
        sample.qw = data.quat[0]; sample.qx = data.quat[1]; sample.qy = data.quat[2]; sample.qz = data.quat[3];
        sample.vx = data.v_body[0]; sample.vy = data.v_body[1]; sample.vz = data.v_body[2];
        sample.wx = data.omega_body[0]; sample.wy = data.omega_body[1]; sample.wz = data.omega_body[2];
        sample.px = data.position[0]; sample.py = data.position[1]; sample.pz = data.position[2];
        sample.timestamp_ns = data.time_stamp;
        OdometryCallback callback;
        { std::lock_guard<std::mutex> lock(owner_.mutex_); callback = owner_.odom_callback_; }
        if (callback) callback(sample);
    }

    void OnJointStateData(const robot_sdk::JointStateData &data) override {
        BackendJointState sample{data.names, data.positions, data.velocities, data.efforts};
        JointCallback callback;
        { std::lock_guard<std::mutex> lock(owner_.mutex_); callback = owner_.joint_callback_; }
        if (callback) callback(sample);
    }

    void OnFaultData(const robot_sdk::FaultDatas &data) override {
        FaultCallback callback;
        { std::lock_guard<std::mutex> lock(owner_.mutex_); callback = owner_.fault_callback_; }
        if (!callback) return;
        for (const auto &fault : data) callback({static_cast<int>(fault.code), static_cast<int>(fault.level), fault.message});
    }

    void OnSpeedData(const robot_sdk::SpeedData &data) override {
        auto state = owner_.state();
        state.connected = true;
        state.vx = data.x;
        state.vy = data.y;
        state.vyaw = data.yaw;
        owner_.updateState(state);
    }

    void OnControlLost(const robot_sdk::ControlLostInfo &) override {
        auto state = owner_.state();
        state.control_owned = false;
        owner_.updateState(state);
    }

    void OnControlAvailable(const robot_sdk::ControlAvailableInfo &) override {
        auto state = owner_.state();
        state.control_owned = false;
        owner_.updateState(state);
    }

private:
    D1MaxBackend &owner_;
};

class D1MaxBackend::ControlCallback final : public robot_sdk::IControlCallback {
public:
    explicit ControlCallback(D1MaxBackend &owner) : owner_(owner) {}

    void OnImuConfig(int freq) override {
        // The SDK reports the accepted configuration asynchronously. Keep this
        // callback implemented so the response is not silently discarded.
        std::lock_guard<std::mutex> lock(owner_.mutex_);
        owner_.imu_configured_hz_ = freq;
        (void)freq;
    }

private:
    D1MaxBackend &owner_;
};

D1MaxBackend::D1MaxBackend(std::string host_ip, int host_port, bool auto_reconnect,
                           int connect_timeout_ms, int reconnect_interval_ms,
                           bool enable_sdk_imu)
    : host_ip_(std::move(host_ip)), host_port_(host_port), auto_reconnect_(auto_reconnect),
      connect_timeout_ms_(connect_timeout_ms), reconnect_interval_ms_(reconnect_interval_ms),
      enable_sdk_imu_(enable_sdk_imu) {
    robot_sdk::ConnectionConfig config;
    config.auto_reconnect = auto_reconnect_;
    config.connect_timeout_ms = connect_timeout_ms_;
    config.reconnect_interval_ms = reconnect_interval_ms_;
    client_ = std::make_unique<robot_sdk::SDKClient>(
        [this](const std::error_code &) {
            auto state = this->state();
            state.connected = false;
            state.control_owned = false;
            this->updateState(state);
        }, config, robot_sdk::TransportProtocol::Udp);
    data_callback_ = std::make_shared<DataCallback>(*this);
    control_callback_ = std::make_shared<ControlCallback>(*this);
    client_->SetDataCallback(data_callback_);
    client_->SetControlCallback(control_callback_);
}

D1MaxBackend::~D1MaxBackend() { disconnect(); }

BackendResult D1MaxBackend::fromError(const std::error_code &ec) const {
    return {!ec, ec ? ec.message() : "OK"};
}

BackendResult D1MaxBackend::connect() {
    auto ec = client_->Connect(host_ip_, std::to_string(host_port_), true);
    if (!ec) {
        auto state = this->state();
        state.connected = true;
        updateState(state);
        // The handshake completion does not guarantee that the robot's
        // command endpoint is ready in the same scheduling slice. The official
        // SDK examples configure sensors after the connection callback; mirror
        // that ordering and use synchronous writes so failures are observable.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (enable_sdk_imu_) {
            ec = client_->SetImuConfig(100, connect_timeout_ms_);
            if (ec) {
                auto state = this->state();
                state.connected = false;
                updateState(state);
                return fromError(ec);
            }
        }
        client_->SetMcConfig(true);
        client_->SetSpeedReportConfig(true, 50);
        client_->SetJointStateConfig(true);
    }
    return fromError(ec);
}

BackendResult D1MaxBackend::disconnect() {
    if (!client_ || !client_->IsConnected()) return {true, "Already disconnected."};
    auto ec = client_->Disconnect(true);
    auto state = this->state();
    state.connected = false;
    state.control_owned = false;
    updateState(state);
    return fromError(ec);
}

BackendResult D1MaxBackend::takeControl() {
    const auto result = fromError(client_->TakeControl(connect_timeout_ms_));
    if (result.success) {
        auto state = this->state();
        state.connected = true;
        state.control_owned = true;
        updateState(state);
    }
    return result;
}

BackendResult D1MaxBackend::releaseControl() {
    if (!client_ || !client_->IsConnected()) return {false, "D1 SDK is not connected."};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!state_.control_owned) {
            return {true, "No active control session to release."};
        }
    }
    auto ec = client_->Move(0.0F, 0.0F, 0.0F, connect_timeout_ms_);
    if (ec) return fromError(ec);
    auto result = fromError(client_->ReleaseControl(connect_timeout_ms_));
    auto state = this->state();
    state.control_owned = false;
    updateState(state);
    return result;
}

BackendResult D1MaxBackend::move(double vx, double vy, double vyaw) {
    const auto clamp = [](double value) { return std::max(-1.0, std::min(1.0, value)); };
    // SDK order is left/right, forward/back, yaw; ROS order is x, y, yaw.
    return fromError(client_->Move(static_cast<float>(clamp(vy)), static_cast<float>(clamp(vx)),
                                   static_cast<float>(clamp(vyaw))));
}

BackendResult D1MaxBackend::stand() {
    if (!client_ || !client_->IsConnected()) return {false, "D1 SDK is not connected."};

    auto status = [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        return motion_status_;
    };
    auto wait_status = [this, &status](std::initializer_list<int> targets, int timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        return motion_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&]() {
            return std::find(targets.begin(), targets.end(), motion_status_) != targets.end();
        });
    };
    const auto is_general = [](int s) {
        return s == static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_WALK) ||
               s == static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_BALANCE_STAND) ||
               s == static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_GAIT);
    };

    int current = status();
    if (current == static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_LOCKED)) {
        auto ec = client_->SoftEmergencyStop(false, connect_timeout_ms_);
        if (ec) return fromError(ec);
        if (!wait_status({static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_LIE_DOWN),
                          static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_STAND_UP),
                          static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_WALK)},
                         connect_timeout_ms_)) {
            return {false, "Timeout recovering from D1 soft emergency stop."};
        }
        current = status();
    }

    auto control = takeControl();
    if (!control.success) return control;

    // Stop any active command stream before changing posture. The SDK does not
    // expose a separate stopped MotionStatus, so allow the controller one
    // reporting interval to settle after a zero command.
    if (!is_general(current) || current == static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_WALK)) {
        auto ec = client_->Move(0.0F, 0.0F, 0.0F, connect_timeout_ms_);
        if (ec) return fromError(ec);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    current = status();
    const bool needs_general_mode = !is_general(current);
    if (!is_general(current)) {
        std::error_code ec;
        if (current == static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_LIE_DOWN)) {
            ec = client_->StandUp(connect_timeout_ms_);
        } else {
            ec = client_->BalanceStandUp(connect_timeout_ms_);
        }
        if (ec) return fromError(ec);
        if (!wait_status({static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_WALK),
                          static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_BALANCE_STAND),
                          static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_GAIT)},
                         connect_timeout_ms_)) {
            return {false, "Timeout waiting for D1 stand completion."};
        }
    }

    current = status();
    if (needs_general_mode) {
        auto ec = client_->Gait(connect_timeout_ms_);
        if (ec) return fromError(ec);
        if (!wait_status({static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_GAIT),
                          static_cast<int>(robot_sdk::MotionStatus::MOTION_STATUS_WALK)},
                         connect_timeout_ms_)) {
            return {false, "Timeout switching D1 to the general navigation mode (MOUNTAIN)."};
        }
    }

    auto speed = setSpeed(static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_MEDIUM));
    if (!speed.success) return speed;

    auto final_state = this->state();
    final_state.connected = true;
    updateState(final_state);
    return {true, "D1 Max is standing in the general navigation mode (MOUNTAIN), medium speed."};
}

BackendResult D1MaxBackend::lie() {
    using robot_sdk::MotionStatus;
    if (!client_ || !client_->IsConnected()) return {false, "D1 SDK is not connected."};

    const int lie_down = static_cast<int>(MotionStatus::MOTION_STATUS_LIE_DOWN);
    const int crawl = static_cast<int>(MotionStatus::MOTION_STATUS_CRAWL);
    const int crawl_walk = static_cast<int>(MotionStatus::MOTION_STATUS_CRAWL_WALK);
    const int walk = static_cast<int>(MotionStatus::MOTION_STATUS_WALK);
    const int balance_stand = static_cast<int>(MotionStatus::MOTION_STATUS_BALANCE_STAND);
    const int gait = static_cast<int>(MotionStatus::MOTION_STATUS_GAIT);
    const int locked = static_cast<int>(MotionStatus::MOTION_STATUS_LOCKED);

    auto status = [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        return motion_status_;
    };
    auto wait_status = [this](std::initializer_list<int> targets, int timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        return motion_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&]() {
            return std::find(targets.begin(), targets.end(), motion_status_) != targets.end();
        });
    };
    auto wait_or_report = [&](std::initializer_list<int> targets, const char *description) {
        if (wait_status(targets, connect_timeout_ms_)) return BackendResult{true, "OK"};
        return BackendResult{false, std::string("Timeout waiting for D1 ") + description + "."};
    };

    int current = status();
    if (current == lie_down) return {true, "D1 Max is already lying down."};

    // A locked robot cannot accept posture commands. Release the soft stop and
    // wait until the SDK reports a commandable state before taking control.
    if (current == locked) {
        auto result = fromError(client_->SoftEmergencyStop(false, connect_timeout_ms_));
        if (!result.success) return result;
        if (!wait_status({lie_down, walk, balance_stand, gait, crawl, crawl_walk},
                         connect_timeout_ms_)) {
            return {false, "Timeout recovering from D1 soft emergency stop."};
        }
        current = status();
        if (current == lie_down) return {true, "D1 Max is already lying down."};
    }

    auto control = takeControl();
    if (!control.success) return control;

    // Stop locomotion first. CRAWL_WALK and the platform-specific modes may
    // continue their current action until a zero velocity is received.
    const bool locomotion = current == walk || current == crawl_walk ||
                            current == static_cast<int>(MotionStatus::MOTION_STATUS_STAIR) ||
                            current == static_cast<int>(MotionStatus::MOTION_STATUS_CLIMB) ||
                            current == static_cast<int>(MotionStatus::MOTION_STATUS_SK_WALK);
    if (locomotion) {
        auto result = fromError(client_->Move(0.0F, 0.0F, 0.0F, connect_timeout_ms_));
        if (!result.success) return result;
        // There is no separate STOPPED status in RobotSDK; wait for a stable
        // posture/mode report rather than assuming the command completed.
        if (!wait_status({walk, balance_stand, gait, crawl, crawl_walk,
                          static_cast<int>(MotionStatus::MOTION_STATUS_STAIR),
                          static_cast<int>(MotionStatus::MOTION_STATUS_CLIMB),
                          static_cast<int>(MotionStatus::MOTION_STATUS_SK_WALK)},
                         connect_timeout_ms_)) {
            return {false, "Timeout waiting for D1 motion to stop."};
        }
        // RobotSDK has no STOPPED status. A short settle period lets the
        // controller consume the zero command before the exit action below.
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        current = status();
    }

    // Special action modes have no generic cancel operation in RobotSDK 0.2.1.
    // Return to the general gait and wait for its acknowledgement first.
    const bool special = current == static_cast<int>(MotionStatus::MOTION_STATUS_STAIR) ||
                         current == static_cast<int>(MotionStatus::MOTION_STATUS_CLIMB) ||
                         current == static_cast<int>(MotionStatus::MOTION_STATUS_SLIM) ||
                         current == static_cast<int>(MotionStatus::MOTION_STATUS_DSB) ||
                         current == static_cast<int>(MotionStatus::MOTION_STATUS_POS_CONTROL) ||
                         current == static_cast<int>(MotionStatus::MOTION_STATUS_SK_WALK) ||
                         current == static_cast<int>(MotionStatus::MOTION_STATUS_SAND);
    if (special) {
        auto result = fromError(client_->Gait(connect_timeout_ms_));
        if (!result.success) return result;
        auto wait = wait_or_report({gait, walk, balance_stand}, "general gait");
        if (!wait.success) return wait;
        current = status();
    }

    // The D1 performs a controlled lie-down most naturally from crawl mode.
    // Stand or general gait states therefore pass through Crawl first.
    if (current != crawl) {
        auto result = fromError(client_->Crawl(connect_timeout_ms_));
        if (!result.success) return result;
        auto wait = wait_or_report({crawl}, "crawl posture");
        if (!wait.success) return wait;
    }

    // Crawl status marks the requested posture, but the controller may still
    // be settling the body and legs. Keep the robot in crawl mode for a full
    // stabilization window before issuing the lie-down action.
    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto result = fromError(client_->LieDown(connect_timeout_ms_));
    if (!result.success) return result;
    auto wait = wait_or_report({lie_down}, "lie-down completion");
    if (!wait.success) return wait;
    return {true, "D1 Max is lying down."};
}

BackendResult D1MaxBackend::softEstop(bool enabled) {
    return fromError(client_->SoftEmergencyStop(enabled, connect_timeout_ms_));
}

BackendResult D1MaxBackend::setMode(int mode) {
    (void)mode;
    return {false, "RobotSDK-0.2.1 removed SetMode; use a dedicated D1 posture command."};
}

BackendResult D1MaxBackend::setGait(int gait) {
    if (!client_ || !client_->IsConnected()) return {false, "D1 SDK is not connected."};

    using robot_sdk::MotionStatus;
    const bool general = gait == 0 || gait == 3 || gait == 32 || gait == 33 || gait == 34;
    const bool stair = gait == 6 || gait == 7 || gait == 8 || gait == 36;
    if (!general && !stair) {
        return {false, "Unsupported D1 gait. Supported: WALK(0), RUN(3), L_WALK(32), "
                       "MOUNTAIN(33), SILENT(34), STAIR_SOLID(6), STAIR_ACC(7), "
                       "STAIR45_ACC(8), L_STAIR(36)."};
    }

    auto control = takeControl();
    if (!control.success) return control;
    auto status = [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        return motion_status_;
    };
    auto wait_status = [this](std::initializer_list<int> targets, int timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        return motion_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&]() {
            return std::find(targets.begin(), targets.end(), motion_status_) != targets.end();
        });
    };
    const int current = status();
    if (current == static_cast<int>(MotionStatus::MOTION_STATUS_WALK) ||
        current == static_cast<int>(MotionStatus::MOTION_STATUS_CRAWL_WALK) ||
        current == static_cast<int>(MotionStatus::MOTION_STATUS_STAIR)) {
        auto stop = fromError(client_->Move(0.0F, 0.0F, 0.0F, connect_timeout_ms_));
        if (!stop.success) return stop;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::error_code ec;
    if (stair) {
        ec = client_->Stair(connect_timeout_ms_);
        if (ec) return fromError(ec);
        if (!wait_status({static_cast<int>(MotionStatus::MOTION_STATUS_STAIR)},
                         connect_timeout_ms_)) {
            return {false, "Timeout waiting for D1 stair mode."};
        }
    } else {
        ec = client_->Gait(connect_timeout_ms_);
        if (ec) return fromError(ec);
        if (!wait_status({static_cast<int>(MotionStatus::MOTION_STATUS_GAIT),
                          static_cast<int>(MotionStatus::MOTION_STATUS_WALK),
                          static_cast<int>(MotionStatus::MOTION_STATUS_BALANCE_STAND)},
                         connect_timeout_ms_)) {
            return {false, "Timeout waiting for D1 general gait mode."};
        }
        int speed = static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_MEDIUM);
        if (gait == 0 || gait == 32) speed = static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_SLOW);
        else if (gait == 3) speed = static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_HIGH);
        auto speed_result = setSpeed(speed);
        if (!speed_result.success) return speed_result;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        navigation_gait_ = gait;
        state_.mode = gait;
    }
    return {true, "D1 gait changed successfully."};
}

BackendResult D1MaxBackend::setSpeed(int speed_level) {
    return fromError(client_->SetSpeed(speed_level, connect_timeout_ms_));
}

BackendResult D1MaxBackend::startRecharge(int confirmation_timeout_ms) {
    if (!client_ || !client_->IsConnected()) return {false, "D1 SDK is not connected."};
    auto control = takeControl();
    if (!control.success) return control;
    auto stop = fromError(client_->Move(0.0F, 0.0F, 0.0F, connect_timeout_ms_));
    if (!stop.success) return stop;
    uint64_t sequence;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sequence = task_state_sequence_;
    }
    const auto start = fromError(client_->StartRechargeTask(connect_timeout_ms_));
    if (!start.success) return start;
    return waitForRechargeStarted(sequence, confirmation_timeout_ms);
}

BackendResult D1MaxBackend::stopRecharge(int confirmation_timeout_ms) {
    if (!client_ || !client_->IsConnected()) return {false, "D1 SDK is not connected."};
    uint64_t sequence;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool recharge_active = task_type_ == static_cast<int>(robot_sdk::TaskType::RECHARGING) &&
                                     (task_status_ == static_cast<int>(robot_sdk::TaskStatus::STARTING) ||
                                      task_status_ == static_cast<int>(robot_sdk::TaskStatus::RUNNING));
        if (!recharge_active) return {false, "D1 recharge task is not active; stop command was not sent."};
        sequence = task_state_sequence_;
    }
    const auto stop = fromError(client_->StopRechargeTask(connect_timeout_ms_));
    if (!stop.success) return stop;
    return waitForRechargeStopped(sequence, confirmation_timeout_ms);
}

BackendResult D1MaxBackend::startUndock(int confirmation_timeout_ms) {
    if (!client_ || !client_->IsConnected()) return {false, "D1 SDK is not connected."};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!charging_pile_connected_) {
            return {false, "D1 is not connected to a charging pile; undock task was not sent."};
        }
    }
    auto control = takeControl();
    if (!control.success) return control;
    uint64_t sequence;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sequence = task_state_sequence_;
    }
    const auto start = fromError(client_->StartUnDockTask(connect_timeout_ms_));
    if (!start.success) return start;
    return waitForUndockCompleted(sequence, confirmation_timeout_ms);
}

BackendResult D1MaxBackend::stopUndock(int confirmation_timeout_ms) {
    if (!client_ || !client_->IsConnected()) return {false, "D1 SDK is not connected."};
    uint64_t sequence;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sequence = task_state_sequence_;
    }
    const auto stop = fromError(client_->StopUnDockTask(connect_timeout_ms_));
    if (!stop.success) return stop;
    return waitForUndockStopped(sequence, confirmation_timeout_ms);
}

bool D1MaxBackend::isChargingLocked() const {
    const int charging = static_cast<int>(robot_sdk::PowerSupplyStatus::CHARGING);
    return (battery1_present_ && battery1_power_supply_status_ == charging) ||
           (battery2_present_ && battery2_power_supply_status_ == charging);
}

BackendResult D1MaxBackend::waitForRechargeStarted(uint64_t min_sequence, int timeout_ms) {
    using robot_sdk::TaskStatus;
    using robot_sdk::TaskType;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        const bool new_task_state = task_state_sequence_ > min_sequence &&
                                    task_type_ == static_cast<int>(TaskType::RECHARGING);
        if (new_task_state && task_status_ == static_cast<int>(TaskStatus::FAILURE)) {
            return {false, "D1 recharge task failed, error_code=" + std::to_string(task_error_code_)};
        }
        const bool task_active = task_status_ == static_cast<int>(TaskStatus::STARTING) ||
                                 task_status_ == static_cast<int>(TaskStatus::RUNNING);
        if (new_task_state && task_active && charging_pile_connected_ && isChargingLocked()) {
            return {true, "D1 is connected to the charging pile and battery charging is confirmed."};
        }
        if (motion_cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
            break;
        }
    }
    return {false, "Timed out waiting for D1 charging-pile connection and battery charging confirmation."};
}

BackendResult D1MaxBackend::waitForRechargeStopped(uint64_t min_sequence, int timeout_ms) {
    using robot_sdk::TaskStatus;
    using robot_sdk::TaskType;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        const bool stopped = task_state_sequence_ > min_sequence &&
                             task_type_ == static_cast<int>(TaskType::RECHARGING) &&
                             task_status_ == static_cast<int>(TaskStatus::STOPPED);
        if (stopped) return {true, "D1 recharge task stopped."};
        if (motion_cv_.wait_until(lock, deadline) == std::cv_status::timeout) break;
    }
    return {false, "Timed out waiting for D1 recharge task to stop."};
}

BackendResult D1MaxBackend::waitForUndockCompleted(uint64_t min_sequence, int timeout_ms) {
    using robot_sdk::TaskStatus;
    using robot_sdk::TaskType;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        const bool new_undock_state = task_state_sequence_ > min_sequence &&
                                      task_type_ == static_cast<int>(TaskType::UNDOCK);
        if (new_undock_state && task_status_ == static_cast<int>(TaskStatus::FAILURE)) {
            return {false, "D1 undock task failed, error_code=" + std::to_string(task_error_code_)};
        }
        if (new_undock_state && task_status_ == static_cast<int>(TaskStatus::SUCCESS) &&
            !charging_pile_connected_) {
            return {true, "D1 undock completed and charging-pile disconnection is confirmed."};
        }
        if (motion_cv_.wait_until(lock, deadline) == std::cv_status::timeout) break;
    }
    return {false, "Timed out waiting for D1 undock completion and charging-pile disconnection."};
}

BackendResult D1MaxBackend::waitForUndockStopped(uint64_t min_sequence, int timeout_ms) {
    using robot_sdk::TaskStatus;
    using robot_sdk::TaskType;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        if (task_state_sequence_ > min_sequence &&
            task_type_ == static_cast<int>(TaskType::UNDOCK) &&
            task_status_ == static_cast<int>(TaskStatus::STOPPED)) {
            return {true, "D1 undock task stopped."};
        }
        if (motion_cv_.wait_until(lock, deadline) == std::cv_status::timeout) break;
    }
    return {false, "Timed out waiting for D1 undock task to stop."};
}

int D1MaxBackend::chargeState() const {
    // X30-compatible values: 0=idle, 1=task running, 2=charging,
    // 4=task failure. D1 task details remain available through SDK logs.
    std::lock_guard<std::mutex> lock(mutex_);
    using robot_sdk::MachineStatus;
    using robot_sdk::TaskStatus;
    using robot_sdk::TaskType;
    if (task_status_ == static_cast<int>(TaskStatus::FAILURE)) return 4;
    if (task_type_ == static_cast<int>(TaskType::RECHARGING)) {
        if (charging_pile_connected_ && isChargingLocked()) return 2;
        if (task_status_ == static_cast<int>(TaskStatus::STARTING) ||
            task_status_ == static_cast<int>(TaskStatus::RUNNING)) return 1;
    }
    if (task_type_ == static_cast<int>(TaskType::UNDOCK) &&
        (task_status_ == static_cast<int>(TaskStatus::STARTING) ||
         task_status_ == static_cast<int>(TaskStatus::RUNNING))) return 3;
    return 0;
}

bool D1MaxBackend::chargeTaskActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    using robot_sdk::TaskStatus;
    using robot_sdk::TaskType;
    const bool running = task_status_ == static_cast<int>(TaskStatus::STARTING) ||
                         task_status_ == static_cast<int>(TaskStatus::RUNNING);
    return running && (task_type_ == static_cast<int>(TaskType::RECHARGING) ||
                       task_type_ == static_cast<int>(TaskType::UNDOCK));
}

bool D1MaxBackend::velocityCommandAllowed() const {
    using robot_sdk::MotionStatus;
    std::lock_guard<std::mutex> lock(mutex_);
    switch (static_cast<MotionStatus>(motion_status_)) {
    case MotionStatus::MOTION_STATUS_WALK:
    case MotionStatus::MOTION_STATUS_BALANCE_STAND:
    case MotionStatus::MOTION_STATUS_GAIT:
    case MotionStatus::MOTION_STATUS_CRAWL_WALK:
    case MotionStatus::MOTION_STATUS_STAIR:
        return true;
    default:
        return false;
    }
}

int D1MaxBackend::bodyHeightState() const {
    using robot_sdk::MotionStatus;
    std::lock_guard<std::mutex> lock(mutex_);
    // Match X30's public convention: -1 is crawl, 0 is normal. RobotSDK
    // does not report HighLowStance feedback, so low/high in-place stance is
    // intentionally represented as normal unless the robot is in Crawl mode.
    return (motion_status_ == static_cast<int>(MotionStatus::MOTION_STATUS_CRAWL) ||
            motion_status_ == static_cast<int>(MotionStatus::MOTION_STATUS_CRAWL_WALK)) ? -1 : 0;
}

BackendState D1MaxBackend::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void D1MaxBackend::setStateCallback(StateCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_callback_ = std::move(callback);
}

void D1MaxBackend::setImuCallback(ImuCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_); imu_callback_ = std::move(callback);
}
void D1MaxBackend::setOdometryCallback(OdometryCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_); odom_callback_ = std::move(callback);
}
void D1MaxBackend::setJointCallback(JointCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_); joint_callback_ = std::move(callback);
}
void D1MaxBackend::setFaultCallback(FaultCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_); fault_callback_ = std::move(callback);
}

void D1MaxBackend::updateState(const BackendState &state) {
    StateCallback callback;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = state;
        callback = state_callback_;
    }
    if (callback) callback(state);
}

}  // namespace nav_bridge
