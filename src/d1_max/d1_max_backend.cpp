#include "nav_bridge/d1_max/d1_max_backend.hpp"

#include <algorithm>
#include <cmath>
#include <system_error>
#include <thread>
#include <chrono>
#include <cstdio>

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
            owner_.last_basic_state_time_ = std::chrono::steady_clock::now();
            owner_.battery1_present_ = data.battery.present1;
            owner_.battery2_present_ = data.battery.present2;
            owner_.battery1_power_supply_status_ =
                static_cast<int>(data.battery.power_supply_status1);
            owner_.battery2_power_supply_status_ =
                static_cast<int>(data.battery.power_supply_status2);
            if (owner_.isChargingLocked() || data.machine_status == robot_sdk::MachineStatus::RECHARGE)
                owner_.undock_required_ = true;
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
            if (owner_.task_type_ != static_cast<int>(data.task_type) ||
                owner_.task_status_ != static_cast<int>(data.task_status) ||
                owner_.task_error_code_ != data.error_code) {
                std::fprintf(stderr, "[D1 charge] SDK task update: type=%d status=%d error=%u\n",
                             static_cast<int>(data.task_type), static_cast<int>(data.task_status),
                             static_cast<unsigned>(data.error_code));
            }
            owner_.task_type_ = static_cast<int>(data.task_type);
            owner_.task_status_ = static_cast<int>(data.task_status);
            owner_.task_error_code_ = data.error_code;
            const uint64_t sequence = ++owner_.task_state_sequence_;
            if ((data.task_type == robot_sdk::TaskType::RECHARGING ||
                 data.task_type == robot_sdk::TaskType::UNDOCK) &&
                (data.task_status == robot_sdk::TaskStatus::STARTING ||
                 data.task_status == robot_sdk::TaskStatus::RUNNING ||
                 data.task_status == robot_sdk::TaskStatus::FAILURE))
                owner_.undock_required_ = true;
            if (data.task_type == robot_sdk::TaskType::RECHARGING &&
                data.task_status == robot_sdk::TaskStatus::STOPPED) {
                owner_.recharge_stopped_sequence_ = sequence;
            }
            if (data.task_type == robot_sdk::TaskType::UNDOCK) {
                if (data.task_status == robot_sdk::TaskStatus::SUCCESS) {
                    owner_.undock_success_sequence_ = sequence;
                    owner_.undock_success_time_ = std::chrono::steady_clock::now();
                } else if (data.task_status == robot_sdk::TaskStatus::FAILURE) {
                    owner_.undock_failure_sequence_ = sequence;
                    owner_.undock_failure_error_code_ = data.error_code;
                } else if (data.task_status == robot_sdk::TaskStatus::STOPPED) {
                    owner_.undock_stopped_sequence_ = sequence;
                }
            }
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
    // ROS uses SI units (m/s, rad/s), while SDK Move() expects normalized
    // percentages in [-1, 1]. Convert each axis using the active speed/gait
    // limits before calling the SDK.
    int gait;
    int speed_level;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        gait = navigation_gait_;
        speed_level = speed_level_;
    }

    // L_WALK is implemented by the dedicated SDK Gait posture, but its
    // externally visible velocity contract follows general-mode medium speed.
    // Do not send SetSpeed() for L_WALK; this is only the local ROS-to-SDK
    // normalization contract.
    if (gait == 32) speed_level = static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_MEDIUM);
    // All Stair() mappings use a fixed 1 m/s local velocity contract,
    // independent of the previous speed level; do not send SDK SetSpeed().
    else if (gait == 6 || gait == 7 || gait == 8 || gait == 36) {
        speed_level = static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_SLOW);
    }

    double max_forward = 1.0;
    if (speed_level == static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_MEDIUM)) max_forward = 2.0;
    else if (speed_level == static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_HIGH)) max_forward = 3.0;

    const double max_lateral = 0.5;
    // Convert yaw using a fixed input scale. The SDK/firmware owns motion
    // constraints that couple forward speed with lateral or yaw commands.
    const double max_yaw = 1.5;

    const auto normalize = [](double value, double limit) {
        if (limit <= 0.0) return 0.0;
        return std::max(-1.0, std::min(1.0, value / limit));
    };
    const float sdk_left_right = static_cast<float>(normalize(vy, max_lateral));
    const float sdk_forward_back = static_cast<float>(normalize(vx, max_forward));
    const float sdk_yaw = static_cast<float>(normalize(vyaw, max_yaw));
    // SDK order is left/right, forward/back, yaw; ROS order is x, y, yaw.
    return fromError(client_->Move(sdk_left_right, sdk_forward_back, sdk_yaw));
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

    // The unified /stand contract means standing in MOUNTAIN, not merely any
    // generic locomotion state. Reuse setGait(33) so the SDK command and
    // feedback wait are identical to the explicit gait service.
    const auto mountain = setGait(33);
    if (!mountain.success) return mountain;
    return {true, "D1 Max is standing in the general navigation mode (MOUNTAIN), high speed."};
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
        // L_WALK is represented by the SDK's dedicated Gait posture. It is
        // intentionally not followed by SetSpeed(SLOW): SDK Gait is a distinct
        // mode, not merely the low-speed variant of the general posture.
        if (gait != 32) {
            int speed = static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_MEDIUM);
            if (gait == 0) speed = static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_SLOW);
            else if (gait == 3 || gait == 33) {
                speed = static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_HIGH);
            }
            auto speed_result = setSpeed(speed);
            if (!speed_result.success) return speed_result;
        } else {
            // Keep the local velocity contract at general-mode medium speed,
            // without sending SetSpeed() to the dedicated Gait posture.
            std::lock_guard<std::mutex> lock(mutex_);
            speed_level_ = static_cast<int>(robot_sdk::SpeedLevel::SPEED_LEVEL_MEDIUM);
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        navigation_gait_ = gait;
        state_.mode = gait;
    }
    return {true, "D1 gait changed successfully."};
}

BackendResult D1MaxBackend::setSpeed(int speed_level) {
    const auto result = fromError(client_->SetSpeed(speed_level, connect_timeout_ms_));
    if (result.success) {
        std::lock_guard<std::mutex> lock(mutex_);
        speed_level_ = speed_level;
    }
    return result;
}

BackendResult D1MaxBackend::startRecharge(int confirmation_timeout_ms, int pile_connection_timeout_ms) {
    if (!client_ || !client_->IsConnected()) return {false, "D1 SDK is not connected."};
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!motion_cv_.wait_for(lock, std::chrono::milliseconds(std::max(1, pile_connection_timeout_ms)), [&] {
                return charging_pile_connected_ &&
                    last_basic_state_time_ != std::chrono::steady_clock::time_point{} &&
                    std::chrono::steady_clock::now() - last_basic_state_time_ < std::chrono::seconds(5);
            })) {
            return {false, "Timed out waiting for D1 charging pile connection; recharge was not started."};
        }
    }
    if (!client_->IsConnected()) return {false, "D1 SDK is not connected."};
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
        undock_required_ = true;
        sequence = task_state_sequence_;
    }
    const auto stop = fromError(client_->StopRechargeTask(connect_timeout_ms_));
    if (!stop.success) return stop;
    return waitForRechargeStopped(sequence, confirmation_timeout_ms);
}

void D1MaxBackend::logChargeState(const char *phase) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto age_ms = last_basic_state_time_ == std::chrono::steady_clock::time_point{} ? -1LL :
        static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_basic_state_time_).count());
    std::fprintf(stderr,
        "[D1 charge] %s: task_type=%d task_status=%d task_error=%u task_seq=%llu "
        "undock_success_seq=%llu undock_failure_seq=%llu pile_bluetooth_connected=%d charging=%d "
        "basic_age_ms=%lld motion=%d machine=%d battery1_status=%d battery2_status=%d\n",
        phase, task_type_, task_status_, static_cast<unsigned>(task_error_code_),
        static_cast<unsigned long long>(task_state_sequence_),
        static_cast<unsigned long long>(undock_success_sequence_),
        static_cast<unsigned long long>(undock_failure_sequence_),
        charging_pile_connected_, isChargingLocked(), age_ms, motion_status_, machine_status_,
        battery1_power_supply_status_, battery2_power_supply_status_);
}

BackendResult D1MaxBackend::startUndock(int confirmation_timeout_ms) {
    logChargeState("startUndock entry");
    if (!client_ || !client_->IsConnected()) return {false, "D1 SDK is not connected."};
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(1, confirmation_timeout_ms));
    const auto remaining_timeout_ms = [&deadline]() {
        return std::max(1, static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count()));
    };
    bool recharge_active = false;
    bool undock_active = false;
    bool undock_succeeded = false;
    uint64_t undock_sequence = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        recharge_active = task_type_ == static_cast<int>(robot_sdk::TaskType::RECHARGING) &&
                                     (task_status_ == static_cast<int>(robot_sdk::TaskStatus::STARTING) ||
                                      task_status_ == static_cast<int>(robot_sdk::TaskStatus::RUNNING));
        undock_active = task_type_ == static_cast<int>(robot_sdk::TaskType::UNDOCK) &&
            (task_status_ == static_cast<int>(robot_sdk::TaskStatus::STARTING) ||
             task_status_ == static_cast<int>(robot_sdk::TaskStatus::RUNNING));
        undock_sequence = task_state_sequence_;
        undock_succeeded = task_type_ == static_cast<int>(robot_sdk::TaskType::UNDOCK) &&
            task_status_ == static_cast<int>(robot_sdk::TaskStatus::SUCCESS) &&
            undock_success_sequence_ > 0;
        if (undock_succeeded) undock_sequence = undock_success_sequence_ - 1;
        if (recharge_active || undock_active || isChargingLocked()) undock_required_ = true;
    }
    // A caller may retry after its own timeout while the SDK is still
    // undocking. Join that task; never issue a second StartUnDockTask.
    if (undock_active) {
        std::fprintf(stderr, "[D1 charge] branch=join_undock active=%d succeeded=%d\n",
                     undock_active, undock_succeeded);
        return waitForUndockCompleted(undock_sequence, remaining_timeout_ms());
    }
    if (recharge_active) {
        std::fprintf(stderr, "[D1 charge] branch=stop_active_recharge; calling stopRecharge\n");
        // Unified UNDOCK_START is an exit operation: stop charging first,
        // then start the physical undock task.
        const auto stop = stopRecharge(remaining_timeout_ms());
        std::fprintf(stderr, "[D1 charge] stopRecharge returned success=%d reason=%s\n",
                     stop.success, stop.message.c_str());
        if (!stop.success) return stop;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool fresh = last_basic_state_time_ != std::chrono::steady_clock::time_point{} &&
            std::chrono::steady_clock::now() - last_basic_state_time_ < std::chrono::seconds(5);
        if (!fresh) return {false, "D1 charge exit requires fresh SDK robot telemetry."};
        // Bluetooth disconnection satisfies the charge-side condition, but
        // only after active SDK tasks above have completed or stopped.
        if (chargeExitStateConfirmedLocked() && (!charging_pile_connected_ || !undock_required_)) {
            undock_required_ = false;
            undock_success_sequence_ = 0;
            std::fprintf(stderr, "[D1 charge] branch=already_exited; no SDK undock command\n");
            return {true, "D1 charge exit confirmed; no undock task needed."};
        }
        if (!undock_required_)
            return {false, "D1 battery charge status is unconfirmed; no undock task was sent."};
    }
    if (undock_succeeded) {
        std::lock_guard<std::mutex> lock(mutex_);
        // A new charging session must not reuse an old undock SUCCESS.
        undock_succeeded = !isChargingLocked();
    }
    if (undock_succeeded)
        return waitForUndockCompleted(undock_sequence, remaining_timeout_ms());
    std::fprintf(stderr, "[D1 charge] branch=physical_undock; calling takeControl\n");
    auto control = takeControl();
    if (!control.success) return control;
    uint64_t sequence;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        undock_required_ = true;
        sequence = task_state_sequence_;
    }
    std::fprintf(stderr, "[D1 charge] calling SDK StartUnDockTask timeout_ms=%d\n", connect_timeout_ms_);
    const auto start = fromError(client_->StartUnDockTask(connect_timeout_ms_));
    std::fprintf(stderr, "[D1 charge] SDK StartUnDockTask returned success=%d reason=%s\n",
                 start.success, start.message.c_str());
    if (!start.success) return start;
    return waitForUndockCompleted(sequence, remaining_timeout_ms());
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

bool D1MaxBackend::chargeExitStateConfirmedLocked() const {
    if (last_basic_state_time_ == std::chrono::steady_clock::time_point{} ||
        std::chrono::steady_clock::now() - last_basic_state_time_ >= std::chrono::seconds(5))
        return false;
    if (!charging_pile_connected_) return true;
    // Unknown or absent batteries must not look like confirmed non-charging.
    const auto not_charging = [](int status) {
        return status == static_cast<int>(robot_sdk::PowerSupplyStatus::DISCHARGING) ||
               status == static_cast<int>(robot_sdk::PowerSupplyStatus::FULL);
    };
    return (battery1_present_ || battery2_present_) &&
        (!battery1_present_ || not_charging(battery1_power_supply_status_)) &&
        (!battery2_present_ || not_charging(battery2_power_supply_status_));
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
        const bool fresh = last_basic_state_time_ != std::chrono::steady_clock::time_point{} &&
            std::chrono::steady_clock::now() - last_basic_state_time_ < std::chrono::seconds(5);
        if (new_task_state && task_active && fresh && isChargingLocked()) {
            return {true, "D1 battery charging is confirmed."};
        }
        if (motion_cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
            break;
        }
    }
    return {false, "Timed out waiting for D1 battery charging confirmation."};
}

BackendResult D1MaxBackend::waitForRechargeStopped(uint64_t min_sequence, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        if (recharge_stopped_sequence_ > min_sequence) {
            return {true, "D1 recharge task stopped."};
        }
        if (motion_cv_.wait_until(lock, deadline) == std::cv_status::timeout) break;
    }
    return {false, "Timed out waiting for D1 recharge task to stop."};
}

BackendResult D1MaxBackend::waitForUndockCompleted(uint64_t min_sequence, int timeout_ms) {
    std::fprintf(stderr, "[D1 charge] waiting for undock SUCCESS + charge exit: min_seq=%llu timeout_ms=%d\n",
                 static_cast<unsigned long long>(min_sequence), timeout_ms);
    logChargeState("undock wait entry");
    const auto started = std::chrono::steady_clock::now();
    const auto deadline = started +
                          std::chrono::milliseconds(std::max(1, timeout_ms));
    auto next_report = started + std::chrono::seconds(5);
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        if (undock_failure_sequence_ > min_sequence) {
            std::fprintf(stderr, "[D1 charge] undock wait failed: error=%u\n",
                         static_cast<unsigned>(undock_failure_error_code_));
            return {false, "D1 undock task failed, error_code=" +
                           std::to_string(undock_failure_error_code_)};
        }
        if (undock_success_sequence_ > min_sequence &&
            last_basic_state_time_ >= std::max(started, undock_success_time_) &&
            chargeExitStateConfirmedLocked()) {
            undock_required_ = false;
            // This terminal event has completed its handoff. A later charge
            // session must not reuse it to skip its own physical undock.
            undock_success_sequence_ = 0;
            std::fprintf(stderr, "[D1 charge] undock wait complete: SDK SUCCESS + fresh charge exit\n");
            return {true, "D1 undock task completed and charge exit confirmed."};
        }
        if (std::chrono::steady_clock::now() >= next_report) {
            const auto age_ms = last_basic_state_time_ == std::chrono::steady_clock::time_point{} ? -1LL :
                static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - last_basic_state_time_).count());
            std::fprintf(stderr,
                "[D1 charge] undock wait pending: success_seq=%llu min_seq=%llu pile_bluetooth_connected=%d "
                "charging=%d basic_age_ms=%lld basic_after_wait=%d task_type=%d task_status=%d\n",
                static_cast<unsigned long long>(undock_success_sequence_),
                static_cast<unsigned long long>(min_sequence), charging_pile_connected_, isChargingLocked(),
                age_ms, last_basic_state_time_ >= started, task_type_, task_status_);
            next_report = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        }
        if (motion_cv_.wait_until(lock, deadline) == std::cv_status::timeout) break;
    }
    lock.unlock();
    logChargeState("undock wait timeout");
    return {false, "Timed out waiting for D1 undock SUCCESS and charge exit confirmation."};
}

BackendResult D1MaxBackend::waitForUndockStopped(uint64_t min_sequence, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        if (undock_stopped_sequence_ > min_sequence) {
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
    using robot_sdk::TaskStatus;
    using robot_sdk::TaskType;
    // SDK can retain the last task's failure after a confirmed charge exit.
    if (undock_required_ && task_status_ == static_cast<int>(TaskStatus::FAILURE)) return 4;
    if (task_type_ == static_cast<int>(TaskType::UNDOCK) &&
        (task_status_ == static_cast<int>(TaskStatus::STARTING) ||
         task_status_ == static_cast<int>(TaskStatus::RUNNING))) return 3;
    if (isChargingLocked()) return 2;
    if (task_type_ == static_cast<int>(TaskType::RECHARGING)) {
        if (task_status_ == static_cast<int>(TaskStatus::STARTING) ||
            task_status_ == static_cast<int>(TaskStatus::RUNNING)) return 1;
    }
    return 0;
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
