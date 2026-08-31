#include "nav_bridge/d1_max/d1_max_backend.hpp"

#include <algorithm>
#include <cmath>
#include <system_error>

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
        state.mode = 0;
        state.control_owned = data.control_source == robot_sdk::CtrlSource::CTRL_SOURCE_SDK;
        state.vx = data.speed.line;
        state.vy = data.speed.translation;
        state.vyaw = data.speed.angle;
        const bool p1 = data.battery.present1;
        const bool p2 = data.battery.present2;
        if (p1 && p2) state.battery_percent = (data.battery.power1 + data.battery.power2) * 0.5;
        else if (p1) state.battery_percent = data.battery.power1;
        else if (p2) state.battery_percent = data.battery.power2;
        owner_.updateState(state);
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

class D1MaxBackend::ControlCallback final : public robot_sdk::IControlCallback {};

D1MaxBackend::D1MaxBackend(std::string host_ip, int host_port, bool auto_reconnect,
                           int connect_timeout_ms, int reconnect_interval_ms)
    : host_ip_(std::move(host_ip)), host_port_(host_port), auto_reconnect_(auto_reconnect),
      connect_timeout_ms_(connect_timeout_ms), reconnect_interval_ms_(reconnect_interval_ms) {
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
    control_callback_ = std::make_shared<ControlCallback>();
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
        client_->SetImuConfig(100);
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

BackendResult D1MaxBackend::stand() { return fromError(client_->StandUp(connect_timeout_ms_)); }

BackendResult D1MaxBackend::lie() { return fromError(client_->LieDown(connect_timeout_ms_)); }

BackendResult D1MaxBackend::softEstop(bool enabled) {
    return fromError(client_->SoftEmergencyStop(enabled, connect_timeout_ms_));
}

BackendResult D1MaxBackend::setMode(int mode) {
    (void)mode;
    return {false, "RobotSDK-0.2.1 removed SetMode; use a dedicated D1 posture command."};
}

BackendResult D1MaxBackend::setSpeed(int speed_level) {
    return fromError(client_->SetSpeed(speed_level, connect_timeout_ms_));
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
