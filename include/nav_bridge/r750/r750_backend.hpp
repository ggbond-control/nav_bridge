#pragma once

#include <functional>
#include <memory>
#include <string>

#include "nav_bridge/robot_backend.hpp"

namespace nav_bridge {

struct R750Options {
    std::string serial_port{"/dev/ttyACM0"};
    int serial_baud{230400};
    int connect_timeout_ms{5000};
    int reconnect_interval_ms{2000};
    double cmd_vel_rate_hz{10.0};
    int cmd_vel_timeout_ms{500};
    double max_linear_speed{0.7};
    double max_angular_speed{3.5};
    int telemetry_timeout_ms{1000};
    double state_publish_rate_hz{10.0};
    bool startup_acquire_control{true};
    bool enable_motion_output{true};
    double battery_empty_voltage{9.9};
    double battery_full_voltage{12.6};
};

// TARKBOT X-Protocol serial chassis adapter (R750 differential drive).
// move() only replaces a bounded, latest-command cache; independent workers
// perform control streaming, telemetry parsing and reconnection. The protocol
// has no acknowledgement and no queryable device state, so BackendState
// velocities describe the last sent command, never measured motion.
class R750Backend final : public RobotBackend {
public:
    explicit R750Backend(R750Options options);
    ~R750Backend() override;
    BackendResult connect() override;
    BackendResult disconnect() override;
    BackendResult takeControl() override;
    BackendResult releaseControl() override;
    BackendResult move(double vx, double vy, double vyaw) override;
    BackendResult stand() override;
    BackendResult lie() override;
    BackendResult softEstop(bool enabled) override;
    BackendResult setMode(int mode) override;
    BackendResult setSpeed(int speed_level) override;
    BackendState state() const override;
    std::string diagnostics() const;
    using RawVelocityCallback = std::function<void(double, double, double)>;
    void setRawVelocityCallback(RawVelocityCallback callback);
    void setStateCallback(StateCallback callback) override;
    void setImuCallback(ImuCallback callback) override;
    void setOdometryCallback(OdometryCallback callback) override;
    void setJointCallback(JointCallback callback) override;
    void setFaultCallback(FaultCallback callback) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace nav_bridge
