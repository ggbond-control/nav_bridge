#pragma once

#include <memory>
#include <functional>
#include <string>

#include "nav_bridge/robot_backend.hpp"

namespace nav_bridge {

struct TankOptions {
    std::string host_ip{"49.235.44.225"};
    int host_port{10050};
    int connect_timeout_ms{500};
    int http_timeout_ms{3000};
    int move_timeout_ms{1000};
    int move_failure_limit{3};
    int state_failure_limit{3};
    int move_failure_timeout_ms{2000};
    int reconnect_interval_ms{2000};
    double cmd_vel_rate_hz{10.0};
    int cmd_vel_timeout_ms{500};
    double max_linear_speed{1.2};
    double max_angular_speed{1.0};
    int state_poll_interval_ms{1000};
    int battery_poll_interval_ms{3000};
    double raw_vel_poll_rate_hz{10.0};
    int raw_vel_timeout_ms{500};
    int telemetry_timeout_ms{10000};
    bool startup_acquire_control{true};
    bool enable_motion_output{true};
};

// HTTP chassis adapter. move() only replaces a bounded, latest-command cache;
// independent workers perform control streaming and telemetry/reconnection.
// BackendState velocities describe acknowledged commands, not measured motion.
class TankBackend final : public RobotBackend {
public:
    explicit TankBackend(TankOptions options);
    ~TankBackend() override;
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
