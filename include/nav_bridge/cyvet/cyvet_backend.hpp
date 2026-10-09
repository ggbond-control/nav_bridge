#pragma once

#include <memory>
#include <string>
#include "nav_bridge/robot_backend.hpp"

namespace nav_bridge {
struct CyvetOptions {
    std::string device_id;
    bool enable_observations{true};
    std::string service_name{"robotServer"};
    std::string event_topic{"/robotServer/Event"};
    int robot_domain_id{42};
    int connect_timeout_ms{5000};
    int reconnect_interval_ms{2000};
    int action_timeout_ms{10000};
    int rpc_timeout_ms{200};
    int lease_ms{5000};
    int cmd_vel_timeout_ms{500};
    int telemetry_timeout_ms{2000};
    double cmd_vel_rate_hz{30};
    double lateral_sign{1};
    std::string default_control_profile{"slow"};
};

// Owns a separate robot ROS context and a serialized SDK/executor worker.
// move() accepts a latest-value command; it does not wait for a remote ACK.
class CyvetBackend final : public RobotBackend {
public:
    explicit CyvetBackend(CyvetOptions options);
    ~CyvetBackend() override;
    BackendResult connect() override;
    BackendResult disconnect() override;
    BackendResult takeControl() override;
    BackendResult releaseControl() override;
    BackendResult move(double vx, double vy, double vyaw) override;
    BackendResult stand() override;
    BackendResult lie() override;
    BackendResult softEstop(bool enabled) override;
    BackendResult setMode(int mode) override;
    BackendResult setGait(int gait);
    BackendResult setSpeed(int speed_level) override;
    BackendState state() const override;
    std::string diagnostics() const;
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
