#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace nav_bridge {

struct BackendResult {
    bool success{false};
    std::string message;
};

enum class BackendMotionState : uint8_t {
    UNKNOWN,
    LYING_DOWN,
    STANDING_UP,
    STANDING,
    MOVING,
    LOCKED,
    ESTOP,
};

struct BackendState {
    bool connected{false};
    bool control_owned{false};
    BackendMotionState motion_state{BackendMotionState::UNKNOWN};
    uint8_t mode{0};
    double vx{0.0};
    double vy{0.0};
    double vyaw{0.0};
    double battery_percent{-1.0};
};

struct BackendImu {
    double ax{0.0}, ay{0.0}, az{0.0};
    double gx{0.0}, gy{0.0}, gz{0.0};
    double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};

struct BackendOdometry {
    double px{0.0}, py{0.0}, pz{0.0};
    double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
    double vx{0.0}, vy{0.0}, vz{0.0};
    double wx{0.0}, wy{0.0}, wz{0.0};
    uint64_t timestamp_ns{0};
};

struct BackendJointState {
    std::vector<std::string> names;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> efforts;
};

struct BackendFault {
    int code{0};
    int level{0};
    std::string message;
};

class RobotBackend {
public:
    using StateCallback = std::function<void(const BackendState &)>;
    using ImuCallback = std::function<void(const BackendImu &)>;
    using OdometryCallback = std::function<void(const BackendOdometry &)>;
    using JointCallback = std::function<void(const BackendJointState &)>;
    using FaultCallback = std::function<void(const BackendFault &)>;

    virtual ~RobotBackend() = default;

    virtual BackendResult connect() = 0;
    virtual BackendResult disconnect() = 0;
    virtual BackendResult takeControl() = 0;
    virtual BackendResult releaseControl() = 0;
    virtual BackendResult move(double vx, double vy, double vyaw) = 0;
    virtual BackendResult stand() = 0;
    virtual BackendResult lie() = 0;
    virtual BackendResult softEstop(bool enabled) = 0;
    virtual BackendResult setMode(int mode) = 0;
    virtual BackendResult setSpeed(int speed_level) = 0;
    virtual BackendState state() const = 0;
    virtual void setStateCallback(StateCallback callback) = 0;
    virtual void setImuCallback(ImuCallback callback) = 0;
    virtual void setOdometryCallback(OdometryCallback callback) = 0;
    virtual void setJointCallback(JointCallback callback) = 0;
    virtual void setFaultCallback(FaultCallback callback) = 0;
};

using RobotBackendPtr = std::unique_ptr<RobotBackend>;

}  // namespace nav_bridge
