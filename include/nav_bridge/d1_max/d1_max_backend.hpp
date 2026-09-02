#pragma once

#include <memory>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <system_error>

#include "nav_bridge/robot_backend.hpp"

namespace robot_sdk {
class SDKClient;
class IDataCallback;
class IControlCallback;
}  // namespace robot_sdk

namespace nav_bridge {

class D1MaxBackend final : public RobotBackend {
public:
    D1MaxBackend(std::string host_ip, int host_port, bool auto_reconnect,
                 int connect_timeout_ms, int reconnect_interval_ms,
                 bool enable_sdk_imu);
    ~D1MaxBackend() override;

    BackendResult connect() override;
    BackendResult disconnect() override;
    BackendResult takeControl() override;
    BackendResult releaseControl() override;
    BackendResult move(double vx, double vy, double vyaw) override;
    BackendResult stand() override;
    BackendResult setGait(int gait);
    BackendResult lie() override;
    BackendResult softEstop(bool enabled) override;
    BackendResult setMode(int mode) override;
    BackendResult setSpeed(int speed_level) override;
    bool velocityCommandAllowed() const;
    int bodyHeightState() const;
    // These methods return success only after the SDK state callbacks confirm
    // the requested task transition, rather than after the command ACK alone.
    BackendResult startRecharge(int confirmation_timeout_ms);
    BackendResult stopRecharge(int confirmation_timeout_ms);
    BackendResult startUndock(int confirmation_timeout_ms);
    BackendResult stopUndock(int confirmation_timeout_ms);
    int chargeState() const;
    BackendState state() const override;
    void setStateCallback(StateCallback callback) override;
    void setImuCallback(ImuCallback callback) override;
    void setOdometryCallback(OdometryCallback callback) override;
    void setJointCallback(JointCallback callback) override;
    void setFaultCallback(FaultCallback callback) override;

private:
    class DataCallback;
    class ControlCallback;

    BackendResult fromError(const std::error_code &ec) const;
    BackendResult waitForTaskTransition(int task_type, int machine_status,
                                        bool starting, uint64_t min_sequence,
                                        int timeout_ms);
    void updateState(const BackendState &state);

    std::string host_ip_;
    int host_port_{8082};
    bool auto_reconnect_{true};
    int connect_timeout_ms_{5000};
    int reconnect_interval_ms_{2000};
    bool enable_sdk_imu_{false};

    std::unique_ptr<robot_sdk::SDKClient> client_;
    std::shared_ptr<DataCallback> data_callback_;
    std::shared_ptr<ControlCallback> control_callback_;

    mutable std::mutex mutex_;
    BackendState state_;
    StateCallback state_callback_;
    ImuCallback imu_callback_;
    OdometryCallback odom_callback_;
    JointCallback joint_callback_;
    FaultCallback fault_callback_;
    int imu_configured_hz_{0};
    int motion_status_{0};
    int navigation_gait_{33};
    int machine_status_{0};
    int task_type_{0};
    int task_status_{0};
    uint32_t task_error_code_{0};
    uint64_t task_state_sequence_{0};
    std::condition_variable motion_cv_;
};

}  // namespace nav_bridge
