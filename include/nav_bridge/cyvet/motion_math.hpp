#pragma once

#include <cmath>
#include <cstdint>

namespace nav_bridge::cyvet {
struct Pose2 { double x{0}, y{0}, yaw{0}; };

// Each walking epoch has its own origin. Align its first sample to the last
// published pose, without inventing displacement while observations are absent.
class ContinuousOdometry {
public:
    Pose2 update(uint32_t epoch, const Pose2 &raw) {
        if (!initialized_) {
            initialized_ = true;
            reset_ = false;
            epoch_ = epoch;
        } else if (reset_ || epoch != epoch_) {
            reset_ = false;
            epoch_ = epoch;
            offset_.yaw = last_.yaw - raw.yaw;
            offset_.x = last_.x - std::cos(offset_.yaw) * raw.x + std::sin(offset_.yaw) * raw.y;
            offset_.y = last_.y - std::sin(offset_.yaw) * raw.x - std::cos(offset_.yaw) * raw.y;
        }
        last_ = {offset_.x + std::cos(offset_.yaw) * raw.x - std::sin(offset_.yaw) * raw.y,
                 offset_.y + std::sin(offset_.yaw) * raw.x + std::cos(offset_.yaw) * raw.y,
                 offset_.yaw + raw.yaw};
        return last_;
    }
    void newSession() { reset_ = true; }
private:
    bool initialized_{false};
    bool reset_{false};
    uint32_t epoch_{0};
    Pose2 offset_, last_;
};

}  // namespace nav_bridge::cyvet
