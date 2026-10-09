#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

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

struct AxisRange { double min{0}, max{0}; };
inline double limitAxis(double value, double cap, AxisRange range) {
    if (!std::isfinite(value) || !std::isfinite(cap) || cap < 0 ||
        !std::isfinite(range.min) || !std::isfinite(range.max) ||
        range.min > 0 || range.max < 0 || range.min > range.max) {
        throw std::invalid_argument("Invalid velocity or capability range");
    }
    return std::clamp(value, std::max(-cap, range.min), std::min(cap, range.max));
}
}  // namespace nav_bridge::cyvet
