#pragma once

#include <cmath>

namespace yahboom_alc_bringup
{

// How far the car went along its commanded direction (+1 forward, -1 reverse),
// from the furthest point behind. Net displacement would cancel when the car
// coasts on from the previous direction and then passes its turning point.
class CommandedTravel
{
public:
    // Coasting this far (the odometry noise) means the new command has not
    // answered yet; encoder jitter of a pinned car does not add up to it.
    static constexpr double kNoiseM = 0.02;

    void start(const double x, const double y) {
        x_ = x;
        y_ = y;
        coast_m_ = 0.0;
    }

    double update(const double x, const double y, const double yaw,
                  const double direction) {
        const double along =
            direction * ((x - x_) * std::cos(yaw) + (y - y_) * std::sin(yaw));
        if (along >= 0.0) {
            return along;
        }
        coast_m_ -= along;
        x_ = x;
        y_ = y;
        return 0.0;
    }

    // True once per kNoiseM of coasting.
    bool takeCoast() {
        if (coast_m_ < kNoiseM) {
            return false;
        }
        coast_m_ = 0.0;
        return true;
    }

private:
    double x_ = 0.0;
    double y_ = 0.0;
    double coast_m_ = 0.0;
};

}  // namespace yahboom_alc_bringup
