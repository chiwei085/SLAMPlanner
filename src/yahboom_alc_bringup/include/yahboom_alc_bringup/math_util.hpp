#pragma once

#include <cmath>
#include <utility>

namespace yahboom_alc_bringup
{

constexpr double kPi = 3.14159265358979323846;

struct DecimalPlaces
{
    explicit DecimalPlaces(const int value_in) : value(value_in) {}
    int value;
};

inline double rounded(const double value, const DecimalPlaces precision) {
    const double scale = std::pow(10.0, precision.value);
    return std::round(value * scale) / scale;
}

inline double rounded(const double value) {
    return rounded(value, DecimalPlaces{3});
}

inline double yawBetween(const double from_x, const double from_y,
                         const double to_x, const double to_y) {
    return std::atan2(to_y - from_y, to_x - from_x);
}

inline double yawBetween(const std::pair<double, double>& from,
                         const std::pair<double, double>& to) {
    return yawBetween(from.first, from.second, to.first, to.second);
}

inline double headingDelta(const double from_yaw, const double to_yaw) {
    return std::abs(std::remainder(to_yaw - from_yaw, 2.0 * kPi));
}

inline double evenlySpacedYaw(const int index, const int count) {
    return count <= 0 ? 0.0
                      : static_cast<double>(index) * 2.0 * kPi /
                            static_cast<double>(count);
}

inline std::pair<double, double> projectPoint(
    const std::pair<double, double>& origin, const double yaw,
    const double distance_m) {
    return {origin.first + distance_m * std::cos(yaw),
            origin.second + distance_m * std::sin(yaw)};
}

}  // namespace yahboom_alc_bringup
