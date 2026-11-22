#include "yahboom_alc_bringup/straight_tail.hpp"

#include <algorithm>
#include <cmath>

namespace yahboom_alc_bringup
{

double maxCarrotBearing(const double lookahead_m, const double max_curvature) {
    return std::asin(std::clamp(max_curvature * lookahead_m / 2.0, 0.0, 1.0));
}

std::optional<double> tailCaptureAlong(const TailPose& robot,
                                       const TailPose& goal,
                                       const double lookahead_m,
                                       const double max_curvature) {
    const double dx = std::cos(goal.yaw);
    const double dy = std::sin(goal.yaw);
    const double rx = robot.x - goal.x;
    const double ry = robot.y - goal.y;
    const double along = rx * dx + ry * dy;
    const double lateral = dx * ry - dy * rx;
    if (along > 0.0 || std::abs(lateral) >= lookahead_m) {
        return std::nullopt;
    }
    const double carrot_along = std::min(
        0.0, along + std::sqrt(lookahead_m * lookahead_m - lateral * lateral));
    const double carrot_x = goal.x + carrot_along * dx;
    const double carrot_y = goal.y + carrot_along * dy;
    const double bearing = std::remainder(
        std::atan2(carrot_y - robot.y, carrot_x - robot.x) - robot.yaw,
        2.0 * M_PI);
    if (std::abs(bearing) > maxCarrotBearing(lookahead_m, max_curvature)) {
        return std::nullopt;
    }
    return along;
}

std::vector<TailPose> tailPoses(const TailPose& goal, const double along_from,
                                const double along_to, const double step_m) {
    const double length = along_to - along_from;
    const int segments =
        std::max(1, static_cast<int>(std::ceil(std::abs(length) / step_m)));
    std::vector<TailPose> poses;
    poses.reserve(static_cast<std::size_t>(segments) + 1);
    for (int i = 0; i <= segments; ++i) {
        const double along = along_from + length * i / segments;
        poses.push_back({goal.x + along * std::cos(goal.yaw),
                         goal.y + along * std::sin(goal.yaw), goal.yaw});
    }
    return poses;
}

}  // namespace yahboom_alc_bringup
