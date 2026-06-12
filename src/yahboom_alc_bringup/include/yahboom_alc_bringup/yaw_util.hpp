#pragma once

#include <geometry_msgs/msg/quaternion.hpp>

#include <cmath>

namespace yahboom_alc_bringup
{

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace yahboom_alc_bringup
