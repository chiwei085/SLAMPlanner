#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

#include "yahboom_alc_bringup/r2_collision_safety.hpp"

namespace yahboom_alc_bringup
{
namespace
{

// Clamps the immutable config invariants once at assignment so detectImpact()
// can use the fields directly instead of re-guarding them on every call.
R2CollisionSafetyConfig sanitizeConfig(R2CollisionSafetyConfig config) {
    config.imu_baseline_samples = std::max(1, config.imu_baseline_samples);
    config.command_vx_threshold_mps = std::abs(config.command_vx_threshold_mps);
    config.imu_impact_decel_threshold_mps2 =
        std::abs(config.imu_impact_decel_threshold_mps2);
    return config;
}

}  // namespace

CollisionSafetyCore::CollisionSafetyCore(R2CollisionSafetyConfig config)
    : config_(sanitizeConfig(std::move(config))) {}

void CollisionSafetyCore::setConfig(const R2CollisionSafetyConfig& config) {
    config_ = sanitizeConfig(config);
    reset();
}

void CollisionSafetyCore::reset() {
    impact_hold_until_sec_ = -1.0e9;
    last_impact_sec_ = -1.0e9;
    imu_baseline_.clear();
}

ImpactDecision CollisionSafetyCore::detectImpact(
    const double time_sec, const geometry_msgs::msg::Twist& command,
    const bool imu_valid, const double imu_forward_accel) {
    ImpactDecision result;
    const bool forward = command.linear.x > config_.command_vx_threshold_mps;

    bool impact = false;
    if (imu_valid && std::isfinite(imu_forward_accel)) {
        if (static_cast<int>(imu_baseline_.size()) >=
                config_.imu_baseline_samples &&
            forward) {
            const double baseline = std::accumulate(imu_baseline_.begin(),
                                                    imu_baseline_.end(), 0.0) /
                                    static_cast<double>(imu_baseline_.size());
            impact = imu_forward_accel - baseline <=
                     -config_.imu_impact_decel_threshold_mps2;
        }
        imu_baseline_.push_back(imu_forward_accel);
        while (static_cast<int>(imu_baseline_.size()) >
               config_.imu_baseline_samples) {
            imu_baseline_.pop_front();
        }
    }
    if (impact && time_sec - last_impact_sec_ >= config_.impact_cooldown_sec) {
        last_impact_sec_ = time_sec;
        impact_hold_until_sec_ = time_sec + config_.impact_stop_hold_sec;
        result.impact_triggered = true;
    }
    result.stop_hold_active = time_sec < impact_hold_until_sec_;
    return result;
}

}  // namespace yahboom_alc_bringup
