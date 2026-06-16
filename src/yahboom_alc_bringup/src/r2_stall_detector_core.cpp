#include <cmath>
#include <utility>

#include "yahboom_alc_bringup/r2_stall_detector.hpp"

namespace yahboom_alc_bringup
{

namespace
{

bool forwardCommandActive(const StallDetectorObservation& observation,
                          const R2StallDetectorConfig& config) {
    return observation.cmd_vx > std::abs(config.command_vx_threshold_mps);
}

}  // namespace

StallDetectorCore::StallDetectorCore(R2StallDetectorConfig config)
    : config_(std::move(config)) {}

void StallDetectorCore::setConfig(const R2StallDetectorConfig& config) {
    config_ = config;
    reset();
}

void StallDetectorCore::reset() {
    window_start_.reset();
}

StallDetectorDecision StallDetectorCore::update(
    const StallDetectorObservation& observation) {
    StallDetectorDecision decision;

    if (!forwardCommandActive(observation, config_)) {
        reset();
        decision.reason = "forward_command_inactive";
        return decision;
    }

    if (!window_start_) {
        window_start_ = observation;
        decision.reason = "window_started";
        return decision;
    }

    decision.forward_duration_sec =
        observation.time_sec - window_start_->time_sec;
    decision.wheel_delta_m =
        std::hypot(observation.wheel_odom_x - window_start_->wheel_odom_x,
                   observation.wheel_odom_y - window_start_->wheel_odom_y);
    decision.slam_delta_m =
        std::hypot(observation.slam_x - window_start_->slam_x,
                   observation.slam_y - window_start_->slam_y);
    decision.imu_motion_consistent =
        std::abs(observation.imu_linear_accel_x) >=
            config_.imu_linear_accel_threshold_mps2 ||
        std::abs(observation.imu_yaw_rate) >=
            config_.imu_yaw_rate_threshold_radps;

    if (decision.forward_duration_sec <
        config_.forward_command_min_duration_sec) {
        decision.reason = "duration_below_threshold";
        return decision;
    }
    if (decision.wheel_delta_m < config_.wheel_odom_delta_threshold_m) {
        decision.reason = "wheel_delta_below_threshold";
        return decision;
    }
    if (decision.slam_delta_m > config_.slam_delta_max_m ||
        decision.imu_motion_consistent) {
        decision.reason = decision.slam_delta_m > config_.slam_delta_max_m
                              ? "slam_motion_observed"
                              : "imu_motion_observed";
        window_start_ = observation;
        return decision;
    }
    if (observation.time_sec - last_trigger_sec_ <
        config_.retrigger_cooldown_sec) {
        decision.reason = "cooldown";
        return decision;
    }

    decision.stall_or_slip = true;
    decision.reason = "wheel_odom_without_slam_or_imu_motion";
    last_trigger_sec_ = observation.time_sec;
    window_start_ = observation;
    return decision;
}

}  // namespace yahboom_alc_bringup
