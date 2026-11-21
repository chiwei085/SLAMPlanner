#include <algorithm>
#include <cmath>
#include <utility>

#include "yahboom_alc_bringup/r2_stall_detector.hpp"

namespace yahboom_alc_bringup
{

StallDetectorCore::StallDetectorCore(R2StallDetectorConfig config)
    : config_(std::move(config)) {}

void StallDetectorCore::setConfig(const R2StallDetectorConfig& config) {
    config_ = config;
    reset();
}

void StallDetectorCore::reset() {
    window_start_.reset();
    expected_motion_m_ = 0.0;
}

void StallDetectorCore::restartWindow(
    const StallDetectorObservation& observation, const bool from_rest) {
    window_start_ = observation;
    window_from_rest_ = from_rest;
    last_observation_sec_ = observation.time_sec;
    expected_motion_m_ = 0.0;
}

StallDetectorDecision StallDetectorCore::update(
    const StallDetectorObservation& observation) {
    StallDetectorDecision decision;

    if (observation.driver_cmd_vx <= config_.command_vx_threshold_mps) {
        reset();
        decision.reason = "forward_command_inactive";
        return decision;
    }
    if (!window_start_) {
        restartWindow(observation, true);
        decision.reason = "window_started";
        return decision;
    }

    decision.forward_duration_sec =
        observation.time_sec - window_start_->time_sec;
    const bool responding =
        !window_from_rest_ ||
        decision.forward_duration_sec >= config_.start_response_delay_sec;
    if (responding) {
        expected_motion_m_ +=
            observation.driver_cmd_vx *
            std::max(0.0, observation.time_sec - last_observation_sec_);
    }
    last_observation_sec_ = observation.time_sec;
    decision.expected_motion_m = expected_motion_m_;
    decision.odom_delta_m =
        std::hypot(observation.odom_x - window_start_->odom_x,
                   observation.odom_y - window_start_->odom_y);
    decision.imu_motion_consistent = std::abs(observation.imu_yaw_rate) >=
                                     config_.imu_yaw_rate_threshold_radps;

    if (expected_motion_m_ < config_.min_expected_motion_m) {
        decision.reason = "expected_motion_below_noise";
        return decision;
    }
    if (decision.odom_delta_m >=
        config_.stuck_motion_ratio * expected_motion_m_) {
        decision.reason = "motion_observed";
        restartWindow(observation, false);
        return decision;
    }
    if (decision.imu_motion_consistent) {
        decision.reason = "imu_yaw_motion_observed";
        return decision;
    }
    if (observation.time_sec - last_trigger_sec_ <
        config_.retrigger_cooldown_sec) {
        decision.reason = "cooldown";
        return decision;
    }

    decision.stall_or_slip = true;
    decision.reason = "commanded_motion_not_observed";
    last_trigger_sec_ = observation.time_sec;
    restartWindow(observation, false);
    return decision;
}

}  // namespace yahboom_alc_bringup
