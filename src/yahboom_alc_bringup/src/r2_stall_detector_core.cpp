#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
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

bool cooldownActive(const StallDetectorObservation& observation,
                    const double last_trigger_sec,
                    const R2StallDetectorConfig& config) {
    return observation.time_sec - last_trigger_sec <
           config.retrigger_cooldown_sec;
}

int centeredSpanStart(const std::uint32_t size, const double ratio) {
    const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
    const auto span = std::max<std::uint32_t>(
        1, static_cast<std::uint32_t>(std::round(size * clamped_ratio)));
    return static_cast<int>((size - std::min(size, span)) / 2);
}

int centeredSpanEnd(const std::uint32_t size, const double ratio) {
    const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
    const auto span = std::max<std::uint32_t>(
        1, static_cast<std::uint32_t>(std::round(size * clamped_ratio)));
    return static_cast<int>((size + std::min(size, span)) / 2);
}

bool depthInConfiguredRange(const double depth_m,
                            const R2StallDetectorConfig& config) {
    return std::isfinite(depth_m) && depth_m >= config.depth_min_valid_m &&
           depth_m <= config.depth_forward_obstacle_range_m;
}

double readDepthMeters(const sensor_msgs::msg::Image& image,
                       const std::size_t offset) {
    if (image.encoding == "16UC1") {
        std::uint16_t depth_mm = 0;
        std::memcpy(&depth_mm, image.data.data() + offset, sizeof(depth_mm));
        if (depth_mm == 0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return static_cast<double>(depth_mm) / 1000.0;
    }
    if (image.encoding == "32FC1") {
        float depth_m = 0.0F;
        std::memcpy(&depth_m, image.data.data() + offset, sizeof(depth_m));
        return static_cast<double>(depth_m);
    }
    return std::numeric_limits<double>::quiet_NaN();
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

DepthRoiResult estimateForwardDepthRoi(const sensor_msgs::msg::Image& image,
                                       const R2StallDetectorConfig& config) {
    DepthRoiResult result;
    if (!config.depth_obstacle_enabled || image.width == 0 ||
        image.height == 0 || image.step == 0) {
        return result;
    }

    std::size_t pixel_size = 0;
    if (image.encoding == "16UC1") {
        pixel_size = sizeof(std::uint16_t);
    }
    else if (image.encoding == "32FC1") {
        pixel_size = sizeof(float);
    }
    else {
        return result;
    }

    const std::size_t min_step =
        static_cast<std::size_t>(image.width) * pixel_size;
    if (image.step < min_step ||
        image.data.size() <
            image.step * static_cast<std::size_t>(image.height)) {
        return result;
    }

    const int x_begin =
        centeredSpanStart(image.width, config.depth_roi_center_width_ratio);
    const int x_end =
        centeredSpanEnd(image.width, config.depth_roi_center_width_ratio);
    const int y_begin =
        centeredSpanStart(image.height, config.depth_roi_center_height_ratio);
    const int y_end =
        centeredSpanEnd(image.height, config.depth_roi_center_height_ratio);

    double min_depth_m = std::numeric_limits<double>::infinity();
    int obstacle_pixel_count = 0;
    for (int y = y_begin; y < y_end; ++y) {
        const auto row_offset = static_cast<std::size_t>(y) * image.step;
        for (int x = x_begin; x < x_end; ++x) {
            const auto offset =
                row_offset + static_cast<std::size_t>(x) * pixel_size;
            const double depth_m = readDepthMeters(image, offset);
            if (!depthInConfiguredRange(depth_m, config)) {
                continue;
            }
            min_depth_m = std::min(min_depth_m, depth_m);
            ++obstacle_pixel_count;
        }
    }

    result.valid = true;
    result.obstacle_pixel_count = obstacle_pixel_count;
    if (std::isfinite(min_depth_m)) {
        result.min_depth_m = min_depth_m;
        result.obstacle_detected =
            obstacle_pixel_count >= config.depth_min_obstacle_pixels;
    }
    return result;
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
    decision.imu_motion_consistent = std::abs(observation.imu_yaw_rate) >=
                                     config_.imu_yaw_rate_threshold_radps;
    decision.driver_command_valid = observation.driver_command_valid;
    decision.forward_min_depth_m = observation.forward_min_depth_m;
    decision.forward_obstacle_detected = observation.forward_obstacle_detected;
    decision.depth_observation_valid = observation.depth_observation_valid;

    if (decision.forward_duration_sec <
        config_.forward_command_min_duration_sec) {
        decision.reason = "duration_below_threshold";
        return decision;
    }
    if (decision.slam_delta_m > config_.slam_delta_max_m) {
        decision.reason = "slam_motion_observed";
        window_start_ = observation;
        return decision;
    }
    if (decision.imu_motion_consistent) {
        decision.reason = "imu_yaw_motion_observed";
        return decision;
    }
    if (cooldownActive(observation, last_trigger_sec_, config_)) {
        decision.reason = "cooldown";
        return decision;
    }

    if (decision.forward_obstacle_detected) {
        decision.stall_or_slip = true;
        decision.reason = "forward_obstacle_without_slam_motion";
    }
    else if (decision.wheel_delta_m >= config_.wheel_odom_delta_threshold_m) {
        decision.stall_or_slip = true;
        decision.reason = "wheel_odom_without_slam_motion";
    }
    else if (observation.driver_command_valid &&
             observation.driver_cmd_vx >
                 std::abs(config_.command_vx_threshold_mps) &&
             config_.forward_no_motion_check_sec > 0.0 &&
             decision.forward_duration_sec >=
                 config_.forward_no_motion_check_sec) {
        decision.stall_or_slip = true;
        decision.reason = "forward_command_without_slam_motion";
    }
    else {
        decision.reason = "wheel_delta_below_threshold";
    }
    if (decision.stall_or_slip) {
        last_trigger_sec_ = observation.time_sec;
        window_start_ = observation;
    }
    return decision;
}

}  // namespace yahboom_alc_bringup
