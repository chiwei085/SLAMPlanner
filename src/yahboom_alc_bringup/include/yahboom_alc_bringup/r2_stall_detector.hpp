#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rik_sd/struct_descriptor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <optional>
#include <string>

namespace yahboom_alc_bringup
{

struct R2StallDetectorConfig
{
    std::string nav_cmd_topic = "/alc/nav_cmd_vel";
    std::string driver_cmd_topic = "/cmd_vel";
    std::string wheel_odom_topic = "/odom";
    std::string imu_topic = "/imu/data";
    std::string depth_image_topic = "/camera/depth/image_raw";
    std::string planner_event_topic = "/planner/events";
    std::string slam_global_frame = "map";
    std::string robot_frame = "base_footprint";
    double forward_command_min_duration_sec = 0.5;
    double forward_no_motion_check_sec = 0.8;
    double command_vx_threshold_mps = 0.08;
    double wheel_odom_delta_threshold_m = 0.05;
    double slam_delta_max_m = 0.02;
    double imu_yaw_rate_threshold_radps = 0.08;
    bool depth_obstacle_enabled = true;
    double depth_roi_center_width_ratio = 0.20;
    double depth_roi_center_height_ratio = 0.50;
    double depth_min_valid_m = 0.20;
    double depth_forward_obstacle_range_m = 0.60;
    int depth_min_obstacle_pixels = 5;
    double depth_stale_timeout_sec = 0.5;
    double stale_observation_timeout_sec = 0.5;
    double zero_burst_duration_sec = 0.5;
    double zero_burst_rate_hz = 30.0;
    double retrigger_cooldown_sec = 3.0;
};

struct StallDetectorObservation
{
    double time_sec = 0.0;
    double cmd_vx = 0.0;
    double wheel_odom_x = 0.0;
    double wheel_odom_y = 0.0;
    double slam_x = 0.0;
    double slam_y = 0.0;
    double imu_yaw_rate = 0.0;
    double driver_cmd_vx = 0.0;
    bool driver_command_valid = false;
    double forward_min_depth_m = 0.0;
    bool forward_obstacle_detected = false;
    bool depth_observation_valid = false;
};

struct StallDetectorDecision
{
    bool stall_or_slip = false;
    std::string reason;
    double forward_duration_sec = 0.0;
    double wheel_delta_m = 0.0;
    double slam_delta_m = 0.0;
    bool imu_motion_consistent = false;
    bool driver_command_valid = false;
    double forward_min_depth_m = 0.0;
    bool forward_obstacle_detected = false;
    bool depth_observation_valid = false;
};

struct DepthRoiResult
{
    bool valid = false;
    double min_depth_m = 0.0;
    bool obstacle_detected = false;
    int obstacle_pixel_count = 0;
};

class StallDetectorCore
{
public:
    explicit StallDetectorCore(R2StallDetectorConfig config = {});

    void setConfig(const R2StallDetectorConfig& config);
    void reset();
    StallDetectorDecision update(const StallDetectorObservation& observation);

private:
    R2StallDetectorConfig config_;
    std::optional<StallDetectorObservation> window_start_;
    double last_trigger_sec_ = -1.0e9;
};

DepthRoiResult estimateForwardDepthRoi(const sensor_msgs::msg::Image& image,
                                       const R2StallDetectorConfig& config);

}  // namespace yahboom_alc_bringup

namespace rik_sd
{

template <>
struct descriptor<yahboom_alc_bringup::R2StallDetectorConfig>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::R2StallDetectorConfig;
        return make_descriptor(
            named(&R2StallDetectorConfig::nav_cmd_topic, "nav_cmd_topic"),
            named(&R2StallDetectorConfig::driver_cmd_topic, "driver_cmd_topic"),
            named(&R2StallDetectorConfig::wheel_odom_topic, "wheel_odom_topic"),
            named(&R2StallDetectorConfig::imu_topic, "imu_topic"),
            named(&R2StallDetectorConfig::depth_image_topic,
                  "depth_image_topic"),
            named(&R2StallDetectorConfig::planner_event_topic,
                  "planner_event_topic"),
            named(&R2StallDetectorConfig::slam_global_frame,
                  "slam_global_frame"),
            named(&R2StallDetectorConfig::robot_frame, "robot_frame"),
            named(&R2StallDetectorConfig::forward_command_min_duration_sec,
                  "forward_command_min_duration_sec"),
            named(&R2StallDetectorConfig::forward_no_motion_check_sec,
                  "forward_no_motion_check_sec"),
            named(&R2StallDetectorConfig::command_vx_threshold_mps,
                  "command_vx_threshold_mps"),
            named(&R2StallDetectorConfig::wheel_odom_delta_threshold_m,
                  "wheel_odom_delta_threshold_m"),
            named(&R2StallDetectorConfig::slam_delta_max_m, "slam_delta_max_m"),
            named(&R2StallDetectorConfig::imu_yaw_rate_threshold_radps,
                  "imu_yaw_rate_threshold_radps"),
            named(&R2StallDetectorConfig::depth_obstacle_enabled,
                  "depth_obstacle_enabled"),
            named(&R2StallDetectorConfig::depth_roi_center_width_ratio,
                  "depth_roi_center_width_ratio"),
            named(&R2StallDetectorConfig::depth_roi_center_height_ratio,
                  "depth_roi_center_height_ratio"),
            named(&R2StallDetectorConfig::depth_min_valid_m,
                  "depth_min_valid_m"),
            named(&R2StallDetectorConfig::depth_forward_obstacle_range_m,
                  "depth_forward_obstacle_range_m"),
            named(&R2StallDetectorConfig::depth_min_obstacle_pixels,
                  "depth_min_obstacle_pixels"),
            named(&R2StallDetectorConfig::depth_stale_timeout_sec,
                  "depth_stale_timeout_sec"),
            named(&R2StallDetectorConfig::stale_observation_timeout_sec,
                  "stale_observation_timeout_sec"),
            named(&R2StallDetectorConfig::zero_burst_duration_sec,
                  "zero_burst_duration_sec"),
            named(&R2StallDetectorConfig::zero_burst_rate_hz,
                  "zero_burst_rate_hz"),
            named(&R2StallDetectorConfig::retrigger_cooldown_sec,
                  "retrigger_cooldown_sec"));
    }
};

}  // namespace rik_sd
