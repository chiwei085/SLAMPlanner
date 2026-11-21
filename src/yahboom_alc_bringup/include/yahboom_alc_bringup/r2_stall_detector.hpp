#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rik_sd/struct_descriptor.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <optional>
#include <string>

namespace yahboom_alc_bringup
{

// A stall is: the chassis was commanded forward (the driver command, after
// the bridge's ramp, floor and safety gates) but did not move. "Moved" is the
// EKF odom (wheel encoders + gyro yaw): with this R2 setup RTAB-Map never
// corrected odometry (map->odom stayed identity, 0 refined neighbor links),
// so a SLAM pose would only repeat the same signal, and no sensor here
// measures body translation independently of the encoders -- wheel slip is
// not observable. Obstacles are the costmaps' job (body-frame clouds); the
// verdict does not depend on what a camera sees.
struct R2StallDetectorConfig
{
    std::string driver_cmd_topic = "/cmd_vel";
    std::string odom_topic = "/odom";
    std::string imu_topic = "/imu/data";
    std::string planner_event_topic = "/planner/events";
    // Forward driver commands above this are judged. The bridge never sends
    // less than its 0.08 m/s floor, so keep this below the floor.
    double command_vx_threshold_mps = 0.05;
    // Judge only after the counted commands add up to this distance (3x the
    // ~2 cm pose noise); moving less than stuck_motion_ratio of it is stuck.
    double min_expected_motion_m = 0.06;
    double stuck_motion_ratio = 0.3;
    // After a start from rest the chassis speed loop barely moves the wheels
    // for this long (R2, no bridge, free floor: 1 cm took 0.8-1.45 s over 20
    // starts, then overshoot catches up). Commands in that span are not
    // expected motion. Not applied after a mid-drive window restart.
    double start_response_delay_sec = 0.0;
    double imu_yaw_rate_threshold_radps = 0.08;
    double stale_observation_timeout_sec = 0.5;
    double zero_burst_duration_sec = 0.5;
    double retrigger_cooldown_sec = 3.0;
};

struct StallDetectorObservation
{
    double time_sec = 0.0;
    double driver_cmd_vx = 0.0;
    double odom_x = 0.0;
    double odom_y = 0.0;
    double imu_yaw_rate = 0.0;
};

struct StallDetectorDecision
{
    bool stall_or_slip = false;
    std::string reason;
    double forward_duration_sec = 0.0;
    double expected_motion_m = 0.0;
    double odom_delta_m = 0.0;
    bool imu_motion_consistent = false;
};

class StallDetectorCore
{
public:
    explicit StallDetectorCore(R2StallDetectorConfig config = {});

    void setConfig(const R2StallDetectorConfig& config);
    void reset();
    StallDetectorDecision update(const StallDetectorObservation& observation);

private:
    void restartWindow(const StallDetectorObservation& observation,
                       bool from_rest);

    R2StallDetectorConfig config_;
    std::optional<StallDetectorObservation> window_start_;
    double last_observation_sec_ = 0.0;
    double expected_motion_m_ = 0.0;
    bool window_from_rest_ = true;
    double last_trigger_sec_ = -1.0e9;
};

}  // namespace yahboom_alc_bringup

namespace rik_sd
{

template <>
struct descriptor<yahboom_alc_bringup::R2StallDetectorConfig>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::R2StallDetectorConfig;
        return make_descriptor(
            named(&R2StallDetectorConfig::driver_cmd_topic, "driver_cmd_topic"),
            named(&R2StallDetectorConfig::odom_topic, "odom_topic"),
            named(&R2StallDetectorConfig::imu_topic, "imu_topic"),
            named(&R2StallDetectorConfig::planner_event_topic,
                  "planner_event_topic"),
            named(&R2StallDetectorConfig::command_vx_threshold_mps,
                  "command_vx_threshold_mps"),
            named(&R2StallDetectorConfig::min_expected_motion_m,
                  "min_expected_motion_m"),
            named(&R2StallDetectorConfig::stuck_motion_ratio,
                  "stuck_motion_ratio"),
            named(&R2StallDetectorConfig::start_response_delay_sec,
                  "start_response_delay_sec"),
            named(&R2StallDetectorConfig::imu_yaw_rate_threshold_radps,
                  "imu_yaw_rate_threshold_radps"),
            named(&R2StallDetectorConfig::stale_observation_timeout_sec,
                  "stale_observation_timeout_sec"),
            named(&R2StallDetectorConfig::zero_burst_duration_sec,
                  "zero_burst_duration_sec"),
            named(&R2StallDetectorConfig::retrigger_cooldown_sec,
                  "retrigger_cooldown_sec"));
    }
};

}  // namespace rik_sd
