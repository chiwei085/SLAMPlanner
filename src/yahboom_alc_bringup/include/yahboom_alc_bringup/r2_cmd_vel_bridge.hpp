#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rik_sd/struct_descriptor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <atomic>
#include <memory>
#include <optional>
#include <string>

#include "yahboom_alc_bringup/recent_free_costmap_history.hpp"
#include "yahboom_alc_bringup/temporal_signal.hpp"

namespace yahboom_alc_bringup
{

struct R2CmdVelBridgeConfig
{
    std::string input_topic = "/alc/nav_cmd_vel";
    std::string output_topic = "/cmd_vel";
    std::string planner_event_topic = "/planner/events";
    double min_linear_speed = 0.05;
    bool enforce_min_linear_speed = true;
    double max_linear_speed = 0.26;
    double steering_gain = 1.25;
    double max_steering = 1.25;
    double wheelbase_m = 0.16;
    double min_turning_radius_m = 0.50;
    double max_curvature = 2.0;
    double max_steering_angle_rad = 1.25;
    double linear_deadband = 0.001;
    double angular_deadband = 0.001;
    double max_linear_accel = 0.20;
    double max_steering_rate = 0.80;
    double assumed_command_period = 0.10;
    bool arc_turn_when_stationary = true;
    bool stationary_arc_requires_forward_clearance = true;
    double stationary_arc_check_distance_m = 0.12;
    bool reverse_safety_enabled = true;
    std::string reverse_costmap_topic = "/local_costmap/costmap";
    std::string reverse_robot_frame = "base_footprint";
    double reverse_costmap_max_age_sec = 1.0;
    double reverse_check_distance_m = 0.12;
    double reverse_safety_margin_m = 0.0;
    int reverse_allowed_cost_max = 98;
    bool reverse_unknown_is_blocked = true;
    // A cell that flips to unknown (falls out of the rolling costmap's
    // observation window) but was seen clearly free within this many
    // seconds is treated as free rather than blocked. 0 disables the
    // distinction and restores strict fail-closed-on-unknown behavior.
    double reverse_recent_free_grace_sec = 2.0;
    // Unlike reverse_unknown_is_blocked, this defaults to false: the
    // stationary-arc forward check runs on every goal that points toward a
    // frontier, and a frontier's entire purpose is bordering unknown space.
    // Blocking on unknown ahead stalls the robot at every goal instead of
    // just the rare, high-consequence blind-reverse case that
    // reverse_unknown_is_blocked exists for.
    bool stationary_arc_unknown_is_blocked = false;
    // While the frontier explorer runs an observation turn (turning to look
    // at unknown space behind the robot), reverse may enter unknown -- never
    // known-occupied -- cells. The robot has no rear sensor, so this is an
    // accepted blind-reverse risk, bounded to this many seconds per turn.
    // 0 disables it.
    double observation_blind_reverse_max_sec = 30.0;
    // The R2 driver latches the last /cmd_vel it received and has no timeout
    // of its own, so a producer that stops publishing (or a single lost zero)
    // leaves the wheels running. When no input arrives for this long, publish
    // centered zero for idle_stop_burst_sec. 0 disables.
    double input_timeout_sec = 0.5;
    double idle_stop_burst_sec = 1.0;
    // Raw chassis IMU. When its content stops changing for
    // feedback_frozen_timeout_sec, every motion command is replaced by zero:
    // the robot cannot sense its own motion, so no recovery can be judged.
    // Empty disables.
    std::string feedback_imu_topic;
    double feedback_frozen_timeout_sec = 0.5;
    double footprint_front_m = 0.19;
    double footprint_rear_m = 0.19;
    double footprint_half_width_m = 0.115;
    double footprint_padding_m = 0.02;
};

struct ReverseSafetyPose
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

bool isReverseSafeInCostmap(
    const nav_msgs::msg::OccupancyGrid& costmap, const ReverseSafetyPose& pose,
    double reverse_speed, const R2CmdVelBridgeConfig& config,
    const RecentFreeCostmapHistory* recent_free_history = nullptr,
    double now_sec = 0.0);

bool isStationaryArcForwardSafeInCostmap(
    const nav_msgs::msg::OccupancyGrid& costmap, const ReverseSafetyPose& pose,
    double forward_speed, const R2CmdVelBridgeConfig& config,
    const RecentFreeCostmapHistory* recent_free_history = nullptr,
    double now_sec = 0.0);

geometry_msgs::msg::Twist convertNavTwistToR2(
    const geometry_msgs::msg::Twist& input, const R2CmdVelBridgeConfig& config,
    bool allow_stationary_arc = true);

// Reconciles a rate-limited drive command with the motor's static-friction
// floor. The wheels cannot turn below min_linear_speed, so any rate-limited
// value inside that band moves the steering servo while the robot stays still.
// When motion is intended (|target| past the deadband) the command snaps up to
// the stall-clearing floor as a gentle breakaway; otherwise sub-stall ramp
// residue snaps to a clean stop. Returns the rate-limited value unchanged when
// the floor is disabled.
double applyDriveStallFloor(double rate_limited_linear_x,
                            double target_linear_x,
                            const R2CmdVelBridgeConfig& config);

// Tracks the explorer's observation-turn window from planner events:
// "observation_turn_started" opens it for max_sec, "observation_turn_finished"
// closes it. The time bound keeps a lost "finished" event from leaving blind
// reverse enabled.
// True while the bridge should be streaming idle zero commands: input has
// been silent for more than timeout_sec, and the zero burst (burst_sec long)
// that follows has not finished yet.
bool shouldPublishIdleStop(double since_last_input_sec, double timeout_sec,
                           double burst_sec);

class BlindReverseWindow
{
public:
    void onEvent(const std::string& event_name, double now_sec, double max_sec);
    bool allowed(double now_sec) const { return now_sec < open_until_sec_; }

private:
    double open_until_sec_ = 0.0;
};

// A live MEMS IMU never repeats a sample exactly (0 of 300 consecutive
// stationary pairs on the R2). After the Yahboom driver's serial reader dies
// it keeps republishing its last cached sample at the normal rate, so arrival
// rate cannot reveal it; content can. Frozen until the first sample arrives.
class FeedbackFreezeWatch
{
public:
    using Sample = std::array<double, 6>;
    void observe(const Sample& sample, double now_sec);
    bool frozen(double now_sec, double timeout_sec) const;

private:
    std::optional<Sample> last_;
    std::optional<double> last_change_sec_;
};

class R2CmdVelBridgeNode : public rclcpp::Node
{
public:
    explicit R2CmdVelBridgeNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    void publishStopBurst();

private:
    R2CmdVelBridgeConfig loadConfig();
    void onCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);
    geometry_msgs::msg::Twist limitRate(const geometry_msgs::msg::Twist& target,
                                        bool apply_stationary_arc_safety);
    geometry_msgs::msg::Twist applyReverseSafety(
        const geometry_msgs::msg::Twist& target);
    geometry_msgs::msg::Twist applyStationaryArcSafety(
        const geometry_msgs::msg::Twist& target);
    std::optional<ReverseSafetyPose> lookupReverseSafetyPose(
        const nav_msgs::msg::OccupancyGrid& costmap, const char* warning_label);
    // Freshness is judged from receive-silence on the host clock (time since
    // the last costmap message actually arrived), not from the message's own
    // header stamp -- the producer's clock domain may not agree with ours,
    // and a stamp-age check would silently inherit that skew.
    bool isCostmapFresh();
    bool reverseUnknownIsBlocked();
    void publishPlannerEvent(const std::string& event,
                             const std::string& reason);

    R2CmdVelBridgeConfig config_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr feedback_imu_sub_;
    FeedbackFreezeWatch feedback_watch_;
    bool feedback_frozen_ = false;
    BlindReverseWindow blind_reverse_window_;
    rclcpp::TimerBase::SharedPtr idle_stop_timer_;
    std::optional<rclcpp::Time> last_input_time_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_costmap_;
    RecentFreeCostmapHistory reverse_recent_free_history_;
    TemporalSignalState costmap_temporal_state_;
    TemporalSignalObservation costmap_temporal_;
    geometry_msgs::msg::Twist last_output_;
    rclcpp::Time last_output_time_;
    std::atomic<bool> stopping_{false};
    bool has_last_output_ = false;
};

}  // namespace yahboom_alc_bringup

namespace rik_sd
{

template <>
struct descriptor<yahboom_alc_bringup::R2CmdVelBridgeConfig>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::R2CmdVelBridgeConfig;
        return make_descriptor(
            named(&R2CmdVelBridgeConfig::input_topic, "input_topic"),
            named(&R2CmdVelBridgeConfig::output_topic, "output_topic"),
            named(&R2CmdVelBridgeConfig::planner_event_topic,
                  "planner_event_topic"),
            named(&R2CmdVelBridgeConfig::min_linear_speed, "min_linear_speed"),
            named(&R2CmdVelBridgeConfig::enforce_min_linear_speed,
                  "enforce_min_linear_speed"),
            named(&R2CmdVelBridgeConfig::max_linear_speed, "max_linear_speed"),
            named(&R2CmdVelBridgeConfig::steering_gain, "steering_gain"),
            named(&R2CmdVelBridgeConfig::max_steering, "max_steering"),
            named(&R2CmdVelBridgeConfig::wheelbase_m, "wheelbase_m"),
            named(&R2CmdVelBridgeConfig::min_turning_radius_m,
                  "min_turning_radius_m"),
            named(&R2CmdVelBridgeConfig::max_curvature, "max_curvature"),
            named(&R2CmdVelBridgeConfig::max_steering_angle_rad,
                  "max_steering_angle_rad"),
            named(&R2CmdVelBridgeConfig::linear_deadband, "linear_deadband"),
            named(&R2CmdVelBridgeConfig::angular_deadband, "angular_deadband"),
            named(&R2CmdVelBridgeConfig::max_linear_accel, "max_linear_accel"),
            named(&R2CmdVelBridgeConfig::max_steering_rate,
                  "max_steering_rate"),
            named(&R2CmdVelBridgeConfig::assumed_command_period,
                  "assumed_command_period"),
            named(&R2CmdVelBridgeConfig::arc_turn_when_stationary,
                  "arc_turn_when_stationary"),
            named(&R2CmdVelBridgeConfig::
                      stationary_arc_requires_forward_clearance,
                  "stationary_arc_requires_forward_clearance"),
            named(&R2CmdVelBridgeConfig::stationary_arc_check_distance_m,
                  "stationary_arc_check_distance_m"),
            named(&R2CmdVelBridgeConfig::reverse_safety_enabled,
                  "reverse_safety_enabled"),
            named(&R2CmdVelBridgeConfig::reverse_costmap_topic,
                  "reverse_costmap_topic"),
            named(&R2CmdVelBridgeConfig::reverse_robot_frame,
                  "reverse_robot_frame"),
            named(&R2CmdVelBridgeConfig::reverse_costmap_max_age_sec,
                  "reverse_costmap_max_age_sec"),
            named(&R2CmdVelBridgeConfig::reverse_check_distance_m,
                  "reverse_check_distance_m"),
            named(&R2CmdVelBridgeConfig::reverse_safety_margin_m,
                  "reverse_safety_margin_m"),
            named(&R2CmdVelBridgeConfig::reverse_allowed_cost_max,
                  "reverse_allowed_cost_max"),
            named(&R2CmdVelBridgeConfig::reverse_unknown_is_blocked,
                  "reverse_unknown_is_blocked"),
            named(&R2CmdVelBridgeConfig::reverse_recent_free_grace_sec,
                  "reverse_recent_free_grace_sec"),
            named(&R2CmdVelBridgeConfig::stationary_arc_unknown_is_blocked,
                  "stationary_arc_unknown_is_blocked"),
            named(&R2CmdVelBridgeConfig::observation_blind_reverse_max_sec,
                  "observation_blind_reverse_max_sec"),
            named(&R2CmdVelBridgeConfig::input_timeout_sec,
                  "input_timeout_sec"),
            named(&R2CmdVelBridgeConfig::idle_stop_burst_sec,
                  "idle_stop_burst_sec"),
            named(&R2CmdVelBridgeConfig::feedback_imu_topic,
                  "feedback_imu_topic"),
            named(&R2CmdVelBridgeConfig::feedback_frozen_timeout_sec,
                  "feedback_frozen_timeout_sec"),
            named(&R2CmdVelBridgeConfig::footprint_front_m,
                  "footprint_front_m"),
            named(&R2CmdVelBridgeConfig::footprint_rear_m, "footprint_rear_m"),
            named(&R2CmdVelBridgeConfig::footprint_half_width_m,
                  "footprint_half_width_m"),
            named(&R2CmdVelBridgeConfig::footprint_padding_m,
                  "footprint_padding_m"));
    }
};

}  // namespace rik_sd
