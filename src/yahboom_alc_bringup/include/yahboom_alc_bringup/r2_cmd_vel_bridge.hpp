#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <array>
#include <optional>
#include <string>

#include "yahboom_alc_bringup/recent_free_costmap_history.hpp"

namespace yahboom_alc_bringup
{

// Highest OccupancyGrid cost a cell under the swept footprint may have. Only
// lethal (100) means the body overlaps an obstacle; inscribed (99) is defined
// for the robot *center* and would count the half width a second time.
constexpr int kFootprintFreeCostMax = 99;

struct R2CmdVelBridgeConfig
{
    std::string input_topic = "/alc/nav_cmd_vel";
    std::string behavior_topic = "/alc/behavior_cmd_vel";
    std::string output_topic = "/cmd_vel";
    std::string planner_event_topic = "/planner/events";
    double min_linear_speed = 0.05;
    bool enforce_min_linear_speed = true;
    double max_linear_speed = 0.26;
    // Yahboom R2 CAD/URDF: rear axle x=-0.11607, steer joints x=0.11922.
    double wheelbase_m = 0.2353;
    double min_turning_radius_m = 0.50;
    double max_curvature = 2.0;
    // URDF steer-joint limit; the firmware accepts up to 45 degrees.
    double max_steering_angle_rad = 0.6;
    double linear_deadband = 0.001;
    double angular_deadband = 0.001;
    double max_linear_accel = 0.20;
    // Wheel-angle slew limit, rad/s.
    double max_steering_rate = 1.0;
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

// The R2 firmware takes the front wheel angle in degrees / 1000 on
// cmd_vel.linear.y (Rosmaster_Lib set_car_motion: R2 v_y in [-0.045, 0.045]).
double wheelAngleToFirmwareSteering(double wheel_angle_rad);

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

geometry_msgs::msg::Twist reversedArc(const geometry_msgs::msg::Twist& arc);

// True while the bridge should be streaming idle zero commands: input has
// been silent for more than timeout_sec, and the zero burst (burst_sec long)
// has not finished yet. The burst is timed from the first tick that sees the
// silence, not from the last input, so a late timer still sends the zeros.
// burst_end_sec holds that deadline; reset it whenever input arrives.
bool shouldPublishIdleStop(double now_sec, double last_input_sec,
                           double timeout_sec, double burst_sec,
                           std::optional<double>& burst_end_sec);

// Tracks the explorer's observation-turn window from planner events:
// "observation_turn_started" opens it for max_sec, "observation_turn_finished"
// closes it. The time bound keeps a lost "finished" event from leaving blind
// reverse enabled.
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

// A behavior (BackUp, Spin) owns the robot while it publishes. The velocity
// smoother keeps ramping the cancelled path command down for about a second,
// and the speed floor would turn both streams into alternating full commands.
class CommandOwner
{
public:
    void onBehavior(double now_sec) { last_behavior_sec_ = now_sec; }
    bool navAllowed(double now_sec) const {
        // Three periods of the 10 Hz behavior server.
        return !last_behavior_sec_ || now_sec - *last_behavior_sec_ > 0.3;
    }

private:
    std::optional<double> last_behavior_sec_;
};

}  // namespace yahboom_alc_bringup
