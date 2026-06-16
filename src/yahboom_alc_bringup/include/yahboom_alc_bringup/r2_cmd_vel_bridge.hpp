#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rik_sd/struct_descriptor.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>

namespace yahboom_alc_bringup
{

struct R2CmdVelBridgeConfig
{
    std::string input_topic = "/alc/nav_cmd_vel";
    std::string output_topic = "/cmd_vel";
    std::string planner_event_topic = "/planner/events";
    double min_linear_speed = 0.05;
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

bool isReverseSafeInCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                            const ReverseSafetyPose& pose,
                            const double reverse_speed,
                            const R2CmdVelBridgeConfig& config);

bool isStationaryArcForwardSafeInCostmap(
    const nav_msgs::msg::OccupancyGrid& costmap, const ReverseSafetyPose& pose,
    const double forward_speed, const R2CmdVelBridgeConfig& config);

geometry_msgs::msg::Twist convertNavTwistToR2(
    const geometry_msgs::msg::Twist& input, const R2CmdVelBridgeConfig& config);

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
    bool isCostmapFresh(const nav_msgs::msg::OccupancyGrid& costmap);
    void publishPlannerEvent(const std::string& event,
                             const std::string& reason);

    R2CmdVelBridgeConfig config_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_costmap_;
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
