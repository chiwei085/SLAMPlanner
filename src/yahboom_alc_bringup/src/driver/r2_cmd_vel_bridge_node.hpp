#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>

#include "yahboom_alc_bringup/r2_cmd_vel_bridge.hpp"
#include "yahboom_alc_bringup/recent_free_costmap_history.hpp"
#include "yahboom_alc_bringup/temporal_signal.hpp"

namespace yahboom_alc_bringup
{

class R2CmdVelBridgeNode : public rclcpp::Node
{
public:
    explicit R2CmdVelBridgeNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    void publishStopBurst();

private:
    R2CmdVelBridgeConfig loadConfig();
    void onCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);
    CommandOwner command_owner_;
    geometry_msgs::msg::Twist limitRate(const geometry_msgs::msg::Twist& target,
                                        bool stationary_arc);
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
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr behavior_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_event_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr feedback_imu_sub_;
    FeedbackFreezeWatch feedback_watch_;
    bool feedback_frozen_ = false;
    BlindReverseWindow blind_reverse_window_;
    rclcpp::TimerBase::SharedPtr idle_stop_timer_;
    std::optional<rclcpp::Time> last_input_time_;
    std::optional<double> idle_stop_burst_end_sec_;
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
