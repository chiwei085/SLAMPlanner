#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rik_sd/struct_descriptor.hpp>

#include <chrono>
#include <string>

namespace yahboom_alc_bringup
{

struct R2CmdVelBridgeConfig
{
    std::string input_topic = "/alc/nav_cmd_vel";
    std::string output_topic = "/cmd_vel";
    double min_linear_speed = 0.05;
    double max_linear_speed = 0.26;
    double steering_gain = 1.25;
    double max_steering = 1.25;
    double linear_deadband = 0.001;
    double angular_deadband = 0.001;
    double max_linear_accel = 0.20;
    double max_steering_rate = 0.80;
    double assumed_command_period = 0.10;
    bool arc_turn_when_stationary = true;
};

geometry_msgs::msg::Twist convertNavTwistToR2(
    const geometry_msgs::msg::Twist& input, const R2CmdVelBridgeConfig& config);

class R2CmdVelBridgeNode : public rclcpp::Node
{
public:
    explicit R2CmdVelBridgeNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

private:
    R2CmdVelBridgeConfig loadConfig();
    void onCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);
    geometry_msgs::msg::Twist limitRate(
        const geometry_msgs::msg::Twist& target);
    void publishStopBurst();

    R2CmdVelBridgeConfig config_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist last_output_;
    rclcpp::Time last_output_time_;
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
            named(&R2CmdVelBridgeConfig::min_linear_speed, "min_linear_speed"),
            named(&R2CmdVelBridgeConfig::max_linear_speed, "max_linear_speed"),
            named(&R2CmdVelBridgeConfig::steering_gain, "steering_gain"),
            named(&R2CmdVelBridgeConfig::max_steering, "max_steering"),
            named(&R2CmdVelBridgeConfig::linear_deadband, "linear_deadband"),
            named(&R2CmdVelBridgeConfig::angular_deadband, "angular_deadband"),
            named(&R2CmdVelBridgeConfig::max_linear_accel, "max_linear_accel"),
            named(&R2CmdVelBridgeConfig::max_steering_rate,
                  "max_steering_rate"),
            named(&R2CmdVelBridgeConfig::assumed_command_period,
                  "assumed_command_period"),
            named(&R2CmdVelBridgeConfig::arc_turn_when_stationary,
                  "arc_turn_when_stationary"));
    }
};

}  // namespace rik_sd
