#include "yahboom_alc_bringup/r2_cmd_vel_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace yahboom_alc_bringup
{
namespace
{

double applyDeadband(const double value, const double deadband) {
    return std::abs(value) < std::abs(deadband) ? 0.0 : value;
}

double signOf(const double value) {
    return value < 0.0 ? -1.0 : 1.0;
}

}  // namespace

geometry_msgs::msg::Twist convertNavTwistToR2(
    const geometry_msgs::msg::Twist& input,
    const R2CmdVelBridgeConfig& config) {
    geometry_msgs::msg::Twist output;

    const double requested_linear =
        applyDeadband(input.linear.x, config.linear_deadband);
    const double requested_angular =
        applyDeadband(input.angular.z, config.angular_deadband);

    double linear_x = 0.0;
    if (requested_linear != 0.0) {
        const double max_linear_speed = std::abs(config.max_linear_speed);
        linear_x =
            std::clamp(requested_linear, -max_linear_speed, max_linear_speed);
        if (std::abs(linear_x) < std::abs(config.min_linear_speed)) {
            linear_x = signOf(linear_x) * std::abs(config.min_linear_speed);
        }
    }
    else if (requested_angular != 0.0 && config.arc_turn_when_stationary) {
        linear_x = std::abs(config.min_linear_speed);
    }

    double steering = 0.0;
    if (requested_angular != 0.0) {
        const double requested_steering =
            requested_angular * config.steering_gain;
        const double steering_limit = std::abs(config.max_steering);
        steering =
            std::clamp(requested_steering, -steering_limit, steering_limit);
    }

    output.linear.x = linear_x;
    output.linear.y = steering;
    output.linear.z = 0.0;
    output.angular.x = 0.0;
    output.angular.y = 0.0;
    output.angular.z = 0.0;
    return output;
}

R2CmdVelBridgeNode::R2CmdVelBridgeNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("yahboom_r2_cmd_vel_bridge", options),
      config_(loadConfig()) {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        config_.output_topic, rclcpp::SystemDefaultsQoS());

    rclcpp::QoS input_qos{rclcpp::KeepLast(1)};
    input_qos.best_effort();
    subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        config_.input_topic, input_qos,
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
            onCmdVel(msg);
        });

    RCLCPP_INFO(get_logger(),
                "Yahboom R2 cmd_vel bridge: %s -> %s, min_linear=%.3f, "
                "max_linear=%.3f, steering_gain=%.3f, max_steering=%.3f",
                config_.input_topic.c_str(), config_.output_topic.c_str(),
                config_.min_linear_speed, config_.max_linear_speed,
                config_.steering_gain, config_.max_steering);
}

R2CmdVelBridgeConfig R2CmdVelBridgeNode::loadConfig() {
    R2CmdVelBridgeConfig config;

    rik_sd::for_each(config, [this](const auto& field, auto& value) {
        const auto& core = rik_sd::core_field(field);
        const std::string parameter_name{core.name_sv()};
        declare_parameter(parameter_name, value);

        const auto parameter = get_parameter(parameter_name);
        using value_t = rik_sd::remove_cvref_t<decltype(value)>;
        if constexpr (std::is_same_v<value_t, std::string>) {
            value = parameter.as_string();
        }
        else if constexpr (std::is_same_v<value_t, double>) {
            value = parameter.as_double();
        }
        else if constexpr (std::is_same_v<value_t, bool>) {
            value = parameter.as_bool();
        }
        else {
            static_assert(sizeof(value_t) == 0,
                          "Unsupported R2CmdVelBridgeConfig field type");
        }
    });

    return config;
}

void R2CmdVelBridgeNode::onCmdVel(
    const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
    if (!msg) {
        return;
    }

    publisher_->publish(convertNavTwistToR2(*msg, config_));
}

}  // namespace yahboom_alc_bringup
