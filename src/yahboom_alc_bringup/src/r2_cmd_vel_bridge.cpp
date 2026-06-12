#include "yahboom_alc_bringup/r2_cmd_vel_bridge.hpp"

#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>

#include "yahboom_alc_bringup/parameter_loader.hpp"
#include "yahboom_alc_bringup/yaw_util.hpp"

namespace yahboom_alc_bringup
{
using namespace std::chrono_literals;

namespace
{

double applyDeadband(const double value, const double deadband) {
    return std::abs(value) < std::abs(deadband) ? 0.0 : value;
}

double signOf(const double value) {
    return value < 0.0 ? -1.0 : 1.0;
}

bool worldToMap(const nav_msgs::msg::OccupancyGrid& costmap, const double wx,
                const double wy, int& mx, int& my) {
    const double resolution = costmap.info.resolution;
    if (resolution <= 0.0) {
        return false;
    }
    const double origin_x = costmap.info.origin.position.x;
    const double origin_y = costmap.info.origin.position.y;
    mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
    my = static_cast<int>(std::floor((wy - origin_y) / resolution));
    return mx >= 0 && my >= 0 && mx < static_cast<int>(costmap.info.width) &&
           my < static_cast<int>(costmap.info.height);
}

bool isFreeCostmapCell(const int value, const R2CmdVelBridgeConfig& config) {
    if (value < 0) {
        return !config.reverse_unknown_is_blocked;
    }
    return value == 0;
}

bool footprintIsFree(const nav_msgs::msg::OccupancyGrid& costmap,
                     const ReverseSafetyPose& pose,
                     const double rear_extension_m,
                     const R2CmdVelBridgeConfig& config) {
    const double resolution = costmap.info.resolution;
    if (resolution <= 0.0 || costmap.data.empty()) {
        return false;
    }

    int center_x = 0;
    int center_y = 0;
    if (!worldToMap(costmap, pose.x, pose.y, center_x, center_y)) {
        return false;
    }

    const double front = config.footprint_front_m + config.footprint_padding_m;
    const double rear =
        config.footprint_rear_m + config.footprint_padding_m + rear_extension_m;
    const double half_width =
        config.footprint_half_width_m + config.footprint_padding_m;
    const int radius_cells = std::max(
        1, static_cast<int>(std::ceil(
               std::hypot(std::max(front, rear), half_width) / resolution)));
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const int mx = center_x + dx;
            const int my = center_y + dy;
            const double cell_x = costmap.info.origin.position.x +
                                  (static_cast<double>(mx) + 0.5) * resolution;
            const double cell_y = costmap.info.origin.position.y +
                                  (static_cast<double>(my) + 0.5) * resolution;
            const double rel_x = cell_x - pose.x;
            const double rel_y = cell_y - pose.y;
            const double body_x = cos_yaw * rel_x + sin_yaw * rel_y;
            const double body_y = -sin_yaw * rel_x + cos_yaw * rel_y;
            if (body_x < -rear || body_x > front) {
                continue;
            }
            if (std::abs(body_y) > half_width) {
                continue;
            }
            if (mx < 0 || my < 0 ||
                mx >= static_cast<int>(costmap.info.width) ||
                my >= static_cast<int>(costmap.info.height)) {
                return false;
            }

            const auto index =
                static_cast<std::size_t>(my) * costmap.info.width + mx;
            if (index >= costmap.data.size() ||
                !isFreeCostmapCell(costmap.data[index], config)) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

bool isReverseSafeInCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                            const ReverseSafetyPose& pose,
                            const double reverse_speed,
                            const R2CmdVelBridgeConfig& config) {
    if (reverse_speed >= 0.0) {
        return true;
    }
    const double check_distance =
        std::max(config.reverse_check_distance_m,
                 std::abs(reverse_speed) * config.assumed_command_period +
                     config.reverse_safety_margin_m);
    // Backing up keeps the yaw constant, so the area swept by the footprint
    // is exactly the footprint rectangle with the rear extended by the
    // check distance.
    return footprintIsFree(costmap, pose, check_distance, config);
}

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
    if (config_.reverse_safety_enabled) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        rclcpp::QoS costmap_qos{rclcpp::KeepLast(1)};
        costmap_qos.transient_local().reliable();
        costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            config_.reverse_costmap_topic, costmap_qos,
            [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
                latest_costmap_ = msg;
            });
    }

    RCLCPP_INFO(get_logger(),
                "Yahboom R2 cmd_vel bridge: %s -> %s, min_linear=%.3f, "
                "max_linear=%.3f, steering_gain=%.3f, max_steering=%.3f, "
                "max_linear_accel=%.3f, max_steering_rate=%.3f, "
                "reverse_safety=%s, reverse_costmap=%s",
                config_.input_topic.c_str(), config_.output_topic.c_str(),
                config_.min_linear_speed, config_.max_linear_speed,
                config_.steering_gain, config_.max_steering,
                config_.max_linear_accel, config_.max_steering_rate,
                config_.reverse_safety_enabled ? "true" : "false",
                config_.reverse_costmap_topic.c_str());
}

R2CmdVelBridgeConfig R2CmdVelBridgeNode::loadConfig() {
    return loadDeclaredParameters<R2CmdVelBridgeConfig>(*this);
}

void R2CmdVelBridgeNode::onCmdVel(
    const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
    if (!msg || stopping_) {
        return;
    }

    publisher_->publish(limitRate(convertNavTwistToR2(*msg, config_)));
}

geometry_msgs::msg::Twist R2CmdVelBridgeNode::applyReverseSafety(
    const geometry_msgs::msg::Twist& target) {
    if (!config_.reverse_safety_enabled || target.linear.x >= 0.0) {
        return target;
    }

    geometry_msgs::msg::Twist output = target;
    const auto costmap = latest_costmap_;
    if (!costmap || !isCostmapFresh(*costmap)) {
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "reverse_blocked_unsafe_space: local costmap unavailable or stale");
        return output;
    }

    const auto pose = lookupReverseSafetyPose(*costmap);
    if (!pose ||
        !isReverseSafeInCostmap(*costmap, *pose, target.linear.x, config_)) {
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "reverse_blocked_unsafe_space: rear footprint is not fully free");
    }
    return output;
}

std::optional<ReverseSafetyPose> R2CmdVelBridgeNode::lookupReverseSafetyPose(
    const nav_msgs::msg::OccupancyGrid& costmap) {
    if (!tf_buffer_) {
        return std::nullopt;
    }
    try {
        const auto transform = tf_buffer_->lookupTransform(
            costmap.header.frame_id, config_.reverse_robot_frame,
            tf2::TimePointZero);
        return ReverseSafetyPose{
            transform.transform.translation.x,
            transform.transform.translation.y,
            yawFromQuaternion(transform.transform.rotation)};
    }
    catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "reverse_blocked_unsafe_space: TF %s -> %s unavailable: %s",
            costmap.header.frame_id.c_str(),
            config_.reverse_robot_frame.c_str(), ex.what());
        return std::nullopt;
    }
}

bool R2CmdVelBridgeNode::isCostmapFresh(
    const nav_msgs::msg::OccupancyGrid& costmap) {
    if (config_.reverse_costmap_max_age_sec <= 0.0) {
        return true;
    }
    const rclcpp::Time stamp{costmap.header.stamp};
    if (stamp.nanoseconds() == 0) {
        return false;
    }
    return (get_clock()->now() - stamp).seconds() <=
           config_.reverse_costmap_max_age_sec;
}

geometry_msgs::msg::Twist R2CmdVelBridgeNode::limitRate(
    const geometry_msgs::msg::Twist& target) {
    const rclcpp::Time now = get_clock()->now();

    double dt = config_.assumed_command_period;
    if (has_last_output_) {
        dt = std::max(0.0, (now - last_output_time_).seconds());
    }
    else {
        last_output_ = geometry_msgs::msg::Twist{};
        has_last_output_ = true;
    }

    geometry_msgs::msg::Twist output = target;
    const double max_linear_delta = std::abs(config_.max_linear_accel) * dt;
    const double max_steering_delta = std::abs(config_.max_steering_rate) * dt;

    output.linear.x =
        std::clamp(target.linear.x, last_output_.linear.x - max_linear_delta,
                   last_output_.linear.x + max_linear_delta);
    output.linear.y =
        std::clamp(target.linear.y, last_output_.linear.y - max_steering_delta,
                   last_output_.linear.y + max_steering_delta);

    output = applyReverseSafety(output);
    last_output_ = output;
    last_output_time_ = now;
    return output;
}

void R2CmdVelBridgeNode::publishStopBurst() {
    if (!publisher_) {
        return;
    }

    stopping_ = true;
    RCLCPP_WARN(get_logger(), "Publishing centered zero cmd_vel before exit");
    const geometry_msgs::msg::Twist stop;
    for (int i = 0; i < 50; ++i) {
        publisher_->publish(stop);
        rclcpp::sleep_for(20ms);
    }
    last_output_ = stop;
    has_last_output_ = true;
    last_output_time_ = get_clock()->now();
}

}  // namespace yahboom_alc_bringup
