#include "yahboom_alc_bringup/r2_cmd_vel_bridge.hpp"

#include <nlohmann/json.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

#include "yahboom_alc_bringup/parameter_loader.hpp"
#include "yahboom_alc_bringup/planner_event.hpp"
#include "yahboom_alc_bringup/temporal_signal.hpp"
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

bool isFreeCostmapCell(const int value, const bool unknown_is_blocked,
                       const bool recently_free,
                       const R2CmdVelBridgeConfig& config) {
    if (value < 0) {
        return !unknown_is_blocked || recently_free;
    }
    return value <= config.reverse_allowed_cost_max;
}

enum class FootprintSafetyStatus : std::uint8_t
{
    kSafe,
    kOutOfBounds,
    kBlocked,
};

struct FootprintExtensions
{
    double front_m = 0.0;
    double rear_m = 0.0;
};

FootprintExtensions frontExtension(const double distance_m) {
    FootprintExtensions extensions;
    extensions.front_m = distance_m;
    return extensions;
}

FootprintExtensions rearExtension(const double distance_m) {
    FootprintExtensions extensions;
    extensions.rear_m = distance_m;
    return extensions;
}

const char* safetyReason(const FootprintSafetyStatus status,
                         const char* blocked_reason) {
    switch (status) {
        case FootprintSafetyStatus::kOutOfBounds:
            return "out_of_bounds";
        case FootprintSafetyStatus::kBlocked:
            return blocked_reason;
        case FootprintSafetyStatus::kSafe:
            return "safe";
    }
    return blocked_reason;
}

FootprintSafetyStatus footprintSafetyStatus(
    const nav_msgs::msg::OccupancyGrid& costmap, const ReverseSafetyPose& pose,
    const FootprintExtensions extensions, const bool unknown_is_blocked,
    const R2CmdVelBridgeConfig& config,
    const RecentFreeCostmapHistory* recent_free_history = nullptr,
    const double now_sec = 0.0) {
    const double resolution = costmap.info.resolution;
    if (resolution <= 0.0 || costmap.data.empty()) {
        return FootprintSafetyStatus::kBlocked;
    }

    int center_x = 0;
    int center_y = 0;
    if (!worldToMap(costmap, pose.x, pose.y, center_x, center_y)) {
        return FootprintSafetyStatus::kOutOfBounds;
    }

    const double front = config.footprint_front_m + config.footprint_padding_m +
                         extensions.front_m;
    const double rear = config.footprint_rear_m + config.footprint_padding_m +
                        extensions.rear_m;
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
                return FootprintSafetyStatus::kOutOfBounds;
            }

            const auto index =
                static_cast<std::size_t>(my) * costmap.info.width + mx;
            if (index >= costmap.data.size()) {
                return FootprintSafetyStatus::kBlocked;
            }
            bool recently_free = false;
            if (unknown_is_blocked && costmap.data[index] < 0 &&
                recent_free_history != nullptr) {
                recently_free = recent_free_history->wasRecentlyFree(
                    cell_x, cell_y, now_sec,
                    config.reverse_recent_free_grace_sec);
            }
            if (!isFreeCostmapCell(costmap.data[index], unknown_is_blocked,
                                   recently_free, config)) {
                return FootprintSafetyStatus::kBlocked;
            }
        }
    }
    return FootprintSafetyStatus::kSafe;
}

double clampAngularForCurvature(const double angular, const double linear_x,
                                const R2CmdVelBridgeConfig& config) {
    if (angular == 0.0 || linear_x == 0.0) {
        return 0.0;
    }
    double curvature_limit = std::abs(config.max_curvature);
    if (config.min_turning_radius_m > 0.0) {
        const double radius_curvature = 1.0 / config.min_turning_radius_m;
        curvature_limit = curvature_limit > 0.0
                              ? std::min(curvature_limit, radius_curvature)
                              : radius_curvature;
    }
    if (curvature_limit <= 0.0) {
        return angular;
    }
    const double angular_limit = std::abs(linear_x) * curvature_limit;
    return std::clamp(angular, -angular_limit, angular_limit);
}

}  // namespace

bool isReverseSafeInCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                            const ReverseSafetyPose& pose,
                            const double reverse_speed,
                            const R2CmdVelBridgeConfig& config,
                            const RecentFreeCostmapHistory* recent_free_history,
                            const double now_sec) {
    if (reverse_speed >= 0.0) {
        return true;
    }
    const double check_distance = std::max(
        0.0, config.reverse_check_distance_m + config.reverse_safety_margin_m);
    // Backing up keeps the yaw constant, so the checked swept volume is the
    // footprint rectangle with the rear extended by a short fail-closed backup
    // horizon.
    return footprintSafetyStatus(costmap, pose, rearExtension(check_distance),
                                 config.reverse_unknown_is_blocked, config,
                                 recent_free_history,
                                 now_sec) == FootprintSafetyStatus::kSafe;
}

bool isStationaryArcForwardSafeInCostmap(
    const nav_msgs::msg::OccupancyGrid& costmap, const ReverseSafetyPose& pose,
    const double forward_speed, const R2CmdVelBridgeConfig& config,
    const RecentFreeCostmapHistory* recent_free_history, const double now_sec) {
    if (forward_speed <= 0.0) {
        return true;
    }
    const double check_distance =
        std::max(0.0, config.stationary_arc_check_distance_m);
    return footprintSafetyStatus(costmap, pose, frontExtension(check_distance),
                                 config.stationary_arc_unknown_is_blocked,
                                 config, recent_free_history,
                                 now_sec) == FootprintSafetyStatus::kSafe;
}

geometry_msgs::msg::Twist convertNavTwistToR2(
    const geometry_msgs::msg::Twist& input, const R2CmdVelBridgeConfig& config,
    const bool allow_stationary_arc) {
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
        if (config.enforce_min_linear_speed &&
            std::abs(linear_x) < std::abs(config.min_linear_speed)) {
            linear_x = signOf(linear_x) * std::abs(config.min_linear_speed);
        }
    }
    else if (requested_angular != 0.0 && config.arc_turn_when_stationary &&
             allow_stationary_arc) {
        linear_x = std::abs(config.min_linear_speed);
    }

    const double curvature_limited_angular =
        clampAngularForCurvature(requested_angular, linear_x, config);

    double steering = 0.0;
    if (curvature_limited_angular != 0.0) {
        const double requested_steering =
            curvature_limited_angular * config.steering_gain;
        double steering_limit = std::abs(config.max_steering);
        if (config.max_steering_angle_rad > 0.0) {
            steering_limit = std::min(steering_limit,
                                      std::abs(config.max_steering_angle_rad));
        }
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

double applyDriveStallFloor(const double rate_limited_linear_x,
                            const double target_linear_x,
                            const R2CmdVelBridgeConfig& config) {
    if (!config.enforce_min_linear_speed) {
        return rate_limited_linear_x;
    }
    const double min_speed = std::abs(config.min_linear_speed);
    if (std::abs(rate_limited_linear_x) >= min_speed) {
        return rate_limited_linear_x;
    }
    const bool drive_intended =
        std::abs(target_linear_x) >= std::abs(config.linear_deadband);
    return drive_intended ? signOf(target_linear_x) * min_speed : 0.0;
}

bool shouldPublishIdleStop(const double since_last_input_sec,
                           const double timeout_sec, const double burst_sec) {
    return timeout_sec > 0.0 && since_last_input_sec > timeout_sec &&
           since_last_input_sec <= timeout_sec + std::max(0.0, burst_sec);
}

void BlindReverseWindow::onEvent(const std::string& event_name,
                                 const double now_sec, const double max_sec) {
    if (event_name == "observation_turn_started" && max_sec > 0.0) {
        open_until_sec_ = now_sec + max_sec;
    }
    else if (event_name == "observation_turn_finished") {
        open_until_sec_ = 0.0;
    }
}

R2CmdVelBridgeNode::R2CmdVelBridgeNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("yahboom_r2_cmd_vel_bridge", options),
      config_(loadConfig()) {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        config_.output_topic, rclcpp::SystemDefaultsQoS());
    event_pub_ = create_publisher<std_msgs::msg::String>(
        config_.planner_event_topic, 10);

    rclcpp::QoS input_qos{rclcpp::KeepLast(1)};
    input_qos.best_effort();
    subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        config_.input_topic, input_qos,
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
            onCmdVel(msg);
        });
    safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        config_.safety_stop_topic, rclcpp::QoS{rclcpp::KeepLast(1)},
        [this](const std_msgs::msg::Bool::ConstSharedPtr& message) {
            safety_stop_active_ = message->data;
        });
    idle_stop_timer_ = create_wall_timer(100ms, [this]() {
        if (!last_input_time_ || stopping_) {
            return;
        }
        const double since = (get_clock()->now() - *last_input_time_).seconds();
        if (!shouldPublishIdleStop(since, config_.input_timeout_sec,
                                   config_.idle_stop_burst_sec)) {
            return;
        }
        const geometry_msgs::msg::Twist stop;
        publisher_->publish(stop);
        last_output_ = stop;
        has_last_output_ = true;
        last_output_time_ = get_clock()->now();
    });
    planner_event_sub_ = create_subscription<std_msgs::msg::String>(
        config_.planner_event_topic, 10,
        [this](const std_msgs::msg::String::ConstSharedPtr& message) {
            const auto name = parseEventName(message->data);
            if (!name) {
                return;
            }
            const double now_sec = get_clock()->now().seconds();
            const bool was_allowed = blind_reverse_window_.allowed(now_sec);
            blind_reverse_window_.onEvent(
                *name, now_sec, config_.observation_blind_reverse_max_sec);
            if (was_allowed != blind_reverse_window_.allowed(now_sec)) {
                RCLCPP_WARN(get_logger(),
                            "observation turn: reverse into unknown cells %s",
                            was_allowed ? "blocked again" : "ALLOWED (blind)");
            }
        });
    const bool costmap_safety_enabled =
        config_.reverse_safety_enabled ||
        config_.stationary_arc_requires_forward_clearance;
    if (costmap_safety_enabled) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        rclcpp::QoS costmap_qos{rclcpp::KeepLast(1)};
        costmap_qos.transient_local().reliable();
        costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            config_.reverse_costmap_topic, costmap_qos,
            [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
                latest_costmap_ = msg;
                const double now_sec = get_clock()->now().seconds();
                costmap_temporal_ = observeTemporalSignal(
                    rclcpp::Time(msg->header.stamp).seconds(), now_sec,
                    costmap_temporal_state_);
                reverse_recent_free_history_.observe(
                    *msg, now_sec, config_.reverse_allowed_cost_max);
            });
    }

    RCLCPP_INFO(
        get_logger(),
        "Yahboom R2 cmd_vel bridge: %s -> %s, min_linear=%.3f, "
        "max_linear=%.3f, steering_gain=%.3f, max_steering=%.3f, "
        "max_linear_accel=%.3f, max_steering_rate=%.3f, "
        "reverse_safety=%s, stationary_arc_forward_clearance=%s, "
        "shared_costmap=%s",
        config_.input_topic.c_str(), config_.output_topic.c_str(),
        config_.min_linear_speed, config_.max_linear_speed,
        config_.steering_gain, config_.max_steering, config_.max_linear_accel,
        config_.max_steering_rate,
        config_.reverse_safety_enabled ? "true" : "false",
        config_.stationary_arc_requires_forward_clearance ? "true" : "false",
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
    last_input_time_ = get_clock()->now();

    // safety_stop_active_ is the IMU-impact reflex from the collision safety
    // node: halt forward motion immediately (the impact already happened) but
    // keep reverse available so recovery can back away from whatever was hit.
    geometry_msgs::msg::Twist command = *msg;
    if (safety_stop_active_ && command.linear.x > 0.0) {
        command.linear.x = 0.0;
    }

    const double requested_linear =
        applyDeadband(command.linear.x, config_.linear_deadband);
    const double requested_angular =
        applyDeadband(command.angular.z, config_.angular_deadband);
    const bool stationary_arc_command =
        requested_linear == 0.0 && requested_angular != 0.0 &&
        config_.arc_turn_when_stationary && !safety_stop_active_;

    publisher_->publish(
        limitRate(convertNavTwistToR2(command, config_, !safety_stop_active_),
                  stationary_arc_command));
}

geometry_msgs::msg::Twist R2CmdVelBridgeNode::applyReverseSafety(
    const geometry_msgs::msg::Twist& target) {
    if (!config_.reverse_safety_enabled || target.linear.x >= 0.0) {
        return target;
    }

    geometry_msgs::msg::Twist output = target;
    const auto costmap = latest_costmap_;
    if (!costmap || !isCostmapFresh()) {
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        publishPlannerEvent("reverse_blocked", "stale_costmap");
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "reverse_blocked_unsafe_space: local costmap unavailable or stale");
        return output;
    }

    const auto pose =
        lookupReverseSafetyPose(*costmap, "reverse_blocked_unsafe_space");
    if (!pose) {
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        publishPlannerEvent("reverse_blocked", "tf_unavailable");
        return output;
    }
    const double check_distance =
        std::max(0.0, config_.reverse_check_distance_m +
                          config_.reverse_safety_margin_m);
    const auto status = footprintSafetyStatus(
        *costmap, *pose, rearExtension(check_distance),
        reverseUnknownIsBlocked(), config_, &reverse_recent_free_history_,
        get_clock()->now().seconds());
    if (status != FootprintSafetyStatus::kSafe) {
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        publishPlannerEvent("reverse_blocked",
                            safetyReason(status, "rear_blocked"));
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "reverse_blocked_unsafe_space: rear footprint is not fully free");
    }
    return output;
}

geometry_msgs::msg::Twist R2CmdVelBridgeNode::applyStationaryArcSafety(
    const geometry_msgs::msg::Twist& target) {
    if (!config_.stationary_arc_requires_forward_clearance ||
        target.linear.x <= 0.0) {
        return target;
    }

    geometry_msgs::msg::Twist output = target;
    const auto costmap = latest_costmap_;
    if (!costmap || !isCostmapFresh()) {
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        publishPlannerEvent("stationary_arc_blocked", "stale_costmap");
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "stationary_arc_blocked: local costmap unavailable or stale");
        return output;
    }

    const auto pose =
        lookupReverseSafetyPose(*costmap, "stationary_arc_blocked");
    if (!pose) {
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        publishPlannerEvent("stationary_arc_blocked", "tf_unavailable");
        return output;
    }

    const double now_sec = get_clock()->now().seconds();
    const double check_distance =
        std::max(0.0, config_.stationary_arc_check_distance_m);
    const auto status =
        footprintSafetyStatus(*costmap, *pose, frontExtension(check_distance),
                              config_.stationary_arc_unknown_is_blocked,
                              config_, &reverse_recent_free_history_, now_sec);
    if (status != FootprintSafetyStatus::kSafe) {
        // Front is blocked; try a reverse arc as fallback — the path the robot
        // just came from is a certified-free corridor and typically
        // unobstructed.
        const double reverse_check =
            std::max(0.0, config_.reverse_check_distance_m +
                              config_.reverse_safety_margin_m);
        const auto reverse_status =
            footprintSafetyStatus(*costmap, *pose, rearExtension(reverse_check),
                                  reverseUnknownIsBlocked(), config_,
                                  &reverse_recent_free_history_, now_sec);
        if (reverse_status == FootprintSafetyStatus::kSafe) {
            output.linear.x = -output.linear.x;
            publishPlannerEvent("stationary_arc_reversed",
                                "front_blocked_reverse_ok");
            return output;
        }
        output.linear.x = 0.0;
        output.linear.y = 0.0;
        publishPlannerEvent("stationary_arc_blocked",
                            safetyReason(status, "front_blocked"));
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "stationary_arc_blocked: front footprint is not fully free");
    }
    return output;
}

std::optional<ReverseSafetyPose> R2CmdVelBridgeNode::lookupReverseSafetyPose(
    const nav_msgs::msg::OccupancyGrid& costmap, const char* warning_label) {
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
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "%s: TF %s -> %s unavailable: %s", warning_label,
                             costmap.header.frame_id.c_str(),
                             config_.reverse_robot_frame.c_str(), ex.what());
        return std::nullopt;
    }
}

bool R2CmdVelBridgeNode::isCostmapFresh() {
    if (config_.reverse_costmap_max_age_sec <= 0.0) {
        return true;
    }
    const auto silence =
        temporalSignalAt(costmap_temporal_state_, get_clock()->now().seconds())
            .receive_silence_sec;
    // No observation yet, or the subscription has gone silent: fail closed.
    return silence && *silence <= config_.reverse_costmap_max_age_sec;
}

bool R2CmdVelBridgeNode::reverseUnknownIsBlocked() {
    return config_.reverse_unknown_is_blocked &&
           !blind_reverse_window_.allowed(get_clock()->now().seconds());
}

void R2CmdVelBridgeNode::publishPlannerEvent(const std::string& event,
                                             const std::string& reason) {
    if (!event_pub_) {
        return;
    }
    std_msgs::msg::String msg;
    const auto now = get_clock()->now().seconds();
    const auto jsonValue = [](const std::optional<double>& value) {
        return value ? nlohmann::json(*value) : nlohmann::json(nullptr);
    };
    auto temporal = costmap_temporal_;
    temporal.receive_silence_sec =
        temporalSignalAt(costmap_temporal_state_, now).receive_silence_sec;
    msg.data =
        nlohmann::json{
            {"event", event},
            {"reason", reason},
            {"time_sec", now},
            {"local_costmap_temporal",
             {{"producer_lag_sec", jsonValue(temporal.producer_lag_sec)},
              {"consume_gap_sec", jsonValue(temporal.consume_gap_sec)},
              {"receive_silence_sec",
               jsonValue(temporal.receive_silence_sec)}}}}
            .dump();
    event_pub_->publish(msg);
}

geometry_msgs::msg::Twist R2CmdVelBridgeNode::limitRate(
    const geometry_msgs::msg::Twist& target,
    const bool apply_stationary_arc_safety) {
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

    // Keep the drive command out of the sub-stall dead band so a commanded
    // motion never decays into "steering moves, wheels don't". Steering
    // (linear.y) is still rate-limited normally above.
    output.linear.x =
        applyDriveStallFloor(output.linear.x, target.linear.x, config_);

    output = applyReverseSafety(output);
    if (apply_stationary_arc_safety) {
        output = applyStationaryArcSafety(output);
    }
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
