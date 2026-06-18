#include "yahboom_alc_bringup/r2_stall_detector.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <utility>

#include "yahboom_alc_bringup/math_util.hpp"
#include "yahboom_alc_bringup/parameter_loader.hpp"

namespace yahboom_alc_bringup
{

class R2StallDetectorNode : public rclcpp::Node
{
public:
    explicit R2StallDetectorNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
        : rclcpp::Node("r2_stall_detector", options),
          config_(loadDeclaredParameters<R2StallDetectorConfig>(*this)),
          core_(config_),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_) {
        rclcpp::QoS cmd_qos{rclcpp::KeepLast(10)};
        cmd_qos.best_effort();
        nav_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            config_.nav_cmd_topic, cmd_qos,
            [this](const geometry_msgs::msg::Twist& msg) {
                latest_cmd_ = msg;
                latest_cmd_time_ = get_clock()->now();
            });
        driver_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            config_.driver_cmd_topic, cmd_qos,
            [this](const geometry_msgs::msg::Twist& msg) {
                latest_driver_cmd_ = msg;
                latest_driver_cmd_time_ = get_clock()->now();
            });
        wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            config_.wheel_odom_topic, rclcpp::SystemDefaultsQoS(),
            [this](const nav_msgs::msg::Odometry& msg) {
                latest_odom_ = msg;
                latest_odom_time_ = get_clock()->now();
            });
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            config_.imu_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu& msg) {
                latest_imu_ = msg;
                latest_imu_time_ = get_clock()->now();
            });
        if (config_.depth_obstacle_enabled) {
            depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
                config_.depth_image_topic, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                    latest_depth_ = msg;
                    latest_depth_time_ = get_clock()->now();
                });
        }
        stop_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            config_.driver_cmd_topic, 10);
        event_pub_ = create_publisher<std_msgs::msg::String>(
            config_.planner_event_topic, 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                   [this]() { onTimer(); });

        RCLCPP_INFO(
            get_logger(),
            "R2 stall detector armed: nav_cmd=%s driver_cmd=%s odom=%s "
            "imu=%s depth=%s "
            "slam_tf=%s->%s",
            config_.nav_cmd_topic.c_str(), config_.driver_cmd_topic.c_str(),
            config_.wheel_odom_topic.c_str(), config_.imu_topic.c_str(),
            config_.depth_obstacle_enabled ? config_.depth_image_topic.c_str()
                                           : "disabled",
            config_.slam_global_frame.c_str(), config_.robot_frame.c_str());
    }

private:
    void onTimer() {
        publishPendingStop();
        evaluate();
    }

    void evaluate() {
        const auto observation = makeObservation();
        if (!observation) {
            return;
        }
        const auto decision = core_.update(*observation);
        if (!decision.stall_or_slip) {
            return;
        }
        publishStopBurst();
        publishStallEvent(decision);
        RCLCPP_WARN(get_logger(),
                    "stall_or_slip: reason=%s wheel_delta=%.3f "
                    "slam_delta=%.3f duration=%.3f depth_valid=%s "
                    "front_depth=%.3f",
                    decision.reason.c_str(), decision.wheel_delta_m,
                    decision.slam_delta_m, decision.forward_duration_sec,
                    decision.depth_observation_valid ? "true" : "false",
                    decision.forward_min_depth_m);
    }

    std::optional<StallDetectorObservation> makeObservation() {
        if (!latest_cmd_ || !latest_odom_ || !latest_imu_ ||
            !latest_cmd_time_ || !latest_odom_time_ || !latest_imu_time_) {
            return std::nullopt;
        }
        const auto& latest_cmd = latest_cmd_.value();
        const auto& latest_odom = latest_odom_.value();
        const auto& latest_imu = latest_imu_.value();
        const auto& latest_cmd_time = latest_cmd_time_.value();
        const auto& latest_odom_time = latest_odom_time_.value();
        const auto& latest_imu_time = latest_imu_time_.value();
        const auto now = get_clock()->now();
        if (isStale(latest_cmd_time, now) || isStale(latest_odom_time, now) ||
            isStale(latest_imu_time, now)) {
            core_.reset();
            return std::nullopt;
        }
        const auto slam_pose = lookupSlamPose();
        if (!slam_pose) {
            core_.reset();
            return std::nullopt;
        }

        StallDetectorObservation observation;
        observation.time_sec = now.seconds();
        observation.cmd_vx = latest_cmd.linear.x;
        observation.wheel_odom_x = latest_odom.pose.pose.position.x;
        observation.wheel_odom_y = latest_odom.pose.pose.position.y;
        observation.slam_x = slam_pose->first;
        observation.slam_y = slam_pose->second;
        observation.imu_yaw_rate = latest_imu.angular_velocity.z;
        applyDriverCommandObservation(observation, now);
        applyDepthObservation(observation, latest_cmd.linear.x, now);
        return observation;
    }

    bool isStale(const rclcpp::Time& stamp, const rclcpp::Time& now) const {
        return (now - stamp).seconds() > config_.stale_observation_timeout_sec;
    }

    std::optional<std::pair<double, double>> lookupSlamPose() {
        try {
            const auto transform = tf_buffer_.lookupTransform(
                config_.slam_global_frame, config_.robot_frame,
                tf2::TimePointZero);
            return std::pair<double, double>{transform.transform.translation.x,
                                             transform.transform.translation.y};
        }
        catch (const tf2::TransformException& exc) {
            RCLCPP_DEBUG(get_logger(), "Waiting for SLAM TF: %s", exc.what());
            return std::nullopt;
        }
    }

    void applyDriverCommandObservation(StallDetectorObservation& observation,
                                       const rclcpp::Time& now) {
        if (!latest_driver_cmd_ || !latest_driver_cmd_time_) {
            return;
        }
        if (isStale(latest_driver_cmd_time_.value(), now)) {
            return;
        }
        observation.driver_cmd_vx = latest_driver_cmd_->linear.x;
        observation.driver_command_valid = true;
    }

    void applyDepthObservation(StallDetectorObservation& observation,
                               const double nav_cmd_vx,
                               const rclcpp::Time& now) {
        if (!config_.depth_obstacle_enabled || !latest_depth_ ||
            !latest_depth_time_) {
            return;
        }
        if (nav_cmd_vx <= std::abs(config_.command_vx_threshold_mps)) {
            return;
        }
        if ((now - latest_depth_time_.value()).seconds() >
            config_.depth_stale_timeout_sec) {
            return;
        }
        const auto depth = estimateForwardDepthRoi(*latest_depth_, config_);
        observation.depth_observation_valid = depth.valid;
        observation.forward_min_depth_m = depth.min_depth_m;
        observation.forward_obstacle_detected = depth.obstacle_detected;
    }

    void publishStopBurst() {
        stop_until_ = get_clock()->now() + rclcpp::Duration::from_seconds(
                                               config_.zero_burst_duration_sec);
        publishPendingStop();
    }

    void publishPendingStop() {
        const auto stop_until = stop_until_;
        if (!stop_until) {
            return;
        }
        if (get_clock()->now() > *stop_until) {
            stop_until_.reset();
            return;
        }
        stop_pub_->publish(geometry_msgs::msg::Twist{});
    }

    void publishStallEvent(const StallDetectorDecision& decision) {
        std_msgs::msg::String msg;
        msg.data =
            nlohmann::json{
                {"event", "stall_or_slip"},
                {"reason", decision.reason},
                {"time_sec", rounded(get_clock()->now().seconds())},
                {"forward_duration_sec",
                 rounded(decision.forward_duration_sec)},
                {"wheel_odom_delta_m", rounded(decision.wheel_delta_m)},
                {"slam_delta_m", rounded(decision.slam_delta_m)},
                {"imu_motion_consistent", decision.imu_motion_consistent},
                {"driver_command_valid", decision.driver_command_valid},
                {"depth_observation_valid", decision.depth_observation_valid},
                {"forward_min_depth_m",
                 decision.depth_observation_valid
                     ? nlohmann::json(rounded(decision.forward_min_depth_m))
                     : nlohmann::json(nullptr)},
                {"forward_obstacle_detected",
                 decision.forward_obstacle_detected}}
                .dump();
        event_pub_->publish(msg);
    }

    R2StallDetectorConfig config_;
    StallDetectorCore core_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::optional<geometry_msgs::msg::Twist> latest_cmd_;
    std::optional<geometry_msgs::msg::Twist> latest_driver_cmd_;
    std::optional<nav_msgs::msg::Odometry> latest_odom_;
    std::optional<sensor_msgs::msg::Imu> latest_imu_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
    std::optional<rclcpp::Time> latest_cmd_time_;
    std::optional<rclcpp::Time> latest_driver_cmd_time_;
    std::optional<rclcpp::Time> latest_odom_time_;
    std::optional<rclcpp::Time> latest_imu_time_;
    std::optional<rclcpp::Time> latest_depth_time_;
    std::optional<rclcpp::Time> stop_until_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr driver_cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace yahboom_alc_bringup

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(
            std::make_shared<yahboom_alc_bringup::R2StallDetectorNode>());
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& exc) {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        std::cerr << "r2_stall_detector failed: " << exc.what() << '\n';
        return 1;
    }
}
