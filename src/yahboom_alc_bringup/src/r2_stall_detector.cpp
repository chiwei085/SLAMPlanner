#include "yahboom_alc_bringup/r2_stall_detector.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>

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
          core_(config_) {
        rclcpp::QoS cmd_qos{rclcpp::KeepLast(10)};
        cmd_qos.best_effort();
        driver_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            config_.driver_cmd_topic, cmd_qos,
            [this](const geometry_msgs::msg::Twist& msg) {
                latest_driver_cmd_ = msg;
                latest_driver_cmd_time_ = get_clock()->now();
            });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            config_.odom_topic, rclcpp::SystemDefaultsQoS(),
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
        stop_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            config_.driver_cmd_topic, 10);
        event_pub_ = create_publisher<std_msgs::msg::String>(
            config_.planner_event_topic, 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                   [this]() { onTimer(); });

        RCLCPP_INFO(get_logger(),
                    "R2 stall detector armed: driver_cmd=%s odom=%s imu=%s",
                    config_.driver_cmd_topic.c_str(),
                    config_.odom_topic.c_str(), config_.imu_topic.c_str());
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
                    "stall_or_slip: reason=%s expected=%.3f odom_delta=%.3f "
                    "duration=%.3f",
                    decision.reason.c_str(), decision.expected_motion_m,
                    decision.odom_delta_m, decision.forward_duration_sec);
    }

    std::optional<StallDetectorObservation> makeObservation() {
        const auto now = get_clock()->now();
        if (!latest_driver_cmd_ || !latest_odom_ || !latest_imu_ ||
            isStale(*latest_driver_cmd_time_, now) ||
            isStale(*latest_odom_time_, now) ||
            isStale(*latest_imu_time_, now)) {
            core_.reset();
            return std::nullopt;
        }
        StallDetectorObservation observation;
        observation.time_sec = now.seconds();
        observation.driver_cmd_vx = latest_driver_cmd_->linear.x;
        observation.odom_x = latest_odom_->pose.pose.position.x;
        observation.odom_y = latest_odom_->pose.pose.position.y;
        observation.imu_yaw_rate = latest_imu_->angular_velocity.z;
        return observation;
    }

    bool isStale(const rclcpp::Time& stamp, const rclcpp::Time& now) const {
        return (now - stamp).seconds() > config_.stale_observation_timeout_sec;
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
                {"expected_motion_m", rounded(decision.expected_motion_m)},
                {"odom_delta_m", rounded(decision.odom_delta_m)},
                {"imu_motion_consistent", decision.imu_motion_consistent}}
                .dump();
        event_pub_->publish(msg);
    }

    R2StallDetectorConfig config_;
    StallDetectorCore core_;
    std::optional<geometry_msgs::msg::Twist> latest_driver_cmd_;
    std::optional<nav_msgs::msg::Odometry> latest_odom_;
    std::optional<sensor_msgs::msg::Imu> latest_imu_;
    std::optional<rclcpp::Time> latest_driver_cmd_time_;
    std::optional<rclcpp::Time> latest_odom_time_;
    std::optional<rclcpp::Time> latest_imu_time_;
    std::optional<rclcpp::Time> stop_until_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr driver_cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
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
