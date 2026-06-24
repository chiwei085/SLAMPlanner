#include "yahboom_alc_bringup/r2_collision_safety.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>

#include "yahboom_alc_bringup/parameter_loader.hpp"

namespace yahboom_alc_bringup
{
// IMU-impact reflex node. Smooth obstacle braking now lives entirely in the
// Nav2 collision monitor (slowdown + approach polygons on the same depth
// cloud); this node only watches the IMU for a forward-deceleration impact -
// the one hazard the depth/costmap layers cannot see, because it happens inside
// the camera's near blind zone (sub-RangeMin) or during pure rotation. On
// impact it (1) raises the safety-stop flag the bridge uses to halt forward
// motion immediately, and (2) emits a stall_or_slip event so the recovery state
// machine takes over. It no longer governs cmd_vel.
class R2CollisionSafetyGateNode : public rclcpp::Node
{
public:
    R2CollisionSafetyGateNode()
        : rclcpp::Node("r2_collision_safety_gate"),
          config_(loadDeclaredParameters<R2CollisionSafetyConfig>(*this)),
          core_(config_) {
        rclcpp::QoS command_qos{rclcpp::KeepLast(1)};
        command_qos.best_effort();
        command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            config_.input_topic, command_qos,
            [this](const geometry_msgs::msg::Twist::ConstSharedPtr& message) {
                latest_command_ = *message;
            });
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            config_.imu_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr& message) {
                latest_imu_accel_ = message->linear_acceleration.x;
                latest_imu_time_ = get_clock()->now();
            });
        stop_pub_ = create_publisher<std_msgs::msg::Bool>(
            config_.safety_stop_topic, 10);
        event_pub_ = create_publisher<std_msgs::msg::String>(
            config_.planner_event_topic, 10);
        // Fixed-cadence ticking keeps the IMU baseline window (one sample per
        // tick) consistent regardless of command/IMU message rates.
        timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                   [this]() { tick(); });
        RCLCPP_INFO(
            get_logger(),
            "R2 collision safety gate (IMU impact only): command=%s imu=%s",
            config_.input_topic.c_str(), config_.imu_topic.c_str());
    }

private:
    void tick() {
        const auto now = get_clock()->now();
        const bool imu_fresh = latest_imu_accel_ && latest_imu_time_ &&
                               (now - *latest_imu_time_).seconds() <=
                                   config_.imu_stale_timeout_sec;
        const geometry_msgs::msg::Twist command =
            latest_command_.value_or(geometry_msgs::msg::Twist{});
        const ImpactDecision impact = core_.detectImpact(
            now.seconds(), command, imu_fresh, latest_imu_accel_.value_or(0.0));

        std_msgs::msg::Bool stop;
        stop.data = impact.stop_hold_active;
        stop_pub_->publish(stop);

        if (impact.impact_triggered) {
            std_msgs::msg::String event;
            event.data = nlohmann::json{{"event", "stall_or_slip"},
                                        {"reason", "imu_impact"},
                                        {"time_sec", now.seconds()}}
                             .dump();
            event_pub_->publish(event);
            RCLCPP_WARN(get_logger(),
                        "IMU impact detected; stopping forward motion");
        }
    }

    R2CollisionSafetyConfig config_;
    CollisionSafetyCore core_;
    std::optional<geometry_msgs::msg::Twist> latest_command_;
    std::optional<double> latest_imu_accel_;
    std::optional<rclcpp::Time> latest_imu_time_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace yahboom_alc_bringup

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(
            std::make_shared<yahboom_alc_bringup::R2CollisionSafetyGateNode>());
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& exception) {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        std::cerr << "r2_collision_safety_gate failed: " << exception.what()
                  << '\n';
        return 1;
    }
}
