#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <csignal>
#include <exception>
#include <iostream>
#include <memory>

#include "yahboom_alc_bringup/parameter_loader.hpp"

namespace yahboom_alc_bringup
{

struct R2SafetyStopParams
{
    std::string cmd_vel_topic = "/cmd_vel";
    double publish_rate_hz = 30.0;
    double stop_duration_sec = 4.0;
};

}  // namespace yahboom_alc_bringup

namespace rik_sd
{

template <>
struct descriptor<yahboom_alc_bringup::R2SafetyStopParams>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::R2SafetyStopParams;
        return make_descriptor(
            named(&R2SafetyStopParams::cmd_vel_topic, "cmd_vel_topic"),
            named(&R2SafetyStopParams::publish_rate_hz, "publish_rate_hz"),
            named(&R2SafetyStopParams::stop_duration_sec, "stop_duration_sec"));
    }
};

}  // namespace rik_sd

namespace
{

std::weak_ptr<class R2SafetyStopNode> g_node;

class R2SafetyStopNode : public rclcpp::Node
{
public:
    R2SafetyStopNode() : rclcpp::Node("r2_safety_stop") {
        const auto params = yahboom_alc_bringup::loadDeclaredParameters<
            yahboom_alc_bringup::R2SafetyStopParams>(*this);
        publish_rate_hz_ = params.publish_rate_hz;
        stop_publish_target_ = std::max(
            1, static_cast<int>(publish_rate_hz_ * params.stop_duration_sec));
        publisher_ = create_publisher<geometry_msgs::msg::Twist>(
            params.cmd_vel_topic, 10);
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_hz_),
            [this]() { onTimer(); });
        RCLCPP_INFO(get_logger(), "R2 safety stop armed on %s",
                    params.cmd_vel_topic.c_str());
    }

    void requestStop() {
        if (!stop_requested_.exchange(true)) {
            RCLCPP_WARN(get_logger(),
                        "Shutdown detected, publishing zero cmd_vel");
        }
    }

    void publishStopBurst() {
        const geometry_msgs::msg::Twist stop;
        for (int i = 0; i < stop_publish_target_; ++i) {
            publisher_->publish(stop);
            if (rclcpp::ok()) {
                rclcpp::spin_some(get_node_base_interface());
            }
        }
        publisher_->publish(stop);
    }

private:
    void onTimer() {
        if (!stop_requested_) {
            return;
        }
        publisher_->publish(geometry_msgs::msg::Twist{});
        ++stop_publish_count_;
        if (stop_publish_count_ >= stop_publish_target_) {
            rclcpp::shutdown();
        }
    }

    double publish_rate_hz_ = 30.0;
    int stop_publish_target_ = 1;
    int stop_publish_count_ = 0;
    std::atomic<bool> stop_requested_{false};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

void handleSignal(int) {
    if (const auto node = g_node.lock()) {
        node->requestStop();
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<R2SafetyStopNode>();
        g_node = node;
        std::signal(SIGINT, handleSignal);
        std::signal(SIGTERM, handleSignal);
        rclcpp::spin(node);
        node->requestStop();
        node->publishStopBurst();
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& exc) {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        std::cerr << "r2_safety_stop failed: " << exc.what() << '\n';
        return 1;
    }
}
