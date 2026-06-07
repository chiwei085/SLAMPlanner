#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <csignal>
#include <memory>

namespace
{

std::weak_ptr<class R2SafetyStopNode> g_node;

class R2SafetyStopNode : public rclcpp::Node
{
public:
    R2SafetyStopNode() : rclcpp::Node("r2_safety_stop") {
        declare_parameter("cmd_vel_topic", "/cmd_vel");
        declare_parameter("publish_rate_hz", 30.0);
        declare_parameter("stop_duration_sec", 4.0);

        const auto topic = get_parameter("cmd_vel_topic").as_string();
        publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
        stop_publish_target_ = std::max(
            1,
            static_cast<int>(publish_rate_hz_ *
                             get_parameter("stop_duration_sec").as_double()));
        publisher_ = create_publisher<geometry_msgs::msg::Twist>(topic, 10);
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_hz_),
            [this]() { onTimer(); });
        RCLCPP_INFO(get_logger(), "R2 safety stop armed on %s", topic.c_str());
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
            rclcpp::spin_some(get_node_base_interface());
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
