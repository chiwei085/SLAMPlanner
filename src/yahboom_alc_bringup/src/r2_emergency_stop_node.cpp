#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdlib>
#include <thread>

#include "yahboom_alc_bringup/parameter_loader.hpp"

namespace yahboom_alc_bringup
{

struct R2EmergencyStopParams
{
    std::string cmd_vel_topic = "/cmd_vel";
    double publish_rate_hz = 30.0;
    int publish_count = 150;
    double driver_wait_sec = 4.0;
};

}  // namespace yahboom_alc_bringup

namespace rik_sd
{

template <>
struct descriptor<yahboom_alc_bringup::R2EmergencyStopParams>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::R2EmergencyStopParams;
        return make_descriptor(
            named(&R2EmergencyStopParams::cmd_vel_topic, "cmd_vel_topic"),
            named(&R2EmergencyStopParams::publish_rate_hz, "publish_rate_hz"),
            named(&R2EmergencyStopParams::publish_count, "publish_count"),
            named(&R2EmergencyStopParams::driver_wait_sec, "driver_wait_sec"));
    }
};

}  // namespace rik_sd

class R2EmergencyStopNode : public rclcpp::Node
{
public:
    R2EmergencyStopNode() : rclcpp::Node("r2_emergency_stop") {
        const auto params = yahboom_alc_bringup::loadDeclaredParameters<
            yahboom_alc_bringup::R2EmergencyStopParams>(*this);
        cmd_vel_topic_ = params.cmd_vel_topic;
        publish_rate_hz_ = params.publish_rate_hz;
        publish_count_ = params.publish_count;
        driver_wait_sec_ = params.driver_wait_sec;
        publisher_ =
            create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    }

    void run() {
        if (!hasCmdVelSubscriber()) {
            RCLCPP_WARN(
                get_logger(),
                "No /cmd_vel subscriber found, starting Ackman_driver_R2 "
                "for stop");
            driver_process_started_ = true;
            const int start_status = std::system(
                "ros2 run yahboomcar_bringup Ackman_driver_R2 >/tmp/"
                "r2_emergency_driver.log 2>&1 &");
            if (start_status != 0) {
                RCLCPP_WARN(get_logger(),
                            "Ackman_driver_R2 start command returned %d",
                            start_status);
            }
            const auto deadline =
                now() + rclcpp::Duration::from_seconds(driver_wait_sec_);
            while (rclcpp::ok() && now() < deadline && !hasCmdVelSubscriber()) {
                rclcpp::spin_some(get_node_base_interface());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        RCLCPP_WARN(get_logger(), "Publishing emergency zero cmd_vel");
        const auto period =
            std::chrono::duration<double>(1.0 / publish_rate_hz_);
        const geometry_msgs::msg::Twist stop;
        for (int i = 0; rclcpp::ok() && i < publish_count_; ++i) {
            publisher_->publish(stop);
            rclcpp::spin_some(get_node_base_interface());
            std::this_thread::sleep_for(period);
        }
        publisher_->publish(stop);
    }

private:
    bool hasCmdVelSubscriber() {
        rclcpp::spin_some(get_node_base_interface());
        return !get_subscriptions_info_by_topic(cmd_vel_topic_).empty();
    }

    std::string cmd_vel_topic_;
    double publish_rate_hz_ = 30.0;
    int publish_count_ = 150;
    double driver_wait_sec_ = 4.0;
    bool driver_process_started_ = false;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<R2EmergencyStopNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
