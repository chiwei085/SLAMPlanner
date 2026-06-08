#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <string>
#include <thread>

#include "yahboom_alc_bringup/parameter_loader.hpp"

namespace yahboom_alc_bringup
{

struct R2SafeDriverParams
{
    std::string cmd_vel_topic = "/cmd_vel";
    int stop_publish_count = 30;
    double stop_publish_period_sec = 0.02;
    std::string driver_package = "yahboomcar_bringup";
    std::string driver_executable = "Ackman_driver_R2";
};

}  // namespace yahboom_alc_bringup

namespace rik_sd
{

template <>
struct descriptor<yahboom_alc_bringup::R2SafeDriverParams>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::R2SafeDriverParams;
        return make_descriptor(
            named(&R2SafeDriverParams::cmd_vel_topic, "cmd_vel_topic"),
            named(&R2SafeDriverParams::stop_publish_count,
                  "stop_publish_count"),
            named(&R2SafeDriverParams::stop_publish_period_sec,
                  "stop_publish_period_sec"),
            named(&R2SafeDriverParams::driver_package, "driver_package"),
            named(&R2SafeDriverParams::driver_executable, "driver_executable"));
    }
};

}  // namespace rik_sd

namespace
{

std::atomic<bool> g_stop_requested{false};

void handleSignal(int) {
    g_stop_requested = true;
}

class R2SafeDriverSupervisor : public rclcpp::Node
{
public:
    R2SafeDriverSupervisor() : rclcpp::Node("r2_safe_driver") {
        const auto params = yahboom_alc_bringup::loadDeclaredParameters<
            yahboom_alc_bringup::R2SafeDriverParams>(*this);
        cmd_vel_topic_ = params.cmd_vel_topic;
        stop_publish_count_ = params.stop_publish_count;
        stop_publish_period_sec_ = params.stop_publish_period_sec;
        driver_package_ = params.driver_package;
        driver_executable_ = params.driver_executable;
        publisher_ =
            create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    }

    int run() {
        child_pid_ = fork();
        if (child_pid_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to fork Yahboom driver process");
            return 1;
        }
        if (child_pid_ == 0) {
            execlp("ros2", "ros2", "run", driver_package_.c_str(),
                   driver_executable_.c_str(), static_cast<char*>(nullptr));
            _exit(127);
        }

        RCLCPP_INFO(get_logger(), "Started Yahboom driver supervisor pid=%d",
                    child_pid_);
        while (rclcpp::ok() && !g_stop_requested) {
            int status = 0;
            const pid_t result = waitpid(child_pid_, &status, WNOHANG);
            if (result == child_pid_) {
                RCLCPP_WARN(get_logger(),
                            "Yahboom driver exited; supervisor stopping");
                publishStopBurst();
                return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
            }
            rclcpp::spin_some(get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        RCLCPP_WARN(get_logger(),
                    "Stopping R2 driver with hardware zero command");
        publishStopBurst();
        stopChild();
        return 0;
    }

    void publishStopBurst() {
        const geometry_msgs::msg::Twist stop;
        for (int i = 0; i < stop_publish_count_; ++i) {
            publisher_->publish(stop);
            rclcpp::spin_some(get_node_base_interface());
            std::this_thread::sleep_for(
                std::chrono::duration<double>(stop_publish_period_sec_));
        }
    }

private:
    void stopChild() {
        if (child_pid_ <= 0) {
            return;
        }
        publishStopBurst();
        kill(child_pid_, SIGTERM);
        for (int i = 0; i < 30; ++i) {
            int status = 0;
            if (waitpid(child_pid_, &status, WNOHANG) == child_pid_) {
                child_pid_ = -1;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        kill(child_pid_, SIGKILL);
        waitpid(child_pid_, nullptr, 0);
        child_pid_ = -1;
    }

    std::string cmd_vel_topic_;
    std::string driver_package_;
    std::string driver_executable_;
    int stop_publish_count_ = 30;
    double stop_publish_period_sec_ = 0.02;
    pid_t child_pid_ = -1;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<R2SafeDriverSupervisor>();
    const int status = node->run();
    rclcpp::shutdown();
    return status;
}
