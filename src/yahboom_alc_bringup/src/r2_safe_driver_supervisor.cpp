#include <ament_index_cpp/get_package_prefix.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
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
    std::string driver_package = "yahboomcar_base_node";
    std::string driver_executable = "base_node_R2";
    bool pub_odom_tf = false;
    double linear_scale_x = 1.0;
    double linear_scale_y = 1.0;
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
            named(&R2SafeDriverParams::driver_executable, "driver_executable"),
            named(&R2SafeDriverParams::pub_odom_tf, "pub_odom_tf"),
            named(&R2SafeDriverParams::linear_scale_x, "linear_scale_x"),
            named(&R2SafeDriverParams::linear_scale_y, "linear_scale_y"));
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
        pub_odom_tf_arg_ = std::string("pub_odom_tf:=") +
                           (params.pub_odom_tf ? "true" : "false");
        linear_scale_x_arg_ =
            "linear_scale_x:=" + std::to_string(params.linear_scale_x);
        linear_scale_y_arg_ =
            "linear_scale_y:=" + std::to_string(params.linear_scale_y);
        publisher_ =
            create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    }

    int run() {
        // Resolve the real executable path (<prefix>/lib/<pkg>/<exe>, the
        // same convention `ros2 run` itself uses) and exec it directly,
        // instead of exec'ing into `ros2 run`. `ros2 run` does not replace
        // itself in-place -- it forks its OWN child to run the target -- so
        // routing through it puts the actual driver process one fork
        // generation below the PR_SET_PDEATHSIG we set below. PDEATHSIG does
        // not cascade to grandchildren: when the supervisor dies, the kernel
        // correctly kills the `ros2 run` wrapper, but the driver process it
        // spawned is simply orphaned and keeps running unsupervised. Exec'ing
        // the resolved path directly means this forked+prctl'd process IS the
        // driver, with no ungoverned grandchild for PDEATHSIG to miss.
        std::string driver_path;
        try {
            driver_path = ament_index_cpp::get_package_prefix(driver_package_) +
                          "/lib/" + driver_package_ + "/" + driver_executable_;
        }
        catch (const std::exception& exc) {
            RCLCPP_ERROR(get_logger(),
                         "Cannot resolve driver executable %s/%s: %s",
                         driver_package_.c_str(), driver_executable_.c_str(),
                         exc.what());
            return 1;
        }

        child_pid_ = fork();
        if (child_pid_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to fork Yahboom driver process");
            return 1;
        }
        if (child_pid_ == 0) {
            // The hardware driver must never outlive this supervisor. In
            // particular, ros2 launch can terminate the parent without
            // delivering a catchable signal while tearing down a process
            // tree. Without a parent-death signal, detached driver instances
            // keep publishing /vel_raw and corrupt the odometry integration.
            if (prctl(PR_SET_PDEATHSIG, SIGTERM) != 0 || getppid() == 1) {
                _exit(127);
            }
            setsid();  // detach from terminal so Ctrl+C SIGINT doesn't reach
                       // child
            execl(driver_path.c_str(), driver_path.c_str(), "--ros-args", "-p",
                  pub_odom_tf_arg_.c_str(), "-p", linear_scale_x_arg_.c_str(),
                  "-p", linear_scale_y_arg_.c_str(),
                  static_cast<char*>(nullptr));
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
            if (rclcpp::ok()) {
                rclcpp::spin_some(get_node_base_interface());
            }
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
    std::string pub_odom_tf_arg_;
    std::string linear_scale_x_arg_;
    std::string linear_scale_y_arg_;
    int stop_publish_count_ = 30;
    double stop_publish_period_sec_ = 0.02;
    pid_t child_pid_ = -1;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

}  // namespace

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        // Install after rclcpp::init() so our handler overwrites rclcpp's.
        // This keeps rclcpp::ok() true during the stop burst so spin_some()
        // works.
        std::signal(SIGINT, handleSignal);
        std::signal(SIGTERM, handleSignal);
        auto node = std::make_shared<R2SafeDriverSupervisor>();
        const int status = node->run();
        rclcpp::shutdown();
        return status;
    }
    catch (const std::exception& exc) {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        std::cerr << "r2_safe_driver failed: " << exc.what() << '\n';
        return 1;
    }
}
