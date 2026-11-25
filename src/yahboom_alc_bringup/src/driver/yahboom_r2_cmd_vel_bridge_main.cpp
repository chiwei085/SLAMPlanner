#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>

#include "yahboom_alc_bringup/r2_cmd_vel_bridge.hpp"

namespace
{

std::atomic<bool> g_stop_requested{false};

void handleSignal(int) {
    g_stop_requested = true;
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    auto node = std::make_shared<yahboom_alc_bringup::R2CmdVelBridgeNode>();
    while (rclcpp::ok() && !g_stop_requested) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    node->publishStopBurst();
    rclcpp::shutdown();
    return 0;
}
