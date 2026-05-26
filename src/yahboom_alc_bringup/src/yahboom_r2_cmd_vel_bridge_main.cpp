#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "yahboom_alc_bringup/r2_cmd_vel_bridge.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<yahboom_alc_bringup::R2CmdVelBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
