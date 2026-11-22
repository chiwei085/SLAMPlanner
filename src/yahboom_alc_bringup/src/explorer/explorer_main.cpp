#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <iostream>
#include <memory>

#include "explorer/cluster_frontier_explorer_node.hpp"

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<
                     yahboom_alc_bringup::ClusterFrontierExplorerNode>());
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& exc) {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        std::cerr << "cluster_frontier_explorer failed: " << exc.what() << '\n';
        return 1;
    }
}
