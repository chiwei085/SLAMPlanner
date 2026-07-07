#include <nav2_planner/planner_server.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);

        constexpr std::size_t kExecutorThreads = 4;
        auto planner_server = std::make_shared<nav2_planner::PlannerServer>();
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions{}, kExecutorThreads);
        executor.add_node(planner_server->get_node_base_interface());

        RCLCPP_INFO(planner_server->get_logger(),
                    "r2_planner_server spinning planner_server with %zu "
                    "executor threads",
                    executor.get_number_of_threads());
        executor.spin();

        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& exc) {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        std::cerr << "r2_planner_server failed: " << exc.what() << '\n';
        return 1;
    }
}
