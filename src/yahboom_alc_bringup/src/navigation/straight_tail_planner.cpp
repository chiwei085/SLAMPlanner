#include <nav2_core/exceptions.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2/utils.h>

#include <cmath>
#include <memory>
#include <string>

#include "yahboom_alc_bringup/straight_tail.hpp"

namespace yahboom_alc_bringup
{

// Global planner for frontier view goals: the inner planner (Smac Hybrid)
// reaches the tail start A = goal - tail_length * goal_dir with the goal yaw,
// then the path runs straight to the goal (see straight_tail.hpp). Once pure
// pursuit is captured by the tail line, every replan is that straight line
// from the robot's foot point, so the 1 Hz BT replan cannot bend the tail.
class StraightTailPlanner : public nav2_core::GlobalPlanner
{
public:
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override {
        auto node = parent.lock();
        costmap_ros_ = costmap_ros;
        nav2_util::declare_parameter_if_not_declared(
            node, name + ".inner_plugin",
            rclcpp::ParameterValue("nav2_smac_planner/SmacPlannerHybrid"));
        nav2_util::declare_parameter_if_not_declared(
            node, name + ".inner_name", rclcpp::ParameterValue("GridBased"));
        nav2_util::declare_parameter_if_not_declared(
            node, name + ".tail_length", rclcpp::ParameterValue(1.40));
        nav2_util::declare_parameter_if_not_declared(
            node, name + ".lookahead_dist", rclcpp::ParameterValue(0.35));
        nav2_util::declare_parameter_if_not_declared(
            node, name + ".max_curvature", rclcpp::ParameterValue(2.0));
        const auto inner_plugin =
            node->get_parameter(name + ".inner_plugin").as_string();
        const auto inner_name =
            node->get_parameter(name + ".inner_name").as_string();
        tail_length_ = node->get_parameter(name + ".tail_length").as_double();
        lookahead_ = node->get_parameter(name + ".lookahead_dist").as_double();
        max_curvature_ =
            node->get_parameter(name + ".max_curvature").as_double();
        // The inner planner reads its parameters under inner_name, so it
        // shares the GridBased configuration instead of duplicating it.
        inner_ = loader_.createSharedInstance(inner_plugin);
        inner_->configure(parent, inner_name, tf, costmap_ros);
        RCLCPP_INFO(node->get_logger(),
                    "%s: tail %.2f m, lookahead %.2f m, max curvature %.2f, "
                    "inner %s (%s)",
                    name.c_str(), tail_length_, lookahead_, max_curvature_,
                    inner_plugin.c_str(), inner_name.c_str());
    }

    void cleanup() override { inner_->cleanup(); }
    void activate() override { inner_->activate(); }
    void deactivate() override { inner_->deactivate(); }

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override {
        const TailPose goal_pose{goal.pose.position.x, goal.pose.position.y,
                                 tf2::getYaw(goal.pose.orientation)};
        const TailPose robot{start.pose.position.x, start.pose.position.y,
                             tf2::getYaw(start.pose.orientation)};
        const double step = costmap_ros_->getCostmap()->getResolution();

        if (const auto along = tailCaptureAlong(robot, goal_pose, lookahead_,
                                                max_curvature_)) {
            auto tail = tailPoses(goal_pose, *along, 0.0, step);
            if (collisionFree(tail)) {
                nav_msgs::msg::Path path;
                path.header = goal.header;
                append(path, tail, 0);
                return path;
            }
        }

        const auto tail = tailPoses(goal_pose, -tail_length_, 0.0, step);
        if (!collisionFree(tail)) {
            throw nav2_core::PlannerException(
                "straight view tail to the goal is blocked");
        }
        auto tail_start = goal;
        tail_start.pose.position.x = tail.front().x;
        tail_start.pose.position.y = tail.front().y;
        auto path = inner_->createPlan(start, tail_start);
        if (path.poses.empty()) {
            throw nav2_core::PlannerException("no path to the view tail start");
        }
        append(path, tail, 1);
        return path;
    }

private:
    bool collisionFree(const std::vector<TailPose>& poses) {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
            *costmap_ros_->getCostmap()->getMutex());
        nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>
            checker(costmap_ros_->getCostmap());
        const auto footprint = costmap_ros_->getRobotFootprint();
        for (const auto& pose : poses) {
            if (checker.footprintCostAtPose(pose.x, pose.y, pose.yaw,
                                            footprint) >=
                nav2_costmap_2d::LETHAL_OBSTACLE) {
                return false;
            }
        }
        return true;
    }

    static void append(nav_msgs::msg::Path& path,
                       const std::vector<TailPose>& poses,
                       const std::size_t first) {
        geometry_msgs::msg::PoseStamped stamped;
        stamped.header = path.header;
        for (std::size_t i = first; i < poses.size(); ++i) {
            stamped.pose.position.x = poses[i].x;
            stamped.pose.position.y = poses[i].y;
            stamped.pose.orientation.z = std::sin(poses[i].yaw / 2.0);
            stamped.pose.orientation.w = std::cos(poses[i].yaw / 2.0);
            path.poses.push_back(stamped);
        }
    }

    pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_{
        "nav2_core", "nav2_core::GlobalPlanner"};
    nav2_core::GlobalPlanner::Ptr inner_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    double tail_length_ = 1.40;
    double lookahead_ = 0.35;
    double max_curvature_ = 2.0;
};

}  // namespace yahboom_alc_bringup

PLUGINLIB_EXPORT_CLASS(yahboom_alc_bringup::StraightTailPlanner,
                       nav2_core::GlobalPlanner)
