#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <optional>
#include <string>

#include "yahboom_alc_bringup/frontier_explorer_core.hpp"

namespace
{

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

}  // namespace

using yahboom_alc_bringup::FrontierConfig;
using yahboom_alc_bringup::FrontierExplorerCore;
using yahboom_alc_bringup::WorldPoint;

class SimpleFrontierExplorerNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit SimpleFrontierExplorerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
        : rclcpp::Node("simple_frontier_explorer", options),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_),
          nav_client_(rclcpp_action::create_client<NavigateToPose>(
              this, "/navigate_to_pose")) {
        declare_parameter("enabled", true);
        declare_parameter("map_topic", "/map");
        declare_parameter("robot_frame", "base_footprint");
        declare_parameter("global_frame", "map");
        declare_parameter("cycle_period_sec", 5.0);
        declare_parameter("min_goal_distance", 0.45);
        declare_parameter("max_goal_distance", 3.0);
        declare_parameter("frontier_stride", 3);
        declare_parameter("occupied_threshold", 50);

        enabled_ = get_parameter("enabled").as_bool();
        map_topic_ = get_parameter("map_topic").as_string();
        robot_frame_ = get_parameter("robot_frame").as_string();
        global_frame_ = get_parameter("global_frame").as_string();
        min_goal_distance_ = get_parameter("min_goal_distance").as_double();
        max_goal_distance_ = get_parameter("max_goal_distance").as_double();
        frontier_stride_ = std::max(
            1, static_cast<int>(get_parameter("frontier_stride").as_int()));

        FrontierConfig cfg;
        cfg.occupied_threshold = get_parameter("occupied_threshold").as_int();
        core_.setConfig(cfg);

        rclcpp::QoS map_qos{rclcpp::KeepLast(1)};
        map_qos.reliable().transient_local();
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, map_qos,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                core_.setMap(*msg);
                has_map_ = true;
            });
        timer_ = create_wall_timer(
            std::chrono::duration<double>(
                get_parameter("cycle_period_sec").as_double()),
            [this]() { onTimer(); });
    }

private:
    void onTimer() {
        if (!enabled_ || goal_active_) {
            return;
        }
        if (!has_map_) {
            logWaiting("Waiting for /map");
            return;
        }
        if (!nav_client_->action_server_is_ready()) {
            logWaiting("Waiting for /navigate_to_pose action server");
            return;
        }
        const auto robot_pose = lookupRobotPose();
        if (!robot_pose) {
            return;
        }
        const auto goal_xy = selectFrontier(*robot_pose);
        if (!goal_xy) {
            logWaiting(describeFrontiers(*robot_pose));
            return;
        }
        sendGoal(*robot_pose, *goal_xy);
    }

    std::optional<WorldPoint> lookupRobotPose() {
        try {
            const auto transform = tf_buffer_.lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero);
            return WorldPoint{transform.transform.translation.x,
                              transform.transform.translation.y};
        }
        catch (const tf2::TransformException& exc) {
            RCLCPP_DEBUG(get_logger(), "Waiting for robot pose TF: %s",
                         exc.what());
            return std::nullopt;
        }
    }

    std::optional<WorldPoint> selectFrontier(
        const WorldPoint& robot_pose) const {
        const auto& map = core_.map();
        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);
        if (width == 0 || height == 0) {
            return std::nullopt;
        }
        std::optional<WorldPoint> best;
        double best_score = std::numeric_limits<double>::infinity();
        for (int y = 1; y < height - 1; y += frontier_stride_) {
            for (int x = 1; x < width - 1; x += frontier_stride_) {
                const int i = y * width + x;
                if (!core_.isFreeValue(map.data[i])) {
                    continue;
                }
                if (!core_.hasUnknownNeighbor(width, i)) {
                    continue;
                }
                const auto xy = core_.mapToWorldUnchecked(x, y);
                const double dist = std::hypot(xy.first - robot_pose.first,
                                               xy.second - robot_pose.second);
                if (dist < min_goal_distance_ || dist > max_goal_distance_) {
                    continue;
                }
                if (last_goal_ &&
                    std::hypot(xy.first - last_goal_->first,
                               xy.second - last_goal_->second) < 0.35) {
                    continue;
                }
                if (dist < best_score) {
                    best_score = dist;
                    best = xy;
                }
            }
        }
        return best;
    }

    std::string describeFrontiers(const WorldPoint& robot_pose) const {
        const auto& map = core_.map();
        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);
        int free = 0;
        int unknown = 0;
        int occupied = 0;
        int frontier = 0;
        int in_range = 0;
        std::optional<double> nearest;
        for (int y = 1; y < height - 1; ++y) {
            for (int x = 1; x < width - 1; ++x) {
                const int i = y * width + x;
                const int v = static_cast<int>(map.data[i]);
                if (v == -1) {
                    ++unknown;
                }
                else if (core_.isFreeValue(v)) {
                    ++free;
                }
                else {
                    ++occupied;
                }
                if (!core_.isFreeValue(v) ||
                    !core_.hasUnknownNeighbor(width, i)) {
                    continue;
                }
                ++frontier;
                const auto xy = core_.mapToWorldUnchecked(x, y);
                const double dist = std::hypot(xy.first - robot_pose.first,
                                               xy.second - robot_pose.second);
                nearest = nearest ? std::min(*nearest, dist) : dist;
                if (min_goal_distance_ <= dist && dist <= max_goal_distance_) {
                    ++in_range;
                }
            }
        }
        const std::string nearest_text =
            nearest ? std::to_string(*nearest) + "m" : "none";
        return "No reachable frontier candidate found (free=" +
               std::to_string(free) + ", unknown=" + std::to_string(unknown) +
               ", occupied=" + std::to_string(occupied) +
               ", frontier=" + std::to_string(frontier) +
               ", in_range=" + std::to_string(in_range) +
               ", nearest=" + nearest_text + ")";
    }

    void sendGoal(const WorldPoint& robot_pose, const WorldPoint& goal_xy) {
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = global_frame_;
        goal.pose.header.stamp = get_clock()->now();
        goal.pose.pose.position.x = goal_xy.first;
        goal.pose.pose.position.y = goal_xy.second;
        const double yaw = std::atan2(goal_xy.second - robot_pose.second,
                                      goal_xy.first - robot_pose.first);
        goal.pose.pose.orientation = yawToQuaternion(yaw);
        goal_active_ = true;
        last_goal_ = goal_xy;
        wait_log_count_ = 0;
        RCLCPP_INFO(get_logger(), "Sending frontier goal x=%.2f, y=%.2f",
                    goal_xy.first, goal_xy.second);

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.goal_response_callback =
            [this](const GoalHandle::SharedPtr& handle) {
                if (!handle) {
                    RCLCPP_WARN(get_logger(), "Frontier goal rejected");
                    goal_active_ = false;
                }
            };
        options.result_callback = [this](
                                      const GoalHandle::WrappedResult& result) {
            RCLCPP_INFO(get_logger(), "Frontier goal finished with status %d",
                        static_cast<int>(result.code));
            goal_active_ = false;
        };
        nav_client_->async_send_goal(goal, options);
    }

    void logWaiting(const std::string& message) {
        ++wait_log_count_;
        if (wait_log_count_ == 1 || wait_log_count_ % 6 == 0) {
            RCLCPP_INFO(get_logger(), "%s", message.c_str());
        }
    }

    bool enabled_ = true;
    bool has_map_ = false;
    bool goal_active_ = false;
    std::string map_topic_;
    std::string robot_frame_;
    std::string global_frame_;
    double min_goal_distance_ = 0.45;
    double max_goal_distance_ = 3.0;
    int frontier_stride_ = 3;
    int wait_log_count_ = 0;
    std::optional<WorldPoint> last_goal_;
    FrontierExplorerCore core_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleFrontierExplorerNode>());
    rclcpp::shutdown();
    return 0;
}
