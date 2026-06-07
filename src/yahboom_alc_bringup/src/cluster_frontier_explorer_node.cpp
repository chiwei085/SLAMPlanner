#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "yahboom_alc_bringup/frontier_explorer_core.hpp"

namespace yahboom_alc_bringup
{

namespace
{

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

double rounded(const double value, const int precision = 3) {
    const double scale = std::pow(10.0, precision);
    return std::round(value * scale) / scale;
}

nlohmann::json pointJson(const WorldPoint& point) {
    return nlohmann::json::array({rounded(point.first), rounded(point.second)});
}

}  // namespace

class ClusterFrontierExplorerNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit ClusterFrontierExplorerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
        : rclcpp::Node("cluster_frontier_explorer", options),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_),
          nav_client_(rclcpp_action::create_client<NavigateToPose>(
              this, "/navigate_to_pose")) {
        loadParams();

        rclcpp::QoS map_qos{rclcpp::KeepLast(1)};
        map_qos.reliable().transient_local();
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, map_qos,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                onMap(*msg);
            });

        rclcpp::QoS marker_qos{rclcpp::KeepLast(1)};
        marker_qos.reliable().transient_local();
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            marker_topic_, marker_qos);
        goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            goal_marker_topic_, marker_qos);
        event_pub_ =
            create_publisher<std_msgs::msg::String>(planner_event_topic_, 10);
        timer_ =
            create_wall_timer(std::chrono::duration<double>(cycle_period_sec_),
                              [this]() { onTimer(); });
    }

private:
    void loadParams() {
        const auto& defs = core_.config();
        auto d = [&](const char* n, double v) {
            declare_parameter(n, v);
            return get_parameter(n).as_double();
        };
        auto i = [&](const char* n, int v) {
            declare_parameter(n, v);
            return get_parameter(n).as_int();
        };
        auto b = [&](const char* n, bool v) {
            declare_parameter(n, v);
            return get_parameter(n).as_bool();
        };
        auto s = [&](const char* n, const char* v) {
            declare_parameter(n, std::string{v});
            return get_parameter(n).as_string();
        };

        enabled_ = b("enabled", true);
        map_topic_ = s("map_topic", "/map");
        robot_frame_ = s("robot_frame", "base_footprint");
        global_frame_ = s("global_frame", "map");
        cycle_period_sec_ = d("cycle_period_sec", 3.0);
        planner_event_topic_ = s("planner_event_topic", "/planner/events");
        marker_topic_ = s("marker_topic", "/frontier_markers");
        goal_marker_topic_ = s("goal_marker_topic", "/frontier_goal_marker");

        FrontierConfig config;
        config.min_frontier_size =
            i("min_frontier_size", defs.min_frontier_size);
        config.occupied_threshold =
            i("occupied_threshold", defs.occupied_threshold);
        config.min_clearance_cells =
            i("min_clearance_cells", defs.min_clearance_cells);
        config.path_clearance_cells =
            i("path_clearance_cells", defs.path_clearance_cells);
        config.min_frontier_distance =
            d("min_frontier_distance", defs.min_frontier_distance);
        config.max_frontier_distance =
            d("max_frontier_distance", defs.max_frontier_distance);
        config.frontier_goal_search_radius =
            d("frontier_goal_search_radius", defs.frontier_goal_search_radius);
        config.robot_start_search_radius =
            d("robot_start_search_radius", defs.robot_start_search_radius);
        config.information_radius =
            d("information_radius", defs.information_radius);
        config.path_weight = d("path_weight", defs.path_weight);
        config.information_weight =
            d("information_weight", defs.information_weight);
        config.frontier_size_weight =
            d("frontier_size_weight", defs.frontier_size_weight);
        config.unreachable_goal_penalty =
            d("unreachable_goal_penalty", defs.unreachable_goal_penalty);
        config.allow_unreachable_goal =
            b("allow_unreachable_goal", defs.allow_unreachable_goal);
        config.goal_update_distance =
            d("goal_update_distance", defs.goal_update_distance);
        config.goal_publish_period =
            d("goal_publish_period", defs.goal_publish_period);
        config.footprint_front_m =
            d("footprint_front_m", defs.footprint_front_m);
        config.footprint_rear_m = d("footprint_rear_m", defs.footprint_rear_m);
        config.footprint_half_width_m =
            d("footprint_half_width_m", defs.footprint_half_width_m);
        config.footprint_padding_m =
            d("footprint_padding_m", defs.footprint_padding_m);
        config.swept_step_m = d("swept_step_m", defs.swept_step_m);
        config.unknown_overlap_max_ratio =
            d("unknown_overlap_max_ratio", defs.unknown_overlap_max_ratio);
        config.unknown_overlap_weight =
            d("unknown_overlap_weight", defs.unknown_overlap_weight);
        config.wall_clearance_m = d("wall_clearance_m", defs.wall_clearance_m);
        config.corner_penalty_weight =
            d("corner_penalty_weight", defs.corner_penalty_weight);
        config.corridor_progress_weight =
            d("corridor_progress_weight", defs.corridor_progress_weight);
        config.goal_timeout_sec = d("goal_timeout_sec", defs.goal_timeout_sec);
        config.stuck_progress_radius_m =
            d("stuck_progress_radius_m", defs.stuck_progress_radius_m);
        config.stuck_progress_timeout_sec =
            d("stuck_progress_timeout_sec", defs.stuck_progress_timeout_sec);
        config.failed_goal_blacklist_radius_m =
            d("failed_goal_blacklist_radius_m",
              defs.failed_goal_blacklist_radius_m);
        config.failed_goal_blacklist_sec =
            d("failed_goal_blacklist_sec", defs.failed_goal_blacklist_sec);
        config.failed_goal_max_retries =
            i("failed_goal_max_retries", defs.failed_goal_max_retries);
        core_.setConfig(config);
    }

    void onMap(const nav_msgs::msg::OccupancyGrid& msg) {
        map_received_ = true;
        core_.setMap(msg);
    }

    void onTimer() {
        if (!enabled_) {
            return;
        }
        if (!map_received_) {
            logWaiting("Waiting for /map");
            return;
        }
        if (goal_active_) {
            const auto robot_xy = lookupRobotPose();
            if (robot_xy) {
                checkActiveGoalProgress(*robot_xy);
            }
            return;
        }
        if (!nav_client_->action_server_is_ready()) {
            logWaiting("Waiting for /navigate_to_pose action server");
            return;
        }
        const auto robot_xy = lookupRobotPose();
        if (!robot_xy) {
            return;
        }

        auto result = core_.findCandidates(*robot_xy, nowSec());
        publishFrontierMarkers(result.candidates);
        if (result.candidates.empty()) {
            clearGoalMarker();
            publishNoCandidateEvent(result.counts);
            logWaiting(
                "No frontier goal candidate found (free=" +
                std::to_string(result.counts.free) +
                ", unknown=" + std::to_string(result.counts.unknown) +
                ", occupied=" + std::to_string(result.counts.occupied) +
                ", frontier_cells=" + std::to_string(result.counts.frontier) +
                ", clusters=" + std::to_string(result.counts.clusters) + ")");
            return;
        }
        const auto best = std::min_element(
            result.candidates.begin(), result.candidates.end(),
            [](const auto& a, const auto& b) { return a.score < b.score; });
        publishCandidateEvent("best_candidate", *best);
        if (!core_.shouldPublishGoal(best->goal_xy, nowSec())) {
            publishCandidateEvent("goal_suppressed", *best);
            return;
        }
        sendGoal(*robot_xy, *best);
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

    void sendGoal(const WorldPoint& robot_xy,
                  const FrontierCandidate& candidate) {
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = global_frame_;
        goal.pose.header.stamp = get_clock()->now();
        goal.pose.pose.position.x = candidate.goal_xy.first;
        goal.pose.pose.position.y = candidate.goal_xy.second;
        goal.pose.pose.orientation = yawToQuaternion(candidate.goal_yaw);

        goal_active_ = true;
        active_goal_ = candidate.goal_xy;
        active_goal_sent_time_sec_ = nowSec();
        last_progress_pose_ = robot_xy;
        last_progress_time_sec_ = nowSec();
        core_.noteGoalPublished(candidate.goal_xy, candidate.goal_yaw,
                                nowSec());
        wait_log_count_ = 0;
        publishGoalMarker(candidate.goal_xy, candidate.goal_yaw);
        publishCandidateEvent("goal_sent", candidate);
        RCLCPP_INFO(get_logger(),
                    "Sending cluster frontier goal x=%.2f, y=%.2f, "
                    "frontier_cells=%zu, path=%.2fm, info=%d, score=%.2f, "
                    "unknown=%.2f, clearance=%.2fm, reachable=%s",
                    candidate.goal_xy.first, candidate.goal_xy.second,
                    candidate.cells.size(), candidate.path_length,
                    candidate.information, candidate.score,
                    candidate.unknown_overlap_ratio, candidate.wall_clearance,
                    candidate.reachable ? "true" : "false");

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.goal_response_callback =
            [this](const GoalHandle::SharedPtr& handle) {
                onGoalResponse(handle);
            };
        options.result_callback =
            [this](const GoalHandle::WrappedResult& result) {
                onGoalResult(result);
            };
        nav_client_->async_send_goal(goal, options);
    }

    void onGoalResponse(const GoalHandle::SharedPtr& handle) {
        if (!handle) {
            RCLCPP_WARN(get_logger(), "Cluster frontier goal rejected");
            publishPlannerEvent("goal_rejected",
                                {{"goal_xy", activeGoalJson()}});
            blacklistActiveGoal();
            clearActiveGoal();
            return;
        }
        goal_handle_ = handle;
    }

    void onGoalResult(const GoalHandle::WrappedResult& result) {
        const int status = static_cast<int>(result.code);
        RCLCPP_INFO(get_logger(),
                    "Cluster frontier goal finished with status %d", status);
        publishPlannerEvent(
            "goal_result", {{"status", status}, {"goal_xy", activeGoalJson()}});
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            blacklistActiveGoal();
        }
        clearActiveGoal();
    }

    void checkActiveGoalProgress(const WorldPoint& robot_xy) {
        const double now = nowSec();
        const auto& config = core_.config();
        if (config.goal_timeout_sec > 0.0 &&
            now - active_goal_sent_time_sec_ >= config.goal_timeout_sec) {
            RCLCPP_WARN(get_logger(), "Abandoning frontier goal after timeout");
            abandonActiveGoal("timeout");
            return;
        }
        if (!last_progress_pose_) {
            last_progress_pose_ = robot_xy;
            last_progress_time_sec_ = now;
            return;
        }
        const double moved =
            std::hypot(robot_xy.first - last_progress_pose_->first,
                       robot_xy.second - last_progress_pose_->second);
        if (moved >= config.stuck_progress_radius_m) {
            last_progress_pose_ = robot_xy;
            last_progress_time_sec_ = now;
            return;
        }
        if (config.stuck_progress_timeout_sec > 0.0 &&
            now - last_progress_time_sec_ >=
                config.stuck_progress_timeout_sec) {
            RCLCPP_WARN(get_logger(),
                        "Abandoning frontier goal after no progress");
            abandonActiveGoal("no_progress");
        }
    }

    void abandonActiveGoal(const std::string& reason) {
        publishPlannerEvent("goal_abandoned", {{"reason", reason},
                                               {"goal_xy", activeGoalJson()}});
        blacklistActiveGoal();
        if (goal_handle_) {
            nav_client_->async_cancel_goal(goal_handle_);
        }
        clearActiveGoal();
    }

    void clearActiveGoal() {
        goal_active_ = false;
        goal_handle_.reset();
        active_goal_.reset();
        last_progress_pose_.reset();
    }

    void blacklistActiveGoal() {
        if (!active_goal_) {
            return;
        }
        core_.blacklistGoal(*active_goal_, nowSec());
        const auto record_index = core_.findGoalAttemptIndex(*active_goal_);
        if (!record_index) {
            return;
        }
        const auto& record = core_.goalAttempts()[*record_index];
        publishPlannerEvent(
            "goal_attempt_updated",
            {{"goal_xy", pointJson(record.xy)},
             {"failures", record.failures},
             {"blacklisted",
              record.failures >= core_.config().failed_goal_max_retries},
             {"expires_at_sec", rounded(record.expires_at_sec)}});
    }

    void publishCandidateEvent(const std::string& event,
                               const FrontierCandidate& candidate) {
        publishPlannerEvent(
            event,
            {{"goal_xy", pointJson(candidate.goal_xy)},
             {"goal_yaw", rounded(candidate.goal_yaw)},
             {"score", rounded(candidate.score)},
             {"path_length", rounded(candidate.path_length)},
             {"information", candidate.information},
             {"cluster_size", candidate.cells.size()},
             {"unknown_overlap", rounded(candidate.unknown_overlap_ratio)},
             {"wall_clearance", rounded(candidate.wall_clearance)},
             {"reachable", candidate.reachable}});
    }

    void publishNoCandidateEvent(const FrontierCounts& counts) {
        publishPlannerEvent("no_frontier_candidate",
                            {{"counts",
                              {{"free", counts.free},
                               {"unknown", counts.unknown},
                               {"occupied", counts.occupied},
                               {"frontier", counts.frontier},
                               {"clusters", counts.clusters}}}});
    }

    void publishPlannerEvent(
        const std::string& event,
        const nlohmann::json& fields = nlohmann::json::object()) {
        std_msgs::msg::String msg;
        nlohmann::json payload = fields;
        payload[std::string("event")] = event;
        payload[std::string("time_sec")] = rounded(nowSec());
        msg.data = payload.dump();
        event_pub_->publish(msg);
    }

    void publishFrontierMarkers(
        const std::vector<FrontierCandidate>& candidates) {
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = global_frame_;
        delete_marker.header.stamp = get_clock()->now();
        delete_marker.ns = "frontier_cells";
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(delete_marker);

        visualization_msgs::msg::Marker cells_marker;
        cells_marker.header = delete_marker.header;
        cells_marker.ns = "frontier_cells";
        cells_marker.id = 1;
        cells_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        cells_marker.action = visualization_msgs::msg::Marker::ADD;
        const double resolution = core_.mapResolution();
        cells_marker.scale.x = resolution;
        cells_marker.scale.y = resolution;
        cells_marker.scale.z = 0.02;
        cells_marker.color.r = 1.0;
        cells_marker.color.g = 0.55;
        cells_marker.color.b = 0.05;
        cells_marker.color.a = 0.85;
        cells_marker.pose.orientation.w = 1.0;
        for (const auto& candidate : candidates) {
            for (const auto& cell : candidate.cells) {
                const auto xy =
                    core_.mapToWorldUnchecked(cell.first, cell.second);
                geometry_msgs::msg::Point point;
                point.x = xy.first;
                point.y = xy.second;
                point.z = 0.03;
                cells_marker.points.push_back(point);
            }
        }
        markers.markers.push_back(cells_marker);
        marker_pub_->publish(markers);
    }

    void publishGoalMarker(const WorldPoint& goal_xy, const double yaw) {
        auto marker = makeGoalMarker(goal_xy, yaw);
        goal_marker_pub_->publish(marker);
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
    }

    visualization_msgs::msg::Marker makeGoalMarker(const WorldPoint& goal_xy,
                                                   const double yaw) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = global_frame_;
        marker.header.stamp = get_clock()->now();
        marker.ns = "frontier_goal";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = goal_xy.first;
        marker.pose.position.y = goal_xy.second;
        marker.pose.position.z = 0.12;
        marker.pose.orientation = yawToQuaternion(yaw);
        marker.scale.x = 0.35;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;
        marker.color.r = 0.05;
        marker.color.g = 0.35;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        return marker;
    }

    void clearGoalMarker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = global_frame_;
        marker.header.stamp = get_clock()->now();
        marker.ns = "frontier_goal";
        marker.id = 1;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        goal_marker_pub_->publish(marker);
    }

    void logWaiting(const std::string& message) {
        ++wait_log_count_;
        if (wait_log_count_ == 1 || wait_log_count_ % 6 == 0) {
            RCLCPP_INFO(get_logger(), "%s", message.c_str());
        }
    }

    nlohmann::json activeGoalJson() const {
        return active_goal_ ? pointJson(*active_goal_)
                            : nlohmann::json(nullptr);
    }

    double nowSec() { return get_clock()->now().seconds(); }

    FrontierExplorerCore core_;
    bool enabled_ = true;
    bool map_received_ = false;
    std::string map_topic_;
    std::string robot_frame_;
    std::string global_frame_;
    std::string planner_event_topic_;
    std::string marker_topic_;
    std::string goal_marker_topic_;
    double cycle_period_sec_ = 3.0;
    int wait_log_count_ = 0;

    bool goal_active_ = false;
    std::optional<WorldPoint> active_goal_;
    double active_goal_sent_time_sec_ = 0.0;
    std::optional<WorldPoint> last_progress_pose_;
    double last_progress_time_sec_ = 0.0;
    GoalHandle::SharedPtr goal_handle_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        goal_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace yahboom_alc_bringup

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<yahboom_alc_bringup::ClusterFrontierExplorerNode>());
    rclcpp::shutdown();
    return 0;
}
