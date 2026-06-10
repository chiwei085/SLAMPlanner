#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
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
#include <optional>
#include <string>
#include <vector>

#include "yahboom_alc_bringup/frontier_explorer_core.hpp"
#include "yahboom_alc_bringup/parameter_loader.hpp"

namespace yahboom_alc_bringup
{

struct ClusterExplorerNodeParams
{
    bool enabled = true;
    std::string map_topic = "/map";
    std::string robot_frame = "base_footprint";
    std::string global_frame = "map";
    double cycle_period_sec = 3.0;
    std::string planner_event_topic = "/planner/events";
    std::string marker_topic = "/frontier_markers";
    std::string goal_marker_topic = "/frontier_goal_marker";
};

struct Nav2ValidationParams
{
    bool nav2_validation_enabled = true;
    double nav2_validation_timeout_sec = 2.0;
    std::string nav2_compute_path_action = "/compute_path_to_pose";
};

struct RobotPose
{
    WorldPoint xy;
    double yaw = 0.0;
};

}  // namespace yahboom_alc_bringup

namespace rik_sd
{

template <>
struct descriptor<yahboom_alc_bringup::ClusterExplorerNodeParams>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::ClusterExplorerNodeParams;
        return make_descriptor(
            named(&ClusterExplorerNodeParams::enabled, "enabled"),
            named(&ClusterExplorerNodeParams::map_topic, "map_topic"),
            named(&ClusterExplorerNodeParams::robot_frame, "robot_frame"),
            named(&ClusterExplorerNodeParams::global_frame, "global_frame"),
            named(&ClusterExplorerNodeParams::cycle_period_sec,
                  "cycle_period_sec"),
            named(&ClusterExplorerNodeParams::planner_event_topic,
                  "planner_event_topic"),
            named(&ClusterExplorerNodeParams::marker_topic, "marker_topic"),
            named(&ClusterExplorerNodeParams::goal_marker_topic,
                  "goal_marker_topic"));
    }
};

template <>
struct descriptor<yahboom_alc_bringup::Nav2ValidationParams>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::Nav2ValidationParams;
        return make_descriptor(
            named(&Nav2ValidationParams::nav2_validation_enabled,
                  "nav2_validation_enabled"),
            named(&Nav2ValidationParams::nav2_validation_timeout_sec,
                  "nav2_validation_timeout_sec"),
            named(&Nav2ValidationParams::nav2_compute_path_action,
                  "nav2_compute_path_action"));
    }
};

template <>
struct descriptor<yahboom_alc_bringup::FrontierConfig>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::FrontierConfig;
        return make_descriptor(
            named(&FrontierConfig::min_frontier_size, "min_frontier_size"),
            named(&FrontierConfig::occupied_threshold, "occupied_threshold"),
            named(&FrontierConfig::min_clearance_cells, "min_clearance_cells"),
            named(&FrontierConfig::path_clearance_cells,
                  "path_clearance_cells"),
            named(&FrontierConfig::min_frontier_distance,
                  "min_frontier_distance"),
            named(&FrontierConfig::max_frontier_distance,
                  "max_frontier_distance"),
            named(&FrontierConfig::frontier_goal_search_radius,
                  "frontier_goal_search_radius"),
            named(&FrontierConfig::robot_start_search_radius,
                  "robot_start_search_radius"),
            named(&FrontierConfig::information_radius, "information_radius"),
            named(&FrontierConfig::path_weight, "path_weight"),
            named(&FrontierConfig::information_weight, "information_weight"),
            named(&FrontierConfig::frontier_size_weight,
                  "frontier_size_weight"),
            named(&FrontierConfig::unreachable_goal_penalty,
                  "unreachable_goal_penalty"),
            named(&FrontierConfig::allow_unreachable_goal,
                  "allow_unreachable_goal"),
            named(&FrontierConfig::goal_update_distance,
                  "goal_update_distance"),
            named(&FrontierConfig::goal_publish_period, "goal_publish_period"),
            named(&FrontierConfig::footprint_front_m, "footprint_front_m"),
            named(&FrontierConfig::footprint_rear_m, "footprint_rear_m"),
            named(&FrontierConfig::footprint_half_width_m,
                  "footprint_half_width_m"),
            named(&FrontierConfig::footprint_padding_m, "footprint_padding_m"),
            named(&FrontierConfig::swept_step_m, "swept_step_m"),
            named(&FrontierConfig::unknown_overlap_max_ratio,
                  "unknown_overlap_max_ratio"),
            named(&FrontierConfig::unknown_overlap_weight,
                  "unknown_overlap_weight"),
            named(&FrontierConfig::wall_clearance_m, "wall_clearance_m"),
            named(&FrontierConfig::corner_penalty_weight,
                  "corner_penalty_weight"),
            named(&FrontierConfig::corridor_progress_weight,
                  "corridor_progress_weight"),
            named(&FrontierConfig::heading_penalty_weight,
                  "heading_penalty_weight"),
            named(&FrontierConfig::heading_penalty_max_rad,
                  "heading_penalty_max_rad"),
            named(&FrontierConfig::goal_timeout_sec, "goal_timeout_sec"),
            named(&FrontierConfig::stuck_progress_radius_m,
                  "stuck_progress_radius_m"),
            named(&FrontierConfig::stuck_progress_timeout_sec,
                  "stuck_progress_timeout_sec"),
            named(&FrontierConfig::near_goal_hysteresis_radius_m,
                  "near_goal_hysteresis_radius_m"),
            named(&FrontierConfig::near_goal_retry_limit,
                  "near_goal_retry_limit"),
            named(&FrontierConfig::near_goal_hysteresis_sec,
                  "near_goal_hysteresis_sec"),
            named(&FrontierConfig::alignment_grace_enabled,
                  "alignment_grace_enabled"),
            named(&FrontierConfig::alignment_heading_threshold_rad,
                  "alignment_heading_threshold_rad"),
            named(&FrontierConfig::alignment_max_grace_sec,
                  "alignment_max_grace_sec"),
            named(&FrontierConfig::alignment_min_rotation_delta_rad,
                  "alignment_min_rotation_delta_rad"),
            named(&FrontierConfig::failed_goal_blacklist_radius_m,
                  "failed_goal_blacklist_radius_m"),
            named(&FrontierConfig::failed_goal_blacklist_sec,
                  "failed_goal_blacklist_sec"),
            named(&FrontierConfig::failed_goal_max_retries,
                  "failed_goal_max_retries"),
            named(&FrontierConfig::enable_subgoals, "enable_subgoals"),
            named(&FrontierConfig::max_subgoals, "max_subgoals"),
            named(&FrontierConfig::subgoal_path_tolerance_m,
                  "subgoal_path_tolerance_m"),
            named(&FrontierConfig::min_subgoal_spacing_m,
                  "min_subgoal_spacing_m"));
    }
};

}  // namespace rik_sd

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

nlohmann::json optPointJson(const std::optional<WorldPoint>& pt) {
    return pt ? pointJson(*pt) : nlohmann::json(nullptr);
}

}  // namespace

class ClusterFrontierExplorerNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using ComputePathGoalHandle =
        rclcpp_action::ClientGoalHandle<ComputePathToPose>;

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
        const auto node_params =
            loadDeclaredParameters<ClusterExplorerNodeParams>(*this);
        enabled_ = node_params.enabled;
        map_topic_ = node_params.map_topic;
        robot_frame_ = node_params.robot_frame;
        global_frame_ = node_params.global_frame;
        cycle_period_sec_ = node_params.cycle_period_sec;
        planner_event_topic_ = node_params.planner_event_topic;
        marker_topic_ = node_params.marker_topic;
        goal_marker_topic_ = node_params.goal_marker_topic;

        auto config = core_.config();
        loadDeclaredParametersInto(*this, config);
        core_.setConfig(config);

        const auto nav2_params =
            loadDeclaredParameters<Nav2ValidationParams>(*this);
        nav2_validation_enabled_ = nav2_params.nav2_validation_enabled;
        nav2_validation_timeout_sec_ = nav2_params.nav2_validation_timeout_sec;
        nav2_compute_path_action_ = nav2_params.nav2_compute_path_action;
        compute_path_client_ = rclcpp_action::create_client<ComputePathToPose>(
            this, nav2_compute_path_action_);
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
            const auto robot_pose = lookupRobotPose();
            if (robot_pose) {
                checkActiveGoalProgress(*robot_pose);
            }
            return;
        }
        if (validation_active_) {
            checkValidationTimeout();
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

        auto result =
            core_.findCandidates(robot_pose->xy, nowSec(), robot_pose->yaw);
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
                ", clusters=" + std::to_string(result.counts.clusters) +
                ", rej_no_opts=" +
                std::to_string(result.counts.rejected_no_options) +
                ", rej_blacklist=" +
                std::to_string(result.counts.rejected_blacklist) +
                ", rej_too_near=" +
                std::to_string(result.counts.rejected_too_near) +
                ", rej_too_far=" +
                std::to_string(result.counts.rejected_too_far) +
                ", rej_no_path=" +
                std::to_string(result.counts.rejected_no_path) +
                ", rej_footprint=" +
                std::to_string(result.counts.rejected_footprint) +
                ", rej_swept=" + std::to_string(result.counts.rejected_swept) +
                ", rej_unknown=" +
                std::to_string(result.counts.rejected_unknown_ratio) + ")");
            return;
        }
        std::sort(
            result.candidates.begin(), result.candidates.end(),
            [](const auto& a, const auto& b) { return a.score < b.score; });
        const auto best = result.candidates.begin();
        if (!core_.shouldPublishGoal(best->goal_xy, nowSec())) {
            publishCandidateEvent("goal_suppressed", *best);
            return;
        }
        beginRouteSelection(robot_pose->xy, result.candidates);
    }

    struct RouteWaypoint
    {
        WorldPoint xy;
        double yaw = 0.0;
        bool final = false;
    };

    std::vector<RouteWaypoint> buildRoute(
        const FrontierCandidate& final_candidate,
        const std::vector<FrontierCandidate>& candidates) {
        std::vector<RouteWaypoint> route;
        const auto subgoals =
            core_.buildSubgoalsForRoute(final_candidate, candidates);
        route.reserve(subgoals.size() + 1);
        for (const auto& subgoal : subgoals) {
            route.push_back({subgoal.goal_xy, subgoal.goal_yaw, false});
        }
        route.push_back(
            {final_candidate.goal_xy, final_candidate.goal_yaw, true});
        return route;
    }

    void beginRouteSelection(
        const WorldPoint& robot_xy,
        const std::vector<FrontierCandidate>& sorted_candidates) {
        pending_robot_xy_ = robot_xy;
        pending_candidates_ = sorted_candidates;
        pending_candidate_index_ = 0;
        validateNextCandidateRoute();
    }

    void validateNextCandidateRoute() {
        validation_active_ = false;
        validation_route_.clear();
        validation_final_candidate_.reset();
        validation_segment_index_ = 0;

        while (pending_candidate_index_ < pending_candidates_.size()) {
            const auto& candidate =
                pending_candidates_[pending_candidate_index_++];
            if (!core_.shouldPublishGoal(candidate.goal_xy, nowSec())) {
                publishCandidateEvent("goal_suppressed", candidate);
                continue;
            }
            publishCandidateEvent("best_candidate", candidate);
            validation_final_candidate_ = candidate;
            validation_route_ = buildRoute(candidate, pending_candidates_);
            if (!nav2_validation_enabled_) {
                startRoute(*pending_robot_xy_, *validation_final_candidate_,
                           validation_route_);
                return;
            }
            validation_active_ = true;
            validation_generation_++;
            validateRouteSegment(validation_generation_);
            return;
        }

        pending_candidates_.clear();
        pending_robot_xy_.reset();
        publishPlannerEvent("nav2_validation_failed",
                            {{"reason", "all_candidate_routes_failed"}});
    }

    geometry_msgs::msg::PoseStamped poseForPoint(const WorldPoint& xy,
                                                 const double yaw) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.header.stamp = get_clock()->now();
        pose.pose.position.x = xy.first;
        pose.pose.position.y = xy.second;
        pose.pose.orientation = yawToQuaternion(yaw);
        return pose;
    }

    void validateRouteSegment(const int generation) {
        if (!validation_active_ || generation != validation_generation_ ||
            !pending_robot_xy_ || !validation_final_candidate_ ||
            validation_segment_index_ >= validation_route_.size()) {
            return;
        }
        if (!compute_path_client_->action_server_is_ready()) {
            publishPlannerEvent(
                "nav2_validation_failed",
                {{"reason", "compute_path_action_unavailable"},
                 {"goal_xy", pointJson(validation_final_candidate_->goal_xy)}});
            validateNextCandidateRoute();
            return;
        }

        const auto& waypoint = validation_route_[validation_segment_index_];
        const WorldPoint start_xy =
            validation_segment_index_ == 0
                ? *pending_robot_xy_
                : validation_route_[validation_segment_index_ - 1].xy;
        const double start_yaw =
            validation_segment_index_ == 0
                ? validation_final_candidate_->goal_yaw
                : validation_route_[validation_segment_index_ - 1].yaw;

        ComputePathToPose::Goal goal;
        goal.start = poseForPoint(start_xy, start_yaw);
        goal.goal = poseForPoint(waypoint.xy, waypoint.yaw);
        goal.planner_id = "";
        goal.use_start = true;

        validation_segment_started_sec_ = nowSec();

        rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
        options.goal_response_callback =
            [this, generation](const ComputePathGoalHandle::SharedPtr& handle) {
                onValidationGoalResponse(generation, handle);
            };
        options.result_callback =
            [this,
             generation](const ComputePathGoalHandle::WrappedResult& result) {
                onValidationResult(generation, result);
            };
        compute_path_client_->async_send_goal(goal, options);
    }

    void onValidationGoalResponse(
        const int generation, const ComputePathGoalHandle::SharedPtr& handle) {
        if (!validation_active_ || generation != validation_generation_) {
            return;
        }
        if (!handle) {
            publishPlannerEvent("nav2_validation_failed",
                                {{"reason", "goal_rejected"}});
            validateNextCandidateRoute();
        }
    }

    void onValidationResult(
        const int generation,
        const ComputePathGoalHandle::WrappedResult& result) {
        if (!validation_active_ || generation != validation_generation_) {
            return;
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
            result.result->path.poses.empty()) {
            publishPlannerEvent("nav2_validation_failed",
                                {{"reason", "empty_or_failed_path"},
                                 {"status", static_cast<int>(result.code)},
                                 {"segment", validation_segment_index_}});
            validateNextCandidateRoute();
            return;
        }

        ++validation_segment_index_;
        if (validation_segment_index_ < validation_route_.size()) {
            validateRouteSegment(generation);
            return;
        }

        validation_active_ = false;
        if (pending_robot_xy_ && validation_final_candidate_) {
            startRoute(*pending_robot_xy_, *validation_final_candidate_,
                       validation_route_);
        }
    }

    void checkValidationTimeout() {
        if (!validation_active_ || nav2_validation_timeout_sec_ <= 0.0) {
            return;
        }
        if (nowSec() - validation_segment_started_sec_ <
            nav2_validation_timeout_sec_) {
            return;
        }
        publishPlannerEvent(
            "nav2_validation_failed",
            {{"reason", "timeout"}, {"segment", validation_segment_index_}});
        ++validation_generation_;
        validateNextCandidateRoute();
    }

    void startRoute(const WorldPoint& robot_xy,
                    const FrontierCandidate& final_candidate,
                    const std::vector<RouteWaypoint>& route) {
        if (route.empty()) {
            return;
        }
        active_route_ = route;
        active_route_index_ = 0;
        active_final_goal_ = final_candidate.goal_xy;
        active_route_path_ = final_candidate.path;
        const auto projection =
            core_.projectOntoPath(robot_xy, active_route_path_);
        last_progress_arc_length_ = projection ? projection->arc_length : 0.0;
        last_progress_time_sec_ = nowSec();
        active_goal_sent_time_sec_ = nowSec();
        active_waypoint_sent_time_sec_ = nowSec();
        last_alignment_robot_yaw_.reset();
        active_goal_hysteresis_retries_ = 0;
        core_.noteGoalPublished(final_candidate.goal_xy,
                                final_candidate.goal_yaw, nowSec());
        publishPlannerEvent(
            "subgoal_route_selected",
            {{"final_goal_xy", pointJson(final_candidate.goal_xy)},
             {"waypoints", active_route_.size()}});
        sendActiveWaypoint();
    }

    void sendActiveWaypoint() {
        if (active_route_index_ >= active_route_.size()) {
            clearActiveGoal();
            return;
        }
        const auto& waypoint = active_route_[active_route_index_];
        FrontierCandidate event_candidate;
        event_candidate.goal_xy = waypoint.xy;
        event_candidate.goal_yaw = waypoint.yaw;
        event_candidate.path = active_route_path_;
        event_candidate.reachable = true;
        publishCandidateEvent(waypoint.final ? "goal_sent" : "subgoal_sent",
                              event_candidate);
        sendGoal(waypoint.xy, waypoint.yaw);
    }

    std::optional<RobotPose> lookupRobotPose() {
        try {
            const auto transform = tf_buffer_.lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero);
            const auto& t = transform.transform.translation;
            const auto& r = transform.transform.rotation;
            const double yaw = std::atan2(2.0 * (r.w * r.z + r.x * r.y),
                                          1.0 - 2.0 * (r.y * r.y + r.z * r.z));
            return RobotPose{WorldPoint{t.x, t.y}, yaw};
        }
        catch (const tf2::TransformException& exc) {
            RCLCPP_DEBUG(get_logger(), "Waiting for robot pose TF: %s",
                         exc.what());
            return std::nullopt;
        }
    }

    void sendGoal(const WorldPoint& goal_xy, const double goal_yaw) {
        NavigateToPose::Goal goal;
        goal.pose = poseForPoint(goal_xy, goal_yaw);

        goal_active_ = true;
        active_goal_ = goal_xy;
        active_waypoint_sent_time_sec_ = nowSec();
        last_alignment_robot_yaw_.reset();
        wait_log_count_ = 0;
        publishGoalMarker(goal_xy, goal_yaw);
        RCLCPP_INFO(get_logger(),
                    "Sending frontier route waypoint %zu/%zu x=%.2f, y=%.2f",
                    active_route_index_ + 1, active_route_.size(),
                    goal_xy.first, goal_xy.second);

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
            abandonActiveGoal("goal_rejected");
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
            abandonActiveGoal("nav2_result_failed");
            return;
        }

        if (active_route_index_ + 1 < active_route_.size()) {
            publishPlannerEvent("subgoal_reached",
                                {{"goal_xy", activeGoalJson()},
                                 {"index", active_route_index_}});
            goal_handle_.reset();
            goal_active_ = false;
            ++active_route_index_;
            const auto robot_pose = lookupRobotPose();
            if (robot_pose) {
                const auto projection =
                    core_.projectOntoPath(robot_pose->xy, active_route_path_);
                if (projection) {
                    last_progress_arc_length_ = projection->arc_length;
                }
                last_progress_time_sec_ = nowSec();
                active_waypoint_sent_time_sec_ = nowSec();
                last_alignment_robot_yaw_.reset();
                sendActiveWaypoint();
                return;
            }
            abandonActiveGoal("missing_pose_after_subgoal");
            return;
        }

        publishPlannerEvent("route_completed",
                            {{"goal_xy", activeFinalGoalJson()}});
        clearActiveGoal();
    }

    void checkActiveGoalProgress(const RobotPose& robot_pose) {
        const double now = nowSec();
        const auto& config = core_.config();
        if (config.goal_timeout_sec > 0.0 &&
            now - active_goal_sent_time_sec_ >= config.goal_timeout_sec) {
            RCLCPP_WARN(get_logger(), "Abandoning frontier goal after timeout");
            abandonActiveGoal("timeout");
            return;
        }
        const auto projection =
            core_.projectOntoPath(robot_pose.xy, active_route_path_);
        if (projection && projection->arc_length - last_progress_arc_length_ >=
                              config.stuck_progress_radius_m) {
            last_progress_arc_length_ = projection->arc_length;
            last_progress_time_sec_ = now;
            last_alignment_robot_yaw_.reset();
            RCLCPP_DEBUG(get_logger(), "Route progress arc=%.3f",
                         last_progress_arc_length_);
            return;
        }
        if (config.stuck_progress_timeout_sec > 0.0 &&
            now - last_progress_time_sec_ >=
                config.stuck_progress_timeout_sec) {
            if (tryAlignmentProgressGrace(robot_pose, now)) {
                last_alignment_robot_yaw_ = robot_pose.yaw;
                return;
            }
            if (tryNearGoalHysteresis(robot_pose.xy, now)) {
                last_alignment_robot_yaw_ = robot_pose.yaw;
                return;
            }
            RCLCPP_WARN(get_logger(),
                        "Abandoning frontier goal after no progress");
            abandonActiveGoal("no_progress");
            return;
        }
        last_alignment_robot_yaw_ = robot_pose.yaw;
    }

    bool tryAlignmentProgressGrace(const RobotPose& robot_pose,
                                   const double now) {
        if (!active_goal_) {
            return false;
        }
        const auto status = core_.evaluateAlignmentGrace(
            robot_pose.xy, robot_pose.yaw, last_alignment_robot_yaw_,
            active_route_path_, *active_goal_,
            now - active_waypoint_sent_time_sec_);
        if (!status.active) {
            return false;
        }

        const auto& config = core_.config();
        const double last_progress_age = now - last_progress_time_sec_;
        // Backdate so no_progress won't re-fire until alignment_max_grace_sec
        // has elapsed from when this waypoint was sent.
        const double capped_progress_time = active_waypoint_sent_time_sec_ +
                                            config.alignment_max_grace_sec -
                                            config.stuck_progress_timeout_sec;
        last_progress_time_sec_ =
            std::max(last_progress_time_sec_, capped_progress_time);
        publishPlannerEvent(
            "alignment_progress_grace",
            {{"goal_xy", activeFinalGoalJson()},
             {"active_waypoint_xy", activeGoalJson()},
             {"robot_xy", pointJson(robot_pose.xy)},
             {"heading_error", rounded(status.heading_error)},
             {"target_yaw", rounded(status.target_yaw)},
             {"elapsed_since_waypoint_sent",
              rounded(now - active_waypoint_sent_time_sec_)},
             {"last_progress_age", rounded(last_progress_age)}});
        RCLCPP_DEBUG(get_logger(),
                     "Holding frontier goal for alignment heading_error=%.3f",
                     status.heading_error);
        return true;
    }

    bool tryNearGoalHysteresis(const WorldPoint& robot_xy, const double now) {
        const auto& config = core_.config();
        if (config.near_goal_retry_limit <= 0 ||
            active_goal_hysteresis_retries_ >= config.near_goal_retry_limit ||
            config.near_goal_hysteresis_radius_m <= 0.0) {
            return false;
        }
        if (!isNearActiveGoal(robot_xy, config.near_goal_hysteresis_radius_m)) {
            return false;
        }

        ++active_goal_hysteresis_retries_;
        extendActiveGoalTimers(now);
        publishPlannerEvent("no_progress_hysteresis_retry",
                            {{"goal_xy", activeFinalGoalJson()},
                             {"active_waypoint_xy", activeGoalJson()},
                             {"retry", active_goal_hysteresis_retries_},
                             {"retry_limit", config.near_goal_retry_limit},
                             {"robot_xy", pointJson(robot_xy)}});
        RCLCPP_WARN(get_logger(),
                    "Holding near frontier goal after no progress retry %d/%d",
                    active_goal_hysteresis_retries_,
                    config.near_goal_retry_limit);
        return true;
    }

    bool isNearActiveGoal(const WorldPoint& robot_xy,
                          const double radius) const {
        const auto near = [&](const std::optional<WorldPoint>& goal) {
            return goal && std::hypot(robot_xy.first - goal->first,
                                      robot_xy.second - goal->second) <= radius;
        };
        return near(active_goal_) || near(active_final_goal_);
    }

    void extendActiveGoalTimers(const double now) {
        const auto& config = core_.config();
        const double grace = config.near_goal_hysteresis_sec > 0.0
                                 ? config.near_goal_hysteresis_sec
                                 : config.stuck_progress_timeout_sec;
        if (config.stuck_progress_timeout_sec > 0.0) {
            last_progress_time_sec_ =
                now - std::max(0.0, config.stuck_progress_timeout_sec - grace);
        }
        if (config.goal_timeout_sec > 0.0) {
            active_goal_sent_time_sec_ =
                now - std::max(0.0, config.goal_timeout_sec - grace);
        }
    }

    void abandonActiveGoal(const std::string& reason) {
        publishPlannerEvent("route_abandoned",
                            {{"reason", reason},
                             {"goal_xy", activeFinalGoalJson()},
                             {"active_waypoint_xy", activeGoalJson()}});
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
        active_route_.clear();
        active_route_index_ = 0;
        active_final_goal_.reset();
        active_route_path_.clear();
        last_progress_arc_length_ = 0.0;
        active_goal_hysteresis_retries_ = 0;
        active_waypoint_sent_time_sec_ = 0.0;
        last_alignment_robot_yaw_.reset();
    }

    void blacklistActiveGoal() {
        if (!active_final_goal_) {
            return;
        }
        core_.blacklistGoal(*active_final_goal_, nowSec());
        const auto record_index =
            core_.findGoalAttemptIndex(*active_final_goal_);
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
             {"heading_delta", rounded(candidate.heading_delta)},
             {"reachable", candidate.reachable}});
    }

    void publishNoCandidateEvent(const FrontierCounts& counts) {
        publishPlannerEvent(
            "no_frontier_candidate",
            {{"counts",
              {{"free", counts.free},
               {"unknown", counts.unknown},
               {"occupied", counts.occupied},
               {"frontier", counts.frontier},
               {"clusters", counts.clusters},
               {"rejected_no_options", counts.rejected_no_options},
               {"rejected_blacklist", counts.rejected_blacklist},
               {"rejected_too_near", counts.rejected_too_near},
               {"rejected_too_far", counts.rejected_too_far},
               {"rejected_no_path", counts.rejected_no_path},
               {"rejected_footprint", counts.rejected_footprint},
               {"rejected_swept", counts.rejected_swept},
               {"rejected_unknown_ratio", counts.rejected_unknown_ratio}}}});
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

    nlohmann::json activeGoalJson() const { return optPointJson(active_goal_); }

    nlohmann::json activeFinalGoalJson() const {
        return optPointJson(active_final_goal_);
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
    std::optional<WorldPoint> active_final_goal_;
    std::vector<RouteWaypoint> active_route_;
    std::size_t active_route_index_ = 0;
    std::vector<Cell> active_route_path_;
    double active_goal_sent_time_sec_ = 0.0;
    double active_waypoint_sent_time_sec_ = 0.0;
    double last_progress_time_sec_ = 0.0;
    double last_progress_arc_length_ = 0.0;
    int active_goal_hysteresis_retries_ = 0;
    std::optional<double> last_alignment_robot_yaw_;
    GoalHandle::SharedPtr goal_handle_;

    bool nav2_validation_enabled_ = true;
    double nav2_validation_timeout_sec_ = 2.0;
    std::string nav2_compute_path_action_;
    bool validation_active_ = false;
    int validation_generation_ = 0;
    std::size_t validation_segment_index_ = 0;
    double validation_segment_started_sec_ = 0.0;
    std::optional<WorldPoint> pending_robot_xy_;
    std::vector<FrontierCandidate> pending_candidates_;
    std::size_t pending_candidate_index_ = 0;
    std::optional<FrontierCandidate> validation_final_candidate_;
    std::vector<RouteWaypoint> validation_route_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_client_;
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
