#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
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
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "yahboom_alc_bringup/backtrace_recovery.hpp"
#include "yahboom_alc_bringup/frontier_explorer_core.hpp"
#include "yahboom_alc_bringup/math_util.hpp"
#include "yahboom_alc_bringup/parameter_loader.hpp"
#include "yahboom_alc_bringup/planner_event.hpp"
#include "yahboom_alc_bringup/recovery_state_machine.hpp"
#include "yahboom_alc_bringup/yaw_util.hpp"

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
    double nav2_success_xy_tolerance_m = 0.55;
};

struct Nav2ValidationParams
{
    bool nav2_validation_enabled = true;
    double nav2_validation_timeout_sec = 2.0;
    std::string nav2_compute_path_action = "/compute_path_to_pose";
    bool nav2_validation_blacklist_enabled = true;
    int nav2_validation_rejected_retry_max = 10;
    double nav2_validation_rejected_retry_delay_sec = 3.0;
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
                  "goal_marker_topic"),
            named(&ClusterExplorerNodeParams::nav2_success_xy_tolerance_m,
                  "nav2_success_xy_tolerance_m"));
    }
};

template <>
struct descriptor<yahboom_alc_bringup::BacktraceRecoveryConfig>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::BacktraceRecoveryConfig;
        return make_descriptor(
            named(&BacktraceRecoveryConfig::enabled,
                  "backtrace_recovery_enabled"),
            named(&BacktraceRecoveryConfig::breadcrumb_max_distance_m,
                  "backtrace_breadcrumb_max_distance_m"),
            named(&BacktraceRecoveryConfig::breadcrumb_max_age_sec,
                  "backtrace_breadcrumb_max_age_sec"),
            named(&BacktraceRecoveryConfig::backup_step_m,
                  "backtrace_backup_step_m"),
            named(&BacktraceRecoveryConfig::backup_speed_mps,
                  "backtrace_backup_speed_mps"),
            named(&BacktraceRecoveryConfig::backup_time_allowance_sec,
                  "backtrace_backup_time_allowance_sec"),
            named(&BacktraceRecoveryConfig::max_segments,
                  "backtrace_max_segments"),
            named(&BacktraceRecoveryConfig::min_actual_displacement_m,
                  "backtrace_min_actual_displacement_m"),
            named(&BacktraceRecoveryConfig::cancel_timeout_sec,
                  "backtrace_cancel_timeout_sec"),
            named(&BacktraceRecoveryConfig::backup_stall_check_sec,
                  "backtrace_backup_stall_check_sec"),
            named(&BacktraceRecoveryConfig::reverse_motion_ratio_min,
                  "backtrace_reverse_motion_ratio_min"),
            named(&BacktraceRecoveryConfig::backup_action,
                  "backtrace_backup_action"));
    }
};

template <>
struct descriptor<yahboom_alc_bringup::RecoveryStateMachineConfig>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::RecoveryStateMachineConfig;
        return make_descriptor(
            named(&RecoveryStateMachineConfig::recovery_reobserve_cycles,
                  "recovery_reobserve_cycles"),
            named(&RecoveryStateMachineConfig::recovery_small_spin_yaw_rad,
                  "recovery_small_spin_yaw_rad"),
            named(&RecoveryStateMachineConfig::recovery_small_spin_attempts,
                  "recovery_small_spin_attempts"),
            named(
                &RecoveryStateMachineConfig::recovery_forward_escape_distance_m,
                "recovery_forward_escape_distance_m"),
            named(&RecoveryStateMachineConfig::rear_blocked_min_latch_sec,
                  "rear_blocked_min_latch_sec"),
            named(&RecoveryStateMachineConfig::
                      rear_blocked_clear_forward_displacement_m,
                  "rear_blocked_clear_forward_displacement_m"),
            named(&RecoveryStateMachineConfig::recovery_clearance_backup_m,
                  "recovery_clearance_backup_m"),
            named(&RecoveryStateMachineConfig::
                      recovery_stagnation_escalation_trees,
                  "recovery_stagnation_escalation_trees"),
            named(&RecoveryStateMachineConfig::recovery_stagnation_radius_m,
                  "recovery_stagnation_radius_m"));
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
                  "nav2_compute_path_action"),
            named(&Nav2ValidationParams::nav2_validation_blacklist_enabled,
                  "nav2_validation_blacklist_enabled"),
            named(&Nav2ValidationParams::nav2_validation_rejected_retry_max,
                  "nav2_validation_rejected_retry_max"),
            named(
                &Nav2ValidationParams::nav2_validation_rejected_retry_delay_sec,
                "nav2_validation_rejected_retry_delay_sec"));
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
            named(&FrontierConfig::exploratory_path_clearance_cells,
                  "exploratory_path_clearance_cells"),
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
            named(&FrontierConfig::soft_unreachable_penalty,
                  "soft_unreachable_penalty"),
            named(&FrontierConfig::allow_unreachable_goal,
                  "allow_unreachable_goal"),
            named(&FrontierConfig::nav2_validation_required_for_final_goal,
                  "nav2_validation_required_for_final_goal"),
            named(&FrontierConfig::approach_offset_m, "approach_offset_m"),
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
            named(&FrontierConfig::min_wall_clearance_m,
                  "min_wall_clearance_m"),
            named(&FrontierConfig::nav2_goal_clearance_m,
                  "nav2_goal_clearance_m"),
            named(&FrontierConfig::viewpoint_information_weight,
                  "viewpoint_information_weight"),
            named(&FrontierConfig::viewpoint_range_m, "viewpoint_range_m"),
            named(&FrontierConfig::viewpoint_half_fov_rad,
                  "viewpoint_half_fov_rad"),
            named(&FrontierConfig::coverage_revisit_enabled,
                  "coverage_revisit_enabled"),
            named(&FrontierConfig::coverage_proximity_m,
                  "coverage_proximity_m"),
            named(&FrontierConfig::coverage_sample_spacing_m,
                  "coverage_sample_spacing_m"),
            named(&FrontierConfig::coverage_max_targets,
                  "coverage_max_targets"),
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
                  "min_subgoal_spacing_m"),
            named(&FrontierConfig::no_frontier_relax_sec,
                  "no_frontier_relax_sec"),
            named(&FrontierConfig::relaxed_min_frontier_size,
                  "relaxed_min_frontier_size"),
            named(&FrontierConfig::relaxed_max_frontier_distance,
                  "relaxed_max_frontier_distance"),
            named(&FrontierConfig::alignment_spin_enabled,
                  "alignment_spin_enabled"),
            named(&FrontierConfig::alignment_spin_max_attempts,
                  "alignment_spin_max_attempts"),
            named(&FrontierConfig::alignment_spin_time_allowance_sec,
                  "alignment_spin_time_allowance_sec"),
            named(&FrontierConfig::coverage_blend_weight,
                  "coverage_blend_weight"),
            named(&FrontierConfig::intention_distance_threshold_m,
                  "intention_distance_threshold_m"),
            named(&FrontierConfig::intention_continuation_bonus,
                  "intention_continuation_bonus"),
            named(&FrontierConfig::intention_clear_radius_m,
                  "intention_clear_radius_m"),
            named(&FrontierConfig::intention_timeout_sec,
                  "intention_timeout_sec"));
    }
};

}  // namespace rik_sd

namespace yahboom_alc_bringup
{

namespace
{

enum class FailureClass : std::uint8_t
{
    kNone,
    kGlobalPathBlocked,
    kControllerNoProgress,
    kReverseUnsafe,
    kStallOrSlip,
    kLocalHazard,     // weight 2: physical blockage confirmed by stall/slip
    kRecoveryFailed,  // weight 2: all recovery strategies exhausted
};

constexpr int blacklistWeight(const FailureClass fc) noexcept {
    return (fc == FailureClass::kLocalHazard ||
            fc == FailureClass::kRecoveryFailed)
               ? 2
               : 1;
}

constexpr const char* failureClassName(const FailureClass fc) noexcept {
    switch (fc) {
        case FailureClass::kNone:
            return "none";
        case FailureClass::kGlobalPathBlocked:
            return "global_path_blocked";
        case FailureClass::kControllerNoProgress:
            return "controller_no_progress";
        case FailureClass::kReverseUnsafe:
            return "reverse_unsafe";
        case FailureClass::kStallOrSlip:
            return "stall_or_slip";
        case FailureClass::kLocalHazard:
            return "local_hazard";
        case FailureClass::kRecoveryFailed:
            return "recovery_failed";
    }
    return "unknown";
}

// Margin granted on top of the behavior server's own spin time allowance
// to cover action transport before declaring a spin lost.
constexpr double kSpinTransportMarginSec = 5.0;
constexpr int kReverseBlockedBacktraceThreshold = 3;
constexpr double kImmediateReverseBlockWindowSec = 3.0;
constexpr double kRecentReverseBlockedWindowSec = 3.0;
// After this many consecutive backup_stalled failures, the rear is blocked;
// skip backtrace and directly abandon to avoid spinning wheels in place.
constexpr int kBacktraceStallSkipThreshold = 3;
// After this many consecutive immediate reverse-blocked abandons across
// different frontiers, the robot's current pose is the trap — trigger a
// forward escape instead of cycling through more doomed frontier candidates.
constexpr int kConsecutiveReverseChurnEscapeThreshold = 2;
// If nav2 declares SUCCEEDED in under this many seconds the robot was already
// at the goal and didn't actually navigate anywhere.  Blacklist the goal to
// break the "frontier never clears" cycling loop.
constexpr double kTrivialSuccessThresholdSec = 2.0;
// Nav2 humble's ControllerServer::isGoalReached() ignores the return value of
// the end-pose transform: when the map->odom lookup fails the goal checker
// compares the robot against a default-constructed (identity) pose.  A robot
// near the odom origin then gets phantom SUCCEEDED results far from the goal.
// Resend the same waypoint up to this many times before treating the phantom
// success as a real failure and entering the recovery tree.
constexpr int kSpuriousSuccessResendMax = 2;

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
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
    using Spin = nav2_msgs::action::Spin;
    using SpinGoalHandle = rclcpp_action::ClientGoalHandle<Spin>;
    using BackUp = nav2_msgs::action::BackUp;
    using BackUpGoalHandle = rclcpp_action::ClientGoalHandle<BackUp>;

    explicit ClusterFrontierExplorerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
        : rclcpp::Node("cluster_frontier_explorer", options),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_),
          nav_client_(rclcpp_action::create_client<NavigateToPose>(
              this, "/navigate_to_pose")),
          spin_client_(rclcpp_action::create_client<Spin>(this, "/spin")) {
        loadParams();
        backup_client_ = rclcpp_action::create_client<BackUp>(
            this, backtrace_backup_action_);

        rclcpp::QoS map_qos{rclcpp::KeepLast(1)};
        map_qos.reliable().transient_local();
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, map_qos,
            [this](const nav_msgs::msg::OccupancyGrid& msg) { onMap(msg); });

        rclcpp::QoS marker_qos{rclcpp::KeepLast(1)};
        marker_qos.reliable().transient_local();
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            marker_topic_, marker_qos);
        goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            goal_marker_topic_, marker_qos);
        event_pub_ =
            create_publisher<std_msgs::msg::String>(planner_event_topic_, 10);
        event_sub_ = create_subscription<std_msgs::msg::String>(
            planner_event_topic_, 10,
            [this](const std_msgs::msg::String& msg) { onPlannerEvent(msg); });
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
        nav2_success_xy_tolerance_m_ = node_params.nav2_success_xy_tolerance_m;

        auto config = core_.config();
        loadDeclaredParametersInto(*this, config);
        core_.setConfig(config);

        const auto nav2_params =
            loadDeclaredParameters<Nav2ValidationParams>(*this);
        nav2_validation_enabled_ = nav2_params.nav2_validation_enabled;
        nav2_validation_timeout_sec_ = nav2_params.nav2_validation_timeout_sec;
        nav2_compute_path_action_ = nav2_params.nav2_compute_path_action;
        nav2_validation_blacklist_enabled_ =
            nav2_params.nav2_validation_blacklist_enabled;
        nav2_validation_rejected_retry_max_ =
            nav2_params.nav2_validation_rejected_retry_max;
        nav2_validation_rejected_retry_delay_sec_ =
            nav2_params.nav2_validation_rejected_retry_delay_sec;
        compute_path_client_ = rclcpp_action::create_client<ComputePathToPose>(
            this, nav2_compute_path_action_);

        backtrace_config_ =
            loadDeclaredParameters<BacktraceRecoveryConfig>(*this);
        breadcrumb_trail_.setConfig(backtrace_config_);
        backtrace_planner_.setConfig(backtrace_config_);
        backtrace_backup_action_ = backtrace_config_.backup_action;

        recovery_config_ =
            loadDeclaredParameters<RecoveryStateMachineConfig>(*this);
        recovery_.setConfig(recovery_config_);
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
        if (alc_gate_.paused()) {
            logWaiting("Frontier exploration paused for ALC");
            return;
        }
        if (recoveryActive()) {
            checkRecoveryStateMachine();
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
        findAndBeginCandidateRoute(*robot_pose, false);
    }

    bool findAndBeginCandidateRoute(const RobotPose& robot_pose,
                                    const bool bypass_goal_suppression) {
        auto result =
            core_.findCandidates(robot_pose.xy, nowSec(), robot_pose.yaw);
        publishFrontierMarkers(result.candidates);
        if (result.candidates.empty()) {
            clearGoalMarker();
            publishNoCandidateEvent(result.counts, result.relaxed);
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
                std::to_string(result.counts.rejected_unknown_ratio) +
                ", rej_clearance=" +
                std::to_string(result.counts.rejected_clearance) +
                ", rej_nav2_clear=" +
                std::to_string(result.counts.rejected_nav2_clearance) +
                ", coverage=" +
                std::to_string(result.counts.coverage_candidates) + ")");
            return false;
        }
        if (result.relaxed) {
            publishPlannerEvent(
                "frontier_search_relaxed",
                nlohmann::json{{"candidates", result.candidates.size()}});
        }
        std::sort(
            result.candidates.begin(), result.candidates.end(),
            [](const auto& a, const auto& b) { return a.score < b.score; });
        const auto best = result.candidates.begin();
        if (!bypass_goal_suppression &&
            !core_.shouldPublishGoal(best->goal_xy, nowSec())) {
            publishCandidateEvent("goal_suppressed", *best);
            return false;
        }
        beginRouteSelection(robot_pose, result.candidates);
        return true;
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
        if (final_candidate.soft_reachable) {
            route.push_back({final_candidate.approach_xy,
                             final_candidate.approach_yaw, true});
        }
        else {
            route.push_back(
                {final_candidate.goal_xy, final_candidate.goal_yaw, true});
        }
        return route;
    }

    void beginRouteSelection(
        const RobotPose& robot_pose,
        const std::vector<FrontierCandidate>& sorted_candidates) {
        pending_robot_pose_ = robot_pose;
        pending_candidates_ = sorted_candidates;
        pending_candidate_index_ = 0;
        validateNextCandidateRoute();
    }

    void validateNextCandidateRoute() {
        if (alc_gate_.paused()) {
            clearPendingRouteSelection();
            return;
        }
        validation_rejection_retry_timer_.reset();
        validation_rejection_count_ = 0;
        validation_active_ = false;
        validation_route_.clear();
        validation_final_candidate_.reset();
        validation_segment_index_ = 0;

        while (pending_candidate_index_ < pending_candidates_.size()) {
            const auto& candidate =
                pending_candidates_[pending_candidate_index_];
            ++pending_candidate_index_;
            if (!core_.shouldPublishGoal(candidate.goal_xy, nowSec())) {
                publishCandidateEvent("goal_suppressed", candidate);
                continue;
            }
            publishCandidateEvent("best_candidate", candidate);
            validation_final_candidate_ = candidate;
            validation_route_ = buildRoute(candidate, pending_candidates_);
            publishGoalMarker(candidate.goal_xy, candidate.goal_yaw);
            if (!nav2_validation_enabled_) {
                const auto pending_robot_pose = pending_robot_pose_;
                const auto validation_final_candidate =
                    validation_final_candidate_;
                if (!pending_robot_pose || !validation_final_candidate) {
                    clearPendingRouteSelection();
                    return;
                }
                startRoute(pending_robot_pose->xy, *validation_final_candidate,
                           validation_route_);
                return;
            }
            validation_active_ = true;
            ++validation_generation_;
            validateRouteSegment(validation_generation_);
            return;
        }

        publishPlannerEvent("nav2_validation_failed",
                            {{"reason", "all_candidate_routes_failed"},
                             {"candidates_tried", pending_candidates_.size()}});
        clearGoalMarker();
        pending_candidates_.clear();
        pending_robot_pose_.reset();
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
            !pending_robot_pose_ || !validation_final_candidate_ ||
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
                ? pending_robot_pose_->xy
                : validation_route_[validation_segment_index_ - 1].xy;
        const double start_yaw =
            validation_segment_index_ == 0
                ? pending_robot_pose_->yaw
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
            ++validation_rejection_count_;
            if (nav2_validation_rejected_retry_max_ > 0 &&
                validation_rejection_count_ <=
                    nav2_validation_rejected_retry_max_) {
                publishPlannerEvent("nav2_validation_failed",
                                    {{"reason", "goal_rejected_retrying"},
                                     {"retry", validation_rejection_count_},
                                     {"goal_xy", validationGoalJson()}});
                const auto delay_ms = static_cast<int>(
                    nav2_validation_rejected_retry_delay_sec_ * 1000.0);
                validation_segment_started_sec_ = nowSec();
                validation_rejection_retry_timer_ = create_wall_timer(
                    std::chrono::milliseconds(delay_ms), [this, generation]() {
                        validation_rejection_retry_timer_.reset();
                        if (validation_active_ &&
                            generation == validation_generation_) {
                            validateRouteSegment(generation);
                        }
                    });
                return;
            }
            publishPlannerEvent("nav2_validation_failed",
                                {{"reason", "goal_rejected"},
                                 {"goal_xy", validationGoalJson()}});
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
                                 {"failure_class", "global_path_blocked"},
                                 {"status", static_cast<int>(result.code)},
                                 {"segment", validation_segment_index_},
                                 {"goal_xy", validationGoalJson()},
                                 {"waypoint_xy", validationWaypointJson()}});
            recordValidationFailure(FailureClass::kGlobalPathBlocked);
            validateNextCandidateRoute();
            return;
        }

        ++validation_segment_index_;
        if (validation_segment_index_ < validation_route_.size()) {
            validateRouteSegment(generation);
            return;
        }

        validation_active_ = false;
        if (pending_robot_pose_ && validation_final_candidate_) {
            startRoute(pending_robot_pose_->xy, *validation_final_candidate_,
                       validation_route_);
        }
    }

    void checkValidationTimeout() {
        if (!validation_active_ || nav2_validation_timeout_sec_ <= 0.0) {
            return;
        }
        if (validation_rejection_retry_timer_) {
            return;
        }
        if (nowSec() - validation_segment_started_sec_ <
            nav2_validation_timeout_sec_) {
            return;
        }
        publishPlannerEvent("nav2_validation_failed",
                            {{"reason", "timeout"},
                             {"failure_class", "global_path_blocked"},
                             {"segment", validation_segment_index_},
                             {"goal_xy", validationGoalJson()},
                             {"waypoint_xy", validationWaypointJson()}});
        recordValidationFailure(FailureClass::kGlobalPathBlocked);
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
        rear_latch_progress_arc_length_ = last_progress_arc_length_;
        last_progress_time_sec_ = nowSec();
        active_goal_sent_time_sec_ = nowSec();
        active_waypoint_sent_time_sec_ = nowSec();
        last_alignment_robot_yaw_.reset();
        active_goal_hysteresis_retries_ = 0;
        alignment_spin_attempts_ = 0;
        breadcrumb_trail_.clear();
        breadcrumb_trail_.add({robot_xy, 0.0, nowSec()});
        backtrace_planner_.reset();
        consecutive_reverse_blocked_ = 0;
        spurious_success_resend_streak_ = 0;
        recovery_escape_active_ = false;
        core_.noteGoalPublished(final_candidate.goal_xy,
                                final_candidate.goal_yaw, nowSec(),
                                final_candidate.path_length);
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
        publishPlannerEvent(waypoint.final ? "goal_sent" : "subgoal_sent",
                            {{"goal_xy", pointJson(waypoint.xy)},
                             {"goal_yaw", rounded(waypoint.yaw)},
                             {"index", active_route_index_}});
        sendGoal(waypoint.xy, waypoint.yaw);
    }

    std::optional<RobotPose> lookupRobotPose() {
        try {
            const auto transform = tf_buffer_.lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero);
            const auto& t = transform.transform.translation;
            return RobotPose{WorldPoint{t.x, t.y},
                             yawFromQuaternion(transform.transform.rotation)};
        }
        catch (const tf2::TransformException& exc) {
            RCLCPP_DEBUG(get_logger(), "Waiting for robot pose TF: %s",
                         exc.what());
            return std::nullopt;
        }
    }

    void sendGoal(const WorldPoint& goal_xy, const double goal_yaw) {
        if (alc_gate_.paused()) {
            return;
        }
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

        const int generation = ++nav_goal_generation_;
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.goal_response_callback =
            [this, generation](const GoalHandle::SharedPtr& handle) {
                onGoalResponse(generation, handle);
            };
        options.result_callback =
            [this, generation](const GoalHandle::WrappedResult& result) {
                onGoalResult(generation, result);
            };
        nav_client_->async_send_goal(goal, options);
    }

    void onGoalResponse(const int generation,
                        const GoalHandle::SharedPtr& handle) {
        if (generation != nav_goal_generation_) {
            if (handle) {
                nav_client_->async_cancel_goal(handle);
            }
            return;
        }
        if (!goal_active_) {
            if (handle) {
                nav_client_->async_cancel_goal(handle);
            }
            return;
        }
        if (!handle) {
            RCLCPP_WARN(get_logger(), "Cluster frontier goal rejected");
            publishPlannerEvent("goal_rejected",
                                {{"goal_xy", activeGoalJson()}});
            abandonActiveGoal("goal_rejected",
                              FailureClass::kGlobalPathBlocked);
            return;
        }
        goal_handle_ = handle;
    }

    void onGoalResult(const int generation,
                      const GoalHandle::WrappedResult& result) {
        if (generation != nav_goal_generation_ || !goal_active_ ||
            spin_recovery_active_) {
            return;
        }
        const int status = static_cast<int>(result.code);
        RCLCPP_INFO(get_logger(),
                    "Cluster frontier goal finished with status %d", status);
        publishPlannerEvent(
            "goal_result", {{"status", status}, {"goal_xy", activeGoalJson()}});
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            startRecoveryTree("nav2_result_failed");
            return;
        }

        const auto robot_pose = lookupRobotPose();
        if (!robot_pose) {
            startRecoveryTree("missing_pose_after_success");
            return;
        }
        const double goal_distance =
            active_goal_ ? distanceBetween(robot_pose->xy, *active_goal_) : 0.0;
        if (active_goal_ &&
            !withinDistanceTolerance(robot_pose->xy, *active_goal_,
                                     nav2_success_xy_tolerance_m_)) {
            const double elapsed = nowSec() - active_waypoint_sent_time_sec_;
            publishPlannerEvent(
                "nav2_success_pose_mismatch",
                {{"goal_xy", activeGoalJson()},
                 {"distance_m", rounded(goal_distance)},
                 {"tolerance_m", rounded(nav2_success_xy_tolerance_m_)},
                 {"elapsed_sec", rounded(elapsed)},
                 {"resend_streak", spurious_success_resend_streak_}});
            if (spurious_success_resend_streak_ < kSpuriousSuccessResendMax) {
                ++spurious_success_resend_streak_;
                publishPlannerEvent(
                    "nav2_spurious_success_resend",
                    {{"goal_xy", activeGoalJson()},
                     {"resend_streak", spurious_success_resend_streak_},
                     {"elapsed_sec", rounded(elapsed)}});
                goal_handle_.reset();
                goal_active_ = false;
                last_progress_time_sec_ = nowSec();
                sendActiveWaypoint();
                return;
            }
            spurious_success_resend_streak_ = 0;
            startRecoveryTree("nav2_success_pose_mismatch");
            return;
        }
        spurious_success_resend_streak_ = 0;

        if (active_route_index_ + 1 < active_route_.size()) {
            publishPlannerEvent("subgoal_reached",
                                {{"goal_xy", activeGoalJson()},
                                 {"index", active_route_index_}});
            goal_handle_.reset();
            goal_active_ = false;
            ++active_route_index_;
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

        const double route_duration = nowSec() - active_goal_sent_time_sec_;
        if (route_duration < kTrivialSuccessThresholdSec) {
            // Robot was already within nav2's tolerance of the goal — it never
            // actually moved.  The frontier won't self-clear, so blacklist it
            // rather than marking it completed, otherwise the same cell gets
            // selected every goal_publish_period seconds indefinitely.
            publishPlannerEvent("route_trivial_success",
                                {{"goal_xy", activeFinalGoalJson()},
                                 {"failure_class", "global_path_blocked"},
                                 {"duration_sec", rounded(route_duration)}});
            blacklistActiveGoal(FailureClass::kGlobalPathBlocked);
            consecutive_immediate_reverse_streak_ = 0;
            consecutive_backtrace_stall_count_ = 0;
            clearActiveGoal();
            return;
        }
        publishPlannerEvent("route_completed",
                            {{"goal_xy", activeFinalGoalJson()}});
        consecutive_immediate_reverse_streak_ = 0;
        consecutive_backtrace_stall_count_ = 0;
        clearActiveGoal();
    }

    void checkActiveGoalProgress(const RobotPose& robot_pose) {
        const double now = nowSec();
        breadcrumb_trail_.add({robot_pose.xy, robot_pose.yaw, now});
        if (spin_recovery_active_) {
            checkSpinRecoveryTimeout(now);
            return;
        }
        const auto& config = core_.config();
        if (config.goal_timeout_sec > 0.0 &&
            now - active_goal_sent_time_sec_ >= config.goal_timeout_sec) {
            RCLCPP_WARN(get_logger(), "Abandoning frontier goal after timeout");
            abandonActiveGoal("timeout", FailureClass::kControllerNoProgress);
            return;
        }
        const auto projection =
            core_.projectOntoPath(robot_pose.xy, active_route_path_);
        if (projection && recovery_.rearBlockedLatchActive()) {
            const double latch_progress_delta =
                projection->arc_length - rear_latch_progress_arc_length_;
            if (latch_progress_delta > 0.0 &&
                recovery_.noteForwardDisplacement(latch_progress_delta, now)) {
                publishPlannerEvent(
                    "rear_blocked_cleared",
                    {{"stage", recoveryStageName(recovery_.stage())},
                     {"reason", "forward_displacement"},
                     {"failure_class", "none"},
                     {"distance_m", rounded(latch_progress_delta)},
                     {"goal_xy", activeFinalGoalJson()},
                     {"active_waypoint_xy", activeGoalJson()},
                     {"robot_xy", pointJson(robot_pose.xy)}});
            }
            if (latch_progress_delta > 0.0) {
                rear_latch_progress_arc_length_ = projection->arc_length;
            }
        }
        if (projection && projection->arc_length - last_progress_arc_length_ >=
                              config.stuck_progress_radius_m) {
            last_progress_arc_length_ = projection->arc_length;
            last_progress_time_sec_ = now;
            last_alignment_robot_yaw_.reset();
            consecutive_reverse_blocked_ = 0;
            RCLCPP_DEBUG(get_logger(), "Route progress arc=%.3f",
                         last_progress_arc_length_);
            return;
        }
        if (config.stuck_progress_timeout_sec > 0.0 &&
            now - last_progress_time_sec_ >=
                config.stuck_progress_timeout_sec) {
            const auto alignment = evaluateActiveGoalAlignment(robot_pose, now);
            if (tryAlignmentProgressGrace(robot_pose, now, alignment)) {
                last_alignment_robot_yaw_ = robot_pose.yaw;
                return;
            }
            if (tryNearGoalHysteresis(robot_pose.xy, now)) {
                last_alignment_robot_yaw_ = robot_pose.yaw;
                return;
            }
            if (trySpinAlignmentRecovery(robot_pose, now, alignment)) {
                return;
            }
            RCLCPP_WARN(get_logger(),
                        "Abandoning frontier goal after no progress");
            startRecoveryTree("no_progress");
            return;
        }
        last_alignment_robot_yaw_ = robot_pose.yaw;
    }

    AlignmentGraceStatus evaluateActiveGoalAlignment(
        const RobotPose& robot_pose, const double now) const {
        if (!active_goal_) {
            return {};
        }
        return core_.evaluateAlignmentGrace(
            robot_pose.xy, robot_pose.yaw, last_alignment_robot_yaw_,
            active_route_path_, *active_goal_,
            now - active_waypoint_sent_time_sec_);
    }

    bool tryAlignmentProgressGrace(const RobotPose& robot_pose,
                                   const double now,
                                   const AlignmentGraceStatus& status) {
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

    bool trySpinAlignmentRecovery(const RobotPose& robot_pose, const double now,
                                  const AlignmentGraceStatus& status) {
        const auto& config = core_.config();
        if (!config.alignment_spin_enabled || !active_goal_ ||
            alignment_spin_attempts_ >= config.alignment_spin_max_attempts) {
            return false;
        }
        if (status.heading_error <= config.alignment_heading_threshold_rad) {
            return false;
        }
        if (!spin_client_->action_server_is_ready()) {
            return false;
        }

        ++alignment_spin_attempts_;
        spin_recovery_active_ = true;
        spin_started_sec_ = now;
        if (goal_handle_) {
            nav_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
        }

        const double commanded_yaw =
            std::remainder(status.target_yaw - robot_pose.yaw, 2.0 * M_PI);
        Spin::Goal spin_goal;
        spin_goal.target_yaw = static_cast<float>(commanded_yaw);
        spin_goal.time_allowance = rclcpp::Duration::from_seconds(
            config.alignment_spin_time_allowance_sec);
        publishPlannerEvent("alignment_spin_recovery",
                            {{"goal_xy", activeFinalGoalJson()},
                             {"active_waypoint_xy", activeGoalJson()},
                             {"heading_error", rounded(status.heading_error)},
                             {"commanded_yaw", rounded(commanded_yaw)},
                             {"attempt", alignment_spin_attempts_}});
        RCLCPP_WARN(get_logger(),
                    "Spinning %.2f rad to recover alignment before retrying "
                    "frontier waypoint",
                    commanded_yaw);

        rclcpp_action::Client<Spin>::SendGoalOptions options;
        options.goal_response_callback =
            [this](const SpinGoalHandle::SharedPtr& handle) {
                onSpinResponse(handle);
            };
        options.result_callback =
            [this](const SpinGoalHandle::WrappedResult& result) {
                onSpinResult(result);
            };
        spin_client_->async_send_goal(spin_goal, options);
        return true;
    }

    void onSpinResponse(const SpinGoalHandle::SharedPtr& handle) {
        if (recovery_spin_active_) {
            if (!handle) {
                finishRecoverySpin("spin_rejected",
                                   FailureClass::kRecoveryFailed);
                return;
            }
            spin_goal_handle_ = handle;
            return;
        }
        if (!spin_recovery_active_) {
            if (handle) {
                spin_client_->async_cancel_goal(handle);
            }
            return;
        }
        if (!handle) {
            finishSpinRecovery("spin_rejected");
            return;
        }
        spin_goal_handle_ = handle;
    }

    void onSpinResult(const SpinGoalHandle::WrappedResult& result) {
        if (recovery_spin_active_) {
            finishRecoverySpin(
                result.code == rclcpp_action::ResultCode::SUCCEEDED
                    ? "spin_completed"
                    : "spin_failed",
                result.code == rclcpp_action::ResultCode::SUCCEEDED
                    ? FailureClass::kNone
                    : FailureClass::kRecoveryFailed);
            return;
        }
        if (!spin_recovery_active_) {
            return;
        }
        finishSpinRecovery(result.code == rclcpp_action::ResultCode::SUCCEEDED
                               ? "spin_completed"
                               : "spin_failed");
    }

    void checkSpinRecoveryTimeout(const double now) {
        const auto& config = core_.config();
        if (now - spin_started_sec_ < config.alignment_spin_time_allowance_sec +
                                          kSpinTransportMarginSec) {
            return;
        }
        if (spin_goal_handle_) {
            spin_client_->async_cancel_goal(spin_goal_handle_);
        }
        finishSpinRecovery("spin_timeout");
    }

    void finishSpinRecovery(const std::string& reason) {
        const double now = nowSec();
        const double spin_elapsed = std::max(0.0, now - spin_started_sec_);
        spin_recovery_active_ = false;
        spin_goal_handle_.reset();
        spin_started_sec_ = 0.0;
        publishPlannerEvent("alignment_spin_recovery_finished",
                            {{"reason", reason},
                             {"elapsed", rounded(spin_elapsed)},
                             {"goal_xy", activeFinalGoalJson()},
                             {"active_waypoint_xy", activeGoalJson()}});
        if (!goal_active_ || active_route_index_ >= active_route_.size()) {
            return;
        }
        // The spin should not eat into route progress budgets.
        active_goal_sent_time_sec_ += spin_elapsed;
        last_progress_time_sec_ = now;
        last_alignment_robot_yaw_.reset();
        sendActiveWaypoint();
    }

    bool isNearActiveGoal(const WorldPoint& robot_xy,
                          const double radius) const {
        const auto near = [&](const std::optional<WorldPoint>& goal) {
            return goal && withinDistanceTolerance(robot_xy, *goal, radius);
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

    bool recoveryActive() const {
        return recovery_.stage() != RecoveryStage::kNone;
    }

    nlohmann::json recoveryEventFields(
        const std::string& reason = "",
        const FailureClass failure_class = FailureClass::kNone) {
        nlohmann::json fields{
            {"stage", recoveryStageName(recovery_.stage())},
            {"reason", reason.empty() ? recovery_reason_ : reason},
            {"failure_class", failureClassName(failure_class)},
            {"goal_xy", activeFinalGoalJson()},
            {"active_waypoint_xy", activeGoalJson()}};
        if (const auto pose = lookupRobotPose()) {
            fields["robot_xy"] = pointJson(pose->xy);
        }
        return fields;
    }

    void resetRecoveryStateOnly() {
        recovery_.reset();
        recovery_spin_active_ = false;
        recovery_clearance_backup_active_ = false;
        pending_recovery_spin_yaw_ = 0.0;
        recovery_reason_.clear();
        recovery_start_xy_.reset();
    }

    void startRecoveryTree(const std::string& reason) {
        if (recoveryActive() || recovery_escape_active_) {
            return;
        }
        recovery_reason_ = reason;
        spurious_success_resend_streak_ = 0;
        recovery_start_xy_.reset();
        if (const auto pose = lookupRobotPose()) {
            recovery_start_xy_ = pose->xy;
        }
        // Reobserve finding a candidate resets the whole tree, so the spin
        // counter never accumulates and the reverse-pulse/backtrace stage is
        // unreachable through normal stage progression.  Escalate directly
        // when consecutive trees keep starting from the same spot.
        bool escalate_to_backtrace = false;
        if (recovery_start_xy_) {
            escalate_to_backtrace = recovery_.noteTreeStartedAndCheckEscalation(
                recovery_start_xy_->first, recovery_start_xy_->second);
        }
        if (escalate_to_backtrace && backtrace_config_.enabled) {
            recovery_.advanceToReversePulse();
            publishPlannerEvent(
                "recovery_stagnation_escalation",
                {{"reason", reason},
                 {"trees",
                  recovery_config_.recovery_stagnation_escalation_trees},
                 {"radius_m",
                  rounded(recovery_config_.recovery_stagnation_radius_m)},
                 {"goal_xy", activeFinalGoalJson()}});
        }
        else if (recovery_reobserve_used_) {
            recovery_.startAtSmallRotation();
        }
        else {
            recovery_.start();
        }
        ++nav_goal_generation_;
        if (goal_handle_) {
            nav_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
        }
        goal_active_ = false;
        publishPlannerEvent("recovery_tree_started",
                            recoveryEventFields(reason));
        publishPlannerEvent("recovery_stage_started",
                            recoveryEventFields(reason));
    }

    void checkRecoveryStateMachine() {
        switch (recovery_.stage()) {
            case RecoveryStage::kNone:
                return;
            case RecoveryStage::kReobserve:
                checkRecoveryReobserve();
                return;
            case RecoveryStage::kSmallRotation:
                checkRecoverySmallRotation();
                return;
            case RecoveryStage::kForwardArc:
                checkRecoveryForwardArc();
                return;
            case RecoveryStage::kReversePulse:
                checkRecoveryReversePulse();
                return;
            case RecoveryStage::kHardFailed:
                finishRecoveryHardFail("recovery_exhausted",
                                       FailureClass::kRecoveryFailed);
                return;
        }
    }

    void checkRecoveryReobserve() {
        if (!recovery_.tickReobserve()) {
            return;
        }
        publishPlannerEvent("recovery_stage_finished",
                            recoveryEventFields("reobserve_complete"));
        const auto robot_pose = lookupRobotPose();
        if (robot_pose && findAndBeginCandidateRoute(*robot_pose, true)) {
            recovery_reobserve_used_ = true;
            resetRecoveryStateOnly();
            return;
        }
        checkRecoverySmallRotation();
    }

    void checkRecoverySmallRotation() {
        if (recovery_spin_active_) {
            checkRecoverySpinTimeout(nowSec());
            return;
        }
        if (recovery_clearance_backup_active_) {
            return;
        }
        const auto yaw = recovery_.nextSmallRotationYaw();
        if (!yaw) {
            publishPlannerEvent(
                "recovery_stage_finished",
                recoveryEventFields("small_rotation_exhausted"));
            recovery_.advanceToForwardArc();
            publishPlannerEvent("recovery_stage_started",
                                recoveryEventFields("forward_arc"));
            return;
        }
        pending_recovery_spin_yaw_ = *yaw;
        sendRecoveryClearanceBackup(*yaw);
    }

    void sendRecoveryClearanceBackup(const double spin_yaw) {
        const double distance = recovery_config_.recovery_clearance_backup_m;
        if (distance <= 0.0 || !backup_client_->action_server_is_ready()) {
            sendRecoverySpin(spin_yaw);
            return;
        }
        recovery_clearance_backup_active_ = true;

        BackUp::Goal goal;
        goal.target.x = -distance;
        goal.speed = static_cast<float>(backtrace_config_.backup_speed_mps);
        goal.time_allowance = rclcpp::Duration::from_seconds(
            backtrace_config_.backup_time_allowance_sec);

        publishPlannerEvent(
            "recovery_clearance_backup_started",
            {{"stage", recoveryStageName(RecoveryStage::kSmallRotation)},
             {"distance_m", rounded(distance)},
             {"spin_yaw", rounded(spin_yaw)},
             {"goal_xy", activeFinalGoalJson()}});

        rclcpp_action::Client<BackUp>::SendGoalOptions options;
        options.goal_response_callback =
            [this](const BackUpGoalHandle::SharedPtr& handle) {
                onClearanceBackupGoalResponse(handle);
            };
        options.result_callback =
            [this](const BackUpGoalHandle::WrappedResult& result) {
                onClearanceBackupResult(result);
            };
        clearance_backup_goal_handle_.reset();
        backup_client_->async_send_goal(goal, options);
    }

    void onClearanceBackupGoalResponse(
        const BackUpGoalHandle::SharedPtr& handle) {
        if (!recovery_clearance_backup_active_) {
            if (handle) {
                backup_client_->async_cancel_goal(handle);
            }
            return;
        }
        if (!handle) {
            recovery_clearance_backup_active_ = false;
            sendRecoverySpin(pending_recovery_spin_yaw_);
            return;
        }
        clearance_backup_goal_handle_ = handle;
    }

    void onClearanceBackupResult(
        const BackUpGoalHandle::WrappedResult& result) {
        if (!recovery_clearance_backup_active_) {
            return;
        }
        clearance_backup_goal_handle_.reset();
        recovery_clearance_backup_active_ = false;
        const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        publishPlannerEvent(
            "recovery_clearance_backup_finished",
            {{"stage", recoveryStageName(RecoveryStage::kSmallRotation)},
             {"success", ok},
             {"goal_xy", activeFinalGoalJson()}});
        sendRecoverySpin(pending_recovery_spin_yaw_);
    }

    void sendRecoverySpin(const double yaw) {
        if (!spin_client_->action_server_is_ready()) {
            publishPlannerEvent(
                "recovery_stage_finished",
                recoveryEventFields("spin_unavailable",
                                    FailureClass::kRecoveryFailed));
            recovery_.advanceToForwardArc();
            publishPlannerEvent("recovery_stage_started",
                                recoveryEventFields("forward_arc"));
            return;
        }
        recovery_spin_active_ = true;
        spin_started_sec_ = nowSec();
        if (goal_handle_) {
            nav_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
        }

        Spin::Goal spin_goal;
        spin_goal.target_yaw = static_cast<float>(yaw);
        spin_goal.time_allowance = rclcpp::Duration::from_seconds(
            core_.config().alignment_spin_time_allowance_sec);
        publishPlannerEvent(
            "recovery_stage_started",
            {{"stage", recoveryStageName(RecoveryStage::kSmallRotation)},
             {"reason", recovery_reason_},
             {"failure_class", "none"},
             {"commanded_yaw", rounded(yaw)},
             {"attempt", recovery_.spinAttempts()},
             {"goal_xy", activeFinalGoalJson()},
             {"active_waypoint_xy", activeGoalJson()}});

        rclcpp_action::Client<Spin>::SendGoalOptions options;
        options.goal_response_callback =
            [this](const SpinGoalHandle::SharedPtr& handle) {
                onSpinResponse(handle);
            };
        options.result_callback =
            [this](const SpinGoalHandle::WrappedResult& result) {
                onSpinResult(result);
            };
        spin_client_->async_send_goal(spin_goal, options);
    }

    void checkRecoverySpinTimeout(const double now) {
        if (now - spin_started_sec_ <
            core_.config().alignment_spin_time_allowance_sec +
                kSpinTransportMarginSec) {
            return;
        }
        if (spin_goal_handle_) {
            spin_client_->async_cancel_goal(spin_goal_handle_);
        }
        finishRecoverySpin("spin_timeout", FailureClass::kRecoveryFailed);
    }

    void finishRecoverySpin(const std::string& reason, const FailureClass fc) {
        const double elapsed = std::max(0.0, nowSec() - spin_started_sec_);
        recovery_spin_active_ = false;
        spin_goal_handle_.reset();
        spin_started_sec_ = 0.0;
        publishPlannerEvent(
            "recovery_stage_finished",
            {{"stage", recoveryStageName(RecoveryStage::kSmallRotation)},
             {"reason", reason},
             {"failure_class", failureClassName(fc)},
             {"elapsed", rounded(elapsed)},
             {"goal_xy", activeFinalGoalJson()},
             {"active_waypoint_xy", activeGoalJson()}});
        recovery_.beginReobserve();
        publishPlannerEvent("recovery_stage_started",
                            recoveryEventFields("post_spin_reobserve"));
    }

    void checkRecoveryForwardArc() {
        const auto robot_pose = lookupRobotPose();
        if (!robot_pose) {
            recovery_.advanceToReversePulse();
            publishPlannerEvent("recovery_stage_started",
                                recoveryEventFields("reverse_pulse"));
            return;
        }
        if (triggerForwardEscape(*robot_pose)) {
            resetRecoveryStateOnly();
            return;
        }
        publishPlannerEvent("recovery_stage_finished",
                            recoveryEventFields("forward_arc_blocked",
                                                FailureClass::kRecoveryFailed));
        recovery_.advanceToReversePulse();
        publishPlannerEvent("recovery_stage_started",
                            recoveryEventFields("reverse_pulse"));
    }

    void checkRecoveryReversePulse() {
        if (backtrace_recovery_active_) {
            checkBacktraceRecovery();
            return;
        }
        const double now = nowSec();
        const bool recent_reverse_blocked =
            last_reverse_blocked_sec_ > 0.0 &&
            now - last_reverse_blocked_sec_ <= kRecentReverseBlockedWindowSec;
        if (recovery_.rearBlockedLatchActive() || recent_reverse_blocked) {
            publishPlannerEvent(
                "reverse_preflight_failed",
                recoveryEventFields(recovery_.rearBlockedLatchActive()
                                        ? "rear_blocked_latched"
                                        : "recent_reverse_blocked",
                                    FailureClass::kReverseUnsafe));
            finishRecoveryHardFail("reverse_preflight_failed",
                                   FailureClass::kReverseUnsafe);
            return;
        }
        startBacktraceRecovery(recovery_reason_);
    }

    void finishRecoveryHardFail(const std::string& reason,
                                const FailureClass fc) {
        recovery_.hardFail();
        publishPlannerEvent("recovery_hard_failed",
                            recoveryEventFields(reason, fc));
        markActiveGoalUnsafe(fc == FailureClass::kReverseUnsafe
                                 ? "REVERSE_UNSAFE"
                                 : "STALL_OR_SLIP",
                             fc);
        clearActiveGoal();
    }

    void latchRearBlocked(const std::string& reason) {
        recovery_.latchRearBlocked(nowSec());
        rear_latch_progress_arc_length_ = last_progress_arc_length_;
        publishPlannerEvent("rear_blocked_latched",
                            {{"stage", recoveryStageName(recovery_.stage())},
                             {"reason", reason},
                             {"failure_class", "reverse_unsafe"},
                             {"goal_xy", activeFinalGoalJson()},
                             {"active_waypoint_xy", activeGoalJson()}});
    }

    void startBacktraceRecovery(const std::string& reason) {
        if (backtrace_recovery_active_) {
            return;
        }
        // Rear persistently blocked: skip futile backup attempts that only
        // spin wheels in place and pollute odometry / SLAM.
        if (consecutive_backtrace_stall_count_ >=
            kBacktraceStallSkipThreshold) {
            publishPlannerEvent(
                "backtrace_skipped_stall",
                {{"stall_count", consecutive_backtrace_stall_count_},
                 {"reason", reason},
                 {"goal_xy", activeFinalGoalJson()}});
            consecutive_backtrace_stall_count_ = 0;
            abandonActiveGoal(reason, FailureClass::kRecoveryFailed);
            return;
        }
        if (!backtrace_planner_.canStart(breadcrumb_trail_) ||
            !backup_client_->action_server_is_ready()) {
            abandonActiveGoal(reason, FailureClass::kRecoveryFailed);
            return;
        }

        backtrace_recovery_active_ = true;
        backtrace_waiting_for_cancel_ = true;
        backtrace_backup_active_ = false;
        backtrace_reason_ = reason;
        backtrace_cancel_started_sec_ = nowSec();
        ++nav_goal_generation_;
        if (goal_handle_) {
            nav_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
        }
        goal_active_ = false;
        publishPlannerEvent(
            "backtrace_recovery_started",
            {{"reason", reason},
             {"goal_xy", activeFinalGoalJson()},
             {"active_waypoint_xy", activeGoalJson()},
             {"breadcrumbs", breadcrumb_trail_.poses().size()}});
    }

    void checkBacktraceRecovery() {
        if (!backtrace_recovery_active_) {
            return;
        }
        if (backtrace_backup_active_) {
            // Detect wheel-spin stall: if the robot hasn't moved
            // min_actual_displacement_m within backup_stall_check_sec of
            // starting the segment, cancel and fail fast rather than waiting
            // the full backup_time_allowance_sec.
            const double stall_check = backtrace_config_.backup_stall_check_sec;
            if (stall_check > 0.0 &&
                nowSec() - backtrace_segment_start_time_sec_ >= stall_check) {
                const auto robot_pose = lookupRobotPose();
                const auto& start_xy =
                    backtrace_planner_.state().segment_start_xy;
                if (robot_pose && start_xy &&
                    withinDistanceTolerance(
                        robot_pose->xy, *start_xy,
                        backtrace_config_.min_actual_displacement_m)) {
                    if (backup_goal_handle_) {
                        backup_client_->async_cancel_goal(backup_goal_handle_);
                        backup_goal_handle_.reset();
                    }
                    backtrace_backup_active_ = false;
                    latchRearBlocked("backup_stalled");
                    finishBacktraceRecovery(false, "backup_stalled",
                                            FailureClass::kLocalHazard);
                    return;
                }
            }
            return;
        }
        if (backtrace_waiting_for_cancel_) {
            if (nowSec() - backtrace_cancel_started_sec_ <
                backtrace_config_.cancel_timeout_sec) {
                return;
            }
            backtrace_waiting_for_cancel_ = false;
        }
        sendNextBacktraceSegment();
    }

    void sendNextBacktraceSegment() {
        const auto robot_pose = lookupRobotPose();
        if (!robot_pose) {
            finishBacktraceRecovery(false, "missing_pose",
                                    FailureClass::kRecoveryFailed);
            return;
        }
        const auto distance = backtrace_planner_.nextBackupDistance(
            breadcrumb_trail_, robot_pose->xy);
        if (!distance) {
            finishBacktraceRecovery(true, "complete", FailureClass::kNone);
            return;
        }
        if (!backtrace_planner_.noteSegmentStarted(robot_pose->xy, *distance)) {
            finishBacktraceRecovery(true, "segment_limit", FailureClass::kNone);
            return;
        }

        BackUp::Goal goal;
        goal.target.x = -*distance;
        goal.speed = static_cast<float>(backtrace_config_.backup_speed_mps);
        goal.time_allowance = rclcpp::Duration::from_seconds(
            backtrace_config_.backup_time_allowance_sec);
        backtrace_backup_active_ = true;
        backtrace_segment_start_time_sec_ = nowSec();
        publishPlannerEvent(
            "backtrace_recovery_segment",
            {{"segment", backtrace_planner_.state().segments_attempted},
             {"distance_m", rounded(*distance)},
             {"speed_mps", rounded(backtrace_config_.backup_speed_mps)}});

        rclcpp_action::Client<BackUp>::SendGoalOptions options;
        options.goal_response_callback =
            [this](const BackUpGoalHandle::SharedPtr& handle) {
                onBacktraceGoalResponse(handle);
            };
        options.result_callback =
            [this](const BackUpGoalHandle::WrappedResult& result) {
                onBacktraceResult(result);
            };
        backup_client_->async_send_goal(goal, options);
    }

    void onBacktraceGoalResponse(const BackUpGoalHandle::SharedPtr& handle) {
        if (!backtrace_recovery_active_) {
            if (handle) {
                backup_client_->async_cancel_goal(handle);
            }
            return;
        }
        if (!handle) {
            backtrace_backup_active_ = false;
            finishBacktraceRecovery(false, "backup_rejected",
                                    FailureClass::kRecoveryFailed);
            return;
        }
        backup_goal_handle_ = handle;
    }

    void onBacktraceResult(const BackUpGoalHandle::WrappedResult& result) {
        if (!backtrace_recovery_active_) {
            return;
        }
        backup_goal_handle_.reset();
        backtrace_backup_active_ = false;
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            finishBacktraceRecovery(false, "backup_failed",
                                    FailureClass::kRecoveryFailed);
            return;
        }
        const auto robot_pose = lookupRobotPose();
        if (!robot_pose) {
            finishBacktraceRecovery(false, "missing_pose_after_backup",
                                    FailureClass::kRecoveryFailed);
            return;
        }
        const auto segment_result =
            backtrace_planner_.noteSegmentFinished(robot_pose->xy);
        if (!segment_result.moved_enough) {
            publishPlannerEvent(
                "rear_slip_detected",
                {{"stage", recoveryStageName(recovery_.stage())},
                 {"reason", "insufficient_motion"},
                 {"failure_class", "stall_or_slip"},
                 {"actual_displacement_m",
                  rounded(segment_result.actual_displacement_m)},
                 {"commanded_m", rounded(segment_result.commanded_m)},
                 {"ratio", rounded(segment_result.ratio)},
                 {"goal_xy", activeFinalGoalJson()},
                 {"active_waypoint_xy", activeGoalJson()}});
            latchRearBlocked("insufficient_reverse_motion");
            finishBacktraceRecovery(false, "backup_stalled",
                                    FailureClass::kLocalHazard);
            return;
        }
        if (segment_result.ratio < backtrace_config_.reverse_motion_ratio_min) {
            publishPlannerEvent(
                "rear_slip_detected",
                {{"stage", recoveryStageName(recovery_.stage())},
                 {"reason", "weak_reverse_motion"},
                 {"failure_class", "stall_or_slip"},
                 {"actual_displacement_m",
                  rounded(segment_result.actual_displacement_m)},
                 {"commanded_m", rounded(segment_result.commanded_m)},
                 {"ratio", rounded(segment_result.ratio)},
                 {"ratio_min",
                  rounded(backtrace_config_.reverse_motion_ratio_min)},
                 {"goal_xy", activeFinalGoalJson()},
                 {"active_waypoint_xy", activeGoalJson()}});
            latchRearBlocked("weak_reverse_motion");
            finishBacktraceRecovery(false, "rear_slip_detected",
                                    FailureClass::kStallOrSlip);
            return;
        }
        sendNextBacktraceSegment();
    }

    void finishBacktraceRecovery(const bool success, const std::string& reason,
                                 const FailureClass fc) {
        publishPlannerEvent(
            success ? "backtrace_recovery_finished"
                    : "backtrace_recovery_failed",
            {{"reason", reason},
             {"failure_class", failureClassName(fc)},
             {"trigger", backtrace_reason_},
             {"segments", backtrace_planner_.state().segments_attempted},
             {"goal_xy", activeFinalGoalJson()},
             {"active_waypoint_xy", activeGoalJson()}});
        blacklistActiveGoal(success ? FailureClass::kControllerNoProgress : fc);
        if (success) {
            consecutive_immediate_reverse_streak_ = 0;
            consecutive_backtrace_stall_count_ = 0;
        }
        else if (fc == FailureClass::kLocalHazard ||
                 fc == FailureClass::kStallOrSlip) {
            ++consecutive_backtrace_stall_count_;
        }
        else {
            consecutive_backtrace_stall_count_ = 0;
        }
        backtrace_recovery_active_ = false;
        backtrace_waiting_for_cancel_ = false;
        backtrace_backup_active_ = false;
        backtrace_reason_.clear();
        backtrace_planner_.reset();
        clearActiveGoal();
    }

    void abandonActiveGoal(
        const std::string& reason,
        const FailureClass failure_class = FailureClass::kRecoveryFailed) {
        if (failure_class == FailureClass::kReverseUnsafe) {
            if (nowSec() - active_waypoint_sent_time_sec_ <
                kImmediateReverseBlockWindowSec) {
                ++consecutive_immediate_reverse_streak_;
            }
        }
        else {
            consecutive_immediate_reverse_streak_ = 0;
        }
        publishPlannerEvent("route_abandoned",
                            {{"reason", reason},
                             {"failure_class", failureClassName(failure_class)},
                             {"goal_xy", activeFinalGoalJson()},
                             {"active_waypoint_xy", activeGoalJson()}});
        blacklistActiveGoal(failure_class);
        if (goal_handle_) {
            nav_client_->async_cancel_goal(goal_handle_);
        }
        clearActiveGoal();
    }

    // Pausing clears all in-flight validation, goal, and spin state, so
    // async callbacks only need their validation_active_/generation/
    // goal_active_ guards; paused() is rechecked at the entry points that
    // start new work (onTimer, validateNextCandidateRoute, sendGoal).
    void pauseForAlc() {
        if (!alc_gate_.applyPause()) {
            return;
        }
        const auto final_goal = activeFinalGoalJson();
        const auto active_waypoint = activeGoalJson();
        clearPendingRouteSelection();
        if (goal_handle_) {
            nav_client_->async_cancel_goal(goal_handle_);
        }
        clearActiveGoal();
        clearGoalMarker();
        publishPlannerEvent(
            "frontier_paused_for_alc",
            {{"goal_xy", final_goal}, {"active_waypoint_xy", active_waypoint}});
        RCLCPP_INFO(get_logger(), "Frontier exploration paused for ALC");
    }

    void resumeAfterAlc() {
        if (!alc_gate_.applyResume()) {
            return;
        }
        wait_log_count_ = 0;
        publishPlannerEvent("frontier_resumed_after_alc");
        RCLCPP_INFO(get_logger(), "Frontier exploration resumed after ALC");
    }

    void clearActiveGoal() {
        if (clearance_backup_goal_handle_) {
            backup_client_->async_cancel_goal(clearance_backup_goal_handle_);
        }
        clearance_backup_goal_handle_.reset();
        recovery_clearance_backup_active_ = false;
        pending_recovery_spin_yaw_ = 0.0;
        if (backup_goal_handle_) {
            backup_client_->async_cancel_goal(backup_goal_handle_);
        }
        backup_goal_handle_.reset();
        backtrace_recovery_active_ = false;
        backtrace_waiting_for_cancel_ = false;
        backtrace_backup_active_ = false;
        backtrace_cancel_started_sec_ = 0.0;
        backtrace_reason_.clear();
        backtrace_planner_.reset();
        goal_active_ = false;
        goal_handle_.reset();
        if (spin_goal_handle_) {
            spin_client_->async_cancel_goal(spin_goal_handle_);
        }
        spin_goal_handle_.reset();
        spin_recovery_active_ = false;
        recovery_spin_active_ = false;
        spin_started_sec_ = 0.0;
        alignment_spin_attempts_ = 0;
        resetRecoveryStateOnly();
        recovery_reobserve_used_ = false;
        active_goal_.reset();
        active_route_.clear();
        active_route_index_ = 0;
        active_final_goal_.reset();
        active_route_path_.clear();
        last_progress_arc_length_ = 0.0;
        rear_latch_progress_arc_length_ = 0.0;
        active_goal_hysteresis_retries_ = 0;
        active_waypoint_sent_time_sec_ = 0.0;
        last_alignment_robot_yaw_.reset();
        breadcrumb_trail_.clear();
        consecutive_reverse_blocked_ = 0;
        recovery_escape_active_ = false;
    }

    void clearPendingRouteSelection() {
        validation_active_ = false;
        ++validation_generation_;
        validation_segment_index_ = 0;
        validation_segment_started_sec_ = 0.0;
        pending_robot_pose_.reset();
        pending_candidates_.clear();
        pending_candidate_index_ = 0;
        validation_final_candidate_.reset();
        validation_route_.clear();
    }

    void blacklistActiveGoal(const FailureClass failure_class) {
        if (!active_final_goal_) {
            return;
        }
        recordGoalFailure(*active_final_goal_, failure_class);
    }

    void recordValidationFailure(const FailureClass failure_class) {
        if (!nav2_validation_blacklist_enabled_ ||
            !validation_final_candidate_) {
            return;
        }
        recordGoalFailure(validation_final_candidate_->goal_xy, failure_class);
    }

    void recordGoalFailure(const WorldPoint& goal_xy,
                           const FailureClass failure_class) {
        core_.recordGoalFailure(goal_xy, nowSec(),
                                blacklistWeight(failure_class));
        const auto record_index = core_.findGoalAttemptIndex(goal_xy);
        if (!record_index) {
            return;
        }
        const auto& record = core_.goalAttempts()[*record_index];
        publishPlannerEvent(
            "goal_attempt_updated",
            {{"goal_xy", pointJson(record.xy)},
             {"failures", record.failures},
             {"failure_class", failureClassName(failure_class)},
             {"blacklisted",
              record.failures >= core_.config().failed_goal_max_retries},
             {"blacklist_radius_m",
              rounded(core_.config().failed_goal_blacklist_radius_m)},
             {"expires_at_sec", rounded(record.expires_at_sec)}});
    }

    nlohmann::json validationGoalJson() const {
        return validation_final_candidate_
                   ? pointJson(validation_final_candidate_->goal_xy)
                   : nlohmann::json(nullptr);
    }

    nlohmann::json validationWaypointJson() const {
        return validation_segment_index_ < validation_route_.size()
                   ? pointJson(validation_route_[validation_segment_index_].xy)
                   : nlohmann::json(nullptr);
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
             {"visible_unknown", candidate.visible_unknown},
             {"is_coverage", candidate.is_coverage},
             {"cluster_size", candidate.cells.size()},
             {"unknown_overlap", rounded(candidate.unknown_overlap_ratio)},
             {"wall_clearance", rounded(candidate.wall_clearance)},
             {"heading_delta", rounded(candidate.heading_delta)},
             {"reachable", candidate.reachable},
             {"soft_reachable", candidate.soft_reachable},
             {"approach_goal_xy", pointJson(candidate.approach_xy)}});
    }

    void publishNoCandidateEvent(const FrontierCounts& counts,
                                 const bool relaxed) {
        publishPlannerEvent(
            "no_frontier_candidate",
            {{"relaxed", relaxed},
             {"counts",
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
               {"rejected_unknown_ratio", counts.rejected_unknown_ratio},
               {"rejected_clearance", counts.rejected_clearance},
               {"rejected_nav2_clearance", counts.rejected_nav2_clearance},
               {"coverage_candidates", counts.coverage_candidates}}}});
    }

    void publishPlannerEvent(
        const std::string& event,
        const nlohmann::json& fields = nlohmann::json::object()) {
        std_msgs::msg::String msg;
        nlohmann::json payload = fields;
        payload["event"] = event;
        payload["time_sec"] = rounded(nowSec());
        msg.data = payload.dump();
        event_pub_->publish(msg);
    }

    void onPlannerEvent(const std_msgs::msg::String& msg) {
        handleReverseBlockedEvent(msg.data);
        handleStallOrSlipEvent(msg.data);
        switch (classifyPlannerEvent(msg.data)) {
            case CoordinationAction::kPause:
                pauseForAlc();
                break;
            case CoordinationAction::kResume:
                resumeAfterAlc();
                break;
            case CoordinationAction::kNone:
                break;
        }
    }

    void handleReverseBlockedEvent(const std::string& data) {
        const auto event = parseEventName(data);
        if (!event || *event != "reverse_blocked") {
            return;
        }
        last_reverse_blocked_sec_ = nowSec();
        if (recoveryActive()) {
            latchRearBlocked("reverse_blocked");
            if (backup_goal_handle_) {
                backup_client_->async_cancel_goal(backup_goal_handle_);
                backup_goal_handle_.reset();
            }
            backtrace_backup_active_ = false;
            if (backtrace_recovery_active_) {
                markActiveGoalUnsafe("REVERSE_UNSAFE",
                                     FailureClass::kReverseUnsafe);
                finishBacktraceRecovery(false, "REVERSE_UNSAFE",
                                        FailureClass::kReverseUnsafe);
            }
            else {
                finishRecoveryHardFail("REVERSE_UNSAFE",
                                       FailureClass::kReverseUnsafe);
            }
            return;
        }
        if (!goal_active_) {
            return;
        }
        ++consecutive_reverse_blocked_;
        if (consecutive_reverse_blocked_ >= kReverseBlockedBacktraceThreshold) {
            // Rear is confirmed blocked by costmap; backtrace uses /backup
            // (backward motion) which the bridge will also block, so skip
            // directly to goal abandonment rather than wasting time on a
            // doomed backup sequence.
            latchRearBlocked("reverse_blocked");
            markActiveGoalUnsafe("REVERSE_UNSAFE",
                                 FailureClass::kReverseUnsafe);
            abandonActiveGoal("REVERSE_UNSAFE", FailureClass::kReverseUnsafe);
            // consecutive_immediate_reverse_streak_ was just incremented inside
            // abandonActiveGoal.  When the count reaches the churn threshold it
            // means every recently-chosen frontier triggered an immediate
            // reverse block — the robot's current pose is the trap, not any
            // individual frontier.  Skip picking yet another doomed candidate
            // and instead drive the robot forward to break the deadlock.
            if (consecutive_immediate_reverse_streak_ >=
                kConsecutiveReverseChurnEscapeThreshold) {
                const int streak = consecutive_immediate_reverse_streak_;
                consecutive_immediate_reverse_streak_ = 0;
                const auto robot_pose = lookupRobotPose();
                publishPlannerEvent(
                    "startup_reverse_churn_detected",
                    {{"streak", streak},
                     {"robot_xy", robot_pose ? pointJson(robot_pose->xy)
                                             : nlohmann::json(nullptr)}});
                if (!robot_pose || !triggerForwardEscape(*robot_pose)) {
                    startRecoveryTree("startup_reverse_churn");
                }
            }
        }
    }

    void handleStallOrSlipEvent(const std::string& data) {
        if (!goal_active_) {
            return;
        }
        const auto event = parseEventName(data);
        if (!event || *event != "stall_or_slip") {
            return;
        }
        markActiveGoalUnsafe("STALL_OR_SLIP", FailureClass::kStallOrSlip);
        startRecoveryTree("stall_or_slip");
    }

    bool triggerForwardEscape(const RobotPose& robot_pose) {
        const double distance =
            recovery_config_.recovery_forward_escape_distance_m;
        const auto escape_yaw =
            core_.findClearestEscapeDirection(robot_pose.xy, distance);
        if (!escape_yaw) {
            publishPlannerEvent(
                "escape_fully_blocked",
                {{"robot_xy", pointJson(robot_pose.xy)},
                 {"streak", consecutive_immediate_reverse_streak_}});
            consecutive_immediate_reverse_streak_ = 0;
            consecutive_backtrace_stall_count_ = 0;
            return false;
        }

        const WorldPoint escape_xy =
            projectPoint(robot_pose.xy, *escape_yaw, distance);
        publishPlannerEvent(
            "forward_escape_started",
            {{"robot_xy", pointJson(robot_pose.xy)},
             {"escape_xy", pointJson(escape_xy)},
             {"streak", consecutive_immediate_reverse_streak_}});
        consecutive_immediate_reverse_streak_ = 0;

        FrontierCandidate escape_cand;
        escape_cand.goal_xy = escape_xy;
        escape_cand.goal_yaw = *escape_yaw;
        escape_cand.reachable = true;

        RouteWaypoint wp;
        wp.xy = escape_xy;
        wp.yaw = *escape_yaw;
        wp.final = true;

        startRoute(robot_pose.xy, escape_cand, {wp});
        recovery_escape_active_ = true;
        return true;
    }

    void markActiveGoalUnsafe(const std::string& reason,
                              const FailureClass failure_class) {
        publishPlannerEvent("frontier_marked_unsafe",
                            {{"reason", reason},
                             {"failure_class", failureClassName(failure_class)},
                             {"goal_xy", activeFinalGoalJson()},
                             {"active_waypoint_xy", activeGoalJson()}});
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
        goal_marker_pub_->publish(makeGoalMarker(goal_xy, yaw));
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
    double nav2_success_xy_tolerance_m_ = 0.40;
    int wait_log_count_ = 0;
    AlcCoordinationGate alc_gate_;

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
    double rear_latch_progress_arc_length_ = 0.0;
    int active_goal_hysteresis_retries_ = 0;
    std::optional<double> last_alignment_robot_yaw_;
    GoalHandle::SharedPtr goal_handle_;
    int nav_goal_generation_ = 0;
    bool spin_recovery_active_ = false;
    int alignment_spin_attempts_ = 0;
    double spin_started_sec_ = 0.0;
    SpinGoalHandle::SharedPtr spin_goal_handle_;
    BacktraceRecoveryConfig backtrace_config_;
    BreadcrumbTrail breadcrumb_trail_;
    BacktraceRecoveryPlanner backtrace_planner_;
    std::string backtrace_backup_action_ = "/backup";
    bool backtrace_recovery_active_ = false;
    bool backtrace_waiting_for_cancel_ = false;
    bool backtrace_backup_active_ = false;
    double backtrace_cancel_started_sec_ = 0.0;
    double backtrace_segment_start_time_sec_ = 0.0;
    std::string backtrace_reason_;
    int consecutive_reverse_blocked_ = 0;
    int consecutive_immediate_reverse_streak_ = 0;
    int consecutive_backtrace_stall_count_ = 0;
    int spurious_success_resend_streak_ = 0;
    BackUpGoalHandle::SharedPtr backup_goal_handle_;
    RecoveryStateMachineConfig recovery_config_;
    RecoveryStateMachine recovery_;
    bool recovery_spin_active_ = false;
    bool recovery_clearance_backup_active_ = false;
    double pending_recovery_spin_yaw_ = 0.0;
    BackUpGoalHandle::SharedPtr clearance_backup_goal_handle_;
    bool recovery_reobserve_used_ = false;
    bool recovery_escape_active_ = false;
    std::string recovery_reason_;
    std::optional<WorldPoint> recovery_start_xy_;
    double last_reverse_blocked_sec_ = -1.0;

    bool nav2_validation_enabled_ = true;
    bool nav2_validation_blacklist_enabled_ = true;
    double nav2_validation_timeout_sec_ = 2.0;
    std::string nav2_compute_path_action_;
    int nav2_validation_rejected_retry_max_ = 10;
    double nav2_validation_rejected_retry_delay_sec_ = 3.0;
    bool validation_active_ = false;
    int validation_generation_ = 0;
    std::size_t validation_segment_index_ = 0;
    double validation_segment_started_sec_ = 0.0;
    int validation_rejection_count_ = 0;
    rclcpp::TimerBase::SharedPtr validation_rejection_retry_timer_;
    std::optional<RobotPose> pending_robot_pose_;
    std::vector<FrontierCandidate> pending_candidates_;
    std::size_t pending_candidate_index_ = 0;
    std::optional<FrontierCandidate> validation_final_candidate_;
    std::vector<RouteWaypoint> validation_route_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::Client<Spin>::SharedPtr spin_client_;
    rclcpp_action::Client<BackUp>::SharedPtr backup_client_;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_client_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        goal_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace yahboom_alc_bringup

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
