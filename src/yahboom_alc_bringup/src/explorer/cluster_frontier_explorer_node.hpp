#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "explorer/explorer_params.hpp"
#include "yahboom_alc_bringup/backtrace_recovery.hpp"
#include "yahboom_alc_bringup/frontier_explorer_core.hpp"
#include "yahboom_alc_bringup/math_util.hpp"
#include "yahboom_alc_bringup/planner_event.hpp"
#include "yahboom_alc_bringup/recovery_state_machine.hpp"
#include "yahboom_alc_bringup/temporal_signal.hpp"
#include "yahboom_alc_bringup/yaw_util.hpp"

namespace yahboom_alc_bringup
{

struct RobotPose
{
    WorldPoint xy;
    double yaw = 0.0;
};

enum class FailureClass : std::uint8_t
{
    kNone,
    kGlobalPathBlocked,
    kControllerNoProgress,
    kReverseUnsafe,
    kStallOrSlip,
    kLocalHazard,     // weight 2: physical blockage confirmed by stall/slip
    kRecoveryFailed,  // weight 2: all recovery strategies exhausted
    // The bridge blocked motion because it couldn't see (stale local
    // costmap / no TF), not because it saw an obstacle. Never blames a goal.
    kPerceptionStall,
    // A frontier view goal succeeded but none of the unknown cells it was
    // chosen to see became known.
    kViewNotObserved,
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
        case FailureClass::kPerceptionStall:
            return "perception_stall";
        case FailureClass::kViewNotObserved:
            return "view_not_observed";
    }
    return "unknown";
}

// Margin granted on top of the behavior server's own spin time allowance
// to cover action transport before declaring a spin lost.
constexpr double kSpinTransportMarginSec = 5.0;
constexpr int kReverseBlockedBacktraceThreshold = 3;
constexpr double kRecentReverseBlockedWindowSec = 3.0;
// After this many consecutive backup_stalled failures, the rear is blocked;
// skip backtrace and directly abandon to avoid spinning wheels in place.
constexpr int kBacktraceStallSkipThreshold = 3;
// If nav2 declares SUCCEEDED in under this many seconds the robot was already
// at the goal and didn't actually navigate anywhere.  Blacklist the goal to
// break the "frontier never clears" cycling loop.
constexpr double kTrivialSuccessThresholdSec = 2.0;
constexpr double kNav2ReadinessPollSec = 1.0;
constexpr double kNav2ReadinessRetrySec = 1.0;
constexpr double kNav2ReadinessRequestTimeoutSec = 2.0;
// Nav2 humble's ControllerServer::isGoalReached() ignores the return value of
// the end-pose transform: when the map->odom lookup fails the goal checker
// compares the robot against a default-constructed (identity) pose.  A robot
// near the odom origin then gets phantom SUCCEEDED results far from the goal.
// Resend the same waypoint up to this many times before treating the phantom
// success as a real failure and entering the recovery tree.
constexpr int kSpuriousSuccessResendMax = 2;

inline geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

inline nlohmann::json pointJson(const WorldPoint& point) {
    return nlohmann::json::array({rounded(point.first), rounded(point.second)});
}

inline nlohmann::json optPointJson(const std::optional<WorldPoint>& pt) {
    return pt ? pointJson(*pt) : nlohmann::json(nullptr);
}

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
    using GetState = lifecycle_msgs::srv::GetState;

    explicit ClusterFrontierExplorerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

private:
    void loadParams();
    void onMap(const nav_msgs::msg::OccupancyGrid& msg);
    void onTimer();
    void maybeStartObservationTurn(const RobotPose& robot_pose);
    void beginObservationSegment(const RobotPose& pose);
    void switchObservationSegment(const RobotPose& pose, const bool stuck);
    void stepObservationTurn();
    void publishObservationCmd(const double linear, const double angular);
    void finishObservationTurn(const std::string& reason);
    void beginObservationRetrace(const RobotPose& pose);
    void stepObservationRetrace(const RobotPose& pose, bool stuck);
    bool findAndBeginCandidateRoute(const RobotPose& robot_pose,
                                    const bool bypass_goal_suppression);

    struct RouteWaypoint
    {
        WorldPoint xy;
        double yaw = 0.0;
        bool final = false;
        // Frontier view goal: straight-tail BT and planner.
        bool view = false;
    };
    std::vector<RouteWaypoint> buildRoute(
        const FrontierCandidate& final_candidate,
        const std::vector<FrontierCandidate>& candidates);
    void beginRouteSelection(
        const RobotPose& robot_pose,
        const std::vector<FrontierCandidate>& sorted_candidates);
    void validateNextCandidateRoute();
    geometry_msgs::msg::PoseStamped poseForPoint(const WorldPoint& xy,
                                                 const double yaw);
    void validateRouteSegment(const int generation);
    void onValidationGoalResponse(
        const int generation, const ComputePathGoalHandle::SharedPtr& handle);
    void onValidationResult(const int generation,
                            const ComputePathGoalHandle::WrappedResult& result);
    bool handleStartBlockedValidation();
    void checkValidationTimeout();
    void startRoute(const WorldPoint& robot_xy,
                    const FrontierCandidate& final_candidate,
                    const std::vector<RouteWaypoint>& route);
    void sendActiveWaypoint();
    std::optional<RobotPose> lookupRobotPose();
    std::optional<RobotPose> lookupPoseIn(const std::string& frame);
    void sendGoal(const WorldPoint& goal_xy, const double goal_yaw,
                  const bool view = false);
    void checkPendingViewObservation();
    void onGoalResponse(const int generation,
                        const GoalHandle::SharedPtr& handle);
    void onGoalResult(const int generation,
                      const GoalHandle::WrappedResult& result);
    void checkActiveGoalProgress(const RobotPose& robot_pose);
    AlignmentGraceStatus evaluateActiveGoalAlignment(
        const RobotPose& robot_pose, const double now) const;
    bool tryAlignmentProgressGrace(const RobotPose& robot_pose,
                                   const double now,
                                   const AlignmentGraceStatus& status);
    bool tryNearGoalHysteresis(const WorldPoint& robot_xy, const double now);
    bool trySpinAlignmentRecovery(const RobotPose& robot_pose,
                                  const AlignmentGraceStatus& status);

    // Spin and BackUp both run on the behavior server, one request at a time.
    // Each request carries its own id: a callback with any other id belongs
    // to an attempt that has already ended, so it is dropped and its goal
    // cancelled through its own handle. The purpose only decides what
    // follows the attempt.
    enum class BehaviorPurpose : std::uint8_t
    {
        kAlignSpin,
        kRecoverySpin,
        kClearanceBackup,
        kBacktraceBackup,
    };

    enum class BehaviorOutcome : std::uint8_t
    {
        kSucceeded,
        kFailed,
        kRejected,
        kTimedOut,
    };

    struct BehaviorAttempt
    {
        int id = 0;
        BehaviorPurpose purpose = BehaviorPurpose::kAlignSpin;
        double started_sec = 0.0;
        SpinGoalHandle::SharedPtr spin_handle;
        BackUpGoalHandle::SharedPtr backup_handle;
    };
    bool behaviorActive(const BehaviorPurpose purpose) const;

    template <typename ActionT>
    void sendBehavior(const typename ActionT::Goal& goal,
                      const BehaviorPurpose purpose) {
        using Handle = rclcpp_action::ClientGoalHandle<ActionT>;
        cancelBehavior();
        const int id = ++behavior_id_;
        behavior_ = BehaviorAttempt{id, purpose, nowSec(), nullptr, nullptr};
        typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
        options.goal_response_callback =
            [this, id](const typename Handle::SharedPtr& handle) {
                if (!behavior_ || behavior_->id != id) {
                    if constexpr (std::is_same_v<ActionT, Spin>) {
                        safeCancelGoal(spin_client_, handle, "stale_spin_goal");
                    }
                    else {
                        safeCancelGoal(backup_client_, handle,
                                       "stale_backup_goal");
                    }
                    return;
                }
                if (!handle) {
                    const auto attempt = *behavior_;
                    behavior_.reset();
                    onBehaviorEnded(attempt, BehaviorOutcome::kRejected);
                    return;
                }
                if constexpr (std::is_same_v<ActionT, Spin>) {
                    behavior_->spin_handle = handle;
                }
                else {
                    behavior_->backup_handle = handle;
                }
            };
        options.result_callback =
            [this, id](const typename Handle::WrappedResult& result) {
                if (!behavior_ || behavior_->id != id) {
                    return;
                }
                const auto attempt = *behavior_;
                behavior_.reset();
                onBehaviorEnded(
                    attempt, result.code == rclcpp_action::ResultCode::SUCCEEDED
                                 ? BehaviorOutcome::kSucceeded
                                 : BehaviorOutcome::kFailed);
            };
        if constexpr (std::is_same_v<ActionT, Spin>) {
            spin_client_->async_send_goal(goal, options);
        }
        else {
            backup_client_->async_send_goal(goal, options);
        }
    }
    void cancelBehavior();
    void checkBehaviorTimeout(const double now);
    void onBehaviorEnded(const BehaviorAttempt& attempt,
                         const BehaviorOutcome outcome);
    void finishSpinRecovery(const std::string& reason,
                            const double started_sec);
    bool isNearActiveGoal(const WorldPoint& robot_xy,
                          const double radius) const;
    void extendActiveGoalTimers(const double now);
    bool recoveryActive() const;
    nlohmann::json recoveryEventFields(
        const std::string& reason = "",
        const FailureClass failure_class = FailureClass::kNone);
    void resetRecoveryStateOnly();
    void startRecoveryTree(const std::string& reason,
                           const bool skip_reobserve = false);
    void checkRecoveryStateMachine();
    void checkRecoveryReobserve();
    void checkRecoverySmallRotation();
    void sendRecoveryClearanceBackup(const double spin_yaw);
    void sendRecoverySpin(const double yaw);
    void finishRecoverySpin(const std::string& reason, const FailureClass fc,
                            const double started_sec);
    void checkRecoveryForwardArc();
    void checkRecoveryReversePulse();
    void finishRecoveryHardFail(const std::string& reason,
                                const FailureClass fc);
    void latchRearBlocked(const std::string& reason);
    void startBacktraceRecovery(const std::string& reason);
    void checkBacktraceRecovery();
    void sendNextBacktraceSegment();
    void onBacktraceBackupEnded(const BehaviorOutcome outcome);
    void finishBacktraceRecovery(const bool success, const std::string& reason,
                                 const FailureClass fc);
    bool activeWaypointBlocked();
    void abandonActiveGoal(
        const std::string& reason,
        const FailureClass failure_class = FailureClass::kRecoveryFailed);
    void pauseForAlc();
    void resumeAfterAlc();
    void clearActiveGoal();
    void clearPendingRouteSelection();
    void blacklistActiveGoal(const FailureClass failure_class);
    static bool isPhysicalFailure(const FailureClass failure_class);
    void recordPhysicalHazard(const FailureClass failure_class);
    void observeHazardTraversal(const WorldPoint& robot_xy);
    void publishHazardCleared(const HazardRegion& region,
                              const std::string& reason);
    void recordValidationFailure(const FailureClass failure_class);
    void recordGoalFailure(const WorldPoint& goal_xy,
                           const FailureClass failure_class);
    nlohmann::json validationGoalJson() const;
    nlohmann::json validationWaypointJson() const;
    void publishCandidateEvent(const std::string& event,
                               const FrontierCandidate& candidate);
    void publishNoCandidateEvent(const FrontierCounts& counts,
                                 const bool relaxed);
    void publishPlannerEvent(
        const std::string& event,
        const nlohmann::json& fields = nlohmann::json::object());

    template <typename ClientT, typename GoalHandleT>
    void safeCancelGoal(const ClientT& client, const GoalHandleT& handle,
                        const char* label) {
        if (!handle) {
            return;
        }
        try {
            client->async_cancel_goal(handle);
        }
        catch (const std::exception& exception) {
            RCLCPP_WARN(get_logger(), "Ignoring stale %s cancel: %s", label,
                        exception.what());
        }
    }
    void onPlannerEvent(const std_msgs::msg::String& msg);
    void handlePerceptionStallReverseBlock(const std::string& reason);
    void handleReverseBlockedEvent();
    void handleStallOrSlipEvent();
    bool triggerForwardEscape(const RobotPose& robot_pose);
    void markActiveGoalUnsafe(const std::string& reason,
                              const FailureClass failure_class);
    void publishFrontierMarkers(
        const std::vector<FrontierCandidate>& candidates);
    void publishGoalMarker(const WorldPoint& goal_xy, const double yaw);
    visualization_msgs::msg::Marker makeGoalMarker(const WorldPoint& goal_xy,
                                                   const double yaw);
    void clearGoalMarker();
    void logWaiting(const std::string& message);

    nlohmann::json activeGoalJson() const { return optPointJson(active_goal_); }
    nlohmann::json activeFinalGoalJson() const;

    double nowSec() { return get_clock()->now().seconds(); }
    double mapReceiveSilenceSec();

    struct LifecycleStateClient
    {
        std::string node_name;
        rclcpp::Client<GetState>::SharedPtr client;
        std::optional<uint8_t> state_id;
        bool request_pending = false;
        double request_sent_sec = 0.0;
    };
    void createNav2LifecycleClients();
    void pollNav2LifecycleStates();
    bool nav2LifecycleReady() const;
    void scheduleNav2ReadinessRetry();
    static const char* lifecycleStateName(const uint8_t state_id);
    nlohmann::json nav2LifecycleStatesJson() const;

    FrontierExplorerCore core_;
    bool enabled_ = true;
    bool map_received_ = false;
    TemporalSignalState map_temporal_state_;
    TemporalSignalObservation map_temporal_;
    std::string map_topic_;
    std::string global_costmap_topic_;
    static constexpr const char* kObservationOdomFrame = "odom";
    static constexpr double kObservationScanRadiusM = 1.5;
    static constexpr double kObservationRepeatRadiusM = 0.5;
    // Roughly the D455/Astra horizontal half-FOV; unknown inside it is
    // occluded rather than unlooked-at.
    static constexpr double kObservationViewHalfAngleRad = 50.0 * kPi / 180.0;
    static constexpr double kObservationDoneYawRad = 25.0 * kPi / 180.0;
    // Below the bridge's observation_blind_reverse_max_sec (30 s), so the
    // blind-reverse window always outlives the turn that opened it.
    static constexpr double kObservationTurnTimeoutSec = 28.0;
    static constexpr double kObservationSegmentM = 0.35;
    static constexpr double kObservationSpeedMps = 0.10;
    // Large enough to saturate the bridge's steering clamp at this speed.
    static constexpr double kObservationAngularCmd = 1.0;
    static constexpr double kObservationStartGraceSec = 1.5;
    static constexpr double kObservationNoProgressSec = 1.0;
    static constexpr double kObservationMinProgressM = 0.02;
    bool observation_turn_active_ = false;
    double observation_turn_started_sec_ = 0.0;
    double observation_target_yaw_ = 0.0;
    double observation_turn_sign_ = 1.0;
    double observation_segment_direction_ = 1.0;
    int observation_stuck_segments_ = 0;
    struct ObservationSegment
    {
        double direction;
        double length;
    };
    std::vector<ObservationSegment> observation_driven_;
    bool observation_retracing_ = false;
    bool observation_retrace_started_ = false;
    WorldPoint observation_segment_start_xy_{0.0, 0.0};
    double observation_segment_start_sec_ = 0.0;
    WorldPoint observation_progress_xy_{0.0, 0.0};
    double observation_progress_sec_ = 0.0;
    std::optional<WorldPoint> last_observation_turn_xy_;
    rclcpp::TimerBase::SharedPtr observation_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        observation_cmd_pub_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr global_costmap_;
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
    std::vector<WorldPoint> active_view_cells_;
    struct ViewCheck
    {
        WorldPoint goal_xy;
        std::vector<WorldPoint> cells;
    };
    // Set when a view goal succeeds; judged on the map the next search uses.
    std::optional<ViewCheck> pending_view_check_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        view_camera_info_sub_;
    std::string nav2_view_behavior_tree_;
    std::string nav2_view_planner_id_;
    double active_goal_sent_time_sec_ = 0.0;
    double active_waypoint_sent_time_sec_ = 0.0;
    double last_progress_time_sec_ = 0.0;
    double last_progress_arc_length_ = 0.0;
    double rear_latch_progress_arc_length_ = 0.0;
    int active_goal_hysteresis_retries_ = 0;
    std::optional<double> last_alignment_robot_yaw_;
    GoalHandle::SharedPtr goal_handle_;
    int nav_goal_generation_ = 0;
    int alignment_spin_attempts_ = 0;
    BacktraceRecoveryConfig backtrace_config_;
    BreadcrumbTrail breadcrumb_trail_;
    BacktraceRecoveryPlanner backtrace_planner_;
    std::string backtrace_backup_action_ = "/backup";
    bool backtrace_recovery_active_ = false;
    bool backtrace_waiting_for_cancel_ = false;
    double backtrace_cancel_started_sec_ = 0.0;
    std::string backtrace_reason_;
    int consecutive_reverse_blocked_ = 0;
    int consecutive_backtrace_stall_count_ = 0;
    int spurious_success_resend_streak_ = 0;
    RecoveryStateMachineConfig recovery_config_;
    RecoveryStateMachine recovery_;
    double pending_recovery_spin_yaw_ = 0.0;
    std::optional<BehaviorAttempt> behavior_;
    int behavior_id_ = 0;
    bool recovery_reobserve_used_ = false;
    bool perception_stall_paused_ = false;
    double perception_stall_pause_started_sec_ = 0.0;
    bool recovery_escape_active_ = false;
    // The stagnation escalation that jumps straight to kReversePulse is
    // triggered BY a reverse_blocked event, which always leaves the rear
    // latch/recency guard in checkRecoveryReversePulse() tripped.  Without
    // this flag every escalated attempt would bounce off that guard into
    // finishRecoveryHardFail() -- which never blacklists -- so the goal
    // would never actually reach the backtrace tail end that does.  Set
    // when escalating, consumed (and cleared) by the first reverse-pulse
    // check so later non-escalated arrivals still get the guard.
    bool reverse_pulse_escalation_pending_ = false;
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
    std::vector<LifecycleStateClient> nav2_lifecycle_clients_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
        global_costmap_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        goal_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr nav2_readiness_poll_timer_;
    rclcpp::TimerBase::SharedPtr nav2_readiness_retry_timer_;
};

}  // namespace yahboom_alc_bringup
