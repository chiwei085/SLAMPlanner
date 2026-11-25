#include "explorer/cluster_frontier_explorer_node.hpp"

namespace yahboom_alc_bringup
{

bool ClusterFrontierExplorerNode::recoveryActive() const {
    return recovery_.stage() != RecoveryStage::kNone;
}

nlohmann::json ClusterFrontierExplorerNode::recoveryEventFields(
    const std::string& reason, const FailureClass failure_class) {
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

void ClusterFrontierExplorerNode::resetRecoveryStateOnly() {
    recovery_.reset();
    pending_recovery_spin_yaw_ = 0.0;
    recovery_reason_.clear();
}

void ClusterFrontierExplorerNode::startRecoveryTree(const std::string& reason,
                                                    const bool skip_reobserve) {
    if (recoveryActive() || recovery_escape_active_) {
        return;
    }
    recovery_reason_ = reason;
    cancelBehavior();
    if (recovery_reobserve_used_ || skip_reobserve) {
        recovery_.startAtSmallRotation();
    }
    else {
        recovery_.start();
    }
    ++nav_goal_generation_;
    if (goal_handle_) {
        safeCancelGoal(nav_client_, goal_handle_, "active_nav_goal");
        goal_handle_.reset();
    }
    goal_active_ = false;
    publishPlannerEvent("recovery_tree_started", recoveryEventFields(reason));
    publishPlannerEvent("recovery_stage_started", recoveryEventFields(reason));
}

void ClusterFrontierExplorerNode::checkRecoveryStateMachine() {
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

void ClusterFrontierExplorerNode::checkRecoveryReobserve() {
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

void ClusterFrontierExplorerNode::checkRecoverySmallRotation() {
    if (behaviorActive(BehaviorPurpose::kRecoverySpin)) {
        checkBehaviorTimeout(nowSec());
        return;
    }
    if (behaviorActive(BehaviorPurpose::kClearanceBackup)) {
        return;
    }
    const auto yaw = recovery_.nextSmallRotationYaw();
    if (!yaw) {
        publishPlannerEvent("recovery_stage_finished",
                            recoveryEventFields("small_rotation_exhausted"));
        recovery_.advanceToForwardArc();
        publishPlannerEvent("recovery_stage_started",
                            recoveryEventFields("forward_arc"));
        return;
    }
    pending_recovery_spin_yaw_ = *yaw;
    sendRecoveryClearanceBackup(*yaw);
}

void ClusterFrontierExplorerNode::sendRecoveryClearanceBackup(
    const double spin_yaw) {
    const double distance = recovery_config_.recovery_clearance_backup_m;
    if (distance <= 0.0 || !backup_client_->action_server_is_ready()) {
        sendRecoverySpin(spin_yaw);
        return;
    }

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
    sendBehavior<BackUp>(goal, BehaviorPurpose::kClearanceBackup);
}

void ClusterFrontierExplorerNode::sendRecoverySpin(const double yaw) {
    if (!spin_client_->action_server_is_ready()) {
        publishPlannerEvent("recovery_stage_finished",
                            recoveryEventFields("spin_unavailable",
                                                FailureClass::kRecoveryFailed));
        recovery_.advanceToForwardArc();
        publishPlannerEvent("recovery_stage_started",
                            recoveryEventFields("forward_arc"));
        return;
    }
    if (goal_handle_) {
        safeCancelGoal(nav_client_, goal_handle_, "active_nav_goal");
        goal_handle_.reset();
    }

    Spin::Goal spin_goal;
    spin_goal.target_yaw = static_cast<float>(yaw);
    spin_goal.time_allowance = rclcpp::Duration::from_seconds(
        recovery_config_.recovery_spin_time_allowance_sec);
    publishPlannerEvent(
        "recovery_stage_started",
        {{"stage", recoveryStageName(RecoveryStage::kSmallRotation)},
         {"reason", recovery_reason_},
         {"failure_class", "none"},
         {"commanded_yaw", rounded(yaw)},
         {"attempt", recovery_.spinAttempts()},
         {"goal_xy", activeFinalGoalJson()},
         {"active_waypoint_xy", activeGoalJson()}});
    sendBehavior<Spin>(spin_goal, BehaviorPurpose::kRecoverySpin);
}

void ClusterFrontierExplorerNode::finishRecoverySpin(const std::string& reason,
                                                     const FailureClass fc,
                                                     const double started_sec) {
    const double elapsed = std::max(0.0, nowSec() - started_sec);
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

void ClusterFrontierExplorerNode::checkRecoveryForwardArc() {
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

void ClusterFrontierExplorerNode::checkRecoveryReversePulse() {
    if (backtrace_recovery_active_) {
        checkBacktraceRecovery();
        return;
    }
    if (rearBlocked()) {
        publishPlannerEvent(
            "reverse_preflight_failed",
            recoveryEventFields("rear_blocked", FailureClass::kReverseUnsafe));
        finishRecoveryHardFail("reverse_preflight_failed",
                               FailureClass::kReverseUnsafe);
        return;
    }
    startBacktraceRecovery(recovery_reason_);
}

void ClusterFrontierExplorerNode::finishRecoveryHardFail(
    const std::string& reason, const FailureClass fc) {
    recovery_.hardFail();
    publishPlannerEvent("recovery_hard_failed",
                        recoveryEventFields(reason, fc));
    markActiveGoalUnsafe(
        fc == FailureClass::kReverseUnsafe ? "REVERSE_UNSAFE" : "STALL_OR_SLIP",
        fc);
    clearActiveGoal();
}

// Asked of the costmap when it matters; a remembered answer goes stale.
bool ClusterFrontierExplorerNode::rearBlocked() {
    const auto pose = lookupRobotPose();
    if (!pose || !global_costmap_) {
        return false;
    }
    const auto behind =
        projectPoint(pose->xy, pose->yaw, -backtrace_config_.backup_step_m);
    return isFootprintBlockedInCostmap(
        *global_costmap_, core_.footprintGeometry(), behind, pose->yaw);
}

void ClusterFrontierExplorerNode::startBacktraceRecovery(
    const std::string& reason) {
    if (backtrace_recovery_active_) {
        return;
    }
    if (!backtrace_planner_.canStart(breadcrumb_trail_) ||
        !backup_client_->action_server_is_ready()) {
        abandonActiveGoal(reason, FailureClass::kRecoveryFailed);
        return;
    }

    backtrace_recovery_active_ = true;
    backtrace_waiting_for_cancel_ = true;
    backtrace_reason_ = reason;
    backtrace_cancel_started_sec_ = nowSec();
    ++nav_goal_generation_;
    if (goal_handle_) {
        safeCancelGoal(nav_client_, goal_handle_, "active_nav_goal");
        goal_handle_.reset();
    }
    goal_active_ = false;
    publishPlannerEvent("backtrace_recovery_started",
                        {{"reason", reason},
                         {"goal_xy", activeFinalGoalJson()},
                         {"active_waypoint_xy", activeGoalJson()},
                         {"breadcrumbs", breadcrumb_trail_.poses().size()}});
}

void ClusterFrontierExplorerNode::checkBacktraceRecovery() {
    if (!backtrace_recovery_active_) {
        return;
    }
    if (behaviorActive(BehaviorPurpose::kBacktraceBackup)) {
        // Detect wheel-spin stall: if the robot hasn't moved
        // min_actual_displacement_m within backup_stall_check_sec of
        // starting the segment, cancel and fail fast rather than waiting
        // the full backup_time_allowance_sec.
        const double stall_check = backtrace_config_.backup_stall_check_sec;
        if (stall_check > 0.0 &&
            nowSec() - behavior_->started_sec >= stall_check) {
            const auto robot_pose = lookupRobotPose();
            const auto& start_xy = backtrace_planner_.state().segment_start_xy;
            if (robot_pose && start_xy &&
                withinDistanceTolerance(
                    robot_pose->xy, *start_xy,
                    backtrace_config_.min_actual_displacement_m)) {
                cancelBehavior();
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

void ClusterFrontierExplorerNode::sendNextBacktraceSegment() {
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
    publishPlannerEvent(
        "backtrace_recovery_segment",
        {{"segment", backtrace_planner_.state().segments_attempted},
         {"distance_m", rounded(*distance)},
         {"speed_mps", rounded(backtrace_config_.backup_speed_mps)}});
    sendBehavior<BackUp>(goal, BehaviorPurpose::kBacktraceBackup);
}

void ClusterFrontierExplorerNode::onBacktraceBackupEnded(
    const BehaviorOutcome outcome) {
    if (!backtrace_recovery_active_) {
        return;
    }
    if (outcome == BehaviorOutcome::kRejected) {
        finishBacktraceRecovery(false, "backup_rejected",
                                FailureClass::kRecoveryFailed);
        return;
    }
    if (outcome != BehaviorOutcome::kSucceeded) {
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
             {"ratio_min", rounded(backtrace_config_.reverse_motion_ratio_min)},
             {"goal_xy", activeFinalGoalJson()},
             {"active_waypoint_xy", activeGoalJson()}});
        finishBacktraceRecovery(false, "rear_slip_detected",
                                FailureClass::kStallOrSlip);
        return;
    }
    sendNextBacktraceSegment();
}

void ClusterFrontierExplorerNode::finishBacktraceRecovery(
    const bool success, const std::string& reason, const FailureClass fc) {
    publishPlannerEvent(
        success ? "backtrace_recovery_finished" : "backtrace_recovery_failed",
        {{"reason", reason},
         {"failure_class", failureClassName(fc)},
         {"trigger", backtrace_reason_},
         {"segments", backtrace_planner_.state().segments_attempted},
         {"goal_xy", activeFinalGoalJson()},
         {"active_waypoint_xy", activeGoalJson()}});
    blacklistActiveGoal(success ? FailureClass::kControllerNoProgress : fc);
    backtrace_recovery_active_ = false;
    backtrace_waiting_for_cancel_ = false;
    backtrace_reason_.clear();
    backtrace_planner_.reset();
    clearActiveGoal();
}

// Only the recovery's own reverse motion is judged here. A route that the
// bridge holds stands still, which the progress check sees, and the rear is a
// fact about the car's pose, not about the goal it was driving to.
void ClusterFrontierExplorerNode::handleReverseBlockedEvent() {
    if (backtrace_recovery_active_) {
        cancelBehavior();
        finishBacktraceRecovery(false, "REVERSE_UNSAFE",
                                FailureClass::kReverseUnsafe);
    }
    else if (behaviorActive(BehaviorPurpose::kClearanceBackup)) {
        // The backup before the spin is optional headroom.
        cancelBehavior();
        publishPlannerEvent("recovery_clearance_backup_skipped",
                            recoveryEventFields("reverse_blocked"));
        sendRecoverySpin(pending_recovery_spin_yaw_);
    }
}

void ClusterFrontierExplorerNode::handleStallOrSlipEvent() {
    if (observation_turn_active_) {
        // The stall detector watches forward commands only, and it fires
        // when the depth camera sees an obstacle ahead: on a forward
        // segment that just means "this way is blocked" -- turn the
        // segment around instead of abandoning the turn.
        const auto pose = lookupPoseIn(kOdomFrame);
        if (observation_segment_direction_ > 0.0 && pose) {
            switchObservationSegment(*pose, /*stuck=*/true);
        }
        else {
            finishObservationTurn("stall_or_slip");
        }
        return;
    }
    if (!goal_active_) {
        return;
    }
    markActiveGoalUnsafe("STALL_OR_SLIP", FailureClass::kStallOrSlip);
    startRecoveryTree("stall_or_slip");
}

bool ClusterFrontierExplorerNode::triggerForwardEscape(
    const RobotPose& robot_pose) {
    const double distance = recovery_config_.recovery_forward_escape_distance_m;
    const auto escape_yaw =
        core_.findClearestEscapeDirection(robot_pose.xy, distance);
    if (!escape_yaw) {
        publishPlannerEvent("escape_fully_blocked",
                            {{"robot_xy", pointJson(robot_pose.xy)}});
        return false;
    }

    const WorldPoint escape_xy =
        projectPoint(robot_pose.xy, *escape_yaw, distance);
    publishPlannerEvent("forward_escape_started",
                        {{"robot_xy", pointJson(robot_pose.xy)},
                         {"escape_xy", pointJson(escape_xy)}});

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

bool ClusterFrontierExplorerNode::behaviorActive(
    const BehaviorPurpose purpose) const {
    return behavior_ && behavior_->purpose == purpose;
}

// Ends the attempt in flight without running its follow-up. A goal not
// yet accepted is cancelled when its response arrives with a stale id.
void ClusterFrontierExplorerNode::cancelBehavior() {
    if (!behavior_) {
        return;
    }
    safeCancelGoal(spin_client_, behavior_->spin_handle, "spin_goal");
    safeCancelGoal(backup_client_, behavior_->backup_handle, "backup_goal");
    behavior_.reset();
}

// Spins are bounded here as well as by the server's time allowance, in
// case the result never arrives.
void ClusterFrontierExplorerNode::checkBehaviorTimeout(const double now) {
    if (!behaviorActive(BehaviorPurpose::kRecoverySpin)) {
        return;
    }
    if (now - behavior_->started_sec <
        recovery_config_.recovery_spin_time_allowance_sec +
            kSpinTransportMarginSec) {
        return;
    }
    const auto attempt = *behavior_;
    cancelBehavior();
    onBehaviorEnded(attempt, BehaviorOutcome::kTimedOut);
}

void ClusterFrontierExplorerNode::onBehaviorEnded(
    const BehaviorAttempt& attempt, const BehaviorOutcome outcome) {
    const bool ok = outcome == BehaviorOutcome::kSucceeded;
    const char* spin_reason =
        ok                                      ? "spin_completed"
        : outcome == BehaviorOutcome::kFailed   ? "spin_failed"
        : outcome == BehaviorOutcome::kRejected ? "spin_rejected"
                                                : "spin_timeout";
    switch (attempt.purpose) {
        case BehaviorPurpose::kRecoverySpin:
            finishRecoverySpin(
                spin_reason,
                ok ? FailureClass::kNone : FailureClass::kRecoveryFailed,
                attempt.started_sec);
            return;
        case BehaviorPurpose::kClearanceBackup:
            if (outcome != BehaviorOutcome::kRejected) {
                publishPlannerEvent(
                    "recovery_clearance_backup_finished",
                    {{"stage",
                      recoveryStageName(RecoveryStage::kSmallRotation)},
                     {"success", ok},
                     {"goal_xy", activeFinalGoalJson()}});
            }
            sendRecoverySpin(pending_recovery_spin_yaw_);
            return;
        case BehaviorPurpose::kBacktraceBackup:
            onBacktraceBackupEnded(outcome);
            return;
    }
}

}  // namespace yahboom_alc_bringup
