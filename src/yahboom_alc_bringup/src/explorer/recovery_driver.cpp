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
    recovery_start_xy_.reset();
    reverse_pulse_escalation_pending_ = false;
}

void ClusterFrontierExplorerNode::startRecoveryTree(const std::string& reason,
                                                    const bool skip_reobserve) {
    if (recoveryActive() || recovery_escape_active_) {
        return;
    }
    recovery_reason_ = reason;
    spurious_success_resend_streak_ = 0;
    // Recovery takes over the robot: an alignment spin still in flight
    // would keep turning it through reobserve.
    cancelBehavior();
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
        reverse_pulse_escalation_pending_ = true;
        publishPlannerEvent(
            "recovery_stagnation_escalation",
            {{"reason", reason},
             {"trees", recovery_config_.recovery_stagnation_escalation_trees},
             {"radius_m",
              rounded(recovery_config_.recovery_stagnation_radius_m)},
             {"goal_xy", activeFinalGoalJson()}});
    }
    else if (recovery_reobserve_used_ || skip_reobserve) {
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
    // Cycle count alone can't tell "observed and found nothing new"
    // apart from "perception stopped producing" -- both look identical
    // to a timer.  Only trust completion when /map has actually kept
    // arriving during the wait.
    const bool map_silent =
        mapReceiveSilenceSec() >
        recovery_config_.recovery_reobserve_map_stall_ceiling_sec;
    switch (recovery_.resolveReobserveCompletion(map_silent)) {
        case ReobserveCompletion::kExtended:
            publishPlannerEvent(
                "reobserve_perception_stalled",
                recoveryEventFields("map_receive_silence_exceeded"));
            return;
        case ReobserveCompletion::kPerceptionStallExhausted: {
            // Perception itself is the failure, not this goal or any
            // other candidate -- never blacklist for this.  But sitting
            // still is exactly why /map stopped: RTAB-Map only inserts a
            // new node (and republishes /map, since map_always_update is
            // false) on sufficient motion.  Waiting longer cannot help;
            // spin in place instead -- it sweeps the camera across
            // whatever is behind/beside the robot (resolving unknown-vs-
            // occupied ambiguity) and gives SLAM the angular displacement
            // it needs to unstick itself.  Only once that bounded spin
            // budget (recovery_small_spin_attempts) is exhausted do we
            // fall back to a passive pause.
            const auto yaw = recovery_.nextSmallRotationYaw();
            if (!yaw) {
                perception_stall_paused_ = true;
                perception_stall_pause_started_sec_ = nowSec();
                publishPlannerEvent("exploration_paused_perception_stalled",
                                    recoveryEventFields("map_stall_exhausted"));
                return;
            }
            publishPlannerEvent("perception_stall_scan_started",
                                recoveryEventFields("map_stall_exhausted"));
            pending_recovery_spin_yaw_ = *yaw;
            sendRecoveryClearanceBackup(*yaw);
            return;
        }
        case ReobserveCompletion::kConfirmed:
            break;
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
    // Stagnation escalation reaches this stage precisely because a
    // reverse_blocked event just fired, so the latch/recency guard
    // below would otherwise always be tripped on this first check --
    // bouncing the escalated attempt into the non-blacklisting
    // finishRecoveryHardFail() path and defeating the escalation.  Let
    // the one escalated attempt actually run the backtrace; its own
    // segment-level checks (checkBacktraceRecovery /
    // finishBacktraceRecovery) still catch a genuinely blocked rear and
    // blacklist correctly.
    if (reverse_pulse_escalation_pending_) {
        reverse_pulse_escalation_pending_ = false;
        startBacktraceRecovery(recovery_reason_);
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

void ClusterFrontierExplorerNode::latchRearBlocked(const std::string& reason) {
    recovery_.latchRearBlocked(nowSec());
    rear_latch_progress_arc_length_ = last_progress_arc_length_;
    publishPlannerEvent("rear_blocked_latched",
                        {{"stage", recoveryStageName(recovery_.stage())},
                         {"reason", reason},
                         {"failure_class", "reverse_unsafe"},
                         {"goal_xy", activeFinalGoalJson()},
                         {"active_waypoint_xy", activeGoalJson()}});
}

void ClusterFrontierExplorerNode::startBacktraceRecovery(
    const std::string& reason) {
    if (backtrace_recovery_active_) {
        return;
    }
    // Rear persistently blocked: skip futile backup attempts that only
    // spin wheels in place and pollute odometry / SLAM.
    if (consecutive_backtrace_stall_count_ >= kBacktraceStallSkipThreshold) {
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
             {"ratio_min", rounded(backtrace_config_.reverse_motion_ratio_min)},
             {"goal_xy", activeFinalGoalJson()},
             {"active_waypoint_xy", activeGoalJson()}});
        latchRearBlocked("weak_reverse_motion");
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
    if (success) {
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
    backtrace_reason_.clear();
    backtrace_planner_.reset();
    clearActiveGoal();
}

void ClusterFrontierExplorerNode::handleReverseBlockedEvent() {
    last_reverse_blocked_sec_ = nowSec();
    if (recoveryActive()) {
        latchRearBlocked("reverse_blocked");
        if (backtrace_recovery_active_) {
            // The recovery's own backtrace backup is what got blocked:
            // this is a real segment-level failure and does blacklist.
            cancelBehavior();
            markActiveGoalUnsafe("REVERSE_UNSAFE",
                                 FailureClass::kReverseUnsafe);
            finishBacktraceRecovery(false, "REVERSE_UNSAFE",
                                    FailureClass::kReverseUnsafe);
        }
        else if (behaviorActive(BehaviorPurpose::kClearanceBackup)) {
            // The pre-spin clearance backup is optional headroom, and
            // the spin it precedes exists precisely to resolve an
            // ambiguous rear.  Skip the backup and run the spin instead
            // of failing the tree; the cancelled backup's late result
            // carries a stale attempt id and is dropped.
            cancelBehavior();
            publishPlannerEvent("recovery_clearance_backup_skipped",
                                recoveryEventFields("reverse_blocked"));
            sendRecoverySpin(pending_recovery_spin_yaw_);
        }
        else if (reverse_pulse_escalation_pending_) {
            // Same race as checkRecoveryReversePulse(): the rear is
            // genuinely blocked, so reverse_blocked events keep
            // streaming in and this handler -- not the timer tick --
            // is almost always the first to observe kReversePulse
            // after an escalation. Consume the flag here too and
            // actually attempt the backtrace instead of hard-failing
            // before it ever runs; any subsequent reverse_blocked in
            // this burst will then hit the backtrace_recovery_active_
            // branch above, which does blacklist.
            reverse_pulse_escalation_pending_ = false;
            startBacktraceRecovery("reverse_blocked");
        }
        else if (recovery_.stage() == RecoveryStage::kReversePulse) {
            // About to attempt reverse and the rear just re-confirmed
            // blocked: same verdict the timer-tick preflight would
            // reach.
            finishRecoveryHardFail("REVERSE_UNSAFE",
                                   FailureClass::kReverseUnsafe);
        }
        // Any other stage (reobserve, small-rotation wait, forward
        // arc) commands no reverse motion of its own, so this event is
        // residual -- the bridge blocking stragglers from the nav goal
        // startRecoveryTree() just cancelled (cancellation is async;
        // the controller keeps publishing for another cycle or two).
        // Killing the tree here is what previously hard-failed 38/60
        // trees within ~50 ms of starting, before reobserve ran even
        // one cycle.  The latch above already records the rear state
        // for the reverse-pulse preflight; let the tree run.
        return;
    }
    if (!goal_active_) {
        return;
    }
    ++consecutive_reverse_blocked_;
    // == not >=: the bridge streams one event per blocked command (20 Hz),
    // and during a forward escape startRecoveryTree() is a no-op, so >=
    // re-marked the frontier unsafe on every event (144/min on the car).
    if (consecutive_reverse_blocked_ == kReverseBlockedBacktraceThreshold) {
        // Rear is confirmed blocked by costmap for several ticks in a
        // row.  Route into the recovery tree instead of blacklisting
        // immediately: reobserve + small-rotation gives the depth camera
        // and SLAM the motion needed to resolve the common case (an
        // ambiguous unknown-vs-occupied read behind the robot), and
        // forward-arc is the same escape this used to reach only via a
        // separate churn counter.  Only the backtrace/hard-fail tail end
        // -- reached after those attempts fail -- blacklists the goal.
        // If the robot's pose is the real trap rather than this one
        // frontier, cross-tree stagnation tracking inside
        // startRecoveryTree() escalates straight to that tail end once
        // consecutive trees keep starting from the same spot, so this
        // still can't spin forever.
        latchRearBlocked("reverse_blocked");
        markActiveGoalUnsafe("REVERSE_UNSAFE", FailureClass::kReverseUnsafe);
        startRecoveryTree("reverse_blocked");
    }
}

void ClusterFrontierExplorerNode::handleStallOrSlipEvent() {
    if (observation_turn_active_) {
        // The stall detector watches forward commands only, and it fires
        // when the depth camera sees an obstacle ahead: on a forward
        // segment that just means "this way is blocked" -- turn the
        // segment around instead of abandoning the turn.
        const auto pose = lookupPoseIn(kObservationOdomFrame);
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
        consecutive_backtrace_stall_count_ = 0;
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

}  // namespace yahboom_alc_bringup
