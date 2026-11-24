#include "explorer/cluster_frontier_explorer_node.hpp"

namespace yahboom_alc_bringup
{

void ClusterFrontierExplorerNode::checkPendingViewObservation() {
    if (!pending_view_check_) {
        return;
    }
    const auto check = std::move(*pending_view_check_);
    pending_view_check_.reset();
    const int observed = core_.countObserved(check.cells);
    publishPlannerEvent("view_check", {{"goal_xy", pointJson(check.goal_xy)},
                                       {"targets", check.cells.size()},
                                       {"observed", observed}});
    if (observed == 0) {
        recordGoalFailure(check.goal_xy, FailureClass::kViewNotObserved);
    }
}

bool ClusterFrontierExplorerNode::findAndBeginCandidateRoute(
    const RobotPose& robot_pose, const bool bypass_goal_suppression) {
    checkPendingViewObservation();
    auto result = core_.findCandidates(robot_pose.xy, nowSec(), robot_pose.yaw);
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
            ", rej_turning_circle=" +
            std::to_string(result.counts.rejected_turning_circle) +
            ", rej_too_far=" + std::to_string(result.counts.rejected_too_far) +
            ", rej_no_path=" + std::to_string(result.counts.rejected_no_path) +
            ", rej_footprint=" +
            std::to_string(result.counts.rejected_footprint) + ", rej_swept=" +
            std::to_string(result.counts.rejected_swept) + ", rej_unknown=" +
            std::to_string(result.counts.rejected_unknown_ratio) +
            ", rej_clearance=" +
            std::to_string(result.counts.rejected_clearance) +
            ", rej_nav2_clear=" +
            std::to_string(result.counts.rejected_nav2_clearance) +
            ", coverage=" + std::to_string(result.counts.coverage_candidates) +
            ")");
        return false;
    }
    if (result.relaxed) {
        publishPlannerEvent(
            "frontier_search_relaxed",
            nlohmann::json{{"candidates", result.candidates.size()}});
    }
    std::sort(result.candidates.begin(), result.candidates.end(),
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

std::vector<ClusterFrontierExplorerNode::RouteWaypoint>
ClusterFrontierExplorerNode::buildRoute(
    const FrontierCandidate& final_candidate,
    const std::vector<FrontierCandidate>& candidates) {
    std::vector<RouteWaypoint> route;
    const auto subgoals =
        core_.buildSubgoalsForRoute(final_candidate, candidates);
    route.reserve(subgoals.size() + 1);
    for (const auto& subgoal : subgoals) {
        route.push_back({subgoal.goal_xy, subgoal.goal_yaw, false});
    }
    // Always send the pose the candidate checks ran on: Nav2 validation and
    // the failure record are keyed to goal_xy, so a substitute pose would
    // be tested unchecked and its failure charged to a pose never planned.
    route.push_back({final_candidate.goal_xy, final_candidate.goal_yaw, true,
                     !final_candidate.view_cells.empty()});
    return route;
}

void ClusterFrontierExplorerNode::beginRouteSelection(
    const RobotPose& robot_pose,
    const std::vector<FrontierCandidate>& sorted_candidates) {
    pending_robot_pose_ = robot_pose;
    pending_candidates_ = sorted_candidates;
    pending_candidate_index_ = 0;
    validateNextCandidateRoute();
}

void ClusterFrontierExplorerNode::validateNextCandidateRoute() {
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
        const auto& candidate = pending_candidates_[pending_candidate_index_];
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
            const auto validation_final_candidate = validation_final_candidate_;
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

geometry_msgs::msg::PoseStamped ClusterFrontierExplorerNode::poseForPoint(
    const WorldPoint& xy, const double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = get_clock()->now();
    pose.pose.position.x = xy.first;
    pose.pose.position.y = xy.second;
    pose.pose.orientation = yawToQuaternion(yaw);
    return pose;
}

void ClusterFrontierExplorerNode::validateRouteSegment(const int generation) {
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
    // Nav2 rejects an empty id once it has two planners; this one matches
    // navigate_to_pose_rgbd.xml.
    goal.planner_id = waypoint.view ? nav2_view_planner_id_ : "GridBased";
    goal.use_start = true;

    validation_segment_started_sec_ = nowSec();

    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
    options.goal_response_callback =
        [this, generation](const ComputePathGoalHandle::SharedPtr& handle) {
            onValidationGoalResponse(generation, handle);
        };
    options.result_callback =
        [this, generation](const ComputePathGoalHandle::WrappedResult& result) {
            onValidationResult(generation, result);
        };
    compute_path_client_->async_send_goal(goal, options);
}

void ClusterFrontierExplorerNode::onValidationGoalResponse(
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
        publishPlannerEvent(
            "nav2_validation_failed",
            {{"reason", "goal_rejected"}, {"goal_xy", validationGoalJson()}});
        validateNextCandidateRoute();
    }
}

void ClusterFrontierExplorerNode::onValidationResult(
    const int generation, const ComputePathGoalHandle::WrappedResult& result) {
    if (!validation_active_ || generation != validation_generation_) {
        return;
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        result.result->path.poses.empty()) {
        if (handleStartBlockedValidation()) {
            return;
        }
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

// A path from the robot's own pose fails when the planner's start cell is
// lethal, whatever the goal. That is a robot-side state (e.g. a mark under
// the chassis the cameras can never clear), so blaming and blacklisting
// each goal in turn only empties the frontier list while the robot sits
// still. Move the robot instead: start recovery at the backup+rotation
// stage (reobserve would only re-validate from the same blocked cell).
bool ClusterFrontierExplorerNode::handleStartBlockedValidation() {
    if (validation_segment_index_ != 0 || !pending_robot_pose_ ||
        !global_costmap_ ||
        !isLethalCostmapCell(*global_costmap_, pending_robot_pose_->xy)) {
        return false;
    }
    publishPlannerEvent("nav2_validation_failed",
                        {{"reason", "start_cell_lethal"},
                         {"failure_class", "start_blocked"},
                         {"goal_xy", validationGoalJson()},
                         {"robot_xy", pointJson(pending_robot_pose_->xy)}});
    validation_active_ = false;
    ++validation_generation_;
    clearGoalMarker();
    pending_candidates_.clear();
    pending_robot_pose_.reset();
    startRecoveryTree("planning_start_blocked", /*skip_reobserve=*/true);
    return true;
}

void ClusterFrontierExplorerNode::checkValidationTimeout() {
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
    if (handleStartBlockedValidation()) {
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

void ClusterFrontierExplorerNode::startRoute(
    const WorldPoint& robot_xy, const FrontierCandidate& final_candidate,
    const std::vector<RouteWaypoint>& route) {
    if (route.empty()) {
        return;
    }
    active_route_ = route;
    active_route_index_ = 0;
    active_final_goal_ = final_candidate.goal_xy;
    active_route_path_ = final_candidate.path;
    active_view_cells_ = final_candidate.view_cells;
    const auto projection = core_.projectOntoPath(robot_xy, active_route_path_);
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
    core_.noteGoalPublished(final_candidate.goal_xy, final_candidate.goal_yaw,
                            nowSec(), final_candidate.path_length);
    publishPlannerEvent("subgoal_route_selected",
                        {{"final_goal_xy", pointJson(final_candidate.goal_xy)},
                         {"waypoints", active_route_.size()}});
    sendActiveWaypoint();
}

void ClusterFrontierExplorerNode::sendActiveWaypoint() {
    if (active_route_index_ >= active_route_.size()) {
        clearActiveGoal();
        return;
    }
    const auto& waypoint = active_route_[active_route_index_];
    if (!nav2LifecycleReady()) {
        publishPlannerEvent("nav2_not_active",
                            {{"goal_xy", pointJson(waypoint.xy)},
                             {"goal_yaw", rounded(waypoint.yaw)},
                             {"index", active_route_index_},
                             {"states", nav2LifecycleStatesJson()}});
        scheduleNav2ReadinessRetry();
        return;
    }
    publishPlannerEvent(waypoint.final ? "goal_sent" : "subgoal_sent",
                        {{"goal_xy", pointJson(waypoint.xy)},
                         {"goal_yaw", rounded(waypoint.yaw)},
                         {"index", active_route_index_}});
    sendGoal(waypoint.xy, waypoint.yaw, waypoint.view);
}

void ClusterFrontierExplorerNode::sendGoal(const WorldPoint& goal_xy,
                                           const double goal_yaw,
                                           const bool view) {
    if (alc_gate_.paused()) {
        return;
    }
    if (!nav2LifecycleReady()) {
        publishPlannerEvent("nav2_not_active",
                            {{"goal_xy", pointJson(goal_xy)},
                             {"goal_yaw", rounded(goal_yaw)},
                             {"states", nav2LifecycleStatesJson()}});
        scheduleNav2ReadinessRetry();
        return;
    }
    NavigateToPose::Goal goal;
    goal.pose = poseForPoint(goal_xy, goal_yaw);
    if (view) {
        goal.behavior_tree = nav2_view_behavior_tree_;
    }

    goal_active_ = true;
    active_goal_ = goal_xy;
    active_waypoint_sent_time_sec_ = nowSec();
    last_alignment_robot_yaw_.reset();
    wait_log_count_ = 0;
    publishGoalMarker(goal_xy, goal_yaw);
    RCLCPP_INFO(get_logger(),
                "Sending frontier route waypoint %zu/%zu x=%.2f, y=%.2f",
                active_route_index_ + 1, active_route_.size(), goal_xy.first,
                goal_xy.second);

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

void ClusterFrontierExplorerNode::onGoalResponse(
    const int generation, const GoalHandle::SharedPtr& handle) {
    if (generation != nav_goal_generation_) {
        if (handle) {
            safeCancelGoal(nav_client_, handle, "stale_nav_goal");
        }
        return;
    }
    if (!goal_active_) {
        if (handle) {
            safeCancelGoal(nav_client_, handle, "inactive_nav_goal");
        }
        return;
    }
    if (!handle) {
        RCLCPP_WARN(get_logger(), "Cluster frontier goal rejected");
        publishPlannerEvent("goal_rejected", {{"goal_xy", activeGoalJson()}});
        abandonActiveGoal("goal_rejected", FailureClass::kGlobalPathBlocked);
        return;
    }
    goal_handle_ = handle;
}

void ClusterFrontierExplorerNode::onGoalResult(
    const int generation, const GoalHandle::WrappedResult& result) {
    if (generation != nav_goal_generation_ || !goal_active_ ||
        behaviorActive(BehaviorPurpose::kAlignSpin)) {
        return;
    }
    const int status = static_cast<int>(result.code);
    RCLCPP_INFO(get_logger(), "Cluster frontier goal finished with status %d",
                status);
    publishPlannerEvent("goal_result",
                        {{"status", status}, {"goal_xy", activeGoalJson()}});
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
        publishPlannerEvent(
            "subgoal_reached",
            {{"goal_xy", activeGoalJson()}, {"index", active_route_index_}});
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
        consecutive_backtrace_stall_count_ = 0;
        clearActiveGoal();
        return;
    }
    publishPlannerEvent("route_completed",
                        {{"goal_xy", activeFinalGoalJson()}});
    if (active_final_goal_ && !active_view_cells_.empty()) {
        pending_view_check_ =
            ViewCheck{*active_final_goal_, std::move(active_view_cells_)};
    }
    consecutive_backtrace_stall_count_ = 0;
    clearActiveGoal();
}

void ClusterFrontierExplorerNode::checkActiveGoalProgress(
    const RobotPose& robot_pose) {
    const double now = nowSec();
    breadcrumb_trail_.add({robot_pose.xy, robot_pose.yaw, now});
    if (behaviorActive(BehaviorPurpose::kAlignSpin)) {
        checkBehaviorTimeout(now);
        return;
    }
    const auto& config = core_.config();
    if (config.goal_timeout_sec > 0.0 &&
        now - active_goal_sent_time_sec_ >= config.goal_timeout_sec) {
        RCLCPP_WARN(get_logger(), "Abandoning frontier goal after timeout");
        abandonActiveGoal("timeout", FailureClass::kControllerNoProgress);
        return;
    }
    // A frontier goal borders unknown space, so approaching it can reveal
    // an obstacle on the goal pose. Nav2's 1 Hz replanning then fails and
    // its BT backs up and retries against a goal no plan can reach;
    // reselect now instead.
    if (activeWaypointBlocked()) {
        RCLCPP_WARN(get_logger(),
                    "Abandoning frontier goal: footprint now blocked");
        abandonActiveGoal("goal_footprint_blocked",
                          FailureClass::kGlobalPathBlocked);
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
        now - last_progress_time_sec_ >= config.stuck_progress_timeout_sec) {
        const auto alignment = evaluateActiveGoalAlignment(robot_pose, now);
        if (tryAlignmentProgressGrace(robot_pose, now, alignment)) {
            last_alignment_robot_yaw_ = robot_pose.yaw;
            return;
        }
        if (tryNearGoalHysteresis(robot_pose.xy, now)) {
            last_alignment_robot_yaw_ = robot_pose.yaw;
            return;
        }
        if (trySpinAlignmentRecovery(robot_pose, alignment)) {
            return;
        }
        RCLCPP_WARN(get_logger(), "Abandoning frontier goal after no progress");
        startRecoveryTree("no_progress");
        return;
    }
    last_alignment_robot_yaw_ = robot_pose.yaw;
}

AlignmentGraceStatus ClusterFrontierExplorerNode::evaluateActiveGoalAlignment(
    const RobotPose& robot_pose, const double now) const {
    if (!active_goal_) {
        return {};
    }
    return core_.evaluateAlignmentGrace(robot_pose.xy, robot_pose.yaw,
                                        last_alignment_robot_yaw_,
                                        active_route_path_, *active_goal_,
                                        now - active_waypoint_sent_time_sec_);
}

bool ClusterFrontierExplorerNode::tryAlignmentProgressGrace(
    const RobotPose& robot_pose, const double now,
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
    publishPlannerEvent("alignment_progress_grace",
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

bool ClusterFrontierExplorerNode::tryNearGoalHysteresis(
    const WorldPoint& robot_xy, const double now) {
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
                active_goal_hysteresis_retries_, config.near_goal_retry_limit);
    return true;
}

bool ClusterFrontierExplorerNode::trySpinAlignmentRecovery(
    const RobotPose& robot_pose, const AlignmentGraceStatus& status) {
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
    if (goal_handle_) {
        safeCancelGoal(nav_client_, goal_handle_, "active_nav_goal");
        goal_handle_.reset();
    }

    const double commanded_yaw = std::remainder(
        status.target_yaw - robot_pose.yaw, 2.0 * std::numbers::pi);
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
    sendBehavior<Spin>(spin_goal, BehaviorPurpose::kAlignSpin);
    return true;
}

void ClusterFrontierExplorerNode::finishSpinRecovery(const std::string& reason,
                                                     const double started_sec) {
    const double now = nowSec();
    const double spin_elapsed = std::max(0.0, now - started_sec);
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

bool ClusterFrontierExplorerNode::isNearActiveGoal(const WorldPoint& robot_xy,
                                                   const double radius) const {
    const auto near = [&](const std::optional<WorldPoint>& goal) {
        return goal && withinDistanceTolerance(robot_xy, *goal, radius);
    };
    return near(active_goal_) || near(active_final_goal_);
}

void ClusterFrontierExplorerNode::extendActiveGoalTimers(const double now) {
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

bool ClusterFrontierExplorerNode::activeWaypointBlocked() {
    if (!global_costmap_ || active_route_index_ >= active_route_.size()) {
        return false;
    }
    const auto& waypoint = active_route_[active_route_index_];
    return isFootprintBlockedInCostmap(
        *global_costmap_, core_.footprintGeometry(), waypoint.xy, waypoint.yaw);
}

void ClusterFrontierExplorerNode::abandonActiveGoal(
    const std::string& reason, const FailureClass failure_class) {
    publishPlannerEvent("route_abandoned",
                        {{"reason", reason},
                         {"failure_class", failureClassName(failure_class)},
                         {"goal_xy", activeFinalGoalJson()},
                         {"active_waypoint_xy", activeGoalJson()}});
    blacklistActiveGoal(failure_class);
    if (goal_handle_) {
        safeCancelGoal(nav_client_, goal_handle_, "active_nav_goal");
    }
    clearActiveGoal();
}

void ClusterFrontierExplorerNode::clearActiveGoal() {
    cancelBehavior();
    pending_recovery_spin_yaw_ = 0.0;
    backtrace_recovery_active_ = false;
    backtrace_waiting_for_cancel_ = false;
    backtrace_cancel_started_sec_ = 0.0;
    backtrace_reason_.clear();
    backtrace_planner_.reset();
    goal_active_ = false;
    goal_handle_.reset();
    alignment_spin_attempts_ = 0;
    resetRecoveryStateOnly();
    recovery_reobserve_used_ = false;
    active_goal_.reset();
    active_route_.clear();
    active_route_index_ = 0;
    active_final_goal_.reset();
    active_route_path_.clear();
    active_view_cells_.clear();
    last_progress_arc_length_ = 0.0;
    rear_latch_progress_arc_length_ = 0.0;
    active_goal_hysteresis_retries_ = 0;
    active_waypoint_sent_time_sec_ = 0.0;
    last_alignment_robot_yaw_.reset();
    breadcrumb_trail_.clear();
    consecutive_reverse_blocked_ = 0;
    recovery_escape_active_ = false;
}

void ClusterFrontierExplorerNode::clearPendingRouteSelection() {
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

// A failed segment proves only that its own waypoint was unreachable from
// the previous one, so the failure is charged to that waypoint. Charging
// the final goal blacklisted goals Nav2 was never asked to reach.
void ClusterFrontierExplorerNode::recordValidationFailure(
    const FailureClass failure_class) {
    if (!nav2_validation_blacklist_enabled_ ||
        validation_segment_index_ >= validation_route_.size()) {
        return;
    }
    recordGoalFailure(validation_route_[validation_segment_index_].xy,
                      failure_class);
}

nlohmann::json ClusterFrontierExplorerNode::validationGoalJson() const {
    return validation_final_candidate_
               ? pointJson(validation_final_candidate_->goal_xy)
               : nlohmann::json(nullptr);
}

nlohmann::json ClusterFrontierExplorerNode::validationWaypointJson() const {
    return validation_segment_index_ < validation_route_.size()
               ? pointJson(validation_route_[validation_segment_index_].xy)
               : nlohmann::json(nullptr);
}

}  // namespace yahboom_alc_bringup
