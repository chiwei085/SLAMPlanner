#include "explorer/cluster_frontier_explorer_node.hpp"

namespace yahboom_alc_bringup
{

// No frontier is reachable from here, but unknown space may lie beside
// or behind the robot (e.g. placed facing furniture: the room is behind
// it, and those frontiers are rejected as too near). Turn to look at it
// instead of waiting for a map that cannot grow while the robot stands
// still. Nav2 cannot plan this turn -- the global costmap only spans the
// observed map, and the space behind is off it -- so it runs here as a
// multi-point turn: forward and reverse segments alternating, until the
// heading points at the unknown. Commands enter the collision monitor
// (front safety) and the bridge (rear costmap check). Reversing goes into
// unseen space and nothing else detects a rear contact (the IMU gate and
// stall detector watch forward motion only), so a segment that stops moving
// is ended here, and two stuck segments in a row end the turn by driving the
// segments backwards: reversing with the wheels turned the other way swings
// the front wheels into whatever they touch, while the track already driven
// is free. At most one turn per spot, so a finished exploration does not loop.
// Distances are odometry: the map pose jumps when SLAM corrects it.
void ClusterFrontierExplorerNode::maybeStartObservationTurn(
    const RobotPose& robot_pose) {
    // Commands sent before the collision monitor is active are dropped;
    // wait (without spending this spot's one turn) until it is.
    if (!nav2LifecycleReady()) {
        return;
    }
    if (last_observation_turn_xy_ &&
        withinDistanceTolerance(robot_pose.xy, *last_observation_turn_xy_,
                                kObservationRepeatRadiusM)) {
        return;
    }
    const auto odom_pose = lookupPoseIn(kOdomFrame);
    if (!odom_pose) {
        return;
    }
    last_observation_turn_xy_ = robot_pose.xy;
    const auto bearing =
        unknownBearing(core_.map(), robot_pose.xy, robot_pose.yaw,
                       kObservationViewHalfAngleRad, kObservationScanRadiusM);
    if (!bearing) {
        publishPlannerEvent("observation_turn_skipped",
                            {{"reason", "no_unknown_outside_view"},
                             {"robot_xy", pointJson(robot_pose.xy)}});
        return;
    }
    observation_turn_active_ = true;
    observation_turn_started_sec_ = nowSec();
    observation_target_yaw_ = *bearing;
    observation_turn_sign_ =
        std::remainder(*bearing - robot_pose.yaw, 2.0 * kPi) >= 0.0 ? 1.0
                                                                    : -1.0;
    observation_segment_direction_ = 1.0;
    observation_stuck_segments_ = 0;
    observation_driven_.clear();
    observation_retracing_ = false;
    beginObservationSegment(*odom_pose);
    publishPlannerEvent("observation_turn_started",
                        {{"robot_xy", pointJson(robot_pose.xy)},
                         {"robot_yaw", rounded(robot_pose.yaw)},
                         {"bearing", rounded(*bearing)},
                         {"turn_sign", observation_turn_sign_}});
    observation_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                           [this]() { stepObservationTurn(); });
}

void ClusterFrontierExplorerNode::beginObservationSegment(
    const RobotPose& pose) {
    observation_segment_start_xy_ = pose.xy;
    observation_segment_start_sec_ = nowSec();
    observation_travel_.start(pose.xy.first, pose.xy.second);
    observation_progress_sec_ = observation_segment_start_sec_;
    observation_moved_ = false;
}

void ClusterFrontierExplorerNode::switchObservationSegment(
    const RobotPose& pose, const bool stuck) {
    if (observation_retracing_) {
        finishObservationTurn("blocked_both_ways");
        return;
    }
    observation_driven_.push_back(
        {observation_segment_direction_,
         distanceBetween(pose.xy, observation_segment_start_xy_)});
    observation_stuck_segments_ = stuck ? observation_stuck_segments_ + 1 : 0;
    if (observation_stuck_segments_ >= 2) {
        beginObservationRetrace(pose);
        return;
    }
    publishObservationCmd(0.0, 0.0);
    observation_segment_direction_ = -observation_segment_direction_;
    beginObservationSegment(pose);
    publishPlannerEvent(
        "observation_turn_segment",
        {{"direction",
          observation_segment_direction_ > 0.0 ? "forward" : "reverse"},
         {"after_stuck", stuck},
         {"robot_yaw", rounded(pose.yaw)}});
}

void ClusterFrontierExplorerNode::stepObservationTurn() {
    if (!observation_turn_active_) {
        return;
    }
    const double now = nowSec();
    if (now - observation_turn_started_sec_ > kObservationTurnTimeoutSec) {
        finishObservationTurn("timeout");
        return;
    }
    const auto map_pose = lookupRobotPose();
    const auto pose = lookupPoseIn(kOdomFrame);
    if (!map_pose || !pose) {
        publishObservationCmd(0.0, 0.0);
        return;
    }
    if (!observation_retracing_ &&
        headingDelta(map_pose->yaw, observation_target_yaw_) <
            kObservationDoneYawRad) {
        finishObservationTurn("heading_reached");
        return;
    }
    const double along =
        observation_travel_.update(pose->xy.first, pose->xy.second, pose->yaw,
                                   observation_segment_direction_);
    if (observation_travel_.takeCoast()) {
        observation_progress_sec_ = now;
    }
    else if (along >= CommandedTravel::kNoiseM) {
        observation_moved_ = true;
        observation_travel_.start(pose->xy.first, pose->xy.second);
        observation_progress_sec_ = now;
    }
    // From rest the chassis needs up to 1.45 s to answer.
    const bool stuck = now - observation_progress_sec_ >
                       (observation_moved_ ? kObservationNoProgressSec
                                           : kObservationStartGraceSec);
    if (observation_retracing_) {
        stepObservationRetrace(*pose, stuck);
        return;
    }
    const bool segment_done = !withinDistanceTolerance(
        pose->xy, observation_segment_start_xy_, kObservationSegmentM);
    if (stuck || segment_done) {
        switchObservationSegment(*pose, stuck);
        return;
    }
    // angular.z is a yaw rate: the bridge flips the wheel angle when
    // reversing, so the same sign rotates the heading in both segments.
    publishObservationCmd(observation_segment_direction_ * kObservationSpeedMps,
                          observation_turn_sign_ * kObservationAngularCmd);
}

void ClusterFrontierExplorerNode::beginObservationRetrace(
    const RobotPose& pose) {
    observation_retracing_ = true;
    observation_retrace_started_ = false;
    publishPlannerEvent("observation_turn_retrace_started",
                        {{"segments", observation_driven_.size()}});
    stepObservationRetrace(pose, false);
}

void ClusterFrontierExplorerNode::stepObservationRetrace(const RobotPose& pose,
                                                         const bool stuck) {
    if (stuck) {
        finishObservationTurn("blocked_both_ways");
        return;
    }
    // The last segment is done once its length is undone; one that never
    // moved has nothing to undo. The first call starts from zero distance.
    while (!observation_driven_.empty()) {
        const auto& segment = observation_driven_.back();
        const double undone =
            observation_retrace_started_
                ? distanceBetween(pose.xy, observation_segment_start_xy_)
                : 0.0;
        if (segment.length >= CommandedTravel::kNoiseM &&
            undone < segment.length) {
            break;
        }
        observation_driven_.pop_back();
        observation_retrace_started_ = false;
    }
    if (observation_driven_.empty()) {
        finishObservationTurn("retraced_after_contact");
        return;
    }
    if (!observation_retrace_started_) {
        publishObservationCmd(0.0, 0.0);
        observation_retrace_started_ = true;
        observation_segment_direction_ = -observation_driven_.back().direction;
        beginObservationSegment(pose);
        return;
    }
    // Same wheel angle as the original segment, opposite travel.
    publishObservationCmd(observation_segment_direction_ * kObservationSpeedMps,
                          -observation_turn_sign_ * kObservationAngularCmd);
}

void ClusterFrontierExplorerNode::publishObservationCmd(const double linear,
                                                        const double angular) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    observation_cmd_pub_->publish(cmd);
}

void ClusterFrontierExplorerNode::finishObservationTurn(
    const std::string& reason) {
    if (!observation_turn_active_) {
        return;
    }
    observation_turn_active_ = false;
    if (observation_timer_) {
        observation_timer_->cancel();
        observation_timer_.reset();
    }
    publishObservationCmd(0.0, 0.0);
    nlohmann::json fields{
        {"reason", reason},
        {"elapsed", rounded(nowSec() - observation_turn_started_sec_)},
        {"target_yaw", rounded(observation_target_yaw_)}};
    if (const auto pose = lookupRobotPose()) {
        fields["robot_xy"] = pointJson(pose->xy);
        fields["robot_yaw"] = rounded(pose->yaw);
    }
    publishPlannerEvent("observation_turn_finished", fields);
}

}  // namespace yahboom_alc_bringup
