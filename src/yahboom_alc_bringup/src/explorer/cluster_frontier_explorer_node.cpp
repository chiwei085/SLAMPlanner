#include "explorer/cluster_frontier_explorer_node.hpp"

#include "yahboom_alc_bringup/parameter_loader.hpp"

namespace yahboom_alc_bringup
{

ClusterFrontierExplorerNode::ClusterFrontierExplorerNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("cluster_frontier_explorer", options),
      tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_),
      nav_client_(rclcpp_action::create_client<NavigateToPose>(
          this, "/navigate_to_pose")),
      spin_client_(rclcpp_action::create_client<Spin>(this, "/spin")) {
    loadParams();
    backup_client_ =
        rclcpp_action::create_client<BackUp>(this, backtrace_backup_action_);
    createNav2LifecycleClients();

    rclcpp::QoS map_qos{rclcpp::KeepLast(1)};
    map_qos.reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, map_qos,
        [this](const nav_msgs::msg::OccupancyGrid& msg) { onMap(msg); });
    // Nav2's costmap publisher is reliable + transient_local.
    global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        global_costmap_topic_, map_qos,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
            global_costmap_ = std::move(msg);
        });

    rclcpp::QoS marker_qos{rclcpp::KeepLast(1)};
    marker_qos.reliable().transient_local();
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic_, marker_qos);
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        goal_marker_topic_, marker_qos);
    event_pub_ =
        create_publisher<std_msgs::msg::String>(planner_event_topic_, 10);
    // Collision monitor input, so observation-turn commands keep its
    // front safety before reaching the bridge.
    observation_cmd_pub_ =
        create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed_raw", 10);
    event_sub_ = create_subscription<std_msgs::msg::String>(
        planner_event_topic_, 10,
        [this](const std_msgs::msg::String& msg) { onPlannerEvent(msg); });
    timer_ = create_wall_timer(std::chrono::duration<double>(cycle_period_sec_),
                               [this]() { onTimer(); });
    nav2_readiness_poll_timer_ =
        create_wall_timer(std::chrono::duration<double>(kNav2ReadinessPollSec),
                          [this]() { pollNav2LifecycleStates(); });
}

void ClusterFrontierExplorerNode::loadParams() {
    const auto node_params =
        loadDeclaredParameters<ClusterExplorerNodeParams>(*this);
    enabled_ = node_params.enabled;
    map_topic_ = node_params.map_topic;
    global_costmap_topic_ = node_params.global_costmap_topic;
    robot_frame_ = node_params.robot_frame;
    global_frame_ = node_params.global_frame;
    cycle_period_sec_ = node_params.cycle_period_sec;
    planner_event_topic_ = node_params.planner_event_topic;
    marker_topic_ = node_params.marker_topic;
    goal_marker_topic_ = node_params.goal_marker_topic;
    nav2_success_xy_tolerance_m_ = node_params.nav2_success_xy_tolerance_m;
    if (!node_params.view_camera_info_topic.empty()) {
        view_camera_info_sub_ =
            create_subscription<sensor_msgs::msg::CameraInfo>(
                node_params.view_camera_info_topic, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::CameraInfo& msg) {
                    if (msg.width == 0 || msg.k[0] <= 0.0) {
                        return;
                    }
                    const double half_fov =
                        std::atan(msg.width / (2.0 * msg.k[0]));
                    core_.setViewHalfFov(half_fov);
                    publishPlannerEvent("view_fov_from_camera_info",
                                        {{"frame_id", msg.header.frame_id},
                                         {"half_fov_rad", rounded(half_fov)}});
                    view_camera_info_sub_.reset();
                });
    }

    auto config = core_.config();
    loadDeclaredParametersInto(*this, config);
    core_.setConfig(config);

    const auto nav2_params =
        loadDeclaredParameters<Nav2ValidationParams>(*this);
    nav2_validation_enabled_ = nav2_params.nav2_validation_enabled;
    nav2_validation_timeout_sec_ = nav2_params.nav2_validation_timeout_sec;
    nav2_compute_path_action_ = nav2_params.nav2_compute_path_action;
    nav2_validation_rejected_retry_max_ =
        nav2_params.nav2_validation_rejected_retry_max;
    nav2_validation_rejected_retry_delay_sec_ =
        nav2_params.nav2_validation_rejected_retry_delay_sec;
    nav2_view_behavior_tree_ = nav2_params.nav2_view_behavior_tree;
    nav2_view_planner_id_ = nav2_params.nav2_view_planner_id;
    compute_path_client_ = rclcpp_action::create_client<ComputePathToPose>(
        this, nav2_compute_path_action_);

    backtrace_config_ = loadDeclaredParameters<BacktraceRecoveryConfig>(*this);
    breadcrumb_trail_.setConfig(backtrace_config_);
    backtrace_planner_.setConfig(backtrace_config_);
    backtrace_backup_action_ = backtrace_config_.backup_action;

    recovery_config_ =
        loadDeclaredParameters<RecoveryStateMachineConfig>(*this);
    recovery_.setConfig(recovery_config_);
}

void ClusterFrontierExplorerNode::onMap(
    const nav_msgs::msg::OccupancyGrid& msg) {
    const bool changed = msg.data != core_.map().data;
    // A malformed map is dropped before it can count as fresh.
    if (!core_.setMap(msg)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Dropping malformed map: %ux%u with %zu cells",
                             msg.info.width, msg.info.height, msg.data.size());
        return;
    }
    if (changed) {
        core_.clearRejectedPlans();
    }
    map_received_ = true;
    map_temporal_ =
        observeTemporalSignal(rclcpp::Time(msg.header.stamp).seconds(),
                              nowSec(), map_temporal_state_);
}

void ClusterFrontierExplorerNode::onTimer() {
    if (!enabled_) {
        return;
    }
    if (!map_received_) {
        logWaiting("Waiting for /map");
        return;
    }
    for (const auto& region : core_.pruneHazardRegions(nowSec())) {
        publishHazardCleared(region, "expired");
    }
    if (alc_gate_.paused()) {
        logWaiting("Frontier exploration paused for ALC");
        return;
    }
    if (recoveryActive()) {
        checkRecoveryStateMachine();
        return;
    }
    if (observation_turn_active_) {
        return;  // driven by observation_timer_
    }
    if (goal_active_) {
        const auto robot_pose = lookupRobotPose();
        if (robot_pose) {
            noteVisited(*robot_pose);
            observeHazardTraversal(robot_pose->xy);
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
    observeHazardTraversal(robot_pose->xy);
    noteVisited(*robot_pose);
    if (!findAndBeginCandidateRoute(*robot_pose, false)) {
        moveWithoutGoal(*robot_pose);
    }
}

std::optional<RobotPose> ClusterFrontierExplorerNode::lookupRobotPose() {
    return lookupPoseIn(global_frame_);
}

std::optional<RobotPose> ClusterFrontierExplorerNode::lookupPoseIn(
    const std::string& frame) {
    try {
        const auto transform =
            tf_buffer_.lookupTransform(frame, robot_frame_, tf2::TimePointZero);
        const auto& t = transform.transform.translation;
        return RobotPose{WorldPoint{t.x, t.y},
                         yawFromQuaternion(transform.transform.rotation)};
    }
    catch (const tf2::TransformException& exc) {
        RCLCPP_DEBUG(get_logger(), "Waiting for robot pose TF: %s", exc.what());
        return std::nullopt;
    }
}

// Pausing clears all in-flight validation, goal, and spin state, so
// async callbacks only need their validation_active_/generation/
// goal_active_ guards; paused() is rechecked at the entry points that
// start new work (onTimer, validateNextCandidateRoute, sendGoal).
void ClusterFrontierExplorerNode::pauseForAlc() {
    if (!alc_gate_.applyPause()) {
        return;
    }
    const auto final_goal = activeFinalGoalJson();
    const auto active_waypoint = activeGoalJson();
    clearPendingRouteSelection();
    finishObservationTurn("alc_started");
    if (goal_handle_) {
        safeCancelGoal(nav_client_, goal_handle_, "active_nav_goal");
    }
    clearActiveGoal();
    clearGoalMarker();
    publishPlannerEvent(
        "frontier_paused_for_alc",
        {{"goal_xy", final_goal}, {"active_waypoint_xy", active_waypoint}});
    RCLCPP_INFO(get_logger(), "Frontier exploration paused for ALC");
}

void ClusterFrontierExplorerNode::resumeAfterAlc() {
    if (!alc_gate_.applyResume()) {
        return;
    }
    wait_log_count_ = 0;
    publishPlannerEvent("frontier_resumed_after_alc");
    RCLCPP_INFO(get_logger(), "Frontier exploration resumed after ALC");
}

void ClusterFrontierExplorerNode::blacklistActiveGoal(
    const FailureClass failure_class) {
    if (!active_final_goal_) {
        return;
    }
    recordPhysicalHazard(failure_class);
    recordGoalFailure(*active_final_goal_, failure_class);
}

// A hazard region marks a place where motion was measured to fall short.
// kRecoveryFailed measures nothing: a physical cause was already recorded
// as kStallOrSlip/kLocalHazard, and a map-derived one (e.g. reverse
// refused near a mapped obstacle) is already in the map. Recording it at
// the robot's own pose excluded every nearby goal and stranded the robot.
bool ClusterFrontierExplorerNode::isPhysicalFailure(
    const FailureClass failure_class) {
    return failure_class == FailureClass::kStallOrSlip ||
           failure_class == FailureClass::kLocalHazard;
}

void ClusterFrontierExplorerNode::recordPhysicalHazard(
    const FailureClass failure_class) {
    if (!isPhysicalFailure(failure_class)) {
        return;
    }
    const auto robot_pose = lookupRobotPose();
    if (!robot_pose) {
        publishPlannerEvent(
            "hazard_region_record_failed",
            {{"reason", "missing_robot_pose"},
             {"failure_class", failureClassName(failure_class)}});
        return;
    }
    // The evidence is a contact at the leading edge, not around the car.
    const auto footprint = core_.footprintGeometry();
    const double reach = failure_class == FailureClass::kStallOrSlip
                             ? footprint.front
                             : -footprint.rear;
    const WorldPoint contact{
        robot_pose->xy.first + reach * std::cos(robot_pose->yaw),
        robot_pose->xy.second + reach * std::sin(robot_pose->yaw)};
    const auto update = core_.recordHazardRegion(
        contact, nowSec(), blacklistWeight(failure_class));
    if (!update) {
        return;
    }
    const auto& region = update->region;
    publishPlannerEvent(
        update->created ? "hazard_region_recorded" : "hazard_region_updated",
        {{"hazard_xy", pointJson(region.xy)},
         {"robot_xy", pointJson(robot_pose->xy)},
         {"failure_class", failureClassName(failure_class)},
         {"severity", region.severity},
         {"radius_m", rounded(region.radius)},
         {"expires_at_sec", rounded(region.expires_at_sec)}});
}

void ClusterFrontierExplorerNode::observeHazardTraversal(
    const WorldPoint& robot_xy) {
    for (const auto& region : core_.noteHazardTraversal(robot_xy)) {
        publishHazardCleared(region, "safe_traversal");
    }
}

void ClusterFrontierExplorerNode::publishHazardCleared(
    const HazardRegion& region, const std::string& reason) {
    publishPlannerEvent("hazard_region_cleared",
                        {{"hazard_xy", pointJson(region.xy)},
                         {"severity", region.severity},
                         {"radius_m", rounded(region.radius)},
                         {"expires_at_sec", rounded(region.expires_at_sec)},
                         {"reason", reason}});
}

void ClusterFrontierExplorerNode::recordGoalFailure(
    const WorldPoint& goal_xy, const FailureClass failure_class) {
    core_.recordGoalFailure(goal_xy, nowSec(), blacklistWeight(failure_class));
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

void ClusterFrontierExplorerNode::publishCandidateEvent(
    const std::string& event, const FrontierCandidate& candidate) {
    publishPlannerEvent(
        event, {{"goal_xy", pointJson(candidate.goal_xy)},
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
                {"soft_reachable", candidate.soft_reachable}});
}

void ClusterFrontierExplorerNode::publishNoCandidateEvent(
    const FrontierCounts& counts, const bool relaxed) {
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
           {"rejected_hazard", counts.rejected_hazard},
           {"rejected_too_near", counts.rejected_too_near},
           {"rejected_turning_circle", counts.rejected_turning_circle},
           {"rejected_too_far", counts.rejected_too_far},
           {"rejected_no_path", counts.rejected_no_path},
           {"rejected_footprint", counts.rejected_footprint},
           {"rejected_swept", counts.rejected_swept},
           {"rejected_unknown_ratio", counts.rejected_unknown_ratio},
           {"rejected_clearance", counts.rejected_clearance},
           {"rejected_nav2_clearance", counts.rejected_nav2_clearance},
           {"coverage_candidates", counts.coverage_candidates}}}});
}

void ClusterFrontierExplorerNode::publishPlannerEvent(
    const std::string& event, const nlohmann::json& fields) {
    std_msgs::msg::String msg;
    nlohmann::json payload = fields;
    if (event.rfind("nav2_validation_", 0) == 0) {
        const auto silence = temporalSignalAt(map_temporal_state_, nowSec());
        payload["map_temporal"] = {
            {"producer_lag_sec",
             map_temporal_.producer_lag_sec
                 ? nlohmann::json(rounded(*map_temporal_.producer_lag_sec))
                 : nlohmann::json(nullptr)},
            {"consume_gap_sec",
             map_temporal_.consume_gap_sec
                 ? nlohmann::json(rounded(*map_temporal_.consume_gap_sec))
                 : nlohmann::json(nullptr)},
            {"receive_silence_sec",
             silence.receive_silence_sec
                 ? nlohmann::json(rounded(*silence.receive_silence_sec))
                 : nlohmann::json(nullptr)}};
    }
    payload["event"] = event;
    payload["time_sec"] = rounded(nowSec());
    msg.data = payload.dump();
    event_pub_->publish(msg);
}

void ClusterFrontierExplorerNode::onPlannerEvent(
    const std_msgs::msg::String& msg) {
    const auto event = parsePlannerEvent(msg.data);
    switch (event.observation) {
        case PlannerObservation::kReverseUnverified:
            // The bridge holds the car until it can see; only the blind turn
            // has to end.
            finishObservationTurn("perception_stalled");
            break;
        case PlannerObservation::kReverseBlocked:
            handleReverseBlockedEvent();
            break;
        case PlannerObservation::kForwardStall:
            handleStallOrSlipEvent();
            break;
        case PlannerObservation::kAlcStarted:
            pauseForAlc();
            break;
        case PlannerObservation::kAlcFinished:
            resumeAfterAlc();
            break;
        case PlannerObservation::kNone:
            break;
    }
}

void ClusterFrontierExplorerNode::markActiveGoalUnsafe(
    const std::string& reason, const FailureClass failure_class) {
    recordPhysicalHazard(failure_class);
    publishPlannerEvent("frontier_marked_unsafe",
                        {{"reason", reason},
                         {"failure_class", failureClassName(failure_class)},
                         {"goal_xy", activeFinalGoalJson()},
                         {"active_waypoint_xy", activeGoalJson()}});
}

void ClusterFrontierExplorerNode::publishFrontierMarkers(
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
            const auto xy = core_.mapToWorldUnchecked(cell.first, cell.second);
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

void ClusterFrontierExplorerNode::publishGoalMarker(const WorldPoint& goal_xy,
                                                    const double yaw) {
    goal_marker_pub_->publish(makeGoalMarker(goal_xy, yaw));
}

visualization_msgs::msg::Marker ClusterFrontierExplorerNode::makeGoalMarker(
    const WorldPoint& goal_xy, const double yaw) {
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

void ClusterFrontierExplorerNode::clearGoalMarker() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = get_clock()->now();
    marker.ns = "frontier_goal";
    marker.id = 1;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    goal_marker_pub_->publish(marker);
}

void ClusterFrontierExplorerNode::logWaiting(const std::string& message) {
    ++wait_log_count_;
    if (wait_log_count_ == 1 || wait_log_count_ % 6 == 0) {
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
    }
}

nlohmann::json ClusterFrontierExplorerNode::activeFinalGoalJson() const {
    return optPointJson(active_final_goal_);
}

void ClusterFrontierExplorerNode::createNav2LifecycleClients() {
    // collision_monitor is activated last and every motion command
    // (Nav2 goals and observation turns alike) passes through it.
    for (const auto& node_name :
         {"planner_server", "controller_server", "bt_navigator",
          "behavior_server", "collision_monitor"}) {
        const std::string node_name_str{node_name};
        nav2_lifecycle_clients_.push_back(LifecycleStateClient{
            node_name_str,
            create_client<GetState>(node_name_str + "/get_state"), std::nullopt,
            false, 0.0});
    }
}

void ClusterFrontierExplorerNode::pollNav2LifecycleStates() {
    const double now = nowSec();
    for (std::size_t i = 0; i < nav2_lifecycle_clients_.size(); ++i) {
        auto& lifecycle_client = nav2_lifecycle_clients_[i];
        if (lifecycle_client.request_pending) {
            if (now - lifecycle_client.request_sent_sec <
                kNav2ReadinessRequestTimeoutSec) {
                continue;
            }
            lifecycle_client.request_pending = false;
            lifecycle_client.state_id.reset();
        }
        if (!lifecycle_client.client->service_is_ready()) {
            lifecycle_client.state_id.reset();
            continue;
        }
        lifecycle_client.request_pending = true;
        lifecycle_client.request_sent_sec = now;
        auto request = std::make_shared<GetState::Request>();
        lifecycle_client.client->async_send_request(
            request, [this, i](rclcpp::Client<GetState>::SharedFuture future) {
                auto& lifecycle_client = nav2_lifecycle_clients_[i];
                lifecycle_client.request_pending = false;
                try {
                    lifecycle_client.state_id = future.get()->current_state.id;
                }
                catch (const std::exception& exc) {
                    lifecycle_client.state_id.reset();
                    RCLCPP_DEBUG(get_logger(),
                                 "Failed to read lifecycle state for %s: %s",
                                 lifecycle_client.node_name.c_str(),
                                 exc.what());
                }
            });
    }
}

bool ClusterFrontierExplorerNode::nav2LifecycleReady() const {
    return std::all_of(
        nav2_lifecycle_clients_.begin(), nav2_lifecycle_clients_.end(),
        [](const LifecycleStateClient& lifecycle_client) {
            return lifecycle_client.state_id &&
                   lifecycle_client.state_id.value() ==
                       lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        });
}

void ClusterFrontierExplorerNode::scheduleNav2ReadinessRetry() {
    if (nav2_readiness_retry_timer_) {
        return;
    }
    nav2_readiness_retry_timer_ = create_wall_timer(
        std::chrono::duration<double>(kNav2ReadinessRetrySec), [this]() {
            nav2_readiness_retry_timer_.reset();
            if (!goal_active_ && !active_route_.empty()) {
                sendActiveWaypoint();
            }
        });
}

const char* ClusterFrontierExplorerNode::lifecycleStateName(
    const uint8_t state_id) {
    switch (state_id) {
        case lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN:
            return "unknown";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
            return "unconfigured";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
            return "inactive";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
            return "active";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
            return "finalized";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING:
            return "configuring";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP:
            return "cleaningup";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN:
            return "shuttingdown";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING:
            return "activating";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING:
            return "deactivating";
        case lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING:
            return "errorprocessing";
        default:
            return "other";
    }
}

nlohmann::json ClusterFrontierExplorerNode::nav2LifecycleStatesJson() const {
    nlohmann::json states = nlohmann::json::object();
    for (const auto& lifecycle_client : nav2_lifecycle_clients_) {
        if (!lifecycle_client.state_id) {
            states[lifecycle_client.node_name] = "unavailable";
            continue;
        }
        states[lifecycle_client.node_name] =
            lifecycleStateName(lifecycle_client.state_id.value());
    }
    return states;
}

}  // namespace yahboom_alc_bringup
