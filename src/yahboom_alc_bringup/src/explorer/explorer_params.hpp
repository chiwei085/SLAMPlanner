#pragma once

#include <rik_sd/struct_descriptor.hpp>

#include <string>

#include "yahboom_alc_bringup/backtrace_recovery.hpp"
#include "yahboom_alc_bringup/frontier_explorer_core.hpp"
#include "yahboom_alc_bringup/recovery_state_machine.hpp"

namespace yahboom_alc_bringup
{

struct ClusterExplorerNodeParams
{
    bool enabled = true;
    std::string map_topic = "/map";
    // Same costmap Nav2's global planner plans on; used to tell a lethal
    // start cell (robot-side problem) apart from an unreachable goal.
    std::string global_costmap_topic = "/global_costmap/costmap";
    std::string robot_frame = "base_footprint";
    std::string global_frame = "map";
    double cycle_period_sec = 3.0;
    std::string planner_event_topic = "/planner/events";
    std::string marker_topic = "/frontier_markers";
    std::string goal_marker_topic = "/frontier_goal_marker";
    double nav2_success_xy_tolerance_m = 0.55;
    // camera_info of the camera whose field of view the view score assumes;
    // empty keeps viewpoint_half_fov_rad.
    std::string view_camera_info_topic;
};

struct Nav2ValidationParams
{
    bool nav2_validation_enabled = true;
    double nav2_validation_timeout_sec = 2.0;
    std::string nav2_compute_path_action = "/compute_path_to_pose";
    int nav2_validation_rejected_retry_max = 10;
    double nav2_validation_rejected_retry_delay_sec = 3.0;
    // Frontier view goals (view_tail_length_m > 0) run this BT and are
    // validated with this planner; empty = Nav2 defaults.
    std::string nav2_view_behavior_tree;
    std::string nav2_view_planner_id;
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
            named(&ClusterExplorerNodeParams::global_costmap_topic,
                  "global_costmap_topic"),
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
                  "nav2_success_xy_tolerance_m"),
            named(&ClusterExplorerNodeParams::view_camera_info_topic,
                  "view_camera_info_topic"));
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

            named(&RecoveryStateMachineConfig::recovery_spin_time_allowance_sec,
                  "recovery_spin_time_allowance_sec"),
            named(&RecoveryStateMachineConfig::recovery_clearance_backup_m,
                  "recovery_clearance_backup_m"));
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
            named(&Nav2ValidationParams::nav2_validation_rejected_retry_max,
                  "nav2_validation_rejected_retry_max"),
            named(
                &Nav2ValidationParams::nav2_validation_rejected_retry_delay_sec,
                "nav2_validation_rejected_retry_delay_sec"),
            named(&Nav2ValidationParams::nav2_view_behavior_tree,
                  "nav2_view_behavior_tree"),
            named(&Nav2ValidationParams::nav2_view_planner_id,
                  "nav2_view_planner_id"));
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
            named(&FrontierConfig::min_turning_radius_m,
                  "min_turning_radius_m"),
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
            named(&FrontierConfig::view_tail_length_m, "view_tail_length_m"),
            named(&FrontierConfig::view_stop_offset_m, "view_stop_offset_m"),
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

            named(&FrontierConfig::failed_goal_blacklist_radius_m,
                  "failed_goal_blacklist_radius_m"),
            named(&FrontierConfig::failed_goal_blacklist_sec,
                  "failed_goal_blacklist_sec"),
            named(&FrontierConfig::failed_goal_max_retries,
                  "failed_goal_max_retries"),
            named(&FrontierConfig::hazard_region_enabled,
                  "hazard_region_enabled"),
            named(&FrontierConfig::hazard_region_severity1_radius_m,
                  "hazard_region_severity1_radius_m"),
            named(&FrontierConfig::hazard_region_severity2_radius_m,
                  "hazard_region_severity2_radius_m"),
            named(&FrontierConfig::hazard_region_ttl_sec,
                  "hazard_region_ttl_sec"),
            named(&FrontierConfig::hazard_region_path_penalty,
                  "hazard_region_path_penalty"),
            named(&FrontierConfig::hazard_region_safe_traversal_clear_enabled,
                  "hazard_region_safe_traversal_clear_enabled"),
            named(&FrontierConfig::enable_subgoals, "enable_subgoals"),
            named(&FrontierConfig::max_subgoals, "max_subgoals"),
            named(&FrontierConfig::subgoal_path_tolerance_m,
                  "subgoal_path_tolerance_m"),
            named(&FrontierConfig::min_subgoal_spacing_m,
                  "min_subgoal_spacing_m"),

            named(&FrontierConfig::relaxed_min_frontier_size,
                  "relaxed_min_frontier_size"),
            named(&FrontierConfig::relaxed_max_frontier_distance,
                  "relaxed_max_frontier_distance"),

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
