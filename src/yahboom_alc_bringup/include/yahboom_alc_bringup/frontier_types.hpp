#pragma once

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace yahboom_alc_bringup
{

using Cell = std::pair<int, int>;
using WorldPoint = std::pair<double, double>;

struct FrontierConfig
{
    double min_frontier_distance = 0.45;
    // Ackermann minimum turning radius. A goal inside either turning circle
    // (centers 0, +-R in the robot frame) cannot be reached driving forward;
    // it needs a reverse cusp. 0 disables the check.
    double min_turning_radius_m = 0.0;
    double max_frontier_distance = 2.5;
    double frontier_goal_search_radius = 0.6;
    double robot_start_search_radius = 2.0;
    double information_radius = 1.5;
    double path_weight = 1.0;
    double information_weight = 0.02;
    double frontier_size_weight = 0.02;
    double unreachable_goal_penalty = 3.0;
    double soft_unreachable_penalty = 1.5;
    double goal_update_distance = 0.5;
    double goal_publish_period = 3.0;
    double footprint_front_m = 0.19;
    double footprint_rear_m = 0.19;
    double footprint_half_width_m = 0.115;
    double footprint_padding_m = 0.02;
    double swept_step_m = 0.05;
    double unknown_overlap_max_ratio = 0.35;
    double unknown_overlap_weight = 4.0;
    double wall_clearance_m = 0.22;
    double min_wall_clearance_m = 0.0;
    // Hard floor on a goal pose's obstacle clearance, kept in lock-step with
    // the Nav2 global costmap inflation_radius. A goal whose nearest obstacle
    // is closer than this sits inside Nav2's inflated band, so
    // SmacPlannerHybrid rejects it with global_path_blocked /
    // empty_or_failed_path. Filtering here stops the explorer from recommending
    // goals Nav2 can never accept (and from poisoning the blacklist with them).
    // 0 disables the floor. Unlike wall_clearance_m (a soft penalty) and
    // min_wall_clearance_m, this floor is preserved under relaxed search.
    double nav2_goal_clearance_m = 0.0;
    // Viewpoint / next-best-view scoring . When
    // viewpoint_information_weight > 0, each goal pose is additionally scored
    // by how much cluster-relevant unknown its camera can actually SEE: a
    // forward ray-fan from the goal cell, bounded by viewpoint_range_m and
    // +/-viewpoint_half_fov_rad around goal_yaw, counting unknown cells reached
    // before any occupied cell blocks the line of sight. This lets the planner
    // prefer a feasible standoff that observes a hard-to-enter (e.g. near-wall)
    // pocket over a pose crammed against it. Weight 0 disables it.
    double viewpoint_information_weight = 0.0;
    double viewpoint_range_m = 3.0;
    double viewpoint_half_fov_rad = 0.5;
    // Straight view tail (straight_tail.hpp; ViewApproach planner). When > 0,
    // a frontier goal's yaw is the direction, among those whose straight tail
    // of this length ending at the goal passes the goal footprint rule, that
    // sees the most unknown from the earliest stop on the tail
    // (view_stop_offset_m before the goal = the goal checker xy tolerance).
    // Must equal the ViewApproach tail_length. 0 keeps the grid-path yaw.
    double view_tail_length_m = 0.0;
    double view_stop_offset_m = 0.0;
    // Coverage revisit. RTAB-Map paints the map from far away, so a room can be
    // known before the robot drives near it. Known-free regions the robot has
    // never approached within coverage_proximity_m are candidates too.
    bool coverage_revisit_enabled = false;
    double coverage_proximity_m = 1.2;
    double coverage_sample_spacing_m = 0.4;
    int coverage_max_targets = 12;
    // Bonus that lets coverage candidates compete with frontiers. 0 generates
    // them only when no frontier candidate exists.
    double coverage_blend_weight = 0.0;
    // Intention-based commitment. When intention_distance_threshold_m > 0,
    // selecting a goal farther than this distance locks in a strategic
    // intention. Subsequent planning cycles give a score bonus
    // (intention_continuation_bonus, subtracted from score) to candidates near
    // the locked intention goal, preventing nearby small opportunities from
    // perpetually outbidding a committed long-range target. Intention is
    // cleared when the robot arrives within intention_clear_radius_m, when
    // intention_timeout_sec elapses (0 = no timeout), or when the intention
    // goal is blacklisted.
    double intention_distance_threshold_m = 0.0;
    double intention_continuation_bonus = 0.0;
    double intention_clear_radius_m = 1.5;
    double intention_timeout_sec = 0.0;
    double corner_penalty_weight = 1.0;
    double corridor_progress_weight = 0.4;
    double heading_penalty_weight = 0.0;
    double heading_penalty_max_rad = 3.141592653589793;
    double goal_timeout_sec = 45.0;
    double stuck_progress_radius_m = 0.12;
    double stuck_progress_timeout_sec = 12.0;
    double failed_goal_blacklist_radius_m = 0.35;
    double failed_goal_blacklist_sec = 600.0;
    double hazard_region_severity1_radius_m = 0.6;
    double hazard_region_severity2_radius_m = 1.0;
    double hazard_region_ttl_sec = 600.0;
    double hazard_region_path_penalty = 100.0;
    double subgoal_path_tolerance_m = 0.30;
    double min_subgoal_spacing_m = 0.45;
    double relaxed_max_frontier_distance = 0.0;
    int min_frontier_size = 8;
    int occupied_threshold = 50;
    int min_clearance_cells = 2;
    int path_clearance_cells = 2;
    int exploratory_path_clearance_cells = 1;
    int failed_goal_max_retries = 2;
    int max_subgoals = 3;
    int relaxed_min_frontier_size = 8;
    bool allow_unreachable_goal = false;
    bool enable_subgoals = true;
    bool hazard_region_enabled = true;
    bool hazard_region_safe_traversal_clear_enabled = true;
};

struct FrontierCounts
{
    int free = 0;
    int unknown = 0;
    int occupied = 0;
    int frontier = 0;
    int clusters = 0;
    int rejected_no_options = 0;
    int rejected_blacklist = 0;
    int rejected_hazard = 0;
    int rejected_too_near = 0;
    int rejected_turning_circle = 0;
    int rejected_too_far = 0;
    int rejected_no_path = 0;
    int rejected_footprint = 0;
    int rejected_swept = 0;
    int rejected_unknown_ratio = 0;
    int rejected_clearance = 0;
    int rejected_nav2_clearance = 0;
    int coverage_candidates = 0;
};

struct FootprintCheck
{
    bool occupied = false;
    int unknown = 0;
    int total = 0;

    double unknownRatio() const;
};

struct FootprintGeometry
{
    double front = 0.0;
    double rear = 0.0;
    double half_width = 0.0;
    int radius_cells = 1;
    double resolution = 0.0;
};

struct ObstacleNeighborhood
{
    double wall_clearance = 0.0;
    double corner_penalty = 0.0;
};

struct GoalAttemptRecord
{
    WorldPoint xy;
    int failures = 0;
    double expires_at_sec = 0.0;
};

struct HazardRegion
{
    WorldPoint xy;
    int severity = 1;
    double radius = 0.0;
    double expires_at_sec = 0.0;
    // Tracked across the current uninterrupted stay inside the region, used
    // by noteHazardTraversal() to tell "actually crossed near the hazard
    // point" apart from "wandered back and forth along the boundary".
    std::optional<WorldPoint> traversal_entry_xy;
    double traversal_min_distance_to_center_m = 0.0;
};

struct HazardRegionUpdate
{
    HazardRegion region;
    bool created = false;
};

struct FrontierCandidate
{
    std::vector<Cell> cells;
    Cell goal_cell;
    WorldPoint goal_xy;
    double goal_yaw = 0.0;
    std::vector<Cell> path;
    double path_length = 0.0;
    int information = 0;
    int visible_unknown = 0;
    // Unknown cells the view tail is chosen to observe (world coordinates,
    // so they survive map resizing); empty without a view tail.
    std::vector<WorldPoint> view_cells;
    bool is_coverage = false;
    bool reachable = false;
    bool soft_reachable = false;
    double unknown_overlap_ratio = 0.0;
    double wall_clearance = 0.0;
    double heading_delta = 0.0;
    double score = 0.0;
};

struct ViewTail
{
    double yaw = 0.0;
    FootprintCheck goal_footprint;
    std::vector<WorldPoint> cells;
};

struct PathProjection
{
    double distance = 0.0;
    double arc_length = 0.0;
    // Index i of the path segment (path[i - 1], path[i]) holding the
    // projection; 0 for single-point paths.
    std::size_t segment = 0;
};

struct CandidateResult
{
    std::vector<FrontierCandidate> candidates;
    FrontierCounts counts;
    bool relaxed = false;
};

struct IntentionState
{
    WorldPoint goal_xy;
    double committed_at_sec = 0.0;
};

}  // namespace yahboom_alc_bringup
