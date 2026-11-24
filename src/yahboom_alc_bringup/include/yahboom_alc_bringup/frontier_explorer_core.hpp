#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstdint>
#include <optional>
#include <unordered_set>
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
    // Viewpoint / next-best-view scoring (Stage 2). When
    // viewpoint_information_weight > 0, each goal pose is additionally scored
    // by how much cluster-relevant unknown its camera can actually SEE: a
    // forward ray-fan from the goal cell, bounded by viewpoint_range_m and
    // +/-viewpoint_half_fov_rad around goal_yaw, counting unknown cells reached
    // before any occupied cell blocks the line of sight. This lets the planner
    // prefer a feasible standoff that observes a hard-to-enter (e.g. near-wall)
    // pocket over a pose crammed against it. Weight 0 disables it entirely so
    // selection is byte-identical to the pre-Stage-2 behaviour (no regression).
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
    // Coverage revisit (Stage 3). With a long Grid/RangeMax the map gets
    // painted from far away, so distant rooms become "known" (non-frontier)
    // before the robot ever drives near them and exploration loses interest.
    // When coverage_revisit_enabled and the normal frontier search yields NO
    // candidate, the robot is instead sent to the nearest reachable known-free
    // region it has never approached within coverage_proximity_m, to densify
    // it. It is a pure fallback - it only runs when there would otherwise be
    // zero candidates - so it cannot change active exploration (no regression).
    bool coverage_revisit_enabled = false;
    double coverage_proximity_m = 1.2;
    double coverage_sample_spacing_m = 0.4;
    int coverage_max_targets = 12;
    // Score bonus (subtracted from score) given to coverage-revisit candidates
    // when blended into the main frontier pool. Positive values make coverage
    // candidates more attractive so they can compete with live frontier
    // candidates rather than acting as a pure fallback. Zero (default)
    // preserves the existing behaviour: coverage candidates are only generated
    // when no frontier candidates exist.
    double coverage_blend_weight = 0.0;
    // Intention-based commitment (F9). When intention_distance_threshold_m > 0,
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
    double near_goal_hysteresis_radius_m = 0.35;
    double near_goal_hysteresis_sec = 0.0;
    double alignment_heading_threshold_rad = 0.35;
    double alignment_max_grace_sec = 12.0;
    double alignment_min_rotation_delta_rad = 0.05;
    double failed_goal_blacklist_radius_m = 0.35;
    double failed_goal_blacklist_sec = 600.0;
    double hazard_region_severity1_radius_m = 0.6;
    double hazard_region_severity2_radius_m = 1.0;
    double hazard_region_ttl_sec = 600.0;
    double hazard_region_path_penalty = 100.0;
    double subgoal_path_tolerance_m = 0.30;
    double min_subgoal_spacing_m = 0.45;
    double no_frontier_relax_sec = 0.0;
    double relaxed_max_frontier_distance = 0.0;
    double alignment_spin_time_allowance_sec = 15.0;
    int min_frontier_size = 8;
    int occupied_threshold = 50;
    int min_clearance_cells = 2;
    int path_clearance_cells = 2;
    int exploratory_path_clearance_cells = 1;
    int near_goal_retry_limit = 0;
    int failed_goal_max_retries = 2;
    int max_subgoals = 3;
    int relaxed_min_frontier_size = 8;
    int alignment_spin_max_attempts = 1;
    bool allow_unreachable_goal = false;
    bool nav2_validation_required_for_final_goal = true;
    bool alignment_grace_enabled = true;
    bool enable_subgoals = true;
    bool alignment_spin_enabled = false;
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

struct AlignmentGraceStatus
{
    bool active = false;
    double target_yaw = 0.0;
    double heading_error = 0.0;
    double previous_heading_error = 0.0;
    double heading_error_delta = 0.0;
};

struct IntentionState
{
    WorldPoint goal_xy;
    double committed_at_sec = 0.0;
};

class FrontierExplorerCore
{
public:
    explicit FrontierExplorerCore(FrontierConfig config = {});

    void setConfig(const FrontierConfig& config);
    const FrontierConfig& config() const { return config_; }
    // Returns false and keeps the previous map when this one is malformed
    // (data length not width * height, or non-positive resolution).
    bool setMap(const nav_msgs::msg::OccupancyGrid& map);
    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }

    CandidateResult findCandidates(
        const WorldPoint& robot_xy, double now_sec,
        std::optional<double> robot_yaw = std::nullopt);
    std::optional<FrontierCandidate> makeCandidate(
        const std::vector<Cell>& cluster, const Cell& start_cell,
        const WorldPoint& robot_xy, const std::vector<std::uint8_t>& blocked,
        int width, int height, FrontierCounts& counts,
        std::optional<double> robot_yaw = std::nullopt);

    std::vector<Cell> findFrontierCells(int width, int height) const;
    std::vector<std::vector<Cell>> clusterFrontiers(
        const std::vector<Cell>& frontier_cells, int width) const;
    std::vector<Cell> goalOptionsForCluster(
        const std::vector<Cell>& cluster,
        const std::vector<std::uint8_t>& blocked, int width, int height) const;
    std::vector<std::uint8_t> makeBlockedGrid(int width, int height,
                                              int clearance) const;
    std::optional<Cell> nearestFreeCell(
        const Cell& origin, const std::vector<std::uint8_t>& blocked, int width,
        int height, double radius_m) const;
    std::optional<std::vector<Cell>> gridPath(
        const Cell& start, const Cell& goal,
        const std::vector<std::uint8_t>& blocked, int width, int height) const;

    double pathLength(const std::vector<Cell>& path) const;
    int countUnknownNear(const Cell& cell) const;
    // Records that the robot occupied `robot_xy`, building the traversal
    // history used by isUnderObserved(). Only called while coverage revisit is
    // enabled.
    void noteRobotVisited(const WorldPoint& robot_xy);
    // True if the robot has never been within coverage_proximity_m of `xy`.
    bool isUnderObserved(const WorldPoint& xy) const;
    // Subsampled known-free, reachable-clearance cells the robot has never
    // approached, nearest to robot_xy first, capped to coverage_max_targets.
    std::vector<Cell> findCoverageSeeds(const WorldPoint& robot_xy,
                                        double max_distance, int width,
                                        int height) const;
    // Counts unique unknown cells visible from `from` within a forward camera
    // ray-fan (viewpoint_range_m, +/-viewpoint_half_fov_rad around `yaw`),
    // stopping each ray at the first occupied cell. Used for Stage 2 viewpoint
    // scoring; see viewpoint_information_weight.
    int countVisibleUnknown(const Cell& from, double yaw) const;
    std::vector<int> visibleUnknownIndices(const Cell& from, double yaw) const;
    std::optional<ViewTail> bestViewTail(const WorldPoint& goal_xy);
    // How many of `cells` are no longer unknown in the current map.
    int countObserved(const std::vector<WorldPoint>& cells) const;
    // Camera field of view is a sensor property, read from camera_info.
    void setViewHalfFov(double half_fov_rad) {
        config_.viewpoint_half_fov_rad = half_fov_rad;
    }
    bool shouldPublishGoal(const WorldPoint& goal_xy, double now_sec) const;
    // path_length is used to decide whether to commit a strategic intention
    // when intention_distance_threshold_m > 0.
    void noteGoalPublished(const WorldPoint& goal_xy, double goal_yaw,
                           double now_sec, double path_length = 0.0);

    const std::optional<IntentionState>& intention() const {
        return intention_;
    }
    void clearIntention() { intention_.reset(); }

    FootprintCheck footprintCheckAtWorld(double wx, double wy, double yaw);
    std::optional<double> sweptPathUnknownRatio(const std::vector<Cell>& path);
    std::optional<double> findClearestEscapeDirection(const WorldPoint& pos,
                                                      double max_dist_m,
                                                      int num_angles = 8) const;
    ObstacleNeighborhood analyzeObstacleNeighborhood(const Cell& cell,
                                                     int width,
                                                     int height) const;
    std::optional<PathProjection> projectOntoPath(
        const WorldPoint& point, const std::vector<Cell>& path) const;
    std::vector<FrontierCandidate> buildSubgoalsForRoute(
        const FrontierCandidate& final_candidate,
        const std::vector<FrontierCandidate>& candidates) const;
    WorldPoint clusterAxis(const std::vector<Cell>& cluster) const;
    double corridorProgress(const WorldPoint& start_xy,
                            const WorldPoint& goal_xy,
                            const WorldPoint& axis) const;

    bool isBlacklisted(const WorldPoint& goal_xy) const;
    // A failed plan is a fact about one start, goal yaw and costmap, so it
    // only steers the rest of the same selection pass.
    void rejectPlan(const WorldPoint& goal_xy) {
        rejected_plans_.push_back(goal_xy);
    }
    void clearRejectedPlans() { rejected_plans_.clear(); }
    std::size_t rejectedPlanCount() const { return rejected_plans_.size(); }
    void blacklistGoal(const WorldPoint& goal_xy, double now_sec);
    void recordGoalFailure(const WorldPoint& goal_xy, double now_sec,
                           int failure_weight = 1);
    void pruneBlacklist(double now_sec);
    std::optional<std::size_t> findGoalAttemptIndex(
        const WorldPoint& goal_xy) const;
    const std::vector<GoalAttemptRecord>& goalAttempts() const {
        return goal_attempts_;
    }
    std::vector<GoalAttemptRecord>& goalAttemptsForTest() {
        return goal_attempts_;
    }

    bool isInHazardRegion(const WorldPoint& xy) const;
    double hazardPathPenalty(const std::vector<Cell>& path) const;
    std::optional<HazardRegionUpdate> recordHazardRegion(
        const WorldPoint& robot_xy, double now_sec, int severity);
    std::vector<HazardRegion> noteHazardTraversal(const WorldPoint& robot_xy);
    std::vector<HazardRegion> pruneHazardRegions(double now_sec);
    const std::vector<HazardRegion>& hazardRegions() const {
        return hazard_regions_;
    }
    std::vector<HazardRegion>& hazardRegionsForTest() {
        return hazard_regions_;
    }

    bool isFreeValue(int value) const;
    bool isOccupiedValue(int value) const;
    bool hasUnknownNeighbor(int width, int index) const;
    bool hasOccupiedClearance(int x, int y, int width, int height,
                              int radius) const;
    std::optional<double> pathTargetYawNear(
        const WorldPoint& robot_xy, const std::vector<Cell>& path) const;
    AlignmentGraceStatus evaluateAlignmentGrace(
        const WorldPoint& robot_xy, double robot_yaw,
        std::optional<double> previous_robot_yaw, const std::vector<Cell>& path,
        const WorldPoint& active_goal_xy,
        double elapsed_since_waypoint_sent_sec) const;
    double pathInitialYaw(const std::vector<Cell>& path,
                          const WorldPoint& robot_xy,
                          const WorldPoint& goal_xy) const;
    double pathGoalYaw(const std::vector<Cell>& path,
                       const WorldPoint& robot_xy,
                       const WorldPoint& goal_xy) const;
    std::vector<Cell> neighborCells(int x, int y) const;
    int index(int x, int y, int width) const;
    bool inBounds(int x, int y, int width, int height) const;
    double euclideanCells(const Cell& a, const Cell& b) const;
    double mapResolution() const;
    WorldPoint mapToWorld(int mx, int my) const;
    WorldPoint mapToWorldUnchecked(int mx, int my) const;
    std::optional<Cell> worldToMap(double wx, double wy) const;
    FootprintGeometry footprintGeometry();

private:
    std::vector<Cell> reconstructPath(const std::vector<int>& parent,
                                      int start_index, int goal_index,
                                      int width) const;
    // Built once per search: frontier and coverage candidates plan on the
    // same map, pose and clearance settings.
    struct SearchGrid
    {
        int width = 0;
        int height = 0;
        std::vector<std::uint8_t> blocked;
        std::vector<std::uint8_t> strict_blocked;
        Cell start_cell;
    };
    // The only two settings relaxed search widens.
    struct SearchLimits
    {
        int min_frontier_size = 0;
        double max_frontier_distance = 0.0;
    };

    std::optional<SearchGrid> prepareSearchGrid(
        const WorldPoint& robot_xy) const;
    SearchLimits strictLimits() const;
    SearchLimits relaxedLimits() const;
    CandidateResult findCandidatesOnce(const WorldPoint& robot_xy,
                                       const std::optional<SearchGrid>& grid,
                                       const SearchLimits& limits,
                                       std::optional<double> robot_yaw);
    // Fills `result` with coverage-revisit candidates when coverage is enabled
    // and frontier search produced none. No-op otherwise, so it can never alter
    // an active-exploration result.
    void appendCoverageCandidates(CandidateResult& result,
                                  const WorldPoint& robot_xy,
                                  const std::optional<SearchGrid>& grid,
                                  const SearchLimits& limits,
                                  std::optional<double> robot_yaw);
    std::optional<FrontierCandidate> makeCandidateImpl(
        const std::vector<Cell>& cluster, const Cell& start_cell,
        const WorldPoint& robot_xy, const std::vector<std::uint8_t>& blocked,
        const std::vector<std::uint8_t>& strict_blocked, int width, int height,
        double max_frontier_distance, FrontierCounts& counts,
        std::optional<double> robot_yaw = std::nullopt);

    FrontierConfig config_;
    nav_msgs::msg::OccupancyGrid map_;
    bool has_map_ = false;
    std::optional<FootprintGeometry> footprint_geometry_;
    std::optional<WorldPoint> last_goal_;
    double last_goal_yaw_ = 0.0;
    double last_goal_time_sec_ = 0.0;
    std::vector<GoalAttemptRecord> goal_attempts_;
    std::vector<WorldPoint> rejected_plans_;
    std::vector<HazardRegion> hazard_regions_;
    std::optional<double> last_strict_candidate_sec_;
    bool relaxation_active_ = false;
    // Quantised world cells (kVisitQuantM) the robot has occupied, for
    // coverage-revisit under-observed detection.
    static constexpr double kVisitQuantM = 0.25;
    std::unordered_set<long long> visited_keys_;
    std::optional<IntentionState> intention_;
};

// True when the world point falls on a lethal (100) cell of a 0-100
// OccupancyGrid costmap. Mirrors Nav2's "Starting point in lethal space"
// check, which looks at the start cell only (not the whole footprint).
// Out of bounds or an empty grid is not lethal.
bool isLethalCostmapCell(const nav_msgs::msg::OccupancyGrid& costmap,
                         const WorldPoint& xy);

// True when a pose is unplannable as a goal on a 0-100 OccupancyGrid costmap,
// the way SmacPlannerHybrid's footprint collision check judges it: the center
// cell is inscribed (>= 99), or a cell whose center lies inside the footprint
// rectangle at yaw is lethal (100). Out of bounds or an empty grid is not
// blocked.
bool isFootprintBlockedInCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                 const FootprintGeometry& footprint,
                                 const WorldPoint& xy, double yaw);

// Dominant direction (world yaw) of the unknown (-1) cells within radius_m of
// xy that lie OUTSIDE the cameras' view: cells within view_half_angle_rad of
// robot_yaw are skipped, since unknown inside the current view is occluded or
// unseeable and turning toward it reveals nothing. Circular mean of the
// directions to the remaining cells. Returns nullopt when fewer than
// min_cells qualify or when they surround the robot so evenly that no
// direction dominates (resultant below min_concentration of the count).
std::optional<double> unknownBearing(const nav_msgs::msg::OccupancyGrid& map,
                                     const WorldPoint& xy, double robot_yaw,
                                     double view_half_angle_rad,
                                     double radius_m, int min_cells = 20,
                                     double min_concentration = 0.2);

}  // namespace yahboom_alc_bringup
