#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <vector>

#include "yahboom_alc_bringup/frontier_types.hpp"

namespace yahboom_alc_bringup
{

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

    std::vector<Cell> goalOptionsForCluster(
        const std::vector<Cell>& cluster,
        const std::vector<std::uint8_t>& blocked, int width, int height) const;
    std::vector<std::uint8_t> makeBlockedGrid(int width, int height,
                                              int clearance) const;
    std::optional<std::vector<Cell>> gridPath(
        const Cell& start, const Cell& goal,
        const std::vector<std::uint8_t>& blocked, int width, int height) const;

    double pathLength(const std::vector<Cell>& path) const;
    // Records that the robot occupied `robot_xy`, building the traversal
    // history used by isUnderObserved(). Only called while coverage revisit is
    // enabled.
    void noteRobotVisited(const WorldPoint& robot_xy);
    // True if the robot has never been within coverage_proximity_m of `xy`.
    bool isUnderObserved(const WorldPoint& xy) const;
    // Counts unique unknown cells visible from `from` within a forward camera
    // ray-fan (viewpoint_range_m, +/-viewpoint_half_fov_rad around `yaw`),
    // stopping each ray at the first occupied cell. See
    // viewpoint_information_weight.
    int countVisibleUnknown(const Cell& from, double yaw) const;
    // How many of `cells` are known in the current map.
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

    FootprintCheck footprintCheckAtWorld(double wx, double wy, double yaw);
    std::optional<double> sweptPathUnknownRatio(const std::vector<Cell>& path);
    std::optional<double> findClearestEscapeDirection(const WorldPoint& pos,
                                                      double max_dist_m,
                                                      int num_angles = 8) const;
    std::optional<PathProjection> projectOntoPath(
        const WorldPoint& point, const std::vector<Cell>& path) const;
    std::vector<FrontierCandidate> buildSubgoalsForRoute(
        const FrontierCandidate& final_candidate,
        const std::vector<FrontierCandidate>& candidates) const;

    bool isBlacklisted(const WorldPoint& goal_xy) const;
    // A failed plan is a fact about one start, goal and map: it excludes that
    // goal cell only, until the robot moves or the map changes.
    void rejectPlan(const WorldPoint& goal_xy) {
        rejected_plans_.push_back(goal_xy);
    }
    // The straight view tail makes a better look, but a goal without one is
    // still somewhere to go.
    void setRequireViewTail(const bool required) {
        require_view_tail_ = required;
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

    double hazardPathPenalty(const std::vector<Cell>& path) const;
    std::optional<HazardRegionUpdate> recordHazardRegion(
        const WorldPoint& robot_xy, double now_sec, int severity);
    std::vector<HazardRegion> noteHazardTraversal(const WorldPoint& robot_xy);
    std::vector<HazardRegion> pruneHazardRegions(double now_sec);
    const std::vector<HazardRegion>& hazardRegions() const {
        return hazard_regions_;
    }

    bool isFreeValue(int value) const;
    std::optional<double> pathTargetYawNear(
        const WorldPoint& robot_xy, const std::vector<Cell>& path) const;
    int index(int x, int y, int width) const;
    double mapResolution() const;
    WorldPoint mapToWorld(int mx, int my) const;
    WorldPoint mapToWorldUnchecked(int mx, int my) const;
    std::optional<Cell> worldToMap(double wx, double wy) const;
    FootprintGeometry footprintGeometry();

private:
    std::vector<Cell> findFrontierCells(int width, int height) const;
    std::vector<std::vector<Cell>> clusterFrontiers(
        const std::vector<Cell>& frontier_cells, int width) const;
    std::optional<Cell> nearestFreeCell(
        const Cell& origin, const std::vector<std::uint8_t>& blocked, int width,
        int height, double radius_m) const;
    int countUnknownNear(const Cell& cell) const;
    // Subsampled known-free, reachable-clearance cells the robot has never
    // approached, nearest to robot_xy first, capped to coverage_max_targets.
    std::vector<Cell> findCoverageSeeds(const WorldPoint& robot_xy,
                                        double max_distance, int width,
                                        int height) const;
    std::vector<int> visibleUnknownIndices(const Cell& from, double yaw) const;
    std::optional<ViewTail> bestViewTail(const WorldPoint& goal_xy);
    ObstacleNeighborhood analyzeObstacleNeighborhood(const Cell& cell,
                                                     int width,
                                                     int height) const;
    WorldPoint clusterAxis(const std::vector<Cell>& cluster) const;
    double corridorProgress(const WorldPoint& start_xy,
                            const WorldPoint& goal_xy,
                            const WorldPoint& axis) const;
    bool isInHazardRegion(const WorldPoint& xy) const;
    bool isOccupiedValue(int value) const;
    bool hasUnknownNeighbor(int width, int index) const;
    bool hasOccupiedClearance(int x, int y, int width, int height,
                              int radius) const;
    double pathInitialYaw(const std::vector<Cell>& path,
                          const WorldPoint& robot_xy,
                          const WorldPoint& goal_xy) const;
    double pathGoalYaw(const std::vector<Cell>& path,
                       const WorldPoint& robot_xy,
                       const WorldPoint& goal_xy) const;
    std::vector<Cell> neighborCells(int x, int y) const;
    bool inBounds(int x, int y, int width, int height) const;
    double euclideanCells(const Cell& a, const Cell& b) const;

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
    bool require_view_tail_ = true;
    std::vector<HazardRegion> hazard_regions_;
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
