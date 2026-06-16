#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

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
    double max_frontier_distance = 2.5;
    double frontier_goal_search_radius = 0.6;
    double robot_start_search_radius = 2.0;
    double information_radius = 1.5;
    double path_weight = 1.0;
    double information_weight = 0.02;
    double frontier_size_weight = 0.02;
    double unreachable_goal_penalty = 3.0;
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
    double subgoal_path_tolerance_m = 0.30;
    double min_subgoal_spacing_m = 0.45;
    double no_frontier_relax_sec = 0.0;
    double relaxed_max_frontier_distance = 0.0;
    double alignment_spin_time_allowance_sec = 15.0;
    int min_frontier_size = 8;
    int occupied_threshold = 50;
    int min_clearance_cells = 2;
    int path_clearance_cells = 2;
    int near_goal_retry_limit = 0;
    int failed_goal_max_retries = 2;
    int max_subgoals = 3;
    int relaxed_min_frontier_size = 8;
    int alignment_spin_max_attempts = 1;
    bool allow_unreachable_goal = false;
    bool alignment_grace_enabled = true;
    bool enable_subgoals = true;
    bool alignment_spin_enabled = false;
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
    int rejected_too_near = 0;
    int rejected_too_far = 0;
    int rejected_no_path = 0;
    int rejected_footprint = 0;
    int rejected_swept = 0;
    int rejected_unknown_ratio = 0;
    int rejected_clearance = 0;
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

struct FrontierCandidate
{
    std::vector<Cell> cells;
    Cell goal_cell;
    WorldPoint goal_xy;
    double goal_yaw = 0.0;
    std::vector<Cell> path;
    double path_length = 0.0;
    int information = 0;
    bool reachable = false;
    double unknown_overlap_ratio = 0.0;
    double wall_clearance = 0.0;
    double heading_delta = 0.0;
    double score = 0.0;
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

class FrontierExplorerCore
{
public:
    explicit FrontierExplorerCore(FrontierConfig config = {});

    void setConfig(const FrontierConfig& config);
    const FrontierConfig& config() const { return config_; }
    void setMap(const nav_msgs::msg::OccupancyGrid& map);
    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }

    CandidateResult findCandidates(
        const WorldPoint& robot_xy, double now_sec,
        std::optional<double> robot_yaw = std::nullopt);
    std::optional<FrontierCandidate> makeCandidate(
        const std::vector<Cell>& cluster, const Cell& start_cell,
        const WorldPoint& robot_xy, const std::vector<bool>& blocked, int width,
        int height, FrontierCounts& counts,
        std::optional<double> robot_yaw = std::nullopt);

    std::vector<Cell> findFrontierCells(int width, int height) const;
    std::vector<std::vector<Cell>> clusterFrontiers(
        const std::vector<Cell>& frontier_cells, int width) const;
    std::vector<Cell> goalOptionsForCluster(const std::vector<Cell>& cluster,
                                            const std::vector<bool>& blocked,
                                            int width, int height) const;
    std::vector<bool> makeBlockedGrid(int width, int height,
                                      int clearance) const;
    std::optional<Cell> nearestFreeCell(const Cell& origin,
                                        const std::vector<bool>& blocked,
                                        int width, int height,
                                        double radius_m) const;
    std::optional<std::vector<Cell>> gridPath(const Cell& start,
                                              const Cell& goal,
                                              const std::vector<bool>& blocked,
                                              int width, int height) const;

    double pathLength(const std::vector<Cell>& path) const;
    int countUnknownNear(const Cell& cell) const;
    bool shouldPublishGoal(const WorldPoint& goal_xy, double now_sec) const;
    void noteGoalPublished(const WorldPoint& goal_xy, double goal_yaw,
                           double now_sec);

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
    void blacklistGoal(const WorldPoint& goal_xy, double now_sec);
    void pruneBlacklist(double now_sec);
    std::optional<std::size_t> findGoalAttemptIndex(
        const WorldPoint& goal_xy) const;
    const std::vector<GoalAttemptRecord>& goalAttempts() const {
        return goal_attempts_;
    }
    std::vector<GoalAttemptRecord>& goalAttemptsForTest() {
        return goal_attempts_;
    }

    bool isFreeValue(int value) const;
    bool isOccupiedValue(int value) const;
    bool hasUnknownNeighbor(int width, int index) const;
    bool hasOccupiedClearance(int x, int y, int width, int height,
                              int radius) const;
    double headingError(double from_yaw, double to_yaw) const;
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
    CandidateResult findCandidatesOnce(const WorldPoint& robot_xy,
                                       double now_sec,
                                       std::optional<double> robot_yaw);
    FrontierConfig relaxedConfig() const;

    FrontierConfig config_;
    nav_msgs::msg::OccupancyGrid map_;
    bool has_map_ = false;
    std::optional<FootprintGeometry> footprint_geometry_;
    std::optional<WorldPoint> last_goal_;
    double last_goal_yaw_ = 0.0;
    double last_goal_time_sec_ = 0.0;
    std::vector<GoalAttemptRecord> goal_attempts_;
    std::optional<double> last_strict_candidate_sec_;
    bool relaxation_active_ = false;
};

}  // namespace yahboom_alc_bringup
