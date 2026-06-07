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
    int min_frontier_size = 8;
    int occupied_threshold = 50;
    int min_clearance_cells = 2;
    int path_clearance_cells = 2;
    double min_frontier_distance = 0.45;
    double max_frontier_distance = 2.5;
    double frontier_goal_search_radius = 0.6;
    double robot_start_search_radius = 2.0;
    double information_radius = 1.5;
    double path_weight = 1.0;
    double information_weight = 0.02;
    double frontier_size_weight = 0.02;
    double unreachable_goal_penalty = 3.0;
    bool allow_unreachable_goal = false;
    double goal_update_distance = 0.5;
    double goal_publish_period = 3.0;
    double footprint_front_m = 0.16;
    double footprint_rear_m = 0.15;
    double footprint_half_width_m = 0.11;
    double footprint_padding_m = 0.03;
    double swept_step_m = 0.05;
    double unknown_overlap_max_ratio = 0.35;
    double unknown_overlap_weight = 4.0;
    double wall_clearance_m = 0.22;
    double corner_penalty_weight = 1.0;
    double corridor_progress_weight = 0.4;
    double goal_timeout_sec = 45.0;
    double stuck_progress_radius_m = 0.12;
    double stuck_progress_timeout_sec = 12.0;
    double failed_goal_blacklist_radius_m = 0.45;
    double failed_goal_blacklist_sec = 60.0;
    int failed_goal_max_retries = 1;
};

struct FrontierCounts
{
    int free = 0;
    int unknown = 0;
    int occupied = 0;
    int frontier = 0;
    int clusters = 0;
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
    double score = 0.0;
};

struct CandidateResult
{
    std::vector<FrontierCandidate> candidates;
    FrontierCounts counts;
};

class FrontierExplorerCore
{
public:
    explicit FrontierExplorerCore(FrontierConfig config = {});

    void setConfig(const FrontierConfig& config);
    const FrontierConfig& config() const { return config_; }
    void setMap(const nav_msgs::msg::OccupancyGrid& map);
    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }

    CandidateResult findCandidates(const WorldPoint& robot_xy, double now_sec);
    std::optional<FrontierCandidate> makeCandidate(
        const std::vector<Cell>& cluster, const Cell& start_cell,
        const WorldPoint& robot_xy, const std::vector<bool>& blocked, int width,
        int height);

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
    ObstacleNeighborhood analyzeObstacleNeighborhood(const Cell& cell,
                                                     int width,
                                                     int height) const;
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

    FrontierConfig config_;
    nav_msgs::msg::OccupancyGrid map_;
    bool has_map_ = false;
    std::optional<FootprintGeometry> footprint_geometry_;
    std::optional<WorldPoint> last_goal_;
    double last_goal_yaw_ = 0.0;
    double last_goal_time_sec_ = 0.0;
    std::vector<GoalAttemptRecord> goal_attempts_;
};

}  // namespace yahboom_alc_bringup
