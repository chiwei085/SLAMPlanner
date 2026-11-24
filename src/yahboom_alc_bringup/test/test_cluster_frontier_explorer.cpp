#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <stdexcept>

#include "yahboom_alc_bringup/frontier_explorer_core.hpp"
#include "yahboom_alc_bringup/math_util.hpp"

namespace yahboom_alc_bringup
{
namespace
{

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
nav_msgs::msg::OccupancyGrid buildGrid(const int width, const int height,
                                       const float resolution = 0.1F,
                                       const std::int8_t default_value = 0) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = static_cast<unsigned int>(width);
    grid.info.height = static_cast<unsigned int>(height);
    grid.info.resolution = resolution;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
        default_value);
    return grid;
}

void setCell(nav_msgs::msg::OccupancyGrid& grid, const int x, const int y,
             const std::int8_t value) {
    grid.data[static_cast<std::size_t>(y) * grid.info.width +
              static_cast<std::size_t>(x)] = value;
}

FrontierExplorerCore makeCore(const nav_msgs::msg::OccupancyGrid& grid) {
    FrontierConfig config;
    config.min_clearance_cells = 0;
    config.path_clearance_cells = 0;
    config.frontier_goal_search_radius = 0.01;
    config.robot_start_search_radius = 1.0;
    config.information_radius = 0.3;
    config.path_weight = 1.0;
    config.information_weight = 0.02;
    config.frontier_size_weight = 0.02;
    config.unreachable_goal_penalty = 3.0;
    config.allow_unreachable_goal = false;
    config.goal_update_distance = 0.5;
    config.goal_publish_period = 3.0;
    config.footprint_front_m = 0.19;
    config.footprint_rear_m = 0.19;
    config.footprint_half_width_m = 0.115;
    config.footprint_padding_m = 0.02;
    config.swept_step_m = 0.05;
    config.unknown_overlap_max_ratio = 0.35;
    config.unknown_overlap_weight = 4.0;
    config.wall_clearance_m = 0.22;
    config.corner_penalty_weight = 1.0;
    config.corridor_progress_weight = 0.4;
    config.heading_penalty_weight = 0.0;
    config.heading_penalty_max_rad = 3.141592653589793;
    config.goal_timeout_sec = 45.0;
    config.stuck_progress_radius_m = 0.12;
    config.stuck_progress_timeout_sec = 12.0;
    config.failed_goal_blacklist_radius_m = 0.35;
    config.failed_goal_blacklist_sec = 600.0;
    config.failed_goal_max_retries = 2;
    config.enable_subgoals = true;
    config.max_subgoals = 3;
    config.subgoal_path_tolerance_m = 0.30;
    config.min_subgoal_spacing_m = 0.45;
    FrontierExplorerCore core(config);
    core.setMap(grid);
    return core;
}

struct CandidateSpec
{
    int x;
    int y;
    double score;
};

FrontierCandidate makeCandidateAt(FrontierExplorerCore& core,
                                  const CandidateSpec& spec) {
    FrontierCandidate candidate;
    candidate.goal_cell = {spec.x, spec.y};
    candidate.goal_xy = core.mapToWorld(spec.x, spec.y);
    candidate.goal_yaw = 0.0;
    candidate.path = {{2, 5}, {spec.x, spec.y}};
    candidate.path_length = core.pathLength(candidate.path);
    candidate.reachable = true;
    candidate.score = spec.score;
    return candidate;
}

template <typename T>
const T& requireValue(const std::optional<T>& value) {
    if (!value) {
        ADD_FAILURE() << "expected optional value";
        throw std::logic_error("expected optional value");
    }
    return value.value();
}

TEST(ClusterFrontierExplorer, LargeCorridorFrontierBeatsSmallCorner) {
    auto grid = buildGrid(30, 12);
    auto core = makeCore(grid);
    auto config = core.config();
    config.unknown_overlap_weight = 0.0;
    config.wall_clearance_m = 0.1;
    core.setConfig(config);
    const Cell start{2, 5};
    const auto robot_xy = core.mapToWorld(start.first, start.second);

    const std::vector<Cell> small_corner{{6, 5}, {6, 6}};
    std::vector<Cell> large_corridor;
    large_corridor.reserve(12);
    for (int i = 0; i < 12; ++i) {
        large_corridor.emplace_back(8 + i, 5);
    }

    FrontierCounts counts1;
    const auto small = core.makeCandidate(
        small_corner, start, robot_xy,
        std::vector<std::uint8_t>(grid.data.size(), false), 30, 12, counts1);
    const auto large = core.makeCandidate(
        large_corridor, start, robot_xy,
        std::vector<std::uint8_t>(grid.data.size(), false), 30, 12, counts1);

    ASSERT_TRUE(small);
    ASSERT_TRUE(large);
    const auto& small_candidate = requireValue(small);
    const auto& large_candidate = requireValue(large);
    EXPECT_LT(large_candidate.score, small_candidate.score);
}

TEST(ClusterFrontierExplorer, OccupiedFootprintOverlapRejectsCandidate) {
    auto grid = buildGrid(12, 12, 0.05);
    setCell(grid, 6, 5, 100);
    auto core = makeCore(grid);

    const auto check = core.footprintCheckAtWorld(0.30, 0.30, 0.0);

    EXPECT_TRUE(check.occupied);
}

TEST(ClusterFrontierExplorer, LimitedUnknownOverlapPenalizesScore) {
    auto free_grid = buildGrid(16, 10, 0.1);
    auto unknown_grid = buildGrid(16, 10, 0.1);
    setCell(unknown_grid, 6, 5, -1);
    const Cell start{2, 5};
    const std::vector<Cell> cluster{{8, 5}, {9, 5}, {10, 5}, {11, 5}};

    auto free_core = makeCore(free_grid);
    auto unknown_core = makeCore(unknown_grid);
    FrontierCounts counts2;
    const auto free_candidate = free_core.makeCandidate(
        cluster, start, free_core.mapToWorld(start.first, start.second),
        std::vector<std::uint8_t>(free_grid.data.size(), false), 16, 10,
        counts2);
    const auto unknown_candidate = unknown_core.makeCandidate(
        cluster, start, unknown_core.mapToWorld(start.first, start.second),
        std::vector<std::uint8_t>(unknown_grid.data.size(), false), 16, 10,
        counts2);

    ASSERT_TRUE(free_candidate);
    ASSERT_TRUE(unknown_candidate);
    const auto& free = requireValue(free_candidate);
    const auto& unknown = requireValue(unknown_candidate);
    EXPECT_GT(unknown.unknown_overlap_ratio, 0.0);
    EXPECT_GT(unknown.score, free.score);
}

TEST(ClusterFrontierExplorer, ExcessiveUnknownOverlapRejectsCandidate) {
    auto grid = buildGrid(30, 30, 0.05);
    for (int y = 10; y < 15; ++y) {
        for (int x = 6; x < 9; ++x) {
            setCell(grid, x, y, -1);
        }
    }
    auto core = makeCore(grid);
    auto config = core.config();
    config.unknown_overlap_max_ratio = 0.1;
    core.setConfig(config);

    const auto xy = core.mapToWorldUnchecked(5, 12);
    const auto check = core.footprintCheckAtWorld(xy.first, xy.second, 0.0);
    EXPECT_GT(check.unknownRatio(), core.config().unknown_overlap_max_ratio);

    const auto ratio = core.sweptPathUnknownRatio({{5, 12}, {12, 12}});
    ASSERT_TRUE(ratio);
    EXPECT_GT(requireValue(ratio), 0.0);

    FrontierCounts counts3;
    const auto candidate = core.makeCandidate(
        {{5, 12}}, {2, 12}, core.mapToWorld(2, 12),
        std::vector<std::uint8_t>(grid.data.size(), false), 30, 30, counts3);
    EXPECT_FALSE(candidate);
}

TEST(ClusterFrontierExplorer, UnreachableGoalRejectsCandidate) {
    auto grid = buildGrid(12, 12, 0.1);
    for (int y = 0; y < 12; ++y) {
        setCell(grid, 5, y, 100);
    }
    auto core = makeCore(grid);
    auto blocked = std::vector<std::uint8_t>(grid.data.size(), false);
    for (int y = 0; y < 12; ++y) {
        blocked[core.index(5, y, 12)] = true;
    }

    FrontierCounts counts4;
    const auto candidate = core.makeCandidate(
        {{8, 5}}, {2, 5}, core.mapToWorld(2, 5), blocked, 12, 12, counts4);
    EXPECT_FALSE(candidate);
}

TEST(ClusterFrontierExplorer, SoftReachableCandidateSurvivesStrictClearance) {
    auto grid = buildGrid(12, 11, 0.2);
    for (int x = 0; x < 12; ++x) {
        setCell(grid, x, 4, 100);
        setCell(grid, x, 6, 100);
    }
    auto core = makeCore(grid);
    auto config = core.config();
    config.path_clearance_cells = 1;
    config.exploratory_path_clearance_cells = 0;
    config.min_clearance_cells = 0;
    config.unknown_overlap_max_ratio = 1.0;
    config.soft_unreachable_penalty = 1.5;
    core.setConfig(config);

    FrontierCounts counts;
    const auto candidate = core.makeCandidate(
        {{8, 5}}, {2, 5}, core.mapToWorld(2, 5),
        std::vector<std::uint8_t>(grid.data.size(), false), 12, 11, counts);

    ASSERT_TRUE(candidate);
    const auto& soft_candidate = requireValue(candidate);
    EXPECT_FALSE(soft_candidate.reachable);
    EXPECT_TRUE(soft_candidate.soft_reachable);
    EXPECT_GT(soft_candidate.score, soft_candidate.path_length);
}

// A frontier sitting in a narrow corridor (walls 0.2 m away on both sides) is
// grid-reachable but its only goal poses sit inside Nav2's inflation band. The
// nav2_goal_clearance_m floor rejects them so the explorer never hands Nav2 a
// goal it will abort with global_path_blocked.
namespace
{
nav_msgs::msg::OccupancyGrid buildNarrowCorridorGrid() {
    auto grid = buildGrid(20, 14, 0.1);
    for (int x = 0; x < 20; ++x) {
        setCell(grid, x, 5, 100);
        setCell(grid, x, 9, 100);
    }
    return grid;
}
}  // namespace

TEST(ClusterFrontierExplorer, Nav2ClearanceFloorDisabledKeepsNearWallGoal) {
    auto grid = buildNarrowCorridorGrid();
    auto core = makeCore(grid);  // nav2_goal_clearance_m defaults to 0 (off)

    FrontierCounts counts;
    const auto candidate = core.makeCandidate(
        {{10, 7}}, {2, 7}, core.mapToWorld(2, 7),
        std::vector<std::uint8_t>(grid.data.size(), false), 20, 14, counts);

    ASSERT_TRUE(candidate);
    EXPECT_NEAR(requireValue(candidate).wall_clearance, 0.2, 1e-6);
    EXPECT_EQ(counts.rejected_nav2_clearance, 0);
}

TEST(ClusterFrontierExplorer, Nav2ClearanceFloorRejectsNearWallGoal) {
    auto grid = buildNarrowCorridorGrid();
    auto core = makeCore(grid);
    auto config = core.config();
    config.nav2_goal_clearance_m = 0.25;  // goal clearance is only 0.2 m
    core.setConfig(config);

    FrontierCounts counts;
    const auto candidate = core.makeCandidate(
        {{10, 7}}, {2, 7}, core.mapToWorld(2, 7),
        std::vector<std::uint8_t>(grid.data.size(), false), 20, 14, counts);

    EXPECT_FALSE(candidate);
    EXPECT_GT(counts.rejected_nav2_clearance, 0);
}

TEST(ClusterFrontierExplorer, VisibilityStopsAtWallsAndIgnoresBehind) {
    // Unknown at (7,5) and (9,5) lie ahead of a viewer at (5,5) looking +x;
    // (3,5) is directly behind it.
    auto open = buildGrid(20, 11, 0.1);
    setCell(open, 7, 5, -1);
    setCell(open, 9, 5, -1);
    setCell(open, 3, 5, -1);
    auto open_core = makeCore(open);
    // Both forward unknowns visible; the one behind the viewer is outside FOV.
    EXPECT_EQ(open_core.countVisibleUnknown({5, 5}, 0.0), 2);

    // A wall at (8,5) blocks the line of sight to the farther unknown (9,5).
    auto walled = buildGrid(20, 11, 0.1);
    setCell(walled, 7, 5, -1);
    setCell(walled, 9, 5, -1);
    setCell(walled, 3, 5, -1);
    setCell(walled, 8, 5, 100);
    auto walled_core = makeCore(walled);
    EXPECT_EQ(walled_core.countVisibleUnknown({5, 5}, 0.0), 1);
}

TEST(ClusterFrontierExplorer, ViewpointTermIsAdditiveAndDisabledAtZeroWeight) {
    // A single unknown cell sits far ahead of every goal option, so all options
    // see exactly the same amount through the camera fan. The viewpoint bonus
    // is therefore uniform and cannot change which option wins: selection stays
    // identical between weight 0 and weight > 0, and the only score difference
    // is the bonus itself.
    auto grid = buildGrid(22, 12, 0.1);
    setCell(grid, 15, 5, -1);
    const std::vector<Cell> cluster{{10, 5}};
    const Cell start{2, 5};
    const std::vector<std::uint8_t> blocked(grid.data.size(), false);

    auto off_core = makeCore(grid);  // viewpoint weight defaults to 0
    FrontierCounts off_counts;
    const auto off = off_core.makeCandidate(
        cluster, start, off_core.mapToWorld(2, 5), blocked, 22, 12, off_counts);

    auto on_core = makeCore(grid);
    auto config = on_core.config();
    config.viewpoint_information_weight = 0.1;
    on_core.setConfig(config);
    FrontierCounts on_counts;
    const auto on = on_core.makeCandidate(
        cluster, start, on_core.mapToWorld(2, 5), blocked, 22, 12, on_counts);

    ASSERT_TRUE(off);
    ASSERT_TRUE(on);
    EXPECT_EQ(requireValue(on).goal_cell, requireValue(off).goal_cell);
    EXPECT_GT(requireValue(on).visible_unknown, 0);
    EXPECT_EQ(requireValue(off).visible_unknown,
              requireValue(on).visible_unknown);
    // Weight 0 omits the term entirely; weight 0.1 subtracts exactly the bonus.
    EXPECT_NEAR(requireValue(off).score - requireValue(on).score,
                0.1 * requireValue(on).visible_unknown, 1e-6);
}

TEST(ClusterFrontierExplorer, UnderObservedTracksRobotProximity) {
    auto grid = buildGrid(20, 20, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.coverage_proximity_m = 1.0;
    core.setConfig(config);

    EXPECT_TRUE(core.isUnderObserved({5.0, 5.0}));  // no history yet
    core.noteRobotVisited({5.0, 5.0});
    EXPECT_FALSE(core.isUnderObserved({5.2, 5.0}));  // within proximity
    EXPECT_TRUE(core.isUnderObserved({8.0, 5.0}));   // 3 m away
}

TEST(ClusterFrontierExplorer, CoverageRevisitFiresOnlyWhenEnabled) {
    // A fully-mapped, all-free grid has no frontier (no unknown), so normal
    // exploration finds nothing. The far free space has never been approached.
    auto grid = buildGrid(30, 12, 0.1);

    auto disabled = makeCore(grid);
    const auto without =
        disabled.findCandidates(disabled.mapToWorld(2, 5), 0.0);
    EXPECT_TRUE(without.candidates.empty());  // regression guard: off = no-op

    auto enabled = makeCore(grid);
    auto config = enabled.config();
    config.coverage_revisit_enabled = true;
    enabled.setConfig(config);
    const auto with = enabled.findCandidates(enabled.mapToWorld(2, 5), 0.0);

    ASSERT_FALSE(with.candidates.empty());
    EXPECT_TRUE(with.candidates.front().is_coverage);
    EXPECT_GT(with.counts.coverage_candidates, 0);
}

TEST(ClusterFrontierExplorer, MalformedMapIsRejectedAndPreviousMapKept) {
    auto grid = buildGrid(30, 12, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.coverage_revisit_enabled = true;
    core.setConfig(config);
    const auto robot = core.mapToWorld(2, 5);
    const auto before = core.findCandidates(robot, 0.0).candidates.size();
    ASSERT_GT(before, 0u);

    auto truncated = grid;
    truncated.data.resize(truncated.data.size() / 2);
    EXPECT_FALSE(core.setMap(truncated));
    auto no_resolution = grid;
    no_resolution.info.resolution = 0.0F;
    EXPECT_FALSE(core.setMap(no_resolution));
    EXPECT_EQ(core.findCandidates(robot, 1.0).candidates.size(), before);
}

TEST(ClusterFrontierExplorer, BlacklistExpiresWhenOnlyCoverageRemains) {
    // No frontier cluster exists, so only coverage revisits can be chosen;
    // their blacklist entries must still expire on schedule.
    auto grid = buildGrid(30, 12, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.coverage_revisit_enabled = true;
    core.setConfig(config);
    const auto robot = core.mapToWorld(2, 5);

    const auto fresh = core.findCandidates(robot, 0.0);
    ASSERT_FALSE(fresh.candidates.empty());
    for (const auto& candidate : fresh.candidates) {
        for (int i = 0; i < config.failed_goal_max_retries; ++i) {
            core.blacklistGoal(candidate.goal_xy, 0.0);
        }
    }
    EXPECT_LT(core.findCandidates(robot, 1.0).candidates.size(),
              fresh.candidates.size());

    const double expired = config.failed_goal_blacklist_sec + 1.0;
    EXPECT_EQ(core.findCandidates(robot, expired).candidates.size(),
              fresh.candidates.size());
}

TEST(ClusterFrontierExplorer, FailureWeightAcceleratesBlacklist) {
    auto grid = buildGrid(20, 10, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.failed_goal_max_retries = 3;
    core.setConfig(config);
    const auto goal = core.mapToWorld(8, 5);

    core.recordGoalFailure(goal, 0.0, 2);

    ASSERT_EQ(core.goalAttempts().size(), 1u);
    EXPECT_EQ(core.goalAttempts().front().failures, 2);
    EXPECT_FALSE(core.isBlacklisted(goal));

    core.recordGoalFailure(goal, 1.0, 1);
    EXPECT_TRUE(core.isBlacklisted(goal));
}

TEST(ClusterFrontierExplorer, HazardRegionsMergeAndUseMaximumSeverity) {
    auto core = makeCore(buildGrid(40, 20, 0.1));

    const auto first = core.recordHazardRegion({1.0, 1.0}, 10.0, 1);
    ASSERT_TRUE(first);
    EXPECT_TRUE(first->created);
    EXPECT_EQ(first->region.severity, 1);
    EXPECT_NEAR(first->region.radius, 0.6, 1e-6);

    const auto repeated = core.recordHazardRegion({1.2, 1.0}, 20.0, 1);
    ASSERT_TRUE(repeated);
    EXPECT_FALSE(repeated->created);
    EXPECT_EQ(repeated->region.severity, 1);
    EXPECT_NEAR(repeated->region.radius, 0.6, 1e-6);

    const auto escalated = core.recordHazardRegion({1.2, 1.0}, 30.0, 2);
    ASSERT_TRUE(escalated);
    EXPECT_FALSE(escalated->created);
    EXPECT_EQ(escalated->region.severity, 2);
    EXPECT_NEAR(escalated->region.radius, 1.0, 1e-6);
    EXPECT_NEAR(escalated->region.expires_at_sec, 630.0, 1e-6);
    EXPECT_EQ(core.hazardRegions().size(), 1u);
}

TEST(ClusterFrontierExplorer, HazardGoalIsRejectedSeparatelyFromBlacklist) {
    auto grid = buildGrid(20, 10, 0.1);
    auto core = makeCore(grid);
    const auto goal_xy = core.mapToWorld(8, 5);
    ASSERT_TRUE(core.recordHazardRegion(goal_xy, 0.0, 1));

    FrontierCounts counts;
    const auto candidate = core.makeCandidate(
        {{8, 5}, {9, 5}, {10, 5}}, {2, 5}, core.mapToWorld(2, 5),
        std::vector<std::uint8_t>(grid.data.size(), false), 20, 10, counts);

    EXPECT_FALSE(candidate);
    EXPECT_GT(counts.rejected_hazard, 0);
    EXPECT_EQ(counts.rejected_blacklist, 0);
}

TEST(ClusterFrontierExplorer, HazardPathPenaltyAddsOncePerRegion) {
    auto core = makeCore(buildGrid(30, 30, 0.1));
    const auto center = core.mapToWorld(10, 5);
    ASSERT_TRUE(core.recordHazardRegion(center, 0.0, 2));

    EXPECT_NEAR(core.hazardPathPenalty({{2, 5}, {10, 5}, {18, 5}}), 200.0,
                1e-6);
    EXPECT_NEAR(core.hazardPathPenalty({{2, 25}, {18, 25}}), 0.0, 1e-6);
}

TEST(ClusterFrontierExplorer, SafeDistanceClearsHazardAtDiameter) {
    auto core = makeCore(buildGrid(40, 20, 0.1));
    ASSERT_TRUE(core.recordHazardRegion({1.0, 1.0}, 0.0, 1));

    EXPECT_TRUE(core.noteHazardTraversal({0.4, 1.0}).empty());
    EXPECT_TRUE(core.noteHazardTraversal({1.0, 1.0}).empty());
    EXPECT_TRUE(core.noteHazardTraversal({0.45, 1.0}).empty());
    const auto cleared = core.noteHazardTraversal({1.6, 1.0});

    ASSERT_EQ(cleared.size(), 1u);
    EXPECT_TRUE(core.hazardRegions().empty());
}

TEST(ClusterFrontierExplorer, LeavingHazardResetsTraversalEntry) {
    auto core = makeCore(buildGrid(40, 20, 0.1));
    ASSERT_TRUE(core.recordHazardRegion({1.0, 1.0}, 0.0, 1));

    core.noteHazardTraversal({0.5, 1.0});
    core.noteHazardTraversal({1.0, 1.0});
    core.noteHazardTraversal({2.0, 1.0});  // leaves the region: entry resets
    core.noteHazardTraversal({1.5, 1.0});
    const auto cleared = core.noteHazardTraversal({1.0, 1.0});

    EXPECT_TRUE(cleared.empty());
    ASSERT_EQ(core.hazardRegions().size(), 1u);
    const auto& region = core.hazardRegions().front();
    ASSERT_TRUE(region.traversal_entry_xy.has_value());
    EXPECT_NEAR(region.traversal_entry_xy->first, 1.5, 1e-6);
    EXPECT_NEAR(region.traversal_min_distance_to_center_m, 0.0, 1e-6);
}

TEST(ClusterFrontierExplorer, SkirtingBoundaryDoesNotClearHazard) {
    // Regression test: walking a wide arc that stays near the boundary the
    // whole time covers a chord as long as the diameter, but never actually
    // revisits the hazard point -- that should NOT be treated as a safe
    // traversal, even though naive path-length accounting would clear it.
    auto core = makeCore(buildGrid(60, 60, 0.1));
    ASSERT_TRUE(core.recordHazardRegion({0.0, 0.0}, 0.0, 2));
    ASSERT_NEAR(core.hazardRegions().front().radius, 1.0, 1e-6);

    core.noteHazardTraversal({1.0, 0.0});
    core.noteHazardTraversal({0.5, 0.8660254});
    core.noteHazardTraversal({-0.5, 0.8660254});
    const auto cleared = core.noteHazardTraversal({-1.0, 0.0});

    EXPECT_TRUE(cleared.empty());
    ASSERT_EQ(core.hazardRegions().size(), 1u);
    EXPECT_GT(core.hazardRegions().front().traversal_min_distance_to_center_m,
              0.5);
}

TEST(ClusterFrontierExplorer, HazardRegionExpires) {
    auto core = makeCore(buildGrid(20, 10, 0.1));
    ASSERT_TRUE(core.recordHazardRegion({1.0, 1.0}, 0.0, 1));

    EXPECT_TRUE(core.pruneHazardRegions(599.0).empty());
    const auto expired = core.pruneHazardRegions(600.0);

    EXPECT_EQ(expired.size(), 1u);
    EXPECT_TRUE(core.hazardRegions().empty());
}

TEST(ClusterFrontierExplorer, GoalOptionsExcludeUnknownCells) {
    auto grid = buildGrid(30, 20, 0.05, -1);
    for (int y = 2; y <= 17; ++y) {
        for (int x = 2; x <= 18; ++x) {
            setCell(grid, x, y, 0);
        }
    }
    auto core = makeCore(grid);
    auto config = core.config();
    config.frontier_goal_search_radius = 0.6;
    core.setConfig(config);

    const auto blocked = core.makeBlockedGrid(30, 20, 0);
    const auto options =
        core.goalOptionsForCluster({{18, 10}}, blocked, 30, 20);

    ASSERT_FALSE(options.empty());
    for (const auto& option : options) {
        EXPECT_LE(option.first, 18);
        EXPECT_TRUE(core.isFreeValue(
            grid.data[core.index(option.first, option.second, 30)]));
    }
}

TEST(ClusterFrontierExplorer, GridPathDoesNotTraverseUnknownCells) {
    auto grid = buildGrid(12, 8, 0.1);
    for (int y = 0; y < 8; ++y) {
        setCell(grid, 5, y, -1);
    }
    auto core = makeCore(grid);
    const auto blocked = core.makeBlockedGrid(12, 8, 0);

    EXPECT_FALSE(core.gridPath({2, 4}, {8, 4}, blocked, 12, 8));
}

TEST(ClusterFrontierExplorer, SweptFootprintRejectsPathThatClipsWall) {
    auto grid = buildGrid(30, 20, 0.05);
    auto core = makeCore(grid);
    const std::vector<Cell> path{{6, 10}, {22, 10}};
    ASSERT_TRUE(core.sweptPathUnknownRatio(path));

    // The obstacle lies between the endpoint footprints, inside the swept path.
    setCell(grid, 14, 10, 100);
    core.setMap(grid);
    EXPECT_FALSE(core.sweptPathUnknownRatio(path));
}

TEST(ClusterFrontierExplorer, FailedGoalBlacklistPreventsReselection) {
    auto grid = buildGrid(20, 10, 0.1);
    auto core = makeCore(grid);
    core.goalAttemptsForTest().push_back({core.mapToWorld(8, 5), 2, 600.0});

    FrontierCounts counts5;
    const auto candidate = core.makeCandidate(
        {{8, 5}, {9, 5}, {10, 5}}, {2, 5}, core.mapToWorld(2, 5),
        std::vector<std::uint8_t>(grid.data.size(), false), 20, 10, counts5);

    EXPECT_FALSE(candidate);
}

TEST(ClusterFrontierExplorer, ExpiredGoalAttemptPrunesFailureCount) {
    auto grid = buildGrid(20, 10, 0.1);
    auto core = makeCore(grid);
    const auto goal_xy = core.mapToWorld(8, 5);
    core.goalAttemptsForTest().push_back({goal_xy, 3, 600.0});

    core.pruneBlacklist(599.0);

    ASSERT_EQ(core.goalAttempts().size(), 1u);
    EXPECT_TRUE(core.isBlacklisted(goal_xy));

    core.pruneBlacklist(601.0);

    EXPECT_TRUE(core.goalAttempts().empty());
    EXPECT_FALSE(core.isBlacklisted(goal_xy));
}

TEST(ClusterFrontierExplorer, NearbyFailedGoalsShareRetryCountInStableZone) {
    auto grid = buildGrid(20, 10, 0.1);
    auto core = makeCore(grid);

    core.blacklistGoal(core.mapToWorld(8, 5), 0.0);
    ASSERT_EQ(core.goalAttempts().size(), 1u);
    const auto original_center = core.goalAttempts()[0].xy;
    EXPECT_EQ(core.goalAttempts()[0].failures, 1);
    EXPECT_FALSE(core.isBlacklisted(core.mapToWorld(8, 5)));

    core.blacklistGoal(core.mapToWorld(10, 5), 30.0);

    ASSERT_EQ(core.goalAttempts().size(), 1u);
    EXPECT_EQ(core.goalAttempts()[0].xy, original_center);
    EXPECT_EQ(core.goalAttempts()[0].failures, 2);
    EXPECT_NEAR(core.goalAttempts()[0].expires_at_sec, 630.0, 1e-6);
    EXPECT_TRUE(core.isBlacklisted(core.mapToWorld(11, 5)));
    EXPECT_FALSE(core.isBlacklisted(core.mapToWorld(12, 5)));
}

TEST(ClusterFrontierExplorer, HeadingPenaltyPrefersForwardCandidate) {
    auto grid = buildGrid(30, 20, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.heading_penalty_weight = 2.0;
    config.heading_penalty_max_rad = 3.141592653589793;
    config.unknown_overlap_weight = 0.0;
    config.wall_clearance_m = 0.1;
    core.setConfig(config);

    const Cell start{10, 10};
    const auto robot_xy = core.mapToWorld(start.first, start.second);
    FrontierCounts counts;
    const auto forward =
        core.makeCandidate({{15, 10}}, start, robot_xy,
                           std::vector<std::uint8_t>(grid.data.size(), false),
                           30, 20, counts, 0.0);
    const auto behind =
        core.makeCandidate({{5, 10}}, start, robot_xy,
                           std::vector<std::uint8_t>(grid.data.size(), false),
                           30, 20, counts, 0.0);

    ASSERT_TRUE(forward);
    ASSERT_TRUE(behind);
    const auto& forward_candidate = requireValue(forward);
    const auto& behind_candidate = requireValue(behind);
    EXPECT_LT(forward_candidate.heading_delta, behind_candidate.heading_delta);
    EXPECT_LT(forward_candidate.score, behind_candidate.score);
}

TEST(ClusterFrontierExplorer, CandidateRejectionCountsIncludeFilters) {
    auto grid = buildGrid(30, 20, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.min_frontier_distance = 0.45;
    config.max_frontier_distance = 0.8;
    config.failed_goal_max_retries = 1;
    core.setConfig(config);
    core.goalAttemptsForTest().push_back({core.mapToWorld(8, 2), 2, 600.0});

    const Cell start{10, 10};
    const auto robot_xy = core.mapToWorld(start.first, start.second);
    FrontierCounts counts;
    const std::vector<std::uint8_t> blocked(grid.data.size(), false);

    EXPECT_FALSE(core.makeCandidate({{11, 2}}, start, robot_xy, blocked, 30, 20,
                                    counts));
    EXPECT_FALSE(core.makeCandidate({{11, 10}}, start, robot_xy, blocked, 30,
                                    20, counts));
    EXPECT_FALSE(core.makeCandidate({{25, 10}}, start, robot_xy, blocked, 30,
                                    20, counts));

    EXPECT_GE(counts.rejected_blacklist, 1);
    EXPECT_GE(counts.rejected_too_near, 1);
    EXPECT_GE(counts.rejected_too_far, 1);
}

// realcar_20261122_111558: goals 0.5 m beside the robot with goal_yaw +-90 deg
// cannot be reached driving forward with a 0.5 m turning radius; Smac planned
// reverse cusps (the V-shaped back-and-forth) and the frontier stayed unseen.
TEST(ClusterFrontierExplorer, RejectsGoalsInsideTurningCircles) {
    auto grid = buildGrid(40, 40, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.min_turning_radius_m = 0.5;
    core.setConfig(config);

    const Cell start{20, 20};
    const auto robot_xy = core.mapToWorld(start.first, start.second);
    const std::vector<std::uint8_t> blocked(grid.data.size(), false);
    const double robot_yaw = 0.0;

    FrontierCounts counts;
    EXPECT_FALSE(core.makeCandidate({{20, 25}}, start, robot_xy, blocked, 40,
                                    40, counts, robot_yaw));  // 0.5 m left
    EXPECT_FALSE(core.makeCandidate({{20, 15}}, start, robot_xy, blocked, 40,
                                    40, counts, robot_yaw));  // 0.5 m right
    EXPECT_GE(counts.rejected_turning_circle, 2);

    FrontierCounts ahead_counts;
    EXPECT_TRUE(core.makeCandidate({{27, 20}}, start, robot_xy, blocked, 40, 40,
                                   ahead_counts, robot_yaw));  // 0.7 m ahead
    EXPECT_EQ(ahead_counts.rejected_turning_circle, 0);

    // Without a heading the turning circles are undefined: no rejection.
    FrontierCounts no_yaw_counts;
    EXPECT_TRUE(core.makeCandidate({{20, 25}}, start, robot_xy, blocked, 40, 40,
                                   no_yaw_counts));
}

TEST(ClusterFrontierExplorer, ShouldPublishGoalHonorsDistanceAndPeriod) {
    auto grid = buildGrid(20, 10, 0.1);
    auto core = makeCore(grid);
    const auto goal = core.mapToWorld(8, 5);

    EXPECT_TRUE(core.shouldPublishGoal(goal, 0.0));
    core.noteGoalPublished(goal, 0.0, 0.0);
    EXPECT_FALSE(core.shouldPublishGoal(goal, 1.0));
    EXPECT_FALSE(core.shouldPublishGoal({goal.first + 0.49, goal.second}, 1.0));
    EXPECT_TRUE(core.shouldPublishGoal({goal.first + 0.51, goal.second}, 1.0));
    EXPECT_FALSE(core.shouldPublishGoal(goal, 2.99));
    EXPECT_TRUE(core.shouldPublishGoal(goal, 3.0));
}

TEST(ClusterFrontierExplorer, NoCandidateCountsRemainCompact) {
    auto grid = buildGrid(10, 10, 0.1);
    auto core = makeCore(grid);
    const auto result = core.findCandidates(core.mapToWorld(2, 2), 0.0);

    EXPECT_TRUE(result.candidates.empty());
    EXPECT_EQ(result.counts.free, 100);
    EXPECT_EQ(result.counts.frontier, 0);
}

// Map layout: free corridor at x=2..18, unknown space at x=19..29.
// Frontier cells at x=18 are free and adjacent to unknown. A very strict
// unknown-overlap threshold still rejects these frontier-edge candidates.

TEST(ClusterFrontierExplorer, StrictUnknownRatioThresholdBlocksAllCandidates) {
    auto grid = buildGrid(30, 20, 0.05, -1);
    for (int y = 2; y <= 17; ++y)
        for (int x = 2; x <= 18; ++x) setCell(grid, x, y, 0);
    for (int x = 0; x < 30; ++x) {
        setCell(grid, x, 0, 100);
        setCell(grid, x, 1, 100);
        setCell(grid, x, 18, 100);
        setCell(grid, x, 19, 100);
    }
    for (int y = 0; y < 20; ++y) {
        setCell(grid, 0, y, 100);
        setCell(grid, 1, y, 100);
    }

    auto core = makeCore(grid);
    auto config = core.config();
    config.min_clearance_cells = 3;
    config.path_clearance_cells = 1;
    config.unknown_overlap_max_ratio = 0.1;
    config.frontier_goal_search_radius = 0.6;
    config.goal_publish_period = 0.0;
    config.goal_update_distance = 0.0;
    core.setConfig(config);

    const auto result = core.findCandidates(core.mapToWorld(8, 10), 0.0);

    EXPECT_TRUE(result.candidates.empty());
    EXPECT_GT(result.counts.clusters, 0);
    EXPECT_GT(
        result.counts.rejected_unknown_ratio + result.counts.rejected_swept, 0);
}

TEST(ClusterFrontierExplorer, NewUnknownRatioThresholdFindsCandidates) {
    auto grid = buildGrid(30, 20, 0.05, -1);
    for (int y = 2; y <= 17; ++y)
        for (int x = 2; x <= 18; ++x) setCell(grid, x, y, 0);
    for (int x = 0; x < 30; ++x) {
        setCell(grid, x, 0, 100);
        setCell(grid, x, 1, 100);
        setCell(grid, x, 18, 100);
        setCell(grid, x, 19, 100);
    }
    for (int y = 0; y < 20; ++y) {
        setCell(grid, 0, y, 100);
        setCell(grid, 1, y, 100);
    }

    auto core = makeCore(grid);
    auto config = core.config();
    config.min_clearance_cells = 2;
    config.path_clearance_cells = 2;
    config.unknown_overlap_max_ratio = 1.0;
    config.frontier_goal_search_radius = 0.6;
    config.goal_publish_period = 0.0;
    config.goal_update_distance = 0.0;
    core.setConfig(config);

    const auto result = core.findCandidates(core.mapToWorld(8, 10), 0.0);

    ASSERT_FALSE(result.candidates.empty());
    EXPECT_GT(result.counts.clusters, 0);
    EXPECT_GE(result.candidates.front().path_length, 0.45);
}

TEST(ClusterFrontierExplorer, ProjectsPointOntoBfsPolylineInWorldMeters) {
    auto grid = buildGrid(20, 20, 0.1);
    auto core = makeCore(grid);
    const std::vector<Cell> path{{2, 5}, {8, 5}, {8, 10}};

    const auto near_first_segment =
        core.projectOntoPath(core.mapToWorld(5, 7), path);
    const auto near_second_segment =
        core.projectOntoPath(core.mapToWorld(10, 8), path);

    ASSERT_TRUE(near_first_segment);
    const auto& first_projection = requireValue(near_first_segment);
    EXPECT_NEAR(first_projection.distance, 0.2, 1e-6);
    EXPECT_NEAR(first_projection.arc_length, 0.3, 1e-6);
    ASSERT_TRUE(near_second_segment);
    const auto& second_projection = requireValue(near_second_segment);
    EXPECT_NEAR(second_projection.distance, 0.2, 1e-6);
    EXPECT_NEAR(second_projection.arc_length, 0.9, 1e-6);
}

TEST(ClusterFrontierExplorer, BuildsSubgoalsNearBfsPathOnly) {
    auto grid = buildGrid(30, 20, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.subgoal_path_tolerance_m = 0.21;
    config.min_subgoal_spacing_m = 0.3;
    core.setConfig(config);

    auto final_goal = makeCandidateAt(core, {18, 5, 1.0});
    final_goal.path = {{2, 5}, {18, 5}};
    auto near_path = makeCandidateAt(core, {8, 6, 2.0});
    auto off_path = makeCandidateAt(core, {10, 9, 3.0});
    const std::vector<FrontierCandidate> candidates{final_goal, off_path,
                                                    near_path};

    const auto subgoals = core.buildSubgoalsForRoute(final_goal, candidates);

    ASSERT_EQ(subgoals.size(), 1u);
    EXPECT_EQ(subgoals.front().goal_cell, near_path.goal_cell);
}

TEST(ClusterFrontierExplorer, SortsAcceptedSubgoalsByBfsArcLength) {
    auto grid = buildGrid(30, 20, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.subgoal_path_tolerance_m = 0.2;
    config.min_subgoal_spacing_m = 0.2;
    core.setConfig(config);

    auto final_goal = makeCandidateAt(core, {18, 5, 1.0});
    final_goal.path = {{2, 5}, {18, 5}};
    auto later = makeCandidateAt(core, {14, 5, 2.0});
    auto earlier = makeCandidateAt(core, {6, 5, 3.0});

    const auto subgoals =
        core.buildSubgoalsForRoute(final_goal, {final_goal, later, earlier});

    ASSERT_EQ(subgoals.size(), 2u);
    EXPECT_EQ(subgoals[0].goal_cell, earlier.goal_cell);
    EXPECT_EQ(subgoals[1].goal_cell, later.goal_cell);
}

TEST(ClusterFrontierExplorer, EnforcesSubgoalSpacingAndLimit) {
    auto grid = buildGrid(30, 20, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.max_subgoals = 2;
    config.subgoal_path_tolerance_m = 0.2;
    config.min_subgoal_spacing_m = 0.45;
    core.setConfig(config);

    auto final_goal = makeCandidateAt(core, {24, 5, 1.0});
    final_goal.path = {{2, 5}, {24, 5}};
    auto first = makeCandidateAt(core, {6, 5, 2.0});
    auto duplicate = makeCandidateAt(core, {8, 5, 3.0});
    auto second = makeCandidateAt(core, {13, 5, 4.0});
    auto over_limit = makeCandidateAt(core, {19, 5, 5.0});

    const auto subgoals = core.buildSubgoalsForRoute(
        final_goal, {final_goal, first, duplicate, second, over_limit});

    ASSERT_EQ(subgoals.size(), 2u);
    EXPECT_EQ(subgoals[0].goal_cell, first.goal_cell);
    EXPECT_EQ(subgoals[1].goal_cell, second.goal_cell);
}

TEST(ClusterFrontierExplorer, ProgressRequiresBfsArcAdvance) {
    auto grid = buildGrid(30, 20, 0.1);
    auto core = makeCore(grid);
    const std::vector<Cell> path{{2, 5}, {20, 5}};
    const auto start = core.projectOntoPath(core.mapToWorld(3, 5), path);
    const auto forward = core.projectOntoPath(core.mapToWorld(5, 5), path);
    const auto lateral = core.projectOntoPath(core.mapToWorld(3, 8), path);

    ASSERT_TRUE(start);
    ASSERT_TRUE(forward);
    ASSERT_TRUE(lateral);
    const auto& start_projection = requireValue(start);
    const auto& forward_projection = requireValue(forward);
    const auto& lateral_projection = requireValue(lateral);
    EXPECT_GE(forward_projection.arc_length - start_projection.arc_length,
              0.12);
    EXPECT_LT(
        std::abs(lateral_projection.arc_length - start_projection.arc_length),
        1e-6);
}

nav_msgs::msg::OccupancyGrid buildHalfUnknownGrid() {
    auto grid = buildGrid(20, 10, 0.1);
    for (int y = 0; y < 10; ++y) {
        for (int x = 12; x < 20; ++x) {
            setCell(grid, x, y, -1);
        }
    }
    return grid;
}

FrontierExplorerCore makeRelaxableCore() {
    auto core = makeCore(buildHalfUnknownGrid());
    auto config = core.config();
    config.min_frontier_size = 50;
    config.unknown_overlap_max_ratio = 0.7;
    config.relaxed_min_frontier_size = 4;
    config.relaxed_max_frontier_distance = 5.0;
    core.setConfig(config);
    return core;
}

TEST(ClusterFrontierExplorer, SmallClustersAreOfferedInTheFallback) {
    auto core = makeRelaxableCore();
    const auto robot_xy = core.mapToWorld(2, 5);
    const auto strict = core.findCandidates(robot_xy, 100.0, 0.0);
    EXPECT_TRUE(strict.candidates.empty());
    EXPECT_FALSE(strict.relaxed);
    core.setRequireViewTail(false);
    const auto fallback = core.findCandidates(robot_xy, 100.0, 0.0);
    EXPECT_TRUE(fallback.relaxed);
    EXPECT_FALSE(fallback.candidates.empty());
}

TEST(ClusterFrontierExplorer, ClearestEscapeDirectionAvoidsBlockedForwardRay) {
    auto grid = buildGrid(31, 31, 0.1F, 100);
    setCell(grid, 15, 15, 0);
    for (int y = 16; y < 31; ++y) {
        setCell(grid, 15, y, 0);
    }
    auto core = makeCore(grid);

    const auto yaw =
        core.findClearestEscapeDirection(core.mapToWorld(15, 15), 1.0, 8);

    ASSERT_TRUE(yaw);
    EXPECT_NEAR(requireValue(yaw), kPi / 2.0, 1e-6);
}

TEST(ClusterFrontierExplorer,
     ClearestEscapeDirectionReturnsNulloptWhenBlocked) {
    auto grid = buildGrid(11, 11, 0.1F, 100);
    setCell(grid, 5, 5, 0);
    auto core = makeCore(grid);

    const auto yaw =
        core.findClearestEscapeDirection(core.mapToWorld(5, 5), 0.5, 8);

    EXPECT_FALSE(yaw);
}

TEST(ClusterFrontierExplorer, ClearestEscapeDirectionPrefersFreeOverUnknown) {
    auto grid = buildGrid(21, 21, 0.1F, -1);
    setCell(grid, 10, 10, 0);
    for (int x = 0; x < 10; ++x) {
        setCell(grid, x, 10, 0);
    }
    auto core = makeCore(grid);

    const auto yaw =
        core.findClearestEscapeDirection(core.mapToWorld(10, 10), 0.8, 4);

    ASSERT_TRUE(yaw);
    EXPECT_NEAR(requireValue(yaw), kPi, 1e-6);
}

TEST(ClusterFrontierExplorer,
     ClearestEscapeDirectionUsesDeterministicTieBreak) {
    auto grid = buildGrid(31, 31, 0.1);
    auto core = makeCore(grid);

    const auto yaw =
        core.findClearestEscapeDirection(core.mapToWorld(15, 15), 0.8, 8);

    ASSERT_TRUE(yaw);
    EXPECT_NEAR(requireValue(yaw), 0.0, 1e-6);
}

TEST(ClusterFrontierExplorer, HeadingErrorWrapsAcrossPi) {
    EXPECT_NEAR(headingDelta(3.10, -3.10), 0.083185307, 1e-6);
    EXPECT_NEAR(headingDelta(-3.10, 3.10), 0.083185307, 1e-6);
}

TEST(ClusterFrontierExplorer, PathTargetYawUsesNearestPathSegment) {
    auto grid = buildGrid(20, 20, 0.1);
    auto core = makeCore(grid);
    const std::vector<Cell> path{{2, 5}, {8, 5}, {8, 10}};

    const auto first_segment =
        core.pathTargetYawNear(core.mapToWorld(5, 6), path);
    const auto second_segment =
        core.pathTargetYawNear(core.mapToWorld(9, 8), path);

    ASSERT_TRUE(first_segment);
    ASSERT_TRUE(second_segment);
    EXPECT_NEAR(requireValue(first_segment), 0.0, 1e-6);
    EXPECT_NEAR(requireValue(second_segment), (std::numbers::pi / 2.0), 1e-6);
}

// ---------------------------------------------------------------------------
// Coverage Blend (F1/F3): coverage candidates participate in main pool
// ---------------------------------------------------------------------------

TEST(ClusterFrontierExplorer, CoverageBlendAddsToMainPool) {
    // Frontier and coverage candidates must coexist when blending is enabled.
    auto grid = buildGrid(40, 20, 0.1);
    // Mark a frontier cluster at column 10 (x=10, y=5..14)
    for (int y = 5; y < 15; ++y) {
        setCell(grid, 10, y, -1);  // unknown — creates a frontier edge
    }
    FrontierExplorerCore core = makeCore(grid);
    auto config = core.config();
    config.min_frontier_size = 1;
    config.coverage_revisit_enabled = true;
    config.coverage_blend_weight =
        5.0;  // large bonus to guarantee coverage wins
    config.coverage_proximity_m = 0.3;
    config.coverage_sample_spacing_m = 0.5;
    config.coverage_max_targets = 5;
    config.max_frontier_distance = 10.0;
    core.setConfig(config);

    const WorldPoint robot_xy = core.mapToWorldUnchecked(2, 10);
    // Note a visit only at robot start so cells far away are under-observed
    core.noteRobotVisited(robot_xy);

    auto result = core.findCandidates(robot_xy, 0.0, std::nullopt);

    // With blend enabled we should get BOTH frontier and coverage candidates
    EXPECT_GT(result.counts.coverage_candidates, 0);
    ASSERT_FALSE(result.candidates.empty());
    EXPECT_TRUE(std::any_of(result.candidates.begin(), result.candidates.end(),
                            [](const auto& c) { return !c.is_coverage; }));
    const bool has_coverage =
        std::any_of(result.candidates.begin(), result.candidates.end(),
                    [](const auto& c) { return c.is_coverage; });
    EXPECT_TRUE(has_coverage);
}

TEST(ClusterFrontierExplorer, CoverageBlendZeroKeepsFallbackBehavior) {
    // coverage_blend_weight = 0: coverage candidates should NOT appear when
    // frontier candidates exist (preserved fallback-only behaviour).
    auto grid = buildGrid(40, 20, 0.1);
    for (int y = 5; y < 15; ++y) {
        setCell(grid, 10, y, -1);
    }
    FrontierExplorerCore core = makeCore(grid);
    auto config = core.config();
    config.min_frontier_size = 1;
    config.coverage_revisit_enabled = true;
    config.coverage_blend_weight = 0.0;  // disabled
    config.coverage_proximity_m = 0.3;
    config.max_frontier_distance = 10.0;
    core.setConfig(config);

    const WorldPoint robot_xy = core.mapToWorldUnchecked(2, 10);
    core.noteRobotVisited(robot_xy);

    auto result = core.findCandidates(robot_xy, 0.0, std::nullopt);

    // With blend disabled, coverage candidates must not appear alongside
    // frontier candidates.
    ASSERT_FALSE(result.candidates.empty());
    const bool has_coverage =
        std::any_of(result.candidates.begin(), result.candidates.end(),
                    [](const auto& c) { return c.is_coverage; });
    EXPECT_FALSE(has_coverage);
}

TEST(ClusterFrontierExplorer, CoverageBlendWeightLowersCoverageScoresOnly) {
    auto grid = buildGrid(40, 20, 0.1);
    setCell(grid, 30, 10, -1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.min_frontier_size = 1;
    config.coverage_revisit_enabled = true;
    config.coverage_blend_weight = 1.0;
    config.coverage_proximity_m = 0.3;
    config.coverage_sample_spacing_m = 2.0;
    config.coverage_max_targets = 3;
    config.max_frontier_distance = 10.0;
    core.setConfig(config);
    const auto robot_xy = core.mapToWorld(2, 10);
    const auto before = core.findCandidates(robot_xy, 0.0);
    ASSERT_TRUE(std::any_of(before.candidates.begin(), before.candidates.end(),
                            [](const auto& c) { return c.is_coverage; }));
    ASSERT_TRUE(std::any_of(before.candidates.begin(), before.candidates.end(),
                            [](const auto& c) { return !c.is_coverage; }));

    config.coverage_blend_weight = 3.0;
    core.setConfig(config);
    const auto after = core.findCandidates(robot_xy, 1.0);
    ASSERT_EQ(after.candidates.size(), before.candidates.size());
    for (const auto& candidate : before.candidates) {
        const auto match =
            std::find_if(after.candidates.begin(), after.candidates.end(),
                         [&](const auto& c) {
                             return c.goal_cell == candidate.goal_cell &&
                                    c.is_coverage == candidate.is_coverage;
                         });
        ASSERT_NE(match, after.candidates.end());
        EXPECT_NEAR(match->score,
                    candidate.score - (candidate.is_coverage ? 2.0 : 0.0),
                    1e-9);
    }
}

// ---------------------------------------------------------------------------
// Intention State (F9): cross-cycle goal commitment
// ---------------------------------------------------------------------------

TEST(ClusterFrontierExplorer, IntentionSetOnFarGoal) {
    auto grid = buildGrid(20, 20, 0.1);
    FrontierExplorerCore core = makeCore(grid);
    auto config = core.config();
    config.intention_distance_threshold_m = 1.0;
    config.intention_continuation_bonus = 2.0;
    config.intention_clear_radius_m = 0.5;
    core.setConfig(config);

    EXPECT_FALSE(core.intention().has_value());
    core.noteGoalPublished({1.5, 1.5}, 0.0, 0.0,
                           2.0);  // path_length=2.0 > threshold
    ASSERT_TRUE(core.intention().has_value());
    EXPECT_EQ(core.intention()->goal_xy, (WorldPoint{1.5, 1.5}));
}

TEST(ClusterFrontierExplorer, IntentionNotSetOnNearGoal) {
    auto grid = buildGrid(20, 20, 0.1);
    FrontierExplorerCore core = makeCore(grid);
    auto config = core.config();
    config.intention_distance_threshold_m = 2.0;
    config.intention_continuation_bonus = 2.0;
    core.setConfig(config);

    core.noteGoalPublished({1.5, 1.5}, 0.0, 0.0,
                           0.8);  // path_length < threshold
    EXPECT_FALSE(core.intention().has_value());
}

TEST(ClusterFrontierExplorer, IntentionClearedOnArrival) {
    auto grid = buildGrid(40, 40, 0.1);
    FrontierExplorerCore core = makeCore(grid);
    auto config = core.config();
    config.intention_distance_threshold_m = 1.0;
    config.intention_continuation_bonus = 2.0;
    config.intention_clear_radius_m = 0.5;
    core.setConfig(config);

    const WorldPoint intention_xy{3.0, 3.0};
    core.noteGoalPublished(intention_xy, 0.0, 0.0, 2.0);
    ASSERT_TRUE(core.intention().has_value());

    // Robot is far: intention persists
    core.findCandidates({0.5, 0.5}, 1.0, std::nullopt);
    EXPECT_TRUE(core.intention().has_value());

    // Robot arrives within clear_radius: intention cleared
    core.findCandidates(intention_xy, 2.0, std::nullopt);
    EXPECT_FALSE(core.intention().has_value());
}

TEST(ClusterFrontierExplorer, IntentionClearedOnTimeout) {
    auto grid = buildGrid(40, 40, 0.1);
    FrontierExplorerCore core = makeCore(grid);
    auto config = core.config();
    config.intention_distance_threshold_m = 1.0;
    config.intention_continuation_bonus = 2.0;
    config.intention_clear_radius_m = 0.5;
    config.intention_timeout_sec = 10.0;
    core.setConfig(config);

    core.noteGoalPublished({3.0, 3.0}, 0.0, 0.0, 2.0);
    ASSERT_TRUE(core.intention().has_value());

    // Before timeout: persists
    core.findCandidates({0.5, 0.5}, 5.0, std::nullopt);
    EXPECT_TRUE(core.intention().has_value());

    // After timeout: cleared
    core.findCandidates({0.5, 0.5}, 15.0, std::nullopt);
    EXPECT_FALSE(core.intention().has_value());
}

TEST(ClusterFrontierExplorer, IntentionBonusLowersScoreForNearbyCandidate) {
    // Verify that a candidate near the intention goal gets a lower score
    // (more attractive) than an equidistant candidate elsewhere.
    auto grid = buildGrid(80, 20, 0.1);
    // Two frontier clusters equidistant from robot
    for (int y = 5; y < 10; ++y) {
        setCell(grid, 20, y, -1);  // cluster A at x=20
        setCell(grid, 60, y, -1);  // cluster B at x=60
    }
    FrontierExplorerCore core = makeCore(grid);
    auto config = core.config();
    config.min_frontier_size = 1;
    config.intention_distance_threshold_m = 1.0;
    config.intention_continuation_bonus = 5.0;
    config.intention_clear_radius_m = 1.5;
    config.max_frontier_distance = 100.0;
    config.path_weight = 1.0;
    config.information_weight = 0.0;
    config.frontier_size_weight = 0.0;
    config.corridor_progress_weight = 0.0;
    core.setConfig(config);

    const WorldPoint robot_xy = core.mapToWorldUnchecked(40, 10);

    // Set intention near cluster B (x=60)
    const WorldPoint intention_xy = core.mapToWorldUnchecked(60, 7);
    core.noteGoalPublished(intention_xy, 0.0, 0.0, 5.0);
    ASSERT_TRUE(core.intention().has_value());

    const auto with_bonus = core.findCandidates(robot_xy, 1.0);
    ASSERT_GE(with_bonus.candidates.size(), 2u);
    config.intention_continuation_bonus = 0.0;
    core.setConfig(config);
    const auto without_bonus = core.findCandidates(robot_xy, 1.0);
    ASSERT_EQ(with_bonus.candidates.size(), without_bonus.candidates.size());

    int near_count = 0;
    int far_count = 0;
    for (const auto& candidate : with_bonus.candidates) {
        const auto match = std::find_if(
            without_bonus.candidates.begin(), without_bonus.candidates.end(),
            [&](const auto& c) { return c.goal_cell == candidate.goal_cell; });
        ASSERT_NE(match, without_bonus.candidates.end());
        const bool near =
            std::hypot(candidate.goal_xy.first - intention_xy.first,
                       candidate.goal_xy.second - intention_xy.second) <= 3.0;
        near ? ++near_count : ++far_count;
        EXPECT_NEAR(match->score - candidate.score, near ? 5.0 : 0.0, 1e-9);
    }
    EXPECT_GT(near_count, 0);
    EXPECT_GT(far_count, 0);
}

TEST(LethalCostmapCell, DetectsLethalStartCellOnly) {
    nav_msgs::msg::OccupancyGrid costmap;
    costmap.info.resolution = 0.05;
    costmap.info.width = 4;
    costmap.info.height = 3;
    costmap.info.origin.position.x = -0.1;
    costmap.info.origin.position.y = 0.0;
    costmap.data.assign(12, 0);
    costmap.data[1 * 4 + 2] =
        100;  // cell (2, 1) = world [0.0, 0.05) x [0.05, 0.1)
    costmap.data[1 * 4 + 1] = 99;  // inscribed, not lethal

    EXPECT_TRUE(isLethalCostmapCell(costmap, {0.02, 0.07}));
    EXPECT_FALSE(isLethalCostmapCell(costmap, {-0.03, 0.07}));
    EXPECT_FALSE(isLethalCostmapCell(costmap, {0.07, 0.07}));
    EXPECT_FALSE(isLethalCostmapCell(costmap, {0.5, 0.07}));   // out of bounds
    EXPECT_FALSE(isLethalCostmapCell(costmap, {-0.2, 0.07}));  // out of bounds
    EXPECT_FALSE(
        isLethalCostmapCell(nav_msgs::msg::OccupancyGrid{}, {0.02, 0.07}));
}

TEST(FootprintBlockedInCostmap, MirrorsSmacGoalFootprintCheck) {
    // 1 m x 1 m at 0.05 m, origin (0, 0); pose at the center of cell (10, 10).
    nav_msgs::msg::OccupancyGrid costmap;
    costmap.info.resolution = 0.05;
    costmap.info.width = 20;
    costmap.info.height = 20;
    costmap.data.assign(400, 0);
    const auto set = [&](int x, int y, int8_t v) {
        costmap.data[static_cast<std::size_t>(y) * 20 + x] = v;
    };
    FootprintGeometry footprint;  // 0.42 x 0.27 m, like the padded R2
    footprint.front = 0.21;
    footprint.rear = 0.21;
    footprint.half_width = 0.135;
    const WorldPoint pose{0.525, 0.525};

    EXPECT_FALSE(isFootprintBlockedInCostmap(costmap, footprint, pose, 0.0));

    // Lethal cell 0.15 m ahead (cell 13, center x 0.675): inside the long
    // axis at yaw 0, outside the half width at yaw pi/2.
    set(13, 10, 100);
    EXPECT_TRUE(isFootprintBlockedInCostmap(costmap, footprint, pose, 0.0));
    EXPECT_FALSE(isFootprintBlockedInCostmap(costmap, footprint, pose,
                                             std::numbers::pi / 2.0));

    // Inscribed (99) under the footprint but off-center is not a collision.
    set(13, 10, 99);
    EXPECT_FALSE(isFootprintBlockedInCostmap(costmap, footprint, pose, 0.0));
    // Inscribed at the center cell is.
    set(10, 10, 99);
    EXPECT_TRUE(isFootprintBlockedInCostmap(costmap, footprint, pose, 0.0));
    set(10, 10, 0);

    // Out of bounds and an empty grid are not blocked.
    EXPECT_FALSE(
        isFootprintBlockedInCostmap(costmap, footprint, {1.5, 0.5}, 0.0));
    EXPECT_FALSE(isFootprintBlockedInCostmap(nav_msgs::msg::OccupancyGrid{},
                                             footprint, pose, 0.0));
}

nav_msgs::msg::OccupancyGrid bearingGrid() {
    // 41x41 cells at 0.05 m centred on the origin, all free.
    nav_msgs::msg::OccupancyGrid map;
    map.info.resolution = 0.05;
    map.info.width = 41;
    map.info.height = 41;
    map.info.origin.position.x = -1.025;
    map.info.origin.position.y = -1.025;
    map.data.assign(41 * 41, 0);
    return map;
}

TEST(UnknownBearing, PointsTowardUnknownBehindRobot) {
    auto map = bearingGrid();
    for (int my = 0; my < 41; ++my) {
        for (int mx = 0; mx < 15; ++mx) {  // unknown band at x < -0.3
            map.data[my * 41 + mx] = -1;
        }
    }
    const auto bearing = unknownBearing(map, {0.0, 0.0}, 0.0, 0.87, 1.0);
    ASSERT_TRUE(bearing.has_value());
    EXPECT_NEAR(std::abs(*bearing), kPi, 0.15);
}

TEST(UnknownBearing, IgnoresUnknownInsideCurrentView) {
    // Robot faces +x; the unknown ahead (occluded) outweighs the strip
    // behind, but only the strip behind is outside the view.
    auto map = bearingGrid();
    for (int my = 0; my < 41; ++my) {
        for (int mx = 0; mx < 41; ++mx) {
            // Ahead band limited to |y| <= 0.3 m so it stays inside the
            // +-50 deg view; the strip behind is outside it.
            if ((mx > 26 && my >= 14 && my <= 26) ||
                (mx < 8 && my > 15 && my < 25)) {
                map.data[my * 41 + mx] = -1;
            }
        }
    }
    const auto bearing = unknownBearing(map, {0.0, 0.0}, 0.0, 0.87, 1.0);
    ASSERT_TRUE(bearing.has_value());
    EXPECT_NEAR(std::abs(*bearing), kPi, 0.2);
}

TEST(UnknownBearing, NoneWhenAllUnknownIsInsideView) {
    auto map = bearingGrid();
    for (int my = 0; my < 41; ++my) {
        for (int mx = 27; mx < 41; ++mx) {
            if (my >= 14 && my <= 26) {
                map.data[my * 41 + mx] = -1;  // unknown only ahead, in view
            }
        }
    }
    EXPECT_FALSE(unknownBearing(map, {0.0, 0.0}, 0.0, 0.87, 0.8).has_value());
}

TEST(UnknownBearing, NoneWhenKnownAndBehindWhenSurrounded) {
    auto map = bearingGrid();
    EXPECT_FALSE(unknownBearing(map, {0.0, 0.0}, 0.0, 0.87, 1.0).has_value());
    // Unknown ring all around: with the view cone excluded, what is left is
    // to the sides and behind, so the turn points behind.
    for (int my = 0; my < 41; ++my) {
        for (int mx = 0; mx < 41; ++mx) {
            const double x = -1.0 + mx * 0.05;
            const double y = -1.0 + my * 0.05;
            if (std::hypot(x, y) > 0.6) {
                map.data[my * 41 + mx] = -1;
            }
        }
    }
    const auto bearing = unknownBearing(map, {0.0, 0.0}, 0.0, 0.87, 1.0);
    ASSERT_TRUE(bearing.has_value());
    EXPECT_NEAR(std::abs(*bearing), kPi, 0.15);
}

TEST(UnknownBearing, SpaceOutsideMapCountsAsUnknown) {
    // Like a fresh RTAB-Map /map: the grid starts at the robot and only
    // covers the observed area ahead, so everything behind is off the grid.
    nav_msgs::msg::OccupancyGrid map;
    map.info.resolution = 0.05;
    map.info.width = 30;
    map.info.height = 60;
    map.info.origin.position.x = -0.025;
    map.info.origin.position.y = -1.5;
    map.data.assign(30 * 60, 0);
    const auto bearing = unknownBearing(map, {0.0, 0.0}, 0.0, 0.87, 1.0);
    ASSERT_TRUE(bearing.has_value());
    EXPECT_NEAR(std::abs(*bearing), kPi, 0.15);
}

FrontierExplorerCore makeViewTailCore(
    const nav_msgs::msg::OccupancyGrid& grid) {
    auto core = makeCore(grid);
    auto config = core.config();
    config.view_tail_length_m = 1.40;
    config.view_stop_offset_m = 0.35;
    config.viewpoint_half_fov_rad = 0.5;
    config.viewpoint_range_m = 3.0;
    core.setConfig(config);
    return core;
}

TEST(ClusterFrontierExplorer, ViewTailFacesTheUnknownAlongAFreeStraightTail) {
    // Known free room x < 3.0 m, unknown beyond. The goal 0.55 m short of
    // the unknown must be approached straight along a yaw that looks into it.
    auto grid = buildGrid(40, 30, 0.1);
    for (int y = 0; y < 30; ++y) {
        for (int x = 30; x < 40; ++x) {
            setCell(grid, x, y, -1);
        }
    }
    auto core = makeViewTailCore(grid);
    const std::vector<Cell> cluster{{24, 15}};
    const std::vector<std::uint8_t> blocked(grid.data.size(), false);
    FrontierCounts counts;
    const auto candidate = core.makeCandidate(
        cluster, {5, 15}, core.mapToWorld(5, 15), blocked, 40, 30, counts);

    ASSERT_TRUE(candidate);
    const auto& view = requireValue(candidate);
    EXPECT_LE(std::abs(view.goal_yaw), 0.5);
    EXPECT_GT(view.visible_unknown, 0);
    EXPECT_EQ(static_cast<std::size_t>(view.visible_unknown),
              view.view_cells.size());
    // Every target is currently unknown, so none counts as observed yet.
    EXPECT_EQ(core.countObserved(view.view_cells), 0);
}

TEST(ClusterFrontierExplorer, ViewTailRejectsAGoalNoStraightTailFits) {
    // A 2 m square map: from its centre every 1.4 m tail (plus the rear of
    // the footprint) leaves the map, which the footprint rule treats as
    // occupied. Without the tail the same goal is accepted.
    auto grid = buildGrid(20, 20, 0.1);
    setCell(grid, 15, 15, -1);
    const std::vector<Cell> cluster{{10, 10}};
    const std::vector<std::uint8_t> blocked(grid.data.size(), false);

    auto plain = makeCore(grid);
    FrontierCounts plain_counts;
    EXPECT_TRUE(plain.makeCandidate(cluster, {4, 10}, plain.mapToWorld(4, 10),
                                    blocked, 20, 20, plain_counts));

    auto core = makeViewTailCore(grid);
    FrontierCounts counts;
    EXPECT_FALSE(core.makeCandidate(cluster, {4, 10}, core.mapToWorld(4, 10),
                                    blocked, 20, 20, counts));
    // The one-cell search radius offers five goal cells; each is rejected.
    EXPECT_EQ(counts.rejected_footprint, 5);
    // The tail is a preference: without it the same goal is offered.
    core.setRequireViewTail(false);
    EXPECT_TRUE(core.makeCandidate(cluster, {4, 10}, core.mapToWorld(4, 10),
                                   blocked, 20, 20, counts));
}

TEST(ClusterFrontierExplorer, RejectedPlanExcludesOnlyThatGoalUntilCleared) {
    auto core = makeCore(buildGrid(10, 10, 0.1));
    core.rejectPlan({1.0, 1.0});
    EXPECT_TRUE(core.isBlacklisted({1.0, 1.0}));
    EXPECT_FALSE(core.isBlacklisted({1.1, 1.0}));
    EXPECT_TRUE(core.goalAttempts().empty());
    core.clearRejectedPlans();
    EXPECT_FALSE(core.isBlacklisted({1.0, 1.0}));
}

TEST(ClusterFrontierExplorer, CountObservedCountsCellsNoLongerUnknown) {
    auto grid = buildGrid(10, 10, 0.1, -1);
    auto core = makeCore(grid);
    const std::vector<WorldPoint> cells{
        core.mapToWorld(2, 2), core.mapToWorld(5, 5), core.mapToWorld(7, 1)};
    EXPECT_EQ(core.countObserved(cells), 0);
    setCell(grid, 5, 5, 0);
    setCell(grid, 7, 1, 100);
    core.setMap(grid);
    EXPECT_EQ(core.countObserved(cells), 2);
}

}  // namespace
}  // namespace yahboom_alc_bringup
