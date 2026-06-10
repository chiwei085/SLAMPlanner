#include <gtest/gtest.h>

#include <cmath>

#include "yahboom_alc_bringup/frontier_explorer_core.hpp"

namespace yahboom_alc_bringup
{
namespace
{

nav_msgs::msg::OccupancyGrid buildGrid(const int width, const int height,
                                       const double resolution = 0.1,
                                       const int default_value = 0) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.resolution = resolution;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(width * height, default_value);
    return grid;
}

void setCell(nav_msgs::msg::OccupancyGrid& grid, const int x, const int y,
             const int value) {
    grid.data[y * grid.info.width + x] = value;
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
    config.footprint_front_m = 0.16;
    config.footprint_rear_m = 0.15;
    config.footprint_half_width_m = 0.11;
    config.footprint_padding_m = 0.03;
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
    config.near_goal_hysteresis_radius_m = 0.35;
    config.near_goal_retry_limit = 0;
    config.near_goal_hysteresis_sec = 0.0;
    config.alignment_grace_enabled = true;
    config.alignment_heading_threshold_rad = 0.35;
    config.alignment_max_grace_sec = 12.0;
    config.alignment_min_rotation_delta_rad = 0.05;
    config.failed_goal_blacklist_radius_m = 0.45;
    config.failed_goal_blacklist_sec = 60.0;
    config.failed_goal_max_retries = 1;
    config.enable_subgoals = true;
    config.max_subgoals = 3;
    config.subgoal_path_tolerance_m = 0.30;
    config.min_subgoal_spacing_m = 0.45;
    FrontierExplorerCore core(config);
    core.setMap(grid);
    return core;
}

FrontierCandidate makeCandidateAt(FrontierExplorerCore& core, const int x,
                                  const int y, const double score) {
    FrontierCandidate candidate;
    candidate.goal_cell = {x, y};
    candidate.goal_xy = core.mapToWorld(x, y);
    candidate.goal_yaw = 0.0;
    candidate.path = {{2, 5}, {x, y}};
    candidate.path_length = core.pathLength(candidate.path);
    candidate.reachable = true;
    candidate.score = score;
    return candidate;
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
    for (int i = 0; i < 12; ++i) {
        large_corridor.emplace_back(8 + i, 5);
    }

    FrontierCounts counts1;
    const auto small = core.makeCandidate(
        small_corner, start, robot_xy,
        std::vector<bool>(grid.data.size(), false), 30, 12, counts1);
    const auto large = core.makeCandidate(
        large_corridor, start, robot_xy,
        std::vector<bool>(grid.data.size(), false), 30, 12, counts1);

    ASSERT_TRUE(small);
    ASSERT_TRUE(large);
    EXPECT_LT(large->score, small->score);
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
        std::vector<bool>(free_grid.data.size(), false), 16, 10, counts2);
    const auto unknown_candidate = unknown_core.makeCandidate(
        cluster, start, unknown_core.mapToWorld(start.first, start.second),
        std::vector<bool>(unknown_grid.data.size(), false), 16, 10, counts2);

    ASSERT_TRUE(free_candidate);
    ASSERT_TRUE(unknown_candidate);
    EXPECT_GT(unknown_candidate->unknown_overlap_ratio, 0.0);
    EXPECT_GT(unknown_candidate->score, free_candidate->score);
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
    EXPECT_GT(*ratio, 0.0);

    FrontierCounts counts3;
    const auto candidate = core.makeCandidate(
        {{5, 12}}, {2, 12}, core.mapToWorld(2, 12),
        std::vector<bool>(grid.data.size(), false), 30, 30, counts3);
    EXPECT_FALSE(candidate);
}

TEST(ClusterFrontierExplorer, UnreachableGoalRejectsCandidate) {
    auto grid = buildGrid(12, 12, 0.1);
    auto core = makeCore(grid);
    auto blocked = std::vector<bool>(grid.data.size(), false);
    for (int y = 0; y < 12; ++y) {
        blocked[core.index(5, y, 12)] = true;
    }

    FrontierCounts counts4;
    const auto candidate = core.makeCandidate(
        {{8, 5}}, {2, 5}, core.mapToWorld(2, 5), blocked, 12, 12, counts4);
    EXPECT_FALSE(candidate);
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
    auto grid = buildGrid(12, 12, 0.05);
    setCell(grid, 6, 6, 100);
    auto core = makeCore(grid);

    EXPECT_FALSE(core.sweptPathUnknownRatio({{2, 6}, {9, 6}}));
}

TEST(ClusterFrontierExplorer, FailedGoalBlacklistPreventsReselection) {
    auto grid = buildGrid(20, 10, 0.1);
    auto core = makeCore(grid);
    core.goalAttemptsForTest().push_back({core.mapToWorld(8, 5), 1, 60.0});

    FrontierCounts counts5;
    const auto candidate = core.makeCandidate(
        {{8, 5}, {9, 5}, {10, 5}}, {2, 5}, core.mapToWorld(2, 5),
        std::vector<bool>(grid.data.size(), false), 20, 10, counts5);

    EXPECT_FALSE(candidate);
}

TEST(ClusterFrontierExplorer, ExpiredGoalAttemptPrunesFailureCount) {
    auto grid = buildGrid(20, 10, 0.1);
    auto core = makeCore(grid);
    const auto goal_xy = core.mapToWorld(8, 5);
    core.goalAttemptsForTest().push_back({goal_xy, 3, 10.0});

    core.pruneBlacklist(11.0);

    EXPECT_TRUE(core.goalAttempts().empty());
    EXPECT_FALSE(core.isBlacklisted(goal_xy));
}

TEST(ClusterFrontierExplorer, NearbyFailedGoalsShareRetryCount) {
    auto grid = buildGrid(20, 10, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.failed_goal_max_retries = 2;
    core.setConfig(config);

    core.blacklistGoal(core.mapToWorld(8, 5), 0.0);
    ASSERT_EQ(core.goalAttempts().size(), 1u);
    EXPECT_EQ(core.goalAttempts()[0].failures, 1);
    EXPECT_FALSE(core.isBlacklisted(core.mapToWorld(8, 5)));

    core.blacklistGoal(core.mapToWorld(9, 5), 0.0);

    ASSERT_EQ(core.goalAttempts().size(), 1u);
    EXPECT_EQ(core.goalAttempts()[0].failures, 2);
    EXPECT_TRUE(core.isBlacklisted(core.mapToWorld(8, 5)));
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
    const auto forward = core.makeCandidate(
        {{15, 10}}, start, robot_xy, std::vector<bool>(grid.data.size(), false),
        30, 20, counts, 0.0);
    const auto behind = core.makeCandidate(
        {{5, 10}}, start, robot_xy, std::vector<bool>(grid.data.size(), false),
        30, 20, counts, 0.0);

    ASSERT_TRUE(forward);
    ASSERT_TRUE(behind);
    EXPECT_LT(forward->heading_delta, behind->heading_delta);
    EXPECT_LT(forward->score, behind->score);
}

TEST(ClusterFrontierExplorer, CandidateRejectionCountsIncludeFilters) {
    auto grid = buildGrid(30, 20, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.min_frontier_distance = 0.45;
    config.max_frontier_distance = 0.8;
    config.failed_goal_max_retries = 1;
    core.setConfig(config);
    core.goalAttemptsForTest().push_back({core.mapToWorld(8, 2), 1, 60.0});

    const Cell start{10, 10};
    const auto robot_xy = core.mapToWorld(start.first, start.second);
    FrontierCounts counts;
    const std::vector<bool> blocked(grid.data.size(), false);

    EXPECT_FALSE(
        core.makeCandidate({{8, 2}}, start, robot_xy, blocked, 30, 20, counts));
    EXPECT_FALSE(core.makeCandidate({{11, 10}}, start, robot_xy, blocked, 30,
                                    20, counts));
    EXPECT_FALSE(core.makeCandidate({{25, 10}}, start, robot_xy, blocked, 30,
                                    20, counts));

    EXPECT_GE(counts.rejected_blacklist, 1);
    EXPECT_GE(counts.rejected_too_near, 1);
    EXPECT_GE(counts.rejected_too_far, 1);
}

TEST(ClusterFrontierExplorer, ShouldPublishGoalHonorsDistanceAndPeriod) {
    auto grid = buildGrid(20, 10, 0.1);
    auto core = makeCore(grid);
    const auto goal = core.mapToWorld(8, 5);

    EXPECT_TRUE(core.shouldPublishGoal(goal, 0.0));
    core.noteGoalPublished(goal, 0.0, 0.0);
    EXPECT_FALSE(core.shouldPublishGoal(goal, 1.0));
    EXPECT_TRUE(core.shouldPublishGoal(goal, 4.0));
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

    EXPECT_FALSE(result.candidates.empty());
    EXPECT_GT(result.counts.clusters, 0);
    if (!result.candidates.empty()) {
        EXPECT_GE(result.candidates.front().path_length, 0.45);
    }
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
    EXPECT_NEAR(near_first_segment->distance, 0.2, 1e-6);
    EXPECT_NEAR(near_first_segment->arc_length, 0.3, 1e-6);
    ASSERT_TRUE(near_second_segment);
    EXPECT_NEAR(near_second_segment->distance, 0.2, 1e-6);
    EXPECT_NEAR(near_second_segment->arc_length, 0.9, 1e-6);
}

TEST(ClusterFrontierExplorer, BuildsSubgoalsNearBfsPathOnly) {
    auto grid = buildGrid(30, 20, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.subgoal_path_tolerance_m = 0.21;
    config.min_subgoal_spacing_m = 0.3;
    core.setConfig(config);

    auto final_goal = makeCandidateAt(core, 18, 5, 1.0);
    final_goal.path = {{2, 5}, {18, 5}};
    auto near_path = makeCandidateAt(core, 8, 6, 2.0);
    auto off_path = makeCandidateAt(core, 10, 9, 3.0);
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

    auto final_goal = makeCandidateAt(core, 18, 5, 1.0);
    final_goal.path = {{2, 5}, {18, 5}};
    auto later = makeCandidateAt(core, 14, 5, 2.0);
    auto earlier = makeCandidateAt(core, 6, 5, 3.0);

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

    auto final_goal = makeCandidateAt(core, 24, 5, 1.0);
    final_goal.path = {{2, 5}, {24, 5}};
    auto first = makeCandidateAt(core, 6, 5, 2.0);
    auto duplicate = makeCandidateAt(core, 8, 5, 3.0);
    auto second = makeCandidateAt(core, 13, 5, 4.0);
    auto over_limit = makeCandidateAt(core, 19, 5, 5.0);

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
    EXPECT_GE(forward->arc_length - start->arc_length, 0.12);
    EXPECT_LT(std::abs(lateral->arc_length - start->arc_length), 1e-6);
}

TEST(ClusterFrontierExplorer, HeadingErrorWrapsAcrossPi) {
    auto grid = buildGrid(10, 10, 0.1);
    auto core = makeCore(grid);

    EXPECT_NEAR(core.headingError(3.10, -3.10), 0.083185307, 1e-6);
    EXPECT_NEAR(core.headingError(-3.10, 3.10), 0.083185307, 1e-6);
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
    EXPECT_NEAR(*first_segment, 0.0, 1e-6);
    EXPECT_NEAR(*second_segment, M_PI_2, 1e-6);
}

TEST(ClusterFrontierExplorer, AlignmentGraceFallsBackToActiveGoalYaw) {
    auto grid = buildGrid(20, 20, 0.1);
    auto core = makeCore(grid);
    const auto robot_xy = core.mapToWorld(5, 5);
    const auto goal_xy = core.mapToWorld(5, 10);

    const auto status = core.evaluateAlignmentGrace(robot_xy, 0.0, std::nullopt,
                                                    {}, goal_xy, 1.0);

    EXPECT_FALSE(status.active);
    EXPECT_NEAR(status.target_yaw, M_PI_2, 1e-6);
    EXPECT_GT(status.heading_error,
              core.config().alignment_heading_threshold_rad);
}

TEST(ClusterFrontierExplorer, AlignmentGraceActivatesWhenYawConverges) {
    auto grid = buildGrid(20, 20, 0.1);
    auto core = makeCore(grid);
    auto config = core.config();
    config.alignment_heading_threshold_rad = 0.35;
    config.alignment_max_grace_sec = 12.0;
    config.alignment_min_rotation_delta_rad = 0.05;
    core.setConfig(config);
    const auto robot_xy = core.mapToWorld(5, 5);
    const auto goal_xy = core.mapToWorld(12, 5);
    const std::vector<Cell> path{{5, 5}, {12, 5}};

    const auto status =
        core.evaluateAlignmentGrace(robot_xy, 1.40, M_PI_2, path, goal_xy, 9.0);

    EXPECT_TRUE(status.active);
    EXPECT_NEAR(status.target_yaw, 0.0, 1e-6);
    EXPECT_GT(status.heading_error_delta,
              core.config().alignment_min_rotation_delta_rad);
}

TEST(ClusterFrontierExplorer, AlignmentGraceRejectsNonConvergingYaw) {
    auto grid = buildGrid(20, 20, 0.1);
    auto core = makeCore(grid);
    const auto robot_xy = core.mapToWorld(5, 5);
    const auto goal_xy = core.mapToWorld(12, 5);
    const std::vector<Cell> path{{5, 5}, {12, 5}};

    const auto status =
        core.evaluateAlignmentGrace(robot_xy, 1.40, 1.30, path, goal_xy, 9.0);

    EXPECT_FALSE(status.active);
    EXPECT_LT(status.heading_error_delta, 0.0);
}

TEST(ClusterFrontierExplorer, AlignmentGraceRejectsAlignedHeading) {
    auto grid = buildGrid(20, 20, 0.1);
    auto core = makeCore(grid);
    const auto robot_xy = core.mapToWorld(5, 5);
    const auto goal_xy = core.mapToWorld(12, 5);
    const std::vector<Cell> path{{5, 5}, {12, 5}};

    const auto status =
        core.evaluateAlignmentGrace(robot_xy, 0.10, 0.30, path, goal_xy, 9.0);

    EXPECT_FALSE(status.active);
    EXPECT_LT(status.heading_error,
              core.config().alignment_heading_threshold_rad);
}

TEST(ClusterFrontierExplorer, AlignmentGraceExpiresAfterMaxGrace) {
    auto grid = buildGrid(20, 20, 0.1);
    auto core = makeCore(grid);
    const auto robot_xy = core.mapToWorld(5, 5);
    const auto goal_xy = core.mapToWorld(12, 5);
    const std::vector<Cell> path{{5, 5}, {12, 5}};

    const auto status = core.evaluateAlignmentGrace(robot_xy, 1.40, M_PI_2,
                                                    path, goal_xy, 12.0);

    EXPECT_FALSE(status.active);
    EXPECT_GT(status.heading_error_delta,
              core.config().alignment_min_rotation_delta_rad);
}

}  // namespace
}  // namespace yahboom_alc_bringup
