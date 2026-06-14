#include <gtest/gtest.h>

#include "yahboom_alc_bringup/backtrace_recovery.hpp"

namespace yahboom_alc_bringup
{
namespace
{

BacktraceRecoveryConfig testConfig() {
    BacktraceRecoveryConfig config;
    config.breadcrumb_max_distance_m = 0.30;
    config.breadcrumb_max_age_sec = 10.0;
    config.backup_step_m = 0.10;
    config.backup_speed_mps = 0.04;
    config.max_segments = 4;
    config.min_actual_displacement_m = 0.03;
    return config;
}

TEST(BacktraceRecovery, BreadcrumbTrailKeepsRecentDistanceWindow) {
    BreadcrumbTrail trail(testConfig());
    trail.add({{0.0, 0.0}, 0.0, 0.0});
    trail.add({{0.1, 0.0}, 0.0, 1.0});
    trail.add({{0.2, 0.0}, 0.0, 2.0});
    trail.add({{0.3, 0.0}, 0.0, 3.0});
    trail.add({{0.4, 0.0}, 0.0, 4.0});

    ASSERT_GE(trail.poses().size(), 2u);
    EXPECT_NEAR(trail.poses().back().xy.first, 0.4, 1e-9);
    double retained_distance = 0.0;
    for (std::size_t i = 1; i < trail.poses().size(); ++i) {
        retained_distance +=
            distanceBetween(trail.poses()[i - 1].xy, trail.poses()[i].xy);
    }
    EXPECT_LE(retained_distance, testConfig().breadcrumb_max_distance_m);
}

TEST(BacktraceRecovery, RequiresEnoughBreadcrumbsToStart) {
    const auto config = testConfig();
    BreadcrumbTrail trail(config);
    BacktraceRecoveryPlanner planner(config);

    EXPECT_FALSE(planner.canStart(trail));
    trail.add({{0.0, 0.0}, 0.0, 0.0});
    EXPECT_FALSE(planner.canStart(trail));
    trail.add({{0.1, 0.0}, 0.0, 1.0});
    EXPECT_TRUE(planner.canStart(trail));
}

TEST(BacktraceRecovery, LimitsSegmentsAndDetectsInsufficientMotion) {
    const auto config = testConfig();
    BreadcrumbTrail trail(config);
    trail.add({{0.0, 0.0}, 0.0, 0.0});
    trail.add({{0.1, 0.0}, 0.0, 1.0});
    trail.add({{0.2, 0.0}, 0.0, 2.0});
    BacktraceRecoveryPlanner planner(config);

    const auto distance = planner.nextBackupDistance(trail, {0.2, 0.0});
    ASSERT_TRUE(distance);
    EXPECT_TRUE(planner.noteSegmentStarted({0.2, 0.0}, *distance));
    EXPECT_FALSE(planner.noteSegmentFinished({0.18, 0.0}));

    planner.reset();
    for (int i = 0; i < config.max_segments; ++i) {
        EXPECT_TRUE(
            planner.noteSegmentStarted({0.2, 0.0}, config.backup_step_m));
        EXPECT_TRUE(planner.noteSegmentFinished({0.16, 0.0}));
    }
    EXPECT_FALSE(planner.nextBackupDistance(trail, {0.2, 0.0}));
}

TEST(BacktraceRecovery, GoalToleranceBoundaryMatchesPlannerPolicy) {
    const WorldPoint goal{0.0, 0.0};

    EXPECT_TRUE(withinDistanceTolerance({0.39, 0.0}, goal, 0.40));
    EXPECT_FALSE(withinDistanceTolerance({0.41, 0.0}, goal, 0.40));
}

}  // namespace
}  // namespace yahboom_alc_bringup
