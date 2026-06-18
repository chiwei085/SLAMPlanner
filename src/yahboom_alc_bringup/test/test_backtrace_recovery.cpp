#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>

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
    config.backup_stall_check_sec = 1.0;
    config.reverse_motion_ratio_min = 0.6;
    return config;
}

template <typename T>
const T& requireValue(const std::optional<T>& value) {
    if (!value) {
        ADD_FAILURE() << "expected optional value";
        throw std::logic_error("expected optional value");
    }
    return value.value();
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
    EXPECT_TRUE(planner.noteSegmentStarted({0.2, 0.0}, requireValue(distance)));
    const auto stalled = planner.noteSegmentFinished({0.18, 0.0});
    EXPECT_FALSE(stalled.moved_enough);
    EXPECT_NEAR(stalled.actual_displacement_m, 0.02, 1e-9);
    EXPECT_NEAR(stalled.commanded_m, requireValue(distance), 1e-9);
    EXPECT_NEAR(stalled.ratio, 0.2, 1e-9);

    planner.reset();
    for (int i = 0; i < config.max_segments; ++i) {
        EXPECT_TRUE(
            planner.noteSegmentStarted({0.2, 0.0}, config.backup_step_m));
        const auto result = planner.noteSegmentFinished({0.10, 0.0});
        EXPECT_TRUE(result.moved_enough);
        EXPECT_NEAR(result.actual_displacement_m, 0.10, 1e-9);
        EXPECT_NEAR(result.commanded_m, config.backup_step_m, 1e-9);
        EXPECT_NEAR(result.ratio, 1.0, 1e-9);
    }
    EXPECT_FALSE(planner.nextBackupDistance(trail, {0.2, 0.0}));
}

TEST(BacktraceRecovery, ReportsSlipRatioForSuccessfulButWeakMotion) {
    const auto config = testConfig();
    BacktraceRecoveryPlanner planner(config);

    ASSERT_TRUE(planner.noteSegmentStarted({0.0, 0.0}, config.backup_step_m));
    const auto result = planner.noteSegmentFinished({0.05, 0.0});

    EXPECT_TRUE(result.moved_enough);
    EXPECT_NEAR(result.actual_displacement_m, 0.05, 1e-9);
    EXPECT_NEAR(result.commanded_m, 0.10, 1e-9);
    EXPECT_LT(result.ratio, config.reverse_motion_ratio_min);
}

TEST(BacktraceRecovery, DefaultsAllowFastStallCheckAfterMinimumMotionTime) {
    BacktraceRecoveryConfig config;

    ASSERT_GT(config.backup_speed_mps, 0.0);
    EXPECT_GE(config.backup_stall_check_sec,
              config.min_actual_displacement_m / config.backup_speed_mps);
    EXPECT_LT(config.min_actual_displacement_m / config.backup_step_m,
              config.reverse_motion_ratio_min);
    EXPECT_GE(config.backup_step_m * config.max_segments, 0.40);
}

TEST(BacktraceRecovery, GoalToleranceBoundaryMatchesPlannerPolicy) {
    const WorldPoint goal{0.0, 0.0};

    EXPECT_TRUE(withinDistanceTolerance({0.39, 0.0}, goal, 0.40));
    EXPECT_FALSE(withinDistanceTolerance({0.41, 0.0}, goal, 0.40));
}

}  // namespace
}  // namespace yahboom_alc_bringup
