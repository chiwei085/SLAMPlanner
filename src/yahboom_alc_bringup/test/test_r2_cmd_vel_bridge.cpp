#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "yahboom_alc_bringup/r2_cmd_vel_bridge.hpp"
#include "yahboom_alc_bringup/recent_free_costmap_history.hpp"

namespace
{

using geometry_msgs::msg::Twist;
using yahboom_alc_bringup::applyDriveStallFloor;
using yahboom_alc_bringup::convertNavTwistToR2;
using yahboom_alc_bringup::isReverseSafeInCostmap;
using yahboom_alc_bringup::isStationaryArcForwardSafeInCostmap;
using yahboom_alc_bringup::R2CmdVelBridgeConfig;
using yahboom_alc_bringup::RecentFreeCostmapHistory;
using yahboom_alc_bringup::ReverseSafetyPose;

constexpr double kTolerance = 1e-9;

// R2 firmware steering for a path curvature: wheel angle in whole degrees
// / 1000, from the bicycle model tan(delta) = L * curvature.
double firmwareSteeringFor(const double curvature) {
    const double degrees =
        std::atan(0.2353 * curvature) * 180.0 / std::numbers::pi;
    return std::round(degrees) / 1000.0;
}

R2CmdVelBridgeConfig defaultConfig() {
    R2CmdVelBridgeConfig config;
    config.min_linear_speed = 0.05;
    config.max_linear_speed = 0.26;
    config.wheelbase_m = 0.2353;
    config.min_turning_radius_m = 0.50;
    config.max_curvature = 2.0;
    config.max_steering_angle_rad = 0.6;
    config.linear_deadband = 0.001;
    config.angular_deadband = 0.001;
    config.arc_turn_when_stationary = true;
    config.stationary_arc_requires_forward_clearance = true;
    config.stationary_arc_check_distance_m = 0.12;
    config.reverse_check_distance_m = 0.12;
    config.reverse_safety_margin_m = 0.0;
    config.footprint_front_m = 0.19;
    config.footprint_rear_m = 0.19;
    config.footprint_half_width_m = 0.115;
    config.footprint_padding_m = 0.02;
    return config;
}

nav_msgs::msg::OccupancyGrid buildCostmap() {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "odom";
    map.info.resolution = 0.05;
    map.info.width = 60;
    map.info.height = 60;
    map.info.origin.position.x = -1.5;
    map.info.origin.position.y = -1.5;
    map.data.assign(static_cast<std::size_t>(map.info.width) *
                        static_cast<std::size_t>(map.info.height),
                    0);
    return map;
}

void setWorldCell(
    nav_msgs::msg::OccupancyGrid& map, const double wx,
    const double wy,  // NOLINT(bugprone-easily-swappable-parameters)
    const int8_t value) {
    const auto mx = static_cast<int>(
        std::floor((wx - map.info.origin.position.x) / map.info.resolution));
    const auto my = static_cast<int>(
        std::floor((wy - map.info.origin.position.y) / map.info.resolution));
    ASSERT_GE(mx, 0);
    ASSERT_GE(my, 0);
    ASSERT_LT(mx, static_cast<int>(map.info.width));
    ASSERT_LT(my, static_cast<int>(map.info.height));
    map.data[static_cast<std::size_t>(my) * map.info.width + mx] = value;
}

TEST(R2CmdVelBridge, ZeroInputPublishesZeroOutput) {
    const Twist output = convertNavTwistToR2(Twist{}, defaultConfig());

    EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(output.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
}

TEST(R2CmdVelBridge, ForwardCommandPreservesAndClampsLinearX) {
    Twist input;
    input.linear.x = 0.2;

    Twist output = convertNavTwistToR2(input, defaultConfig());
    EXPECT_NEAR(output.linear.x, 0.2, kTolerance);

    input.linear.x = 1.0;
    output = convertNavTwistToR2(input, defaultConfig());
    EXPECT_NEAR(output.linear.x, 0.26, kTolerance);
}

TEST(R2CmdVelBridge, ReverseCommandPreservesAndClampsLinearX) {
    Twist input;
    input.linear.x = -0.2;

    Twist output = convertNavTwistToR2(input, defaultConfig());
    EXPECT_NEAR(output.linear.x, -0.2, kTolerance);

    input.linear.x = -1.0;
    output = convertNavTwistToR2(input, defaultConfig());
    EXPECT_NEAR(output.linear.x, -0.26, kTolerance);
}

TEST(R2CmdVelBridge, AngularCommandMapsToLinearY) {
    Twist input;
    input.linear.x = 0.1;
    input.angular.z = 0.4;

    const Twist output = convertNavTwistToR2(input, defaultConfig());

    EXPECT_NEAR(output.linear.x, 0.1, kTolerance);
    // 0.4 / 0.1 = 4 exceeds the 1 / 0.5 m turning-radius limit -> curvature 2.
    EXPECT_NEAR(output.linear.y, firmwareSteeringFor(2.0), kTolerance);
    EXPECT_NEAR(output.linear.y, 0.025, kTolerance);
    EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
}

TEST(R2CmdVelBridge, TinyNonzeroForwardCommandIsLiftedToMinimumSpeed) {
    Twist input;
    input.linear.x = 0.01;

    Twist output = convertNavTwistToR2(input, defaultConfig());
    EXPECT_NEAR(output.linear.x, 0.05, kTolerance);

    input.linear.x = -0.01;
    output = convertNavTwistToR2(input, defaultConfig());
    EXPECT_NEAR(output.linear.x, -0.05, kTolerance);
}

TEST(R2CmdVelBridge, CanPreserveLowSpeedWhenSafetyGateDisablesMinimum) {
    R2CmdVelBridgeConfig config = defaultConfig();
    config.enforce_min_linear_speed = false;
    Twist input;
    input.linear.x = 0.01;

    const Twist output = convertNavTwistToR2(input, config);
    EXPECT_NEAR(output.linear.x, 0.01, kTolerance);
}

TEST(R2CmdVelBridge, SafetyStopSuppressesStationaryArcForwardMotion) {
    Twist input;
    input.angular.z = 0.4;

    const Twist output = convertNavTwistToR2(input, defaultConfig(), false);
    EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
}

TEST(R2CmdVelBridge, StallFloorLiftsSubStallRampToBreakawayWhenMoving) {
    const R2CmdVelBridgeConfig config = defaultConfig();
    // Rate limiter has only ramped to 0.02 but a 0.1 forward motion is wanted:
    // the wheels would not turn, so snap up to the stall-clearing floor.
    EXPECT_NEAR(applyDriveStallFloor(0.02, 0.1, config),
                config.min_linear_speed, kTolerance);
    EXPECT_NEAR(applyDriveStallFloor(-0.02, -0.1, config),
                -config.min_linear_speed, kTolerance);
}

TEST(R2CmdVelBridge, StallFloorSnapsSubStallResidueToStopWhenIdle) {
    const R2CmdVelBridgeConfig config = defaultConfig();
    // Decelerating toward a stop (target 0) must not be lifted back up.
    EXPECT_DOUBLE_EQ(applyDriveStallFloor(0.02, 0.0, config), 0.0);
}

TEST(R2CmdVelBridge, StallFloorLeavesDrivableSpeedsUntouched) {
    const R2CmdVelBridgeConfig config = defaultConfig();
    EXPECT_NEAR(applyDriveStallFloor(0.1, 0.1, config), 0.1, kTolerance);
}

TEST(R2CmdVelBridge, StallFloorIsBypassedWhenDisabled) {
    R2CmdVelBridgeConfig config = defaultConfig();
    config.enforce_min_linear_speed = false;
    EXPECT_NEAR(applyDriveStallFloor(0.02, 0.1, config), 0.02, kTolerance);
}

TEST(R2CmdVelBridge, StationarySpinBecomesLowSpeedArcWhenEnabled) {
    Twist input;
    input.angular.z = -0.4;

    const Twist output = convertNavTwistToR2(input, defaultConfig());

    EXPECT_NEAR(output.linear.x, 0.05, kTolerance);
    EXPECT_NEAR(output.linear.y, firmwareSteeringFor(-2.0), kTolerance);
    EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
}

TEST(R2CmdVelBridge, ReversedArcKeepsTheYawDirection) {
    geometry_msgs::msg::Twist arc;
    arc.linear.x = 0.08;
    arc.linear.y = 0.025;
    const auto reversed = yahboom_alc_bringup::reversedArc(arc);
    EXPECT_LT(reversed.linear.x, 0.0);
    // Yaw rate has the sign of speed times wheel angle.
    EXPECT_GT(reversed.linear.x * reversed.linear.y, 0.0);
}

TEST(R2CmdVelBridge, StationarySpinStaysZeroForwardWhenDisabled) {
    R2CmdVelBridgeConfig config = defaultConfig();
    config.arc_turn_when_stationary = false;

    Twist input;
    input.angular.z = 0.4;

    const Twist output = convertNavTwistToR2(input, config);

    EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(output.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
}

TEST(R2CmdVelBridge, ReverseCommandFlipsSteeringAndClampsToTurningRadius) {
    // Reversing, a negative yaw rate needs the wheels turned left (positive),
    // as measured on the real car.
    Twist input;
    input.linear.x = -0.10;
    input.angular.z = -1.0;

    const Twist output = convertNavTwistToR2(input, defaultConfig());

    EXPECT_NEAR(output.linear.x, -0.10, kTolerance);
    EXPECT_NEAR(output.linear.y, firmwareSteeringFor(2.0), kTolerance);
}

TEST(R2CmdVelBridge, ForwardCommandClampsSteeringToTurningRadius) {
    // angular_limit = |linear.x| / min_turning_radius_m = 0.10 / 0.50 = 0.20,
    // so curvature 2.0 instead of the requested 10.
    Twist input;
    input.linear.x = 0.10;
    input.angular.z = 1.0;

    const Twist output = convertNavTwistToR2(input, defaultConfig());

    EXPECT_NEAR(output.linear.x, 0.10, kTolerance);
    EXPECT_NEAR(output.linear.y, firmwareSteeringFor(2.0), kTolerance);
}

// run_20261122_095413: steering was angular * 1.25 on a firmware whose full
// lock is 0.045, so 74% of commands saturated. Small curvatures must now stay
// small.
TEST(R2CmdVelBridge, SmallCurvatureGivesProportionalWheelAngle) {
    Twist input;
    input.linear.x = 0.10;
    input.angular.z = 0.05;  // curvature 0.5 -> 6.7 deg

    const Twist output = convertNavTwistToR2(input, defaultConfig());

    EXPECT_NEAR(output.linear.y, 0.007, kTolerance);
    EXPECT_LT(std::abs(output.linear.y), 0.045);
}

TEST(R2CmdVelBridge, WheelAngleIsClampedToSteerJointLimit) {
    R2CmdVelBridgeConfig config = defaultConfig();
    config.min_turning_radius_m = 0.0;
    config.max_curvature = 100.0;
    Twist input;
    input.linear.x = 0.10;
    input.angular.z = 10.0;

    const Twist output = convertNavTwistToR2(input, config);

    // 0.6 rad = 34.4 deg -> 34 whole degrees.
    EXPECT_NEAR(output.linear.y, 0.034, kTolerance);
}

TEST(R2CmdVelBridge, ReverseSafetyAllowsFullyFreeRearFootprint) {
    const auto costmap = buildCostmap();
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_TRUE(isReverseSafeInCostmap(costmap, pose, -0.08, defaultConfig()));
}

TEST(R2CmdVelBridge, ReverseSafetyBlocksOccupiedRearFootprint) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, -0.22, 0.0, 100);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_FALSE(isReverseSafeInCostmap(costmap, pose, -0.08, defaultConfig()));
}

TEST(R2CmdVelBridge, ReverseSafetyAllowsInflatedRearFootprintCost) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, -0.22, 0.0, 80);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_TRUE(isReverseSafeInCostmap(costmap, pose, -0.08, defaultConfig()));
}

// run_20261122_095413: all 133 rear_blocked events had only inscribed (99)
// cells in the strip, with the nearest lethal cell 0.075-0.10 m outside it.
TEST(R2CmdVelBridge, ReverseSafetyAllowsInscribedRearFootprintCost) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, -0.22, 0.0, 99);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_TRUE(isReverseSafeInCostmap(costmap, pose, -0.08, defaultConfig()));
}

TEST(R2CmdVelBridge, ReverseSafetyBlocksLethalRearFootprintCost) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, -0.22, 0.0, 100);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_FALSE(isReverseSafeInCostmap(costmap, pose, -0.08, defaultConfig()));
}

TEST(R2CmdVelBridge, ReverseSafetyBlocksUnknownRearFootprint) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, -0.22, 0.0, -1);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_FALSE(isReverseSafeInCostmap(costmap, pose, -0.08, defaultConfig()));
}

// A cell that flips to unknown (e.g. it just fell out of the rolling
// costmap's observation window) but was seen clearly free moments ago should
// not fail closed the same way a cell that was never observed does.
TEST(R2CmdVelBridge, RecentFreeHistoryRejectsTruncatedCostmap) {
    RecentFreeCostmapHistory history;
    auto truncated = buildCostmap();
    truncated.data.resize(truncated.data.size() / 2);
    EXPECT_FALSE(history.observe(truncated, /*now_sec=*/100.0,
                                 yahboom_alc_bringup::kFootprintFreeCostMax));
    EXPECT_FALSE(history.wasRecentlyFree(-0.22, 0.0, 100.5, 2.0));
    EXPECT_TRUE(history.observe(buildCostmap(), /*now_sec=*/100.0,
                                yahboom_alc_bringup::kFootprintFreeCostMax));
}

TEST(R2CmdVelBridge, ReverseSafetyAllowsRecentlyFreeUnknownRearFootprint) {
    RecentFreeCostmapHistory history;
    history.observe(buildCostmap(), /*now_sec=*/100.0,
                    yahboom_alc_bringup::kFootprintFreeCostMax);

    auto costmap = buildCostmap();
    setWorldCell(costmap, -0.22, 0.0, -1);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_TRUE(isReverseSafeInCostmap(costmap, pose, -0.08, defaultConfig(),
                                       &history, /*now_sec=*/101.0));
}

TEST(R2CmdVelBridge,
     ReverseSafetyBlocksNeverObservedUnknownRearFootprintEvenWithHistory) {
    RecentFreeCostmapHistory history;  // Never observed anything free.

    auto costmap = buildCostmap();
    setWorldCell(costmap, -0.22, 0.0, -1);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_FALSE(isReverseSafeInCostmap(costmap, pose, -0.08, defaultConfig(),
                                        &history, /*now_sec=*/101.0));
}

TEST(R2CmdVelBridge,
     ReverseSafetyBlocksUnknownRearFootprintOnceRecentFreeGraceExpires) {
    RecentFreeCostmapHistory history;
    history.observe(buildCostmap(), /*now_sec=*/100.0,
                    yahboom_alc_bringup::kFootprintFreeCostMax);

    auto costmap = buildCostmap();
    setWorldCell(costmap, -0.22, 0.0, -1);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};
    auto config = defaultConfig();
    config.reverse_recent_free_grace_sec = 2.0;

    // The grace window (100.0 + 2.0s) has fully elapsed by now_sec=103.0.
    EXPECT_FALSE(isReverseSafeInCostmap(costmap, pose, -0.08, config, &history,
                                        /*now_sec=*/103.0));
}

// run_20261118_204106: face to face with a chair, its inflation reached
// back over the robot's own body while the strip behind was free.
TEST(R2CmdVelBridge, ReverseSafetyIgnoresCostUnderTheBody) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, 0.10, 0.10, 100);
    setWorldCell(costmap, -0.10, 0.10, 99);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_TRUE(isReverseSafeInCostmap(costmap, pose, -0.08, defaultConfig()));
    // The same body cells must not block the forward arc either; only the
    // front strip does.
    EXPECT_TRUE(isStationaryArcForwardSafeInCostmap(costmap, pose, 0.05,
                                                    defaultConfig()));
}

TEST(R2CmdVelBridge, ReverseSafetyBlocksSweptRearCorner) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, -0.30, 0.12, 100);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_FALSE(isReverseSafeInCostmap(costmap, pose, -0.08, defaultConfig()));
}

TEST(R2CmdVelBridge, ReverseSafetyBlocksOutOfBoundsSweep) {
    const auto costmap = buildCostmap();
    const ReverseSafetyPose pose{-1.32, 0.0, 0.0};

    EXPECT_FALSE(isReverseSafeInCostmap(costmap, pose, -0.08, defaultConfig()));
}

TEST(R2CmdVelBridge, ReverseSafetyIgnoresForwardCommands) {
    nav_msgs::msg::OccupancyGrid costmap;
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_TRUE(isReverseSafeInCostmap(costmap, pose, 0.08, defaultConfig()));
}

TEST(R2CmdVelBridge, StationaryArcForwardSafetyAllowsClearFrontFootprint) {
    const auto costmap = buildCostmap();
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_TRUE(isStationaryArcForwardSafeInCostmap(costmap, pose, 0.05,
                                                    defaultConfig()));
}

TEST(R2CmdVelBridge, StationaryArcForwardSafetyAllowsInscribedFrontCost) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, 0.22, 0.0, 99);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_TRUE(isStationaryArcForwardSafeInCostmap(costmap, pose, 0.05,
                                                    defaultConfig()));
}

TEST(R2CmdVelBridge, StationaryArcForwardSafetyBlocksLethalFrontCost) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, 0.22, 0.0, 100);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_FALSE(isStationaryArcForwardSafeInCostmap(costmap, pose, 0.05,
                                                     defaultConfig()));
}

TEST(R2CmdVelBridge, StationaryArcForwardSafetyAllowsInflatedFrontCost) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, 0.22, 0.0, 80);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_TRUE(isStationaryArcForwardSafeInCostmap(costmap, pose, 0.05,
                                                    defaultConfig()));
}

// Unlike reverse safety, the stationary-arc forward check must not block on
// unknown cells by default: it runs on every goal that turns toward a
// frontier, and a frontier borders unknown space by definition. Blocking here
// stalled the robot at every goal (run_20260708_161710.jsonl) instead of just
// guarding the rare blind-reverse case.
TEST(R2CmdVelBridge, StationaryArcForwardSafetyAllowsUnknownFrontByDefault) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, 0.22, 0.0, -1);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};

    EXPECT_TRUE(isStationaryArcForwardSafeInCostmap(costmap, pose, 0.05,
                                                    defaultConfig()));
}

TEST(R2CmdVelBridge,
     StationaryArcForwardSafetyBlocksUnknownFrontWhenConfigured) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, 0.22, 0.0, -1);
    const ReverseSafetyPose pose{0.0, 0.0, 0.0};
    auto config = defaultConfig();
    config.stationary_arc_unknown_is_blocked = true;

    EXPECT_FALSE(
        isStationaryArcForwardSafeInCostmap(costmap, pose, 0.05, config));
}

TEST(R2CmdVelBridge, StationaryArcForwardSafetyBlocksOutOfBoundsSweep) {
    const auto costmap = buildCostmap();
    const ReverseSafetyPose pose{1.32, 0.0, 0.0};

    EXPECT_FALSE(isStationaryArcForwardSafeInCostmap(costmap, pose, 0.05,
                                                     defaultConfig()));
}

TEST(BlindReverseWindow, OpensOnTurnStartAndClosesOnFinishOrTimeout) {
    yahboom_alc_bringup::BlindReverseWindow window;
    EXPECT_FALSE(window.allowed(100.0));
    window.onEvent("observation_turn_started", 100.0, 30.0);
    EXPECT_TRUE(window.allowed(100.0));
    EXPECT_TRUE(window.allowed(129.9));
    EXPECT_FALSE(window.allowed(130.0));  // bounded even if "finished" is lost
    window.onEvent("observation_turn_started", 200.0, 30.0);
    window.onEvent("observation_turn_finished", 205.0, 30.0);
    EXPECT_FALSE(window.allowed(205.0));
    window.onEvent("reverse_blocked", 300.0, 30.0);
    EXPECT_FALSE(window.allowed(300.0));
    window.onEvent("observation_turn_started", 400.0, 0.0);  // disabled
    EXPECT_FALSE(window.allowed(400.0));
}

TEST(IdleStop, StreamsZeroOnlyDuringBurstAfterInputSilence) {
    using yahboom_alc_bringup::shouldPublishIdleStop;
    std::optional<double> end;
    EXPECT_FALSE(shouldPublishIdleStop(0.3, 0.0, 0.5, 1.0, end));  // fresh
    EXPECT_TRUE(shouldPublishIdleStop(0.6, 0.0, 0.5, 1.0, end));
    EXPECT_TRUE(shouldPublishIdleStop(1.6, 0.0, 0.5, 1.0, end));
    EXPECT_FALSE(shouldPublishIdleStop(1.7, 0.0, 0.5, 1.0, end));  // done
    std::optional<double> disabled;
    EXPECT_FALSE(shouldPublishIdleStop(5.0, 0.0, 0.0, 1.0, disabled));
}

TEST(IdleStop, LateTimerStillSendsTheFullBurst) {
    using yahboom_alc_bringup::shouldPublishIdleStop;
    std::optional<double> end;
    // The first tick after silence comes 2 s late: the burst starts there.
    EXPECT_TRUE(shouldPublishIdleStop(2.0, 0.0, 0.5, 1.0, end));
    EXPECT_TRUE(shouldPublishIdleStop(2.9, 0.0, 0.5, 1.0, end));
    EXPECT_FALSE(shouldPublishIdleStop(3.1, 0.0, 0.5, 1.0, end));
    // New input resets the deadline; the next silence gets its own burst.
    end.reset();
    EXPECT_TRUE(shouldPublishIdleStop(10.0, 9.0, 0.5, 1.0, end));
}

}  // namespace

TEST(R2CmdVelBridge, FeedbackWatchFrozenBeforeFirstSample) {
    const yahboom_alc_bringup::FeedbackFreezeWatch watch;
    EXPECT_TRUE(watch.frozen(10.0, 0.5));
}

TEST(R2CmdVelBridge, FeedbackWatchFreezesOnRepeatedContentAtNormalRate) {
    yahboom_alc_bringup::FeedbackFreezeWatch watch;
    const yahboom_alc_bringup::FeedbackFreezeWatch::Sample cached{
        0.1, 0.0, -9.8, 0.0, 0.0, 0.01};
    // Messages keep arriving every 0.1 s, but the content never changes.
    for (int i = 0; i <= 10; ++i) {
        watch.observe(cached, 10.0 + 0.1 * i);
    }
    EXPECT_FALSE(watch.frozen(10.4, 0.5));
    EXPECT_TRUE(watch.frozen(11.0, 0.5));

    auto live = cached;
    live[2] = -9.81;
    watch.observe(live, 11.1);
    EXPECT_FALSE(watch.frozen(11.2, 0.5));
}
