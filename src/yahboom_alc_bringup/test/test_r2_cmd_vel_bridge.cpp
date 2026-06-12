#include <gtest/gtest.h>

#include <cmath>

#include "yahboom_alc_bringup/r2_cmd_vel_bridge.hpp"

namespace
{

using geometry_msgs::msg::Twist;
using yahboom_alc_bringup::convertNavTwistToR2;
using yahboom_alc_bringup::isReverseSafeInCostmap;
using yahboom_alc_bringup::R2CmdVelBridgeConfig;
using yahboom_alc_bringup::ReverseSafetyPose;

constexpr double kTolerance = 1e-9;

R2CmdVelBridgeConfig defaultConfig() {
    R2CmdVelBridgeConfig config;
    config.min_linear_speed = 0.05;
    config.max_linear_speed = 0.26;
    config.steering_gain = 1.25;
    config.max_steering = 1.25;
    config.linear_deadband = 0.001;
    config.angular_deadband = 0.001;
    config.arc_turn_when_stationary = true;
    config.reverse_check_distance_m = 0.20;
    config.reverse_safety_margin_m = 0.08;
    config.footprint_front_m = 0.16;
    config.footprint_rear_m = 0.15;
    config.footprint_half_width_m = 0.11;
    config.footprint_padding_m = 0.03;
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
    EXPECT_NEAR(output.linear.y, 0.5, kTolerance);
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

TEST(R2CmdVelBridge, StationarySpinBecomesLowSpeedArcWhenEnabled) {
    Twist input;
    input.angular.z = -0.4;

    const Twist output = convertNavTwistToR2(input, defaultConfig());

    EXPECT_NEAR(output.linear.x, 0.05, kTolerance);
    EXPECT_NEAR(output.linear.y, -0.5, kTolerance);
    EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
}

TEST(R2CmdVelBridge, StationarySpinStaysZeroForwardWhenDisabled) {
    R2CmdVelBridgeConfig config = defaultConfig();
    config.arc_turn_when_stationary = false;

    Twist input;
    input.angular.z = 0.4;

    const Twist output = convertNavTwistToR2(input, config);

    EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
    EXPECT_NEAR(output.linear.y, 0.5, kTolerance);
    EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
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

TEST(R2CmdVelBridge, ReverseSafetyBlocksUnknownRearFootprint) {
    auto costmap = buildCostmap();
    setWorldCell(costmap, -0.22, 0.0, -1);
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

}  // namespace
