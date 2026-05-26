#include <gtest/gtest.h>

#include <cmath>

#include "yahboom_alc_bringup/r2_cmd_vel_bridge.hpp"

namespace
{

using geometry_msgs::msg::Twist;
using yahboom_alc_bringup::convertNavTwistToR2;
using yahboom_alc_bringup::R2CmdVelBridgeConfig;

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
    return config;
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

}  // namespace
