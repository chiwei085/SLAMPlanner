#include <gtest/gtest.h>

#include "yahboom_alc_bringup/r2_collision_safety.hpp"

namespace
{

using geometry_msgs::msg::Twist;
using yahboom_alc_bringup::CollisionSafetyCore;
using yahboom_alc_bringup::R2CollisionSafetyConfig;

R2CollisionSafetyConfig testConfig() {
    R2CollisionSafetyConfig config;
    config.imu_baseline_samples = 3;
    config.imu_impact_decel_threshold_mps2 = 1.0;
    return config;
}

Twist forwardCommand(const double speed = 0.11) {
    Twist command;
    command.linear.x = speed;
    command.angular.z = 0.4;
    return command;
}

// detectImpact is the only path the live gate node uses: it must raise the
// edge + hold on a forward deceleration and clear the hold once it expires.
TEST(R2CollisionSafety, DetectImpactTriggersHoldOnForwardDeceleration) {
    CollisionSafetyCore core(testConfig());
    const auto command = forwardCommand();
    for (int index = 0; index < 3; ++index) {
        core.detectImpact(index * 0.1, command, true, 0.0);
    }
    auto impact = core.detectImpact(0.4, command, true, -2.0);
    EXPECT_TRUE(impact.impact_triggered);
    EXPECT_TRUE(impact.stop_hold_active);

    // Within the hold window: still held, but no fresh edge.
    impact = core.detectImpact(0.5, command, true, 0.0);
    EXPECT_FALSE(impact.impact_triggered);
    EXPECT_TRUE(impact.stop_hold_active);

    // impact_stop_hold_sec defaults to 0.5 s, so the hold has expired here.
    impact = core.detectImpact(1.1, command, true, 0.0);
    EXPECT_FALSE(impact.impact_triggered);
    EXPECT_FALSE(impact.stop_hold_active);
}

TEST(R2CollisionSafety, DetectImpactRespectsCooldown) {
    CollisionSafetyCore core(testConfig());
    const auto command = forwardCommand();
    for (int index = 0; index < 3; ++index) {
        core.detectImpact(index * 0.1, command, true, 0.0);
    }
    EXPECT_TRUE(core.detectImpact(0.4, command, true, -2.0).impact_triggered);
    // impact_cooldown_sec defaults to 3.0 s: a second spike inside the cooldown
    // must not raise a fresh edge.
    EXPECT_FALSE(core.detectImpact(1.0, command, true, -2.0).impact_triggered);
}

TEST(R2CollisionSafety, DetectImpactIgnoresImpactWhenNotCommandingForward) {
    CollisionSafetyCore core(testConfig());
    Twist stationary;
    stationary.angular.z = 0.4;
    for (int index = 0; index < 3; ++index) {
        core.detectImpact(index * 0.1, stationary, true, 0.0);
    }
    const auto impact = core.detectImpact(0.4, stationary, true, -2.0);
    EXPECT_FALSE(impact.impact_triggered);
    EXPECT_FALSE(impact.stop_hold_active);
}

}  // namespace
