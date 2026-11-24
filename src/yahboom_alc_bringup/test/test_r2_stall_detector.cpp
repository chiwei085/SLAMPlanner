#include <gtest/gtest.h>

#include "yahboom_alc_bringup/r2_stall_detector.hpp"

namespace yahboom_alc_bringup
{
namespace
{

R2StallDetectorConfig testConfig() {
    R2StallDetectorConfig config;
    config.command_vx_threshold_mps = 0.05;
    config.min_expected_motion_m = 0.06;
    config.stuck_motion_ratio = 0.3;
    config.imu_yaw_rate_threshold_radps = 0.08;
    config.retrigger_cooldown_sec = 0.0;
    return config;
}

StallDetectorObservation driving(const double time_sec, const double odom_x,
                                 const double cmd_vx = 0.08) {
    StallDetectorObservation obs;
    obs.time_sec = time_sec;
    obs.driver_cmd_vx = cmd_vx;
    obs.odom_x = odom_x;
    return obs;
}

// Feeds 0.1 s ticks from t0 (exclusive) to t1 (inclusive) at a fixed odom
// position; returns the first stall decision, or the last decision.
StallDetectorDecision driveUntilStall(StallDetectorCore& detector,
                                      const double t0, const double t1,
                                      const double odom_x) {
    StallDetectorDecision decision;
    for (int i = 1; t0 + 0.1 * i <= t1 + 1e-9; ++i) {
        decision = detector.update(driving(t0 + 0.1 * i, odom_x));
        if (decision.stall_or_slip) {
            return decision;
        }
    }
    return decision;
}

TEST(R2StallDetector, IgnoresCommandsAtOrBelowThreshold) {
    StallDetectorCore detector(testConfig());
    detector.update(driving(0.0, 0.0, 0.05));
    const auto decision = detector.update(driving(2.0, 0.0, 0.05));
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "forward_command_inactive");
}

// The bridge's 0.08 m/s floor is the speed the robot usually drives at; the
// old nav-command > 0.08 gate left 72% of forward driving unwatched.
TEST(R2StallDetector, JudgesTheBridgeFloorSpeed) {
    StallDetectorCore detector(testConfig());
    detector.update(driving(0.0, 0.0));
    const auto decision = driveUntilStall(detector, 0.0, 1.0, 0.0);
    EXPECT_TRUE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "commanded_motion_not_observed");
}

TEST(R2StallDetector, WaitsUntilCommandedMotionExceedsNoise) {
    StallDetectorCore detector(testConfig());
    detector.update(driving(0.0, 0.0));
    // 0.6 s x 0.08 m/s = 4.8 cm commanded: too little to judge.
    const auto decision = detector.update(driving(0.6, 0.0));
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "expected_motion_below_noise");
    EXPECT_NEAR(decision.expected_motion_m, 0.048, 1e-9);
}

TEST(R2StallDetector, ProportionalProgressIsNotAStall) {
    StallDetectorCore detector(testConfig());
    detector.update(driving(0.0, 0.0));
    detector.update(driving(0.5, 0.02));
    // 8 cm commanded, 4 cm moved: slower than commanded, not stuck.
    const auto decision = detector.update(driving(1.0, 0.04));
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "motion_observed");
}

TEST(R2StallDetector, FiresWhenMotionIsBelowRatio) {
    StallDetectorCore detector(testConfig());
    detector.update(driving(0.0, 0.0));
    detector.update(driving(0.5, 0.005));
    // 8 cm commanded, 1 cm moved (< 30%).
    const auto decision = detector.update(driving(1.0, 0.01));
    EXPECT_TRUE(decision.stall_or_slip);
    EXPECT_NEAR(decision.odom_delta_m, 0.01, 1e-9);
}

// Reverse to forward: the car coasts back, then passes its window start again
// with zero net displacement while driving (real car, 2026-11-24).
TEST(R2StallDetector, DrivingPastTheWindowStartIsNotAStall) {
    StallDetectorCore detector(testConfig());
    detector.update(driving(0.0, 0.0));
    StallDetectorDecision decision;
    for (int i = 1; i <= 9; ++i) {
        const double x = i < 8 ? -0.02 : 0.012 * (i - 8);
        decision = detector.update(driving(0.1 * i, x));
        ASSERT_FALSE(decision.stall_or_slip);
    }
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "motion_observed");
}

TEST(R2StallDetector, YawMotionIsNotAStall) {
    StallDetectorCore detector(testConfig());
    detector.update(driving(0.0, 0.0));
    auto turning = driving(1.0, 0.0);
    turning.imu_yaw_rate = 0.2;
    const auto decision = detector.update(turning);
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "imu_yaw_motion_observed");
}

TEST(R2StallDetector, CooldownSuppressesRepeatedTriggers) {
    auto config = testConfig();
    config.retrigger_cooldown_sec = 3.0;
    StallDetectorCore detector(config);
    detector.update(driving(0.0, 0.0));
    ASSERT_TRUE(driveUntilStall(detector, 0.0, 1.0, 0.0).stall_or_slip);
    const auto decision = driveUntilStall(detector, 1.0, 2.5, 0.0);
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "cooldown");
}

// 2026-11-21 bridge-less starts: 0.2-2.0 cm after 1.0 s at 0.08 m/s, so a
// window started from rest must not count those commands.
TEST(R2StallDetector, StartResponseDelayIgnoresChassisDeadTime) {
    auto config = testConfig();
    config.start_response_delay_sec = 1.5;
    StallDetectorCore detector(config);
    detector.update(driving(0.0, 0.0));
    for (int i = 1; i <= 15; ++i) {
        const auto decision = detector.update(driving(0.1 * i, 0.002));
        EXPECT_FALSE(decision.stall_or_slip) << i;
    }
    // Counting from 1.5 s, 6 cm is reached at 2.2 s; 0.2 cm moved -> stuck.
    const auto decision = driveUntilStall(detector, 1.5, 2.5, 0.002);
    EXPECT_TRUE(decision.stall_or_slip);
    EXPECT_NEAR(decision.forward_duration_sec, 2.2, 1e-6);
}

TEST(R2StallDetector, StartResponseDelayNotAppliedAfterObservedMotion) {
    auto config = testConfig();
    config.start_response_delay_sec = 1.5;
    StallDetectorCore detector(config);
    detector.update(driving(0.0, 0.0));
    double t = 0.0;
    StallDetectorDecision decision;
    while (decision.reason != "motion_observed") {
        t += 0.1;
        ASSERT_LT(t, 3.0);
        decision = detector.update(driving(t, 0.08 * t));
    }
    // Blocked mid-drive: judged once 6 cm is commanded, with no dead time.
    const double blocked_at = t;
    decision = driveUntilStall(detector, t, t + 1.0, 0.08 * blocked_at);
    EXPECT_TRUE(decision.stall_or_slip);
    EXPECT_LE(decision.forward_duration_sec, 0.8 + 1e-6);
}

}  // namespace
}  // namespace yahboom_alc_bringup
