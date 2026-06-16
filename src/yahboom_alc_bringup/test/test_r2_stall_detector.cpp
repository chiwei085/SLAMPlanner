#include <gtest/gtest.h>

#include "yahboom_alc_bringup/r2_stall_detector.hpp"

namespace yahboom_alc_bringup
{
namespace
{

R2StallDetectorConfig testConfig() {
    R2StallDetectorConfig config;
    config.forward_command_min_duration_sec = 0.5;
    config.command_vx_threshold_mps = 0.08;
    config.wheel_odom_delta_threshold_m = 0.05;
    config.slam_delta_max_m = 0.02;
    config.imu_linear_accel_threshold_mps2 = 0.25;
    config.imu_yaw_rate_threshold_radps = 0.08;
    config.retrigger_cooldown_sec = 0.0;
    return config;
}

StallDetectorObservation observation(const double time_sec) {
    StallDetectorObservation obs;
    obs.time_sec = time_sec;
    obs.cmd_vx = 0.10;
    return obs;
}

TEST(R2StallDetector, TriggersWhenWheelOdomMovesWithoutSlamOrImuMotion) {
    StallDetectorCore detector(testConfig());

    auto first = observation(0.0);
    EXPECT_FALSE(detector.update(first).stall_or_slip);

    auto second = observation(0.6);
    second.wheel_odom_x = 0.06;
    second.slam_x = 0.005;

    const auto decision = detector.update(second);
    EXPECT_TRUE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "wheel_odom_without_slam_or_imu_motion");
    EXPECT_NEAR(decision.wheel_delta_m, 0.06, 1e-9);
    EXPECT_NEAR(decision.slam_delta_m, 0.005, 1e-9);
}

TEST(R2StallDetector, DoesNotTriggerBeforeCommandDurationThreshold) {
    StallDetectorCore detector(testConfig());

    EXPECT_FALSE(detector.update(observation(0.0)).stall_or_slip);
    auto second = observation(0.4);
    second.wheel_odom_x = 0.06;

    const auto decision = detector.update(second);
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "duration_below_threshold");
}

TEST(R2StallDetector, DoesNotTriggerBelowCommandSpeedThreshold) {
    StallDetectorCore detector(testConfig());

    auto first = observation(0.0);
    first.cmd_vx = 0.04;

    const auto decision = detector.update(first);
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "forward_command_inactive");
}

TEST(R2StallDetector, DoesNotTriggerWhenSlamMotionIsObserved) {
    StallDetectorCore detector(testConfig());

    EXPECT_FALSE(detector.update(observation(0.0)).stall_or_slip);
    auto second = observation(0.6);
    second.wheel_odom_x = 0.06;
    second.slam_x = 0.03;

    const auto decision = detector.update(second);
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "slam_motion_observed");
}

TEST(R2StallDetector, DoesNotTriggerWhenImuMotionIsObserved) {
    StallDetectorCore detector(testConfig());

    EXPECT_FALSE(detector.update(observation(0.0)).stall_or_slip);
    auto second = observation(0.6);
    second.wheel_odom_x = 0.06;
    second.imu_linear_accel_x = 0.3;

    const auto decision = detector.update(second);
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "imu_motion_observed");
    EXPECT_TRUE(decision.imu_motion_consistent);
}

TEST(R2StallDetector, DoesNotTriggerWhenWheelDeltaIsSmall) {
    StallDetectorCore detector(testConfig());

    EXPECT_FALSE(detector.update(observation(0.0)).stall_or_slip);
    auto second = observation(0.6);
    second.wheel_odom_x = 0.03;

    const auto decision = detector.update(second);
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "wheel_delta_below_threshold");
}

}  // namespace
}  // namespace yahboom_alc_bringup
