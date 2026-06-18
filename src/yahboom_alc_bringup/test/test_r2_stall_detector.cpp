#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "yahboom_alc_bringup/r2_stall_detector.hpp"

namespace yahboom_alc_bringup
{
namespace
{

R2StallDetectorConfig testConfig() {
    R2StallDetectorConfig config;
    config.forward_command_min_duration_sec = 0.5;
    config.forward_no_motion_check_sec = 0.8;
    config.command_vx_threshold_mps = 0.08;
    config.wheel_odom_delta_threshold_m = 0.05;
    config.slam_delta_max_m = 0.02;
    config.imu_yaw_rate_threshold_radps = 0.08;
    config.depth_obstacle_enabled = true;
    config.depth_roi_center_width_ratio = 0.20;
    config.depth_roi_center_height_ratio = 0.50;
    config.depth_min_valid_m = 0.20;
    config.depth_forward_obstacle_range_m = 0.60;
    config.depth_min_obstacle_pixels = 5;
    config.retrigger_cooldown_sec = 0.0;
    return config;
}

StallDetectorObservation observation(const double time_sec) {
    StallDetectorObservation obs;
    obs.time_sec = time_sec;
    obs.cmd_vx = 0.10;
    return obs;
}

struct Depth16ImageSpec
{
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint16_t fill_mm = 0;
};

struct Depth32ImageSpec
{
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    float fill_m = 0.0F;
};

struct Depth16Pixel
{
    std::uint32_t x = 0;
    std::uint32_t y = 0;
    std::uint16_t depth_mm = 0;
};

struct Depth32Pixel
{
    std::uint32_t x = 0;
    std::uint32_t y = 0;
    float depth_m = 0.0F;
};

sensor_msgs::msg::Image makeDepth16Image(const Depth16ImageSpec& spec) {
    sensor_msgs::msg::Image image;
    image.width = spec.width;
    image.height = spec.height;
    image.encoding = "16UC1";
    image.step = spec.width * sizeof(std::uint16_t);
    image.data.resize(static_cast<std::size_t>(image.step) * spec.height);
    for (std::uint32_t y = 0; y < spec.height; ++y) {
        for (std::uint32_t x = 0; x < spec.width; ++x) {
            const auto offset =
                static_cast<std::size_t>(y) * image.step +
                static_cast<std::size_t>(x) * sizeof(std::uint16_t);
            std::memcpy(image.data.data() + offset, &spec.fill_mm,
                        sizeof(spec.fill_mm));
        }
    }
    return image;
}

sensor_msgs::msg::Image makeDepth32Image(const Depth32ImageSpec& spec) {
    sensor_msgs::msg::Image image;
    image.width = spec.width;
    image.height = spec.height;
    image.encoding = "32FC1";
    image.step = spec.width * sizeof(float);
    image.data.resize(static_cast<std::size_t>(image.step) * spec.height);
    for (std::uint32_t y = 0; y < spec.height; ++y) {
        for (std::uint32_t x = 0; x < spec.width; ++x) {
            const auto offset = static_cast<std::size_t>(y) * image.step +
                                static_cast<std::size_t>(x) * sizeof(float);
            std::memcpy(image.data.data() + offset, &spec.fill_m,
                        sizeof(spec.fill_m));
        }
    }
    return image;
}

void setDepth16(sensor_msgs::msg::Image& image, const Depth16Pixel& pixel) {
    const auto offset =
        static_cast<std::size_t>(pixel.y) * image.step +
        static_cast<std::size_t>(pixel.x) * sizeof(std::uint16_t);
    std::memcpy(image.data.data() + offset, &pixel.depth_mm,
                sizeof(pixel.depth_mm));
}

void setDepth32(sensor_msgs::msg::Image& image, const Depth32Pixel& pixel) {
    const auto offset = static_cast<std::size_t>(pixel.y) * image.step +
                        static_cast<std::size_t>(pixel.x) * sizeof(float);
    std::memcpy(image.data.data() + offset, &pixel.depth_m,
                sizeof(pixel.depth_m));
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
    EXPECT_EQ(decision.reason, "wheel_odom_without_slam_motion");
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

TEST(R2StallDetector, DoesNotTriggerWhenYawMotionIsObserved) {
    StallDetectorCore detector(testConfig());

    EXPECT_FALSE(detector.update(observation(0.0)).stall_or_slip);
    auto second = observation(0.6);
    second.wheel_odom_x = 0.06;
    second.imu_yaw_rate = 0.09;

    const auto decision = detector.update(second);
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "imu_yaw_motion_observed");
    EXPECT_TRUE(decision.imu_motion_consistent);
}

TEST(R2StallDetector, YawMotionDoesNotResetAccumulatedWindow) {
    StallDetectorCore detector(testConfig());

    EXPECT_FALSE(detector.update(observation(0.0)).stall_or_slip);
    auto yawing = observation(0.6);
    yawing.wheel_odom_x = 0.01;
    yawing.imu_yaw_rate = 0.09;
    EXPECT_EQ(detector.update(yawing).reason, "imu_yaw_motion_observed");

    auto wedged = observation(0.9);
    wedged.wheel_odom_x = 0.055;
    const auto decision = detector.update(wedged);
    EXPECT_TRUE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "wheel_odom_without_slam_motion");
    EXPECT_NEAR(decision.forward_duration_sec, 0.9, 1e-9);
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

TEST(R2StallDetector, DepthObstacleTriggersWhenWheelDeltaIsSmall) {
    StallDetectorCore detector(testConfig());

    EXPECT_FALSE(detector.update(observation(0.0)).stall_or_slip);
    auto second = observation(0.6);
    second.wheel_odom_x = 0.01;
    second.depth_observation_valid = true;
    second.forward_min_depth_m = 0.35;
    second.forward_obstacle_detected = true;

    const auto decision = detector.update(second);
    EXPECT_TRUE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "forward_obstacle_without_slam_motion");
    EXPECT_TRUE(decision.depth_observation_valid);
    EXPECT_TRUE(decision.forward_obstacle_detected);
    EXPECT_NEAR(decision.forward_min_depth_m, 0.35, 1e-9);
}

TEST(R2StallDetector, TriggersWhenForwardCommandHasNoSlamMotion) {
    StallDetectorCore detector(testConfig());

    EXPECT_FALSE(detector.update(observation(0.0)).stall_or_slip);
    auto second = observation(0.9);
    second.wheel_odom_x = 0.01;
    second.driver_cmd_vx = 0.10;
    second.driver_command_valid = true;

    const auto decision = detector.update(second);
    EXPECT_TRUE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "forward_command_without_slam_motion");
}

TEST(R2StallDetector, NoMotionGuardRequiresDriverForwardCommand) {
    StallDetectorCore detector(testConfig());

    EXPECT_FALSE(detector.update(observation(0.0)).stall_or_slip);
    auto second = observation(0.9);
    second.wheel_odom_x = 0.01;

    const auto decision = detector.update(second);
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "wheel_delta_below_threshold");
    EXPECT_FALSE(decision.driver_command_valid);
}

TEST(R2StallDetector, CooldownSuppressesRepeatedTriggers) {
    auto config = testConfig();
    config.retrigger_cooldown_sec = 3.0;
    StallDetectorCore detector(config);

    EXPECT_FALSE(detector.update(observation(0.0)).stall_or_slip);
    auto first_trigger = observation(0.6);
    first_trigger.wheel_odom_x = 0.06;
    EXPECT_TRUE(detector.update(first_trigger).stall_or_slip);

    auto second_trigger = observation(1.2);
    second_trigger.wheel_odom_x = 0.12;
    const auto decision = detector.update(second_trigger);
    EXPECT_FALSE(decision.stall_or_slip);
    EXPECT_EQ(decision.reason, "cooldown");
}

TEST(R2StallDetectorDepthRoi, Converts16UC1Millimeters) {
    auto config = testConfig();
    config.depth_min_obstacle_pixels = 1;
    auto image = makeDepth16Image({10, 10, 1000});
    setDepth16(image, {4, 4, 420});
    setDepth16(image, {5, 5, 350});

    const auto result = estimateForwardDepthRoi(image, config);
    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(result.obstacle_detected);
    EXPECT_NEAR(result.min_depth_m, 0.35, 1e-9);
}

TEST(R2StallDetectorDepthRoi, Converts32FC1Meters) {
    auto config = testConfig();
    config.depth_min_obstacle_pixels = 1;
    auto image = makeDepth32Image({10, 10, 1.0F});
    setDepth32(image, {4, 4, 0.42F});

    const auto result = estimateForwardDepthRoi(image, config);
    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(result.obstacle_detected);
    EXPECT_NEAR(result.min_depth_m, 0.42, 1e-6);
}

TEST(R2StallDetectorDepthRoi, IgnoresInvalidValues) {
    auto config = testConfig();
    config.depth_min_obstacle_pixels = 1;
    auto image = makeDepth32Image({10, 10, 1.0F});
    setDepth32(image, {4, 4, 0.0F});
    setDepth32(image, {5, 4, std::numeric_limits<float>::quiet_NaN()});
    setDepth32(image, {4, 5, 0.7F});
    setDepth32(image, {5, 5, 0.3F});

    const auto result = estimateForwardDepthRoi(image, config);
    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(result.obstacle_detected);
    EXPECT_NEAR(result.min_depth_m, 0.3, 1e-6);
}

TEST(R2StallDetectorDepthRoi, IgnoresOffCenterObstacle) {
    auto config = testConfig();
    auto image = makeDepth16Image({10, 10, 1000});
    setDepth16(image, {0, 4, 300});

    const auto result = estimateForwardDepthRoi(image, config);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.obstacle_pixel_count, 0);
    EXPECT_FALSE(result.obstacle_detected);
}

TEST(R2StallDetectorDepthRoi, RequiresMultipleObstaclePixels) {
    auto config = testConfig();
    config.depth_min_obstacle_pixels = 5;
    auto image = makeDepth16Image({10, 10, 1000});
    setDepth16(image, {4, 4, 350});

    const auto hot_pixel = estimateForwardDepthRoi(image, config);
    EXPECT_TRUE(hot_pixel.valid);
    EXPECT_EQ(hot_pixel.obstacle_pixel_count, 1);
    EXPECT_FALSE(hot_pixel.obstacle_detected);

    setDepth16(image, {5, 4, 350});
    setDepth16(image, {4, 5, 350});
    setDepth16(image, {5, 5, 350});
    setDepth16(image, {4, 6, 350});

    const auto obstacle = estimateForwardDepthRoi(image, config);
    EXPECT_TRUE(obstacle.valid);
    EXPECT_EQ(obstacle.obstacle_pixel_count, 5);
    EXPECT_TRUE(obstacle.obstacle_detected);
}

}  // namespace
}  // namespace yahboom_alc_bringup
