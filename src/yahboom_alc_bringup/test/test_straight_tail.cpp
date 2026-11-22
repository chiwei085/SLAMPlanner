#include <gtest/gtest.h>

#include <cmath>

#include "yahboom_alc_bringup/straight_tail.hpp"

namespace yahboom_alc_bringup
{
namespace
{

constexpr double kLookahead = 0.35;
constexpr double kMaxCurvature = 2.0;

TEST(StraightTail, CarrotBearingLimitIsSteeringLimit) {
    // kappa = 2 sin(alpha) / L_d <= kappa_max  =>  sin(alpha) <= 0.35.
    EXPECT_NEAR(maxCarrotBearing(kLookahead, kMaxCurvature), std::asin(0.35),
                1e-12);
    // A limit above 1 saturates at a quarter turn instead of NaN.
    EXPECT_NEAR(maxCarrotBearing(2.0, 2.0), M_PI / 2.0, 1e-12);
}

TEST(StraightTail, AlignedRobotOnTheLineIsCaptured) {
    const TailPose goal{2.0, 1.0, M_PI / 2.0};
    const auto along = tailCaptureAlong({2.0, 0.0, M_PI / 2.0}, goal,
                                        kLookahead, kMaxCurvature);
    ASSERT_TRUE(along);
    EXPECT_NEAR(*along, -1.0, 1e-9);
}

TEST(StraightTail, SmallOffsetWithinSteeringLimitIsCaptured) {
    // 0.05 m beside the line, heading along it: the carrot bearing is
    // asin(0.05 / 0.35) ~ 0.143 rad < asin(0.35) ~ 0.358 rad.
    const auto along = tailCaptureAlong({-1.0, 0.05, 0.0}, {0.0, 0.0, 0.0},
                                        kLookahead, kMaxCurvature);
    ASSERT_TRUE(along);
    EXPECT_NEAR(*along, -1.0, 1e-9);
}

TEST(StraightTail, HeadingBeyondSteeringLimitIsNotCaptured) {
    // On the line but heading 30 deg off it: carrot bearing 0.52 > 0.358.
    EXPECT_FALSE(tailCaptureAlong({-1.0, 0.0, 0.52}, {0.0, 0.0, 0.0},
                                  kLookahead, kMaxCurvature));
    // Driving the wrong way along the line.
    EXPECT_FALSE(tailCaptureAlong({-1.0, 0.0, M_PI}, {0.0, 0.0, 0.0},
                                  kLookahead, kMaxCurvature));
}

TEST(StraightTail, NoCarrotOrPastGoalIsNotCaptured) {
    // Lateral offset >= L_d: the lookahead circle misses the line.
    EXPECT_FALSE(tailCaptureAlong({-1.0, 0.35, 0.0}, {0.0, 0.0, 0.0},
                                  kLookahead, kMaxCurvature));
    // Already past the goal along its yaw.
    EXPECT_FALSE(tailCaptureAlong({0.1, 0.0, 0.0}, {0.0, 0.0, 0.0}, kLookahead,
                                  kMaxCurvature));
}

TEST(StraightTail, TailPosesSpanTheTailAtTheGoalYaw) {
    const TailPose goal{1.0, 2.0, M_PI / 4.0};
    const auto poses = tailPoses(goal, -1.4, 0.0, 0.05);
    ASSERT_EQ(poses.size(), 29U);
    EXPECT_NEAR(poses.front().x, 1.0 - 1.4 * std::cos(M_PI / 4.0), 1e-9);
    EXPECT_NEAR(poses.front().y, 2.0 - 1.4 * std::sin(M_PI / 4.0), 1e-9);
    EXPECT_NEAR(poses.back().x, 1.0, 1e-9);
    EXPECT_NEAR(poses.back().y, 2.0, 1e-9);
    for (std::size_t i = 1; i < poses.size(); ++i) {
        EXPECT_NEAR(poses[i].yaw, M_PI / 4.0, 1e-12);
        EXPECT_LE(std::hypot(poses[i].x - poses[i - 1].x,
                             poses[i].y - poses[i - 1].y),
                  0.05 + 1e-9);
    }
}

}  // namespace
}  // namespace yahboom_alc_bringup
