#include <gtest/gtest.h>

#include "yahboom_alc_bringup/recovery_state_machine.hpp"

namespace yahboom_alc_bringup
{
namespace
{

RecoveryStateMachineConfig testConfig() {
    RecoveryStateMachineConfig config;
    config.recovery_reobserve_cycles = 1;
    config.recovery_small_spin_yaw_rad = 0.349066;
    config.recovery_small_spin_attempts = 2;
    config.recovery_forward_escape_distance_m = 1.5;
    config.rear_blocked_min_latch_sec = 15.0;
    config.rear_blocked_clear_forward_displacement_m = 0.25;
    return config;
}

TEST(RecoveryStateMachine, StartsWithReobserveNotReverse) {
    RecoveryStateMachine recovery(testConfig());

    recovery.start();

    EXPECT_EQ(recovery.stage(), RecoveryStage::kReobserve);
    EXPECT_TRUE(recovery.tickReobserve());
}

TEST(RecoveryStateMachine, SmallRotationSequenceAlternatesBeforeForwardArc) {
    RecoveryStateMachine recovery(testConfig());
    recovery.start();
    ASSERT_TRUE(recovery.tickReobserve());

    const auto first = recovery.nextSmallRotationYaw();
    ASSERT_TRUE(first);
    EXPECT_GT(first.value_or(0.0), 0.0);

    const auto second = recovery.nextSmallRotationYaw();
    ASSERT_TRUE(second);
    EXPECT_LT(second.value_or(0.0), 0.0);

    EXPECT_FALSE(recovery.nextSmallRotationYaw());
    recovery.advanceToForwardArc();
    EXPECT_EQ(recovery.stage(), RecoveryStage::kForwardArc);
}

TEST(RecoveryStateMachine, RearLatchRequiresTimeAndForwardDisplacement) {
    RecoveryStateMachine recovery(testConfig());

    recovery.latchRearBlocked(10.0);
    EXPECT_TRUE(recovery.rearBlockedLatchActive());
    EXPECT_FALSE(recovery.noteForwardDisplacement(0.30, 20.0));
    EXPECT_TRUE(recovery.rearBlockedLatchActive());
    EXPECT_FALSE(recovery.noteForwardDisplacement(0.01, 24.9));
    EXPECT_TRUE(recovery.noteForwardDisplacement(0.01, 25.0));
    EXPECT_FALSE(recovery.rearBlockedLatchActive());

    recovery.latchRearBlocked(30.0);
    EXPECT_FALSE(recovery.noteForwardDisplacement(0.24, 45.0));
    EXPECT_TRUE(recovery.rearBlockedLatchActive());
    EXPECT_TRUE(recovery.noteForwardDisplacement(0.02, 45.1));
    EXPECT_FALSE(recovery.rearBlockedLatchActive());
}

TEST(RecoveryStateMachine, ResetClearsRearBlockedLatch) {
    RecoveryStateMachine recovery(testConfig());

    recovery.latchRearBlocked(1.0);
    EXPECT_TRUE(recovery.rearBlockedLatchActive());
    recovery.reset();

    EXPECT_EQ(recovery.stage(), RecoveryStage::kNone);
    EXPECT_FALSE(recovery.rearBlockedLatchActive());
    EXPECT_NEAR(recovery.forwardDisplacementSinceLatch(), 0.0, 1e-9);
}

TEST(RecoveryStateMachine, StagnationEscalatesAfterConsecutiveTreesInPlace) {
    auto config = testConfig();
    config.recovery_stagnation_escalation_trees = 3;
    config.recovery_stagnation_radius_m = 0.25;
    RecoveryStateMachine recovery(config);

    EXPECT_FALSE(recovery.noteTreeStartedAndCheckEscalation(0.0, 0.0));
    EXPECT_FALSE(recovery.noteTreeStartedAndCheckEscalation(0.05, -0.02));
    EXPECT_TRUE(recovery.noteTreeStartedAndCheckEscalation(0.10, 0.03));
}

TEST(RecoveryStateMachine, StagnationCounterRearmsAfterEscalation) {
    auto config = testConfig();
    config.recovery_stagnation_escalation_trees = 2;
    RecoveryStateMachine recovery(config);

    EXPECT_FALSE(recovery.noteTreeStartedAndCheckEscalation(0.0, 0.0));
    EXPECT_TRUE(recovery.noteTreeStartedAndCheckEscalation(0.0, 0.0));
    EXPECT_FALSE(recovery.noteTreeStartedAndCheckEscalation(0.0, 0.0));
    EXPECT_TRUE(recovery.noteTreeStartedAndCheckEscalation(0.0, 0.0));
}

TEST(RecoveryStateMachine, StagnationResetsWhenRobotMovesPastRadius) {
    auto config = testConfig();
    config.recovery_stagnation_escalation_trees = 3;
    config.recovery_stagnation_radius_m = 0.25;
    RecoveryStateMachine recovery(config);

    EXPECT_FALSE(recovery.noteTreeStartedAndCheckEscalation(0.0, 0.0));
    EXPECT_FALSE(recovery.noteTreeStartedAndCheckEscalation(0.1, 0.0));
    // Robot escaped beyond the radius: anchor moves, count restarts.
    EXPECT_FALSE(recovery.noteTreeStartedAndCheckEscalation(1.0, 0.0));
    EXPECT_FALSE(recovery.noteTreeStartedAndCheckEscalation(1.05, 0.0));
    EXPECT_TRUE(recovery.noteTreeStartedAndCheckEscalation(1.10, 0.0));
}

TEST(RecoveryStateMachine, StagnationSurvivesPerTreeReset) {
    auto config = testConfig();
    config.recovery_stagnation_escalation_trees = 3;
    RecoveryStateMachine recovery(config);

    EXPECT_FALSE(recovery.noteTreeStartedAndCheckEscalation(0.0, 0.0));
    recovery.reset();
    EXPECT_FALSE(recovery.noteTreeStartedAndCheckEscalation(0.0, 0.0));
    recovery.reset();
    EXPECT_TRUE(recovery.noteTreeStartedAndCheckEscalation(0.0, 0.0));
}

TEST(RecoveryStateMachine, StagnationDisabledWithZeroThreshold) {
    auto config = testConfig();
    config.recovery_stagnation_escalation_trees = 0;
    RecoveryStateMachine recovery(config);

    for (int i = 0; i < 10; ++i) {
        EXPECT_FALSE(recovery.noteTreeStartedAndCheckEscalation(0.0, 0.0));
    }
}

TEST(RecoveryStateMachine, ReobserveConfirmsImmediatelyWhenMapNotSilent) {
    RecoveryStateMachine recovery(testConfig());
    recovery.start();
    ASSERT_TRUE(recovery.tickReobserve());

    EXPECT_EQ(recovery.resolveReobserveCompletion(/*map_silent=*/false),
              ReobserveCompletion::kConfirmed);
}

TEST(RecoveryStateMachine, ReobserveExtendsAndRearmsWhileMapSilent) {
    auto config = testConfig();
    config.recovery_reobserve_perception_stall_max_extensions = 3;
    RecoveryStateMachine recovery(config);
    recovery.start();
    ASSERT_TRUE(recovery.tickReobserve());

    EXPECT_EQ(recovery.resolveReobserveCompletion(/*map_silent=*/true),
              ReobserveCompletion::kExtended);
    // beginReobserve() re-armed the cycle count: still in reobserve, and the
    // next tick must expire again before completion is re-resolved.
    EXPECT_EQ(recovery.stage(), RecoveryStage::kReobserve);
    EXPECT_TRUE(recovery.tickReobserve());
}

TEST(RecoveryStateMachine,
     ReobservePausesAfterExhaustingStallBudgetNotBlacklist) {
    auto config = testConfig();
    config.recovery_reobserve_perception_stall_max_extensions = 2;
    RecoveryStateMachine recovery(config);
    recovery.start();
    ASSERT_TRUE(recovery.tickReobserve());

    EXPECT_EQ(recovery.resolveReobserveCompletion(/*map_silent=*/true),
              ReobserveCompletion::kExtended);
    ASSERT_TRUE(recovery.tickReobserve());
    EXPECT_EQ(recovery.resolveReobserveCompletion(/*map_silent=*/true),
              ReobserveCompletion::kPerceptionStallExhausted);
}

TEST(RecoveryStateMachine, ReobserveStallCounterResetsOnceMapRecovers) {
    auto config = testConfig();
    config.recovery_reobserve_perception_stall_max_extensions = 2;
    RecoveryStateMachine recovery(config);
    recovery.start();
    ASSERT_TRUE(recovery.tickReobserve());

    EXPECT_EQ(recovery.resolveReobserveCompletion(/*map_silent=*/true),
              ReobserveCompletion::kExtended);
    ASSERT_TRUE(recovery.tickReobserve());
    // Map recovered before the budget was exhausted: back to normal, and a
    // later stall starts a fresh budget rather than picking up where the
    // first stall left off.
    EXPECT_EQ(recovery.resolveReobserveCompletion(/*map_silent=*/false),
              ReobserveCompletion::kConfirmed);
    recovery.beginReobserve();
    ASSERT_TRUE(recovery.tickReobserve());
    EXPECT_EQ(recovery.resolveReobserveCompletion(/*map_silent=*/true),
              ReobserveCompletion::kExtended);
}

}  // namespace
}  // namespace yahboom_alc_bringup
