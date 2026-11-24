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
    EXPECT_DOUBLE_EQ(*first, testConfig().recovery_small_spin_yaw_rad);

    const auto second = recovery.nextSmallRotationYaw();
    ASSERT_TRUE(second);
    EXPECT_DOUBLE_EQ(*second, -testConfig().recovery_small_spin_yaw_rad);

    EXPECT_FALSE(recovery.nextSmallRotationYaw());
    recovery.advanceToForwardArc();
    EXPECT_EQ(recovery.stage(), RecoveryStage::kForwardArc);
}

}  // namespace
}  // namespace yahboom_alc_bringup
