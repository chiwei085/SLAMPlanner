#include "yahboom_alc_bringup/recovery_state_machine.hpp"

#include <algorithm>
#include <cmath>

namespace yahboom_alc_bringup
{

RecoveryStateMachine::RecoveryStateMachine(RecoveryStateMachineConfig config)
    : config_(config) {}

void RecoveryStateMachine::setConfig(const RecoveryStateMachineConfig& config) {
    config_ = config;
}

void RecoveryStateMachine::reset() {
    stage_ = RecoveryStage::kNone;
    reobserve_cycles_remaining_ = 0;
    spin_attempts_ = 0;
    rear_blocked_latch_active_ = false;
    rear_blocked_latch_started_sec_ = 0.0;
    forward_displacement_since_latch_m_ = 0.0;
}

void RecoveryStateMachine::start() {
    spin_attempts_ = 0;
    beginReobserve();
}

void RecoveryStateMachine::startAtSmallRotation() {
    spin_attempts_ = 0;
    reobserve_cycles_remaining_ = 0;
    stage_ = RecoveryStage::kSmallRotation;
}

void RecoveryStateMachine::beginReobserve() {
    stage_ = RecoveryStage::kReobserve;
    reobserve_cycles_remaining_ =
        std::max(1, config_.recovery_reobserve_cycles);
}

bool RecoveryStateMachine::tickReobserve() {
    if (stage_ != RecoveryStage::kReobserve) {
        return false;
    }
    if (reobserve_cycles_remaining_ > 0) {
        --reobserve_cycles_remaining_;
    }
    return reobserve_cycles_remaining_ <= 0;
}

ReobserveCompletion RecoveryStateMachine::resolveReobserveCompletion(
    const bool map_silent) {
    if (!map_silent) {
        reobserve_perception_stall_extensions_ = 0;
        return ReobserveCompletion::kConfirmed;
    }
    ++reobserve_perception_stall_extensions_;
    if (reobserve_perception_stall_extensions_ >=
        config_.recovery_reobserve_perception_stall_max_extensions) {
        reobserve_perception_stall_extensions_ = 0;
        return ReobserveCompletion::kPerceptionStallExhausted;
    }
    beginReobserve();
    return ReobserveCompletion::kExtended;
}

std::optional<double> RecoveryStateMachine::nextSmallRotationYaw() {
    stage_ = RecoveryStage::kSmallRotation;
    if (spin_attempts_ >= config_.recovery_small_spin_attempts) {
        return std::nullopt;
    }
    const double sign = lifetime_spin_count_ % 2 == 0 ? 1.0 : -1.0;
    ++spin_attempts_;
    ++lifetime_spin_count_;
    return sign * config_.recovery_small_spin_yaw_rad;
}

void RecoveryStateMachine::advanceToForwardArc() {
    stage_ = RecoveryStage::kForwardArc;
}

void RecoveryStateMachine::advanceToReversePulse() {
    stage_ = RecoveryStage::kReversePulse;
}

void RecoveryStateMachine::hardFail() {
    stage_ = RecoveryStage::kHardFailed;
}

bool RecoveryStateMachine::noteTreeStartedAndCheckEscalation(const double x,
                                                             const double y) {
    const double dx = x - stagnation_anchor_x_;
    const double dy = y - stagnation_anchor_y_;
    if (!stagnation_anchor_valid_ ||
        std::hypot(dx, dy) > config_.recovery_stagnation_radius_m) {
        stagnation_anchor_valid_ = true;
        stagnation_anchor_x_ = x;
        stagnation_anchor_y_ = y;
        stagnant_tree_count_ = 0;
    }
    ++stagnant_tree_count_;
    if (config_.recovery_stagnation_escalation_trees <= 0 ||
        stagnant_tree_count_ < config_.recovery_stagnation_escalation_trees) {
        return false;
    }
    stagnant_tree_count_ = 0;
    stagnation_anchor_valid_ = false;
    return true;
}

void RecoveryStateMachine::latchRearBlocked(const double now_sec) {
    rear_blocked_latch_active_ = true;
    rear_blocked_latch_started_sec_ = now_sec;
    forward_displacement_since_latch_m_ = 0.0;
}

bool RecoveryStateMachine::noteForwardDisplacement(const double displacement_m,
                                                   const double now_sec) {
    if (!rear_blocked_latch_active_) {
        return false;
    }
    forward_displacement_since_latch_m_ += std::max(0.0, displacement_m);
    const bool held_long_enough = now_sec - rear_blocked_latch_started_sec_ >=
                                  config_.rear_blocked_min_latch_sec;
    const bool moved_far_enough =
        forward_displacement_since_latch_m_ >=
        config_.rear_blocked_clear_forward_displacement_m;
    if (!held_long_enough || !moved_far_enough) {
        return false;
    }
    rear_blocked_latch_active_ = false;
    rear_blocked_latch_started_sec_ = 0.0;
    forward_displacement_since_latch_m_ = 0.0;
    return true;
}

std::string recoveryStageName(const RecoveryStage stage) {
    switch (stage) {
        case RecoveryStage::kNone:
            return "none";
        case RecoveryStage::kReobserve:
            return "stop_reobserve";
        case RecoveryStage::kSmallRotation:
            return "small_rotation_reobserve";
        case RecoveryStage::kForwardArc:
            return "forward_arc_escape";
        case RecoveryStage::kReversePulse:
            return "verified_reverse_pulse";
        case RecoveryStage::kHardFailed:
            return "hard_fail";
    }
    return "unknown";
}

}  // namespace yahboom_alc_bringup
