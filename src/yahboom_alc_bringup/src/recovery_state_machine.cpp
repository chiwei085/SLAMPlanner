#include "yahboom_alc_bringup/recovery_state_machine.hpp"

#include <algorithm>

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
