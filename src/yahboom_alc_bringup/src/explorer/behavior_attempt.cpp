#include "explorer/cluster_frontier_explorer_node.hpp"

namespace yahboom_alc_bringup
{

bool ClusterFrontierExplorerNode::behaviorActive(
    const BehaviorPurpose purpose) const {
    return behavior_ && behavior_->purpose == purpose;
}

// Ends the attempt in flight without running its follow-up. A goal not
// yet accepted is cancelled when its response arrives with a stale id.
void ClusterFrontierExplorerNode::cancelBehavior() {
    if (!behavior_) {
        return;
    }
    safeCancelGoal(spin_client_, behavior_->spin_handle, "spin_goal");
    safeCancelGoal(backup_client_, behavior_->backup_handle, "backup_goal");
    behavior_.reset();
}

// Spins are bounded here as well as by the server's time allowance, in
// case the result never arrives.
void ClusterFrontierExplorerNode::checkBehaviorTimeout(const double now) {
    if (!behaviorActive(BehaviorPurpose::kAlignSpin) &&
        !behaviorActive(BehaviorPurpose::kRecoverySpin)) {
        return;
    }
    if (now - behavior_->started_sec <
        core_.config().alignment_spin_time_allowance_sec +
            kSpinTransportMarginSec) {
        return;
    }
    const auto attempt = *behavior_;
    cancelBehavior();
    onBehaviorEnded(attempt, BehaviorOutcome::kTimedOut);
}

void ClusterFrontierExplorerNode::onBehaviorEnded(
    const BehaviorAttempt& attempt, const BehaviorOutcome outcome) {
    const bool ok = outcome == BehaviorOutcome::kSucceeded;
    const char* spin_reason =
        ok                                      ? "spin_completed"
        : outcome == BehaviorOutcome::kFailed   ? "spin_failed"
        : outcome == BehaviorOutcome::kRejected ? "spin_rejected"
                                                : "spin_timeout";
    switch (attempt.purpose) {
        case BehaviorPurpose::kAlignSpin:
            finishSpinRecovery(spin_reason, attempt.started_sec);
            return;
        case BehaviorPurpose::kRecoverySpin:
            finishRecoverySpin(
                spin_reason,
                ok ? FailureClass::kNone : FailureClass::kRecoveryFailed,
                attempt.started_sec);
            return;
        case BehaviorPurpose::kClearanceBackup:
            if (outcome != BehaviorOutcome::kRejected) {
                publishPlannerEvent(
                    "recovery_clearance_backup_finished",
                    {{"stage",
                      recoveryStageName(RecoveryStage::kSmallRotation)},
                     {"success", ok},
                     {"goal_xy", activeFinalGoalJson()}});
            }
            sendRecoverySpin(pending_recovery_spin_yaw_);
            return;
        case BehaviorPurpose::kBacktraceBackup:
            onBacktraceBackupEnded(outcome);
            return;
    }
}

}  // namespace yahboom_alc_bringup
