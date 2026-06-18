#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace yahboom_alc_bringup
{

enum class RecoveryStage : std::uint8_t
{
    kNone,
    kReobserve,
    kSmallRotation,
    kForwardArc,
    kReversePulse,
    kHardFailed
};

struct RecoveryStateMachineConfig
{
    int recovery_reobserve_cycles = 1;
    double recovery_small_spin_yaw_rad = 0.349066;
    int recovery_small_spin_attempts = 2;
    double recovery_forward_escape_distance_m = 1.5;
    double rear_blocked_min_latch_sec = 15.0;
    double rear_blocked_clear_forward_displacement_m = 0.25;
    double recovery_clearance_backup_m = 0.05;
};

class RecoveryStateMachine
{
public:
    explicit RecoveryStateMachine(RecoveryStateMachineConfig config = {});

    void setConfig(const RecoveryStateMachineConfig& config);
    const RecoveryStateMachineConfig& config() const { return config_; }

    void reset();
    void start();
    void startAtSmallRotation();
    void beginReobserve();

    RecoveryStage stage() const { return stage_; }
    int reobserveCyclesRemaining() const { return reobserve_cycles_remaining_; }
    int spinAttempts() const { return spin_attempts_; }

    bool tickReobserve();
    std::optional<double> nextSmallRotationYaw();
    void advanceToForwardArc();
    void advanceToReversePulse();
    void hardFail();

    void latchRearBlocked(double now_sec);
    bool rearBlockedLatchActive() const { return rear_blocked_latch_active_; }
    double forwardDisplacementSinceLatch() const {
        return forward_displacement_since_latch_m_;
    }
    bool noteForwardDisplacement(double displacement_m, double now_sec);

private:
    RecoveryStateMachineConfig config_;
    RecoveryStage stage_ = RecoveryStage::kNone;
    int reobserve_cycles_remaining_ = 0;
    int spin_attempts_ = 0;
    bool rear_blocked_latch_active_ = false;
    double rear_blocked_latch_started_sec_ = 0.0;
    double forward_displacement_since_latch_m_ = 0.0;
};

std::string recoveryStageName(RecoveryStage stage);

}  // namespace yahboom_alc_bringup
