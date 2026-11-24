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
    double recovery_clearance_backup_m = 0.05;
    double recovery_spin_time_allowance_sec = 15.0;
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
    int spinAttempts() const { return spin_attempts_; }

    bool tickReobserve();
    std::optional<double> nextSmallRotationYaw();
    void advanceToForwardArc();
    void advanceToReversePulse();
    void hardFail();

private:
    RecoveryStateMachineConfig config_;
    RecoveryStage stage_ = RecoveryStage::kNone;
    int reobserve_cycles_remaining_ = 0;
    int spin_attempts_ = 0;
    // Counts every spin ever executed; never reset so direction alternates
    // across recovery cycles even when spin_attempts_ is zeroed mid-sequence.
    int lifetime_spin_count_ = 0;
};

std::string recoveryStageName(RecoveryStage stage);

}  // namespace yahboom_alc_bringup
