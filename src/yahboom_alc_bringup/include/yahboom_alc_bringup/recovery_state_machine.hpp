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

// Outcome of resolving a reobserve cycle-count expiry against whether /map
// has actually kept producing during the wait.
enum class ReobserveCompletion : std::uint8_t
{
    kConfirmed,  // perception was alive: safe to treat as complete
    kExtended,   // perception silent, extension budget remains
    kPerceptionStallExhausted  // extension budget exhausted: caller should
                               // try an active small-rotation scan (waiting
                               // longer cannot help -- standing still is why
                               // /map stopped) and only pause, never
                               // blacklist, once that budget is also spent --
                               // the goal was never the problem
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
    // Cross-tree stagnation escalation: when this many consecutive recovery
    // trees start with the robot still within recovery_stagnation_radius_m of
    // where the first of them started, the tree entry is escalated straight
    // to the reverse-pulse/backtrace stage.  Reobserve/spin stages reset the
    // per-tree spin counter every time the explorer re-sends a goal, so
    // without this counter the backtrace stage is unreachable in practice.
    // 0 disables escalation.
    int recovery_stagnation_escalation_trees = 3;
    double recovery_stagnation_radius_m = 0.25;
    // A stop_reobserve stage that finishes its cycle count while /map has not
    // delivered a new message in this long is not "observed and found
    // nothing" -- the producer is silent, so the cycle-count timer alone
    // cannot tell quiescence apart from a dead perception pipeline.  Treat it
    // as inconclusive: re-arm reobserve instead of declaring completion.
    double recovery_reobserve_map_stall_ceiling_sec = 8.0;
    // Bound on how many times reobserve may be re-armed for map silence
    // before giving up and pausing exploration outright (never blacklists a
    // goal for this -- the goal was never the problem).
    int recovery_reobserve_perception_stall_max_extensions = 3;
    // Wall-clock bound on a perception-stall pause.  The pause's normal exit
    // is /map receiving again, but /map only publishes on robot motion and
    // the pause itself keeps the robot stationary -- and a stale_costmap
    // pause is really about the local costmap, which this node does not
    // observe at all.  Waiting on /map silence alone can therefore deadlock
    // forever; after this long the pause lifts unconditionally and
    // exploration retries, re-pausing (bounded, observable) if the bridge
    // still cannot see.
    double recovery_perception_stall_pause_max_sec = 30.0;
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
    // Call only after tickReobserve() returns true.  map_silent is whether
    // /map's receive-silence has exceeded
    // recovery_reobserve_map_stall_ceiling_sec.  Re-arms reobserve
    // internally (via beginReobserve()) on kExtended.
    ReobserveCompletion resolveReobserveCompletion(bool map_silent);
    std::optional<double> nextSmallRotationYaw();
    void advanceToForwardArc();
    void advanceToReversePulse();
    void hardFail();

    // Cross-tree stagnation tracking.  Survives reset(); call once at every
    // recovery-tree start with the robot position.  Returns true when the
    // escalation threshold is reached (the counter re-arms on that call).
    bool noteTreeStartedAndCheckEscalation(double x, double y);
    int stagnantTreeCount() const { return stagnant_tree_count_; }

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
    // Counts every spin ever executed; never reset so direction alternates
    // across recovery cycles even when spin_attempts_ is zeroed mid-sequence.
    int lifetime_spin_count_ = 0;
    bool rear_blocked_latch_active_ = false;
    double rear_blocked_latch_started_sec_ = 0.0;
    double forward_displacement_since_latch_m_ = 0.0;
    // Cross-tree stagnation state; deliberately not touched by reset() so it
    // survives the per-goal recovery tree teardown.
    bool stagnation_anchor_valid_ = false;
    double stagnation_anchor_x_ = 0.0;
    double stagnation_anchor_y_ = 0.0;
    int stagnant_tree_count_ = 0;
    // Deliberately not touched by reset(): a stalled perception pipeline
    // outlives any single recovery tree's reobserve stage.
    int reobserve_perception_stall_extensions_ = 0;
};

std::string recoveryStageName(RecoveryStage stage);

}  // namespace yahboom_alc_bringup
