#pragma once

#include <string>

namespace yahboom_alc_bringup
{

enum class CoordinationAction
{
    kNone,
    kPause,
    kResume,
};

// Classifies a planner coordination event payload (JSON with an "event"
// field). Malformed payloads and unrelated events map to kNone.
CoordinationAction classifyPlannerEvent(const std::string& payload);

// Tracks whether frontier exploration is paused for ALC. The apply methods
// return true only when the call actually transitions the gate state, so
// duplicate alc_started / alc_finished events stay idempotent.
class AlcCoordinationGate
{
public:
    bool paused() const { return paused_; }
    bool applyPause();
    bool applyResume();
    bool apply(CoordinationAction action);

private:
    bool paused_ = false;
};

}  // namespace yahboom_alc_bringup
