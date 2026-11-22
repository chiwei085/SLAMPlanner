#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace yahboom_alc_bringup
{

enum class CoordinationAction : std::uint8_t
{
    kNone,
    kPause,
    kResume,
};

// Extracts the "event" field from a JSON planner event payload.
// Returns nullopt for malformed JSON or payloads without an "event" field.
std::optional<std::string> parseEventName(const std::string& payload);

// Extracts the optional "failure_class" field from planner events.
std::optional<std::string> parseFailureClass(const std::string& payload);

// Extracts the optional "reason" field from planner events.
std::optional<std::string> parseReason(const std::string& payload);

// What an incoming planner event lets the explorer conclude, and no more.
enum class PlannerObservation : std::uint8_t
{
    kNone,
    // The bridge refused a reverse command because it could not see (stale
    // local costmap / no TF): space is unconfirmed, nothing was hit.
    kReverseUnverified,
    // The bridge refused a reverse command on a fresh costmap: this reverse
    // motion is restricted.
    kReverseBlocked,
    // Forward command without matching odometry progress. Not proof of a
    // collision or slip.
    kForwardStall,
    kAlcStarted,
    kAlcFinished,
};

struct ParsedPlannerEvent
{
    PlannerObservation observation = PlannerObservation::kNone;
    std::string reason;
};

// Parses an event payload once. Malformed and unrelated payloads map to
// kNone.
ParsedPlannerEvent parsePlannerEvent(const std::string& payload);

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

private:
    bool paused_ = false;
};

}  // namespace yahboom_alc_bringup
