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
