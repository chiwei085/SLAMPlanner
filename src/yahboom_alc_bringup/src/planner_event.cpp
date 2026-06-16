#include "yahboom_alc_bringup/planner_event.hpp"

#include <nlohmann/json.hpp>

#include <optional>
#include <string>

namespace yahboom_alc_bringup
{

std::optional<std::string> parseEventName(const std::string& payload) {
    try {
        const auto parsed = nlohmann::json::parse(payload);
        if (parsed.contains("event") && parsed["event"].is_string()) {
            return parsed[std::string("event")].get<std::string>();
        }
    }
    catch (const nlohmann::json::parse_error&) {}
    return std::nullopt;
}

CoordinationAction classifyPlannerEvent(const std::string& payload) {
    // Subscribers on the event topic see every event, including their own;
    // skip the JSON parse unless the payload can name an ALC transition.
    if (payload.find("alc_") == std::string::npos) {
        return CoordinationAction::kNone;
    }
    const auto event = parseEventName(payload);
    if (!event) return CoordinationAction::kNone;
    if (*event == "alc_started") return CoordinationAction::kPause;
    if (*event == "alc_finished") return CoordinationAction::kResume;
    return CoordinationAction::kNone;
}

bool AlcCoordinationGate::applyPause() {
    if (paused_) {
        return false;
    }
    paused_ = true;
    return true;
}

bool AlcCoordinationGate::applyResume() {
    if (!paused_) {
        return false;
    }
    paused_ = false;
    return true;
}

}  // namespace yahboom_alc_bringup
