#include "yahboom_alc_bringup/planner_event.hpp"

#include <nlohmann/json.hpp>

namespace yahboom_alc_bringup
{

CoordinationAction classifyPlannerEvent(const std::string& payload) {
    // Subscribers on the event topic see every event, including their own;
    // skip the JSON parse unless the payload can name an ALC transition.
    if (payload.find("alc_") == std::string::npos) {
        return CoordinationAction::kNone;
    }
    nlohmann::json parsed;
    try {
        parsed = nlohmann::json::parse(payload);
    }
    catch (const nlohmann::json::parse_error&) {
        return CoordinationAction::kNone;
    }
    if (!parsed.contains("event")) {
        return CoordinationAction::kNone;
    }
    const auto event = parsed[std::string("event")].get<std::string>();
    if (event == "alc_started") {
        return CoordinationAction::kPause;
    }
    if (event == "alc_finished") {
        return CoordinationAction::kResume;
    }
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
