#include "yahboom_alc_bringup/planner_event.hpp"

#include <nlohmann/json.hpp>

#include <exception>

namespace yahboom_alc_bringup
{

CoordinationAction classifyPlannerEvent(const std::string& payload) {
    nlohmann::json parsed;
    try {
        parsed = nlohmann::json::parse(payload);
    }
    // The bundled JSON shim can surface std::stod failures and similar
    // standard exceptions for malformed payloads, not only parse_error.
    catch (const std::exception&) {
        return CoordinationAction::kNone;
    }
    if (!parsed.contains("event")) {
        return CoordinationAction::kNone;
    }
    const std::string event = parsed[std::string("event")].dump();
    if (event == "\"alc_started\"") {
        return CoordinationAction::kPause;
    }
    if (event == "\"alc_finished\"") {
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

bool AlcCoordinationGate::apply(const CoordinationAction action) {
    switch (action) {
        case CoordinationAction::kPause:
            return applyPause();
        case CoordinationAction::kResume:
            return applyResume();
        case CoordinationAction::kNone:
            break;
    }
    return false;
}

}  // namespace yahboom_alc_bringup
