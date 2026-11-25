#include "yahboom_alc_bringup/planner_event.hpp"

#include <nlohmann/json.hpp>

#include <optional>
#include <string>

namespace yahboom_alc_bringup
{

namespace
{
std::optional<std::string> parseJsonStringField(const std::string& payload,
                                                const char* key) {
    try {
        const auto parsed = nlohmann::json::parse(payload);
        if (parsed.contains(key) && parsed[key].is_string()) {
            return parsed[key].get<std::string>();
        }
    }
    catch (const nlohmann::json::parse_error&) {
    }
    return std::nullopt;
}
}  // namespace

std::optional<std::string> parseEventName(const std::string& payload) {
    return parseJsonStringField(payload, "event");
}

std::optional<std::string> parseFailureClass(const std::string& payload) {
    return parseJsonStringField(payload, "failure_class");
}

ParsedPlannerEvent parsePlannerEvent(const std::string& payload) {
    ParsedPlannerEvent parsed;
    nlohmann::json json;
    try {
        json = nlohmann::json::parse(payload);
    }
    catch (const nlohmann::json::parse_error&) {
        return parsed;
    }
    const auto field = [&json](const char* key) {
        return json.contains(key) && json[key].is_string()
                   ? json[key].get<std::string>()
                   : std::string{};
    };
    const auto event = field("event");
    parsed.reason = field("reason");
    if (event == "reverse_blocked") {
        // Everything but these (rear_blocked, out_of_bounds) is a verdict
        // from a fresh costmap.
        parsed.observation = parsed.reason == "stale_costmap" ||
                                     parsed.reason == "tf_unavailable"
                                 ? PlannerObservation::kReverseUnverified
                                 : PlannerObservation::kReverseBlocked;
    }
    else if (event == "stall_or_slip") {
        parsed.observation = PlannerObservation::kForwardStall;
    }
    else if (event == "alc_started") {
        parsed.observation = PlannerObservation::kAlcStarted;
    }
    else if (event == "alc_finished") {
        parsed.observation = PlannerObservation::kAlcFinished;
    }
    return parsed;
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
