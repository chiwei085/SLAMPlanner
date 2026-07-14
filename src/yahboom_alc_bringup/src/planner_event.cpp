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

std::optional<std::string> parseReason(const std::string& payload) {
    return parseJsonStringField(payload, "reason");
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
