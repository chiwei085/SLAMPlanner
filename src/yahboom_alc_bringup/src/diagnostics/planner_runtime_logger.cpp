#include "yahboom_alc_bringup/planner_runtime_logger.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <ctime>
#include <format>
#include <stdexcept>
#include <utility>

#include "yahboom_alc_bringup/math_util.hpp"

namespace yahboom_alc_bringup
{

std::optional<CommandConflictWatch::Previous> CommandConflictWatch::observe(
    const std::string& source, const double linear_x, const double now_sec) {
    constexpr double kMaxGapSec = 0.2;
    constexpr double kReportPeriodSec = 1.0;
    std::optional<Previous> result;
    const double gap = now_sec - time_sec_;
    if (source != source_ && linear_x * linear_x_ < 0.0 && gap <= kMaxGapSec &&
        now_sec - last_report_sec_ >= kReportPeriodSec) {
        result = Previous{source_, linear_x_, gap};
        last_report_sec_ = now_sec;
    }
    // A zero carries no direction, so it does not hide a reversal.
    if (linear_x != 0.0) {
        source_ = source;
        linear_x_ = linear_x;
        time_sec_ = now_sec;
    }
    return result;
}

double pathLengthM(const nav_msgs::msg::Path& path) {
    double length = 0.0;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
        const auto& a = path.poses[i - 1].pose.position;
        const auto& b = path.poses[i].pose.position;
        length += std::hypot(b.x - a.x, b.y - a.y);
    }
    return length;
}

MapCellCounts countMapCells(const nav_msgs::msg::OccupancyGrid& msg,
                            const int occupied_threshold) {
    MapCellCounts counts;
    for (const auto value : msg.data) {
        if (value == -1) {
            ++counts.unknown;
        }
        else if (0 <= value && value < occupied_threshold) {
            ++counts.free;
        }
        else {
            ++counts.occupied;
        }
    }
    counts.width = static_cast<int>(msg.info.width);
    counts.height = static_cast<int>(msg.info.height);
    counts.resolution = msg.info.resolution;
    return counts;
}

TwistSummary summarizeTwist(const geometry_msgs::msg::Twist& msg) {
    return {msg.linear.x, msg.linear.y, msg.angular.z};
}

nlohmann::json mapCountsJson(const MapCellCounts& counts) {
    return nlohmann::json{
        {"free", counts.free},         {"unknown", counts.unknown},
        {"occupied", counts.occupied}, {"width", counts.width},
        {"height", counts.height},     {"resolution", counts.resolution}};
}

nlohmann::json twistSummaryJson(const std::optional<TwistSummary>& twist) {
    if (!twist) {
        return nullptr;
    }
    return twistSummaryJson(*twist);
}

nlohmann::json twistSummaryJson(const TwistSummary& twist) {
    return nlohmann::json{
        {"linear_x", rounded(twist.linear_x, DecimalPlaces{4})},
        {"linear_y", rounded(twist.linear_y, DecimalPlaces{4})},
        {"angular_z", rounded(twist.angular_z, DecimalPlaces{4})}};
}

nlohmann::json sourceHealthJson(const SourceHealth& health) {
    return {{"seen", health.message_count > 0},
            {"message_count", health.message_count},
            {"receive_silence_sec",
             health.receive_silence_sec
                 ? nlohmann::json(rounded(*health.receive_silence_sec))
                 : nlohmann::json(nullptr)}};
}

nlohmann::json sourcesJson(const SourceHealth& event, const SourceHealth& map,
                           const SourceHealth& nav_cmd,
                           const SourceHealth& driver_cmd,
                           const SourceHealth& navigate_status) {
    return {{"event", sourceHealthJson(event)},
            {"map", sourceHealthJson(map)},
            {"nav_cmd", sourceHealthJson(nav_cmd)},
            {"driver_cmd", sourceHealthJson(driver_cmd)},
            {"navigate_status", sourceHealthJson(navigate_status)}};
}

nlohmann::json runStartJson(const double time_sec,
                            const double snapshot_period_sec,
                            const LoggerTopics& topics,
                            const nlohmann::json& runtime_metadata) {
    auto record =
        nlohmann::json{{"type", "run_start"},
                       {"time_sec", rounded(time_sec)},
                       {"snapshot_period_sec", snapshot_period_sec},
                       {"event_topic", topics.event_topic},
                       {"map_topic", topics.map_topic},
                       {"nav_cmd_topic", topics.nav_cmd_topic},
                       {"driver_cmd_topic", topics.driver_cmd_topic},
                       {"navigate_status_topic", topics.navigate_status_topic},
                       {"global_costmap_topic", topics.global_costmap_topic},
                       {"wheel_odom_topic", topics.wheel_odom_topic}};
    if (!runtime_metadata.empty()) {
        record[std::string("runtime_metadata")] = runtime_metadata;
    }
    return record;
}

nlohmann::json parseObjectOrRaw(const std::string& text) {
    auto parsed = nlohmann::json::parse(text, nullptr, false);
    if (parsed.is_object()) {
        return parsed;
    }
    return {{"raw", text.substr(0, 500)}};
}

std::string makeRunLogName() {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&time, &tm);
    return std::format("run_{:04}{:02}{:02}_{:02}{:02}{:02}.jsonl",
                       tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                       tm.tm_min, tm.tm_sec);
}

PlannerLogWriter::PlannerLogWriter(const std::filesystem::path& log_dir,
                                   std::string run_name) {
    std::filesystem::create_directories(log_dir);
    path_ = log_dir / std::move(run_name);
    file_.open(path_, std::ios::out | std::ios::app);
    if (!file_) {
        throw std::runtime_error("failed to open planner log: " +
                                 path_.string());
    }
}

PlannerLogWriter::~PlannerLogWriter() {
    close();
}

void PlannerLogWriter::write(const nlohmann::json& record) {
    // Raw payloads may hold invalid UTF-8, on which a strict dump throws.
    file_ << record.dump(-1, ' ', false,
                         nlohmann::json::error_handler_t::replace)
          << '\n';
    file_.flush();
}

void PlannerLogWriter::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

}  // namespace yahboom_alc_bringup
