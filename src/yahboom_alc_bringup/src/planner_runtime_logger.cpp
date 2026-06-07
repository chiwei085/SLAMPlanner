#include "yahboom_alc_bringup/planner_runtime_logger.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace yahboom_alc_bringup
{

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
    return nlohmann::json{{"linear_x", rounded(twist.linear_x, 4)},
                          {"linear_y", rounded(twist.linear_y, 4)},
                          {"angular_z", rounded(twist.angular_z, 4)}};
}

std::string makeRunLogName() {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&time, &tm);
    std::ostringstream out;
    out << "run_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".jsonl";
    return out.str();
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
    file_ << record.dump() << '\n';
    file_.flush();
}

void PlannerLogWriter::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

}  // namespace yahboom_alc_bringup
