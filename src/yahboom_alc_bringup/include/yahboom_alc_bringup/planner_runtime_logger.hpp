#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#include "yahboom_alc_bringup/math_util.hpp"

namespace yahboom_alc_bringup
{

struct MapCellCounts
{
    int free = 0;
    int unknown = 0;
    int occupied = 0;
    int width = 0;
    int height = 0;
    double resolution = 0.0;
};

struct TwistSummary
{
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;
};

struct LoggerTopics
{
    std::string event_topic;
    std::string map_topic;
    std::string nav_cmd_topic;
    std::string driver_cmd_topic;
    std::string navigate_status_topic;
};

struct SourceHealth
{
    int message_count = 0;
    std::optional<double> age_sec;
};

MapCellCounts countMapCells(const nav_msgs::msg::OccupancyGrid& msg,
                            int occupied_threshold = 50);
TwistSummary summarizeTwist(const geometry_msgs::msg::Twist& msg);
nlohmann::json mapCountsJson(const MapCellCounts& counts);
nlohmann::json twistSummaryJson(const std::optional<TwistSummary>& twist);
nlohmann::json twistSummaryJson(const TwistSummary& twist);
nlohmann::json sourceHealthJson(const SourceHealth& health);
nlohmann::json sourcesJson(const SourceHealth& event, const SourceHealth& map,
                           const SourceHealth& nav_cmd,
                           const SourceHealth& driver_cmd,
                           const SourceHealth& navigate_status);
nlohmann::json runStartJson(double time_sec, double snapshot_period_sec,
                            const LoggerTopics& topics,
                            const nlohmann::json& runtime_metadata = {});
std::string makeRunLogName();

class PlannerLogWriter
{
public:
    PlannerLogWriter(const std::filesystem::path& log_dir,
                     std::string run_name = makeRunLogName());
    ~PlannerLogWriter();

    void write(const nlohmann::json& record);
    void close();
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
    std::ofstream file_;
};

}  // namespace yahboom_alc_bringup
