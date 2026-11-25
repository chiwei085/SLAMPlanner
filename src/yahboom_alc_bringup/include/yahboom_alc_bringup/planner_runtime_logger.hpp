#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json_fwd.hpp>

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

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
    std::string global_costmap_topic;
    std::string wheel_odom_topic;
};

struct SourceHealth
{
    int message_count = 0;
    std::optional<double> receive_silence_sec;
};

// Two publishers pulling one velocity topic in opposite directions: the bridge
// turns that into a driver command that flips sign and the car stands still.
class CommandConflictWatch
{
public:
    struct Previous
    {
        std::string source;
        double linear_x = 0.0;
        double gap_sec = 0.0;
    };

    // The previous command when this one, from another source, reverses it
    // within max_gap_sec. At most one report per second.
    std::optional<Previous> observe(const std::string& source, double linear_x,
                                    double now_sec);

private:
    std::string source_;
    double linear_x_ = 0.0;
    double time_sec_ = 0.0;
    double last_report_sec_ = -1.0e9;
};

double pathLengthM(const nav_msgs::msg::Path& path);
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
                            const nlohmann::json& runtime_metadata);
// The parsed object, or {"raw": text} when text is not a JSON object.
nlohmann::json parseObjectOrRaw(const std::string& text);
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
