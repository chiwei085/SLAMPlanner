#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

namespace yahboom_alc_bringup
{

inline double rounded(double value, int precision = 3) {
    const double scale = std::pow(10.0, precision);
    return std::round(value * scale) / scale;
}

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

MapCellCounts countMapCells(const nav_msgs::msg::OccupancyGrid& msg,
                            int occupied_threshold = 50);
TwistSummary summarizeTwist(const geometry_msgs::msg::Twist& msg);
nlohmann::json mapCountsJson(const MapCellCounts& counts);
nlohmann::json twistSummaryJson(const std::optional<TwistSummary>& twist);
nlohmann::json twistSummaryJson(const TwistSummary& twist);
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
