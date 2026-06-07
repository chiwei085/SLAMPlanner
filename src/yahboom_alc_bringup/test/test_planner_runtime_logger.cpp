#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "yahboom_alc_bringup/planner_runtime_logger.hpp"

namespace yahboom_alc_bringup
{
namespace
{

nav_msgs::msg::OccupancyGrid buildGrid() {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = 4;
    grid.info.height = 3;
    grid.info.resolution = 0.25;
    grid.data.assign(12, 0);
    grid.data[0] = -1;
    grid.data[1] = 100;
    return grid;
}

TEST(PlannerRuntimeLogger, LogWriterCreatesJsonlAndWritesEvent) {
    const auto dir =
        std::filesystem::temp_directory_path() / "planner_runtime_logger_test";
    std::filesystem::remove_all(dir);
    PlannerLogWriter writer(dir, "run_20260607_103000.jsonl");
    writer.write(
        {{"type", "planner_event"}, {"payload", {{"event", "goal_sent"}}}});
    writer.close();

    const auto path = dir / "run_20260607_103000.jsonl";
    ASSERT_TRUE(std::filesystem::exists(path));
    std::ifstream file(path);
    std::string line;
    std::getline(file, line);
    EXPECT_NE(line.find("\"type\":\"planner_event\""), std::string::npos);
    EXPECT_NE(line.find("\"goal_sent\""), std::string::npos);
    std::filesystem::remove_all(dir);
}

TEST(PlannerRuntimeLogger, MapAndTwistHelpersAreCompact) {
    const auto counts = countMapCells(buildGrid());
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.123456;
    twist.linear.y = -0.2;
    twist.angular.z = 0.987654;
    const auto command = summarizeTwist(twist);

    EXPECT_EQ(counts.free, 10);
    EXPECT_EQ(counts.unknown, 1);
    EXPECT_EQ(counts.occupied, 1);
    EXPECT_EQ(counts.width, 4);
    EXPECT_EQ(counts.height, 3);
    EXPECT_DOUBLE_EQ(counts.resolution, 0.25);
    EXPECT_DOUBLE_EQ(command.linear_x, 0.123456);
    const auto map_json = mapCountsJson(counts);
    const auto twist_json = twistSummaryJson(command);
    EXPECT_FALSE(map_json.contains("data"));
    EXPECT_DOUBLE_EQ(twist_json[std::string("linear_x")], 0.1235);
    EXPECT_DOUBLE_EQ(twist_json[std::string("linear_y")], -0.2);
    EXPECT_DOUBLE_EQ(twist_json[std::string("angular_z")], 0.9877);
}

}  // namespace
}  // namespace yahboom_alc_bringup
