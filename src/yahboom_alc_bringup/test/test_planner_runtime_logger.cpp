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

TEST(PlannerRuntimeLogger, RunStartRecordsAllObservedTopics) {
    const LoggerTopics topics{"/planner/events", "/map", "/alc/nav_cmd_vel",
                              "/cmd_vel", "/navigate_to_pose/_action/status"};

    const auto record = runStartJson(12.3456, 2.0, topics);

    EXPECT_EQ(record[std::string("type")].dump(), "\"run_start\"");
    EXPECT_DOUBLE_EQ(record[std::string("time_sec")], 12.346);
    EXPECT_DOUBLE_EQ(record[std::string("snapshot_period_sec")], 2.0);
    EXPECT_EQ(record[std::string("event_topic")].dump(), "\"/planner/events\"");
    EXPECT_EQ(record[std::string("map_topic")].dump(), "\"/map\"");
    EXPECT_EQ(record[std::string("nav_cmd_topic")].dump(),
              "\"/alc/nav_cmd_vel\"");
    EXPECT_EQ(record[std::string("driver_cmd_topic")].dump(), "\"/cmd_vel\"");
    EXPECT_EQ(record[std::string("navigate_status_topic")].dump(),
              "\"/navigate_to_pose/_action/status\"");
}

TEST(PlannerRuntimeLogger,
     SourcesReportUnseenInputsWithoutChangingNullPayloads) {
    const auto sources = sourcesJson({}, {}, {}, {}, {});
    const nlohmann::json snapshot = {
        {"type", "snapshot"},
        {"map", nullptr},
        {"commands",
         {{"nav", {{"twist", nullptr}, {"age_sec", nullptr}}},
          {"driver", {{"twist", nullptr}, {"age_sec", nullptr}}}}},
        {"sources", sources}};

    EXPECT_EQ(snapshot[std::string("map")].dump(), "null");
    EXPECT_EQ(snapshot[std::string("commands")][std::string("nav")]
                      [std::string("twist")]
                          .dump(),
              "null");
    EXPECT_EQ(snapshot[std::string("sources")][std::string("map")]
                      [std::string("seen")]
                          .dump(),
              "false");
    EXPECT_EQ(snapshot[std::string("sources")][std::string("map")]
                      [std::string("message_count")],
              0);
    EXPECT_EQ(snapshot[std::string("sources")][std::string("nav_cmd")]
                      [std::string("age_sec")]
                          .dump(),
              "null");
}

TEST(PlannerRuntimeLogger, SourcesReportReceivedMapAndTwistInputs) {
    const auto sources =
        sourcesJson({}, SourceHealth{2, 0.125}, SourceHealth{3, 1.2345},
                    SourceHealth{4, 2.0}, {});

    EXPECT_EQ(sources[std::string("map")][std::string("seen")].dump(), "true");
    EXPECT_EQ(sources[std::string("map")][std::string("message_count")], 2);
    EXPECT_DOUBLE_EQ(sources[std::string("map")][std::string("age_sec")],
                     0.125);
    EXPECT_EQ(sources[std::string("nav_cmd")][std::string("seen")].dump(),
              "true");
    EXPECT_EQ(sources[std::string("nav_cmd")][std::string("message_count")], 3);
    EXPECT_DOUBLE_EQ(sources[std::string("nav_cmd")][std::string("age_sec")],
                     1.235);
    EXPECT_EQ(sources[std::string("driver_cmd")][std::string("seen")].dump(),
              "true");
    EXPECT_EQ(
        sources[std::string("navigate_status")][std::string("age_sec")].dump(),
        "null");
}

}  // namespace
}  // namespace yahboom_alc_bringup
