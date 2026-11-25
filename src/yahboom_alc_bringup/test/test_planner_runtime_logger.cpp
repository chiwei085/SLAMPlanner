#include <gtest/gtest.h>

#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>

#include "yahboom_alc_bringup/planner_runtime_logger.hpp"
#include "yahboom_alc_bringup/temporal_signal.hpp"

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

TEST(PlannerRuntimeLogger, RunLogNamePreservesLocalTimestampFormat) {
    const auto before = std::time(nullptr);
    const auto name = makeRunLogName();
    const auto after = std::time(nullptr);

    std::tm local_time{};
    const auto* suffix =
        strptime(name.c_str(), "run_%Y%m%d_%H%M%S.jsonl", &local_time);
    ASSERT_NE(suffix, nullptr);
    EXPECT_EQ(*suffix, '\0');
    EXPECT_EQ(name.size(), 25U);
    local_time.tm_isdst = -1;
    const auto timestamp = std::mktime(&local_time);
    EXPECT_GE(timestamp, before);
    EXPECT_LE(timestamp, after);
}

TEST(CommandConflictWatch, ReportsOppositeCommandsFromTwoSources) {
    CommandConflictWatch watch;
    EXPECT_FALSE(watch.observe("a", 0.07, 10.00));
    const auto conflict = watch.observe("b", -0.04, 10.02);
    ASSERT_TRUE(conflict);
    EXPECT_EQ(conflict->source, "a");
    EXPECT_DOUBLE_EQ(conflict->linear_x, 0.07);
    EXPECT_FALSE(watch.observe("a", 0.06, 10.05));  // one report per second
    EXPECT_TRUE(watch.observe("b", -0.04, 11.06) ||
                watch.observe("a", 0.06, 11.08));
}

TEST(CommandConflictWatch, OneSourceReversingIsNotAConflict) {
    CommandConflictWatch watch;
    EXPECT_FALSE(watch.observe("a", 0.08, 1.0));
    EXPECT_FALSE(watch.observe("a", -0.08, 1.1));
    EXPECT_FALSE(watch.observe("b", 0.08, 5.0));  // long after
}

TEST(PlannerRuntimeLogger, LogWriterAppendsCompleteJsonRecordsOnSeparateLines) {
    const auto dir =
        std::filesystem::temp_directory_path() / "planner_runtime_logger_test";
    std::filesystem::remove_all(dir);
    PlannerLogWriter writer(dir, "run_20260607_103000.jsonl");
    const nlohmann::json event = {
        {"type", "planner_event"},
        {"payload",
         {{"event", "goal_sent"}, {"reason", "line 1\n\"line 2\""}}}};
    writer.write(event);
    writer.close();
    const nlohmann::json next_event = {{"type", "run_end"}};
    PlannerLogWriter reopened(dir, "run_20260607_103000.jsonl");
    reopened.write(next_event);
    reopened.close();

    const auto path = dir / "run_20260607_103000.jsonl";
    ASSERT_TRUE(std::filesystem::exists(path));
    std::ifstream file(path);
    std::string line;
    ASSERT_TRUE(std::getline(file, line));
    EXPECT_EQ(nlohmann::json::parse(line).dump(), event.dump());
    ASSERT_TRUE(std::getline(file, line));
    EXPECT_EQ(nlohmann::json::parse(line).dump(), next_event.dump());
    EXPECT_FALSE(std::getline(file, line));
    std::filesystem::remove_all(dir);
}

TEST(PlannerRuntimeLogger, UnusableEventPayloadIsKeptAsRaw) {
    EXPECT_EQ(parseObjectOrRaw("{\"event\":\"\\u4e2d\"}")["event"], "中");
    for (const std::string text :
         {"", "not json", "null", "5", "\"text\"", "[1]"}) {
        SCOPED_TRACE(text);
        auto payload = parseObjectOrRaw(text);
        EXPECT_EQ(payload["raw"], text);
        payload["robot_yaw"] = 0.5;  // the logger adds fields afterwards
    }
}

TEST(PlannerRuntimeLogger, LogWriterSurvivesInvalidUtf8) {
    const auto dir =
        std::filesystem::temp_directory_path() / "planner_runtime_logger_utf8";
    std::filesystem::remove_all(dir);
    PlannerLogWriter writer(dir, "run.jsonl");
    const auto payload = parseObjectOrRaw("bad \xff byte");
    EXPECT_NO_THROW(writer.write(runStartJson(1.0, 2.0, {}, payload)));
    writer.close();
    std::ifstream file(dir / "run.jsonl");
    std::string line;
    ASSERT_TRUE(std::getline(file, line));
    EXPECT_TRUE(nlohmann::json::accept(line));
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

TEST(PlannerRuntimeLogger, SourcesReportUnseenInputs) {
    const auto sources = sourcesJson({}, {}, {}, {}, {});
    const nlohmann::json expected_health = {{"seen", false},
                                            {"message_count", 0},
                                            {"receive_silence_sec", nullptr}};

    for (const auto* name :
         {"event", "map", "nav_cmd", "driver_cmd", "navigate_status"}) {
        SCOPED_TRACE(name);
        ASSERT_TRUE(sources.contains(name));
        EXPECT_EQ(sources[name].dump(), expected_health.dump());
    }
}

TEST(PlannerRuntimeLogger, SourcesReportReceivedMapAndTwistInputs) {
    const auto sources =
        sourcesJson({}, SourceHealth{2, 0.125}, SourceHealth{3, 1.2345},
                    SourceHealth{4, 2.0}, {});

    EXPECT_EQ(sources[std::string("map")][std::string("seen")].dump(), "true");
    EXPECT_EQ(sources[std::string("map")][std::string("message_count")], 2);
    EXPECT_DOUBLE_EQ(
        sources[std::string("map")][std::string("receive_silence_sec")], 0.125);
    EXPECT_EQ(sources[std::string("nav_cmd")][std::string("seen")].dump(),
              "true");
    EXPECT_EQ(sources[std::string("nav_cmd")][std::string("message_count")], 3);
    EXPECT_DOUBLE_EQ(
        sources[std::string("nav_cmd")][std::string("receive_silence_sec")],
        1.235);
    EXPECT_EQ(sources[std::string("driver_cmd")][std::string("seen")].dump(),
              "true");
    EXPECT_EQ(sources[std::string("navigate_status")]
                     [std::string("receive_silence_sec")]
                         .dump(),
              "null");
}

TEST(PlannerRuntimeLogger, TemporalSignalSeparatesLagGapAndSilence) {
    TemporalSignalState state;
    const auto first = observeTemporalSignal(8.0, 10.0, state);
    ASSERT_TRUE(first.producer_lag_sec);
    EXPECT_DOUBLE_EQ(*first.producer_lag_sec, 2.0);
    EXPECT_FALSE(first.consume_gap_sec);
    const auto second = observeTemporalSignal(10.5, 12.0, state);
    ASSERT_TRUE(second.consume_gap_sec);
    EXPECT_DOUBLE_EQ(*second.consume_gap_sec, 2.0);
    const auto silent = temporalSignalAt(state, 16.0);
    ASSERT_TRUE(silent.receive_silence_sec);
    EXPECT_DOUBLE_EQ(*silent.receive_silence_sec, 4.0);
}

}  // namespace
}  // namespace yahboom_alc_bringup
