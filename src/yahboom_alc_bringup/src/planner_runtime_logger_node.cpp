#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>

#include "yahboom_alc_bringup/planner_runtime_logger.hpp"

namespace yahboom_alc_bringup
{

namespace
{

nlohmann::json ageJson(const std::optional<rclcpp::Time>& stamp,
                       const rclcpp::Time& now) {
    if (!stamp) {
        return nullptr;
    }
    return rounded((now - *stamp).seconds());
}

}  // namespace

class PlannerRuntimeLoggerNode : public rclcpp::Node
{
public:
    explicit PlannerRuntimeLoggerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
        : rclcpp::Node("planner_runtime_logger", options),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_) {
        declare_parameter("log_dir", "planner-log");
        declare_parameter("snapshot_period_sec", 2.0);
        declare_parameter("event_topic", "/planner/events");
        declare_parameter("map_topic", "/map");
        declare_parameter("nav_cmd_topic", "/alc/nav_cmd_vel");
        declare_parameter("driver_cmd_topic", "/cmd_vel");
        declare_parameter("global_frame", "map");
        declare_parameter("robot_frame", "base_footprint");
        declare_parameter("occupied_threshold", 50);

        global_frame_ = get_parameter("global_frame").as_string();
        robot_frame_ = get_parameter("robot_frame").as_string();
        occupied_threshold_ = get_parameter("occupied_threshold").as_int();
        writer_ = std::make_unique<PlannerLogWriter>(
            get_parameter("log_dir").as_string());

        create_subscription<std_msgs::msg::String>(
            get_parameter("event_topic").as_string(), 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                onEvent(*msg);
            });
        create_subscription<nav_msgs::msg::OccupancyGrid>(
            get_parameter("map_topic").as_string(), 10,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                latest_map_ = *msg;
                map_time_ = get_clock()->now();
            });
        create_subscription<geometry_msgs::msg::Twist>(
            get_parameter("nav_cmd_topic").as_string(), 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                latest_nav_cmd_ = summarizeTwist(*msg);
                nav_cmd_time_ = get_clock()->now();
            });
        create_subscription<geometry_msgs::msg::Twist>(
            get_parameter("driver_cmd_topic").as_string(), 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                latest_driver_cmd_ = summarizeTwist(*msg);
                driver_cmd_time_ = get_clock()->now();
            });
        create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status", 10,
            [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
                latest_status_ = *msg;
                status_time_ = get_clock()->now();
            });

        timer_ = create_wall_timer(
            std::chrono::duration<double>(
                get_parameter("snapshot_period_sec").as_double()),
            [this]() { writeSnapshot(); });
        RCLCPP_INFO(get_logger(), "Planner runtime log: %s",
                    writer_->path().c_str());
    }

private:
    void onEvent(const std_msgs::msg::String& msg) {
        nlohmann::json payload;
        try {
            payload = nlohmann::json::parse(msg.data);
        }
        catch (const nlohmann::json::parse_error&) {
            payload = nlohmann::json{{"raw", msg.data.substr(0, 500)}};
        }
        writer_->write({{"type", "planner_event"},
                        {"received_sec", rounded(nowSec())},
                        {"payload", payload}});
    }

    void writeSnapshot() { writer_->write(buildSnapshot()); }

    nlohmann::json buildSnapshot() {
        const auto now = get_clock()->now();
        return {{"type", "snapshot"},
                {"time_sec", rounded(nowSec())},
                {"map", mapSummary(now)},
                {"commands",
                 {{"nav",
                   {{"twist", twistSummaryJson(latest_nav_cmd_)},
                    {"age_sec", ageJson(nav_cmd_time_, now)}}},
                  {"driver",
                   {{"twist", twistSummaryJson(latest_driver_cmd_)},
                    {"age_sec", ageJson(driver_cmd_time_, now)}}}}},
                {"navigate_to_pose", statusSummary(now)},
                {"tf", tfSummary()}};
    }

    nlohmann::json mapSummary(const rclcpp::Time& now) const {
        if (!latest_map_) {
            return nullptr;
        }
        auto summary =
            mapCountsJson(countMapCells(*latest_map_, occupied_threshold_));
        summary[std::string("age_sec")] = ageJson(map_time_, now);
        return summary;
    }

    nlohmann::json statusSummary(const rclcpp::Time& now) const {
        if (!latest_status_) {
            return {{"count", 0},
                    {"statuses", nlohmann::json::array()},
                    {"age_sec", nullptr}};
        }
        nlohmann::json statuses = nlohmann::json::array();
        const auto& list = latest_status_->status_list;
        const std::size_t first = list.size() > 5 ? list.size() - 5 : 0;
        for (std::size_t i = first; i < list.size(); ++i) {
            statuses.push_back(static_cast<int>(list[i].status));
        }
        return {{"count", list.size()},
                {"statuses", statuses},
                {"age_sec", ageJson(status_time_, now)}};
    }

    nlohmann::json tfSummary() {
        try {
            const auto transform = tf_buffer_.lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero);
            return {{"ok", true},
                    {"x", rounded(transform.transform.translation.x)},
                    {"y", rounded(transform.transform.translation.y)}};
        }
        catch (const tf2::TransformException& exc) {
            std::string error = exc.what();
            if (error.size() > 160) {
                error.resize(160);
            }
            return {{"ok", false}, {"error", error}};
        }
    }

    double nowSec() { return get_clock()->now().seconds(); }

    std::unique_ptr<PlannerLogWriter> writer_;
    std::string global_frame_;
    std::string robot_frame_;
    int occupied_threshold_ = 50;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::optional<nav_msgs::msg::OccupancyGrid> latest_map_;
    std::optional<TwistSummary> latest_nav_cmd_;
    std::optional<TwistSummary> latest_driver_cmd_;
    std::optional<action_msgs::msg::GoalStatusArray> latest_status_;
    std::optional<rclcpp::Time> map_time_;
    std::optional<rclcpp::Time> nav_cmd_time_;
    std::optional<rclcpp::Time> driver_cmd_time_;
    std::optional<rclcpp::Time> status_time_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace yahboom_alc_bringup

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<yahboom_alc_bringup::PlannerRuntimeLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
