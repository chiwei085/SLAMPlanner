#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

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

struct RobotPose
{
    double x;
    double y;
    double yaw;
};

struct TFResult
{
    std::optional<RobotPose> pose;
    std::string error;
};

const char* navStatusName(int8_t status) {
    switch (status) {
        case 1:
            return "ACCEPTED";
        case 2:
            return "EXECUTING";
        case 3:
            return "CANCELING";
        case 4:
            return "SUCCEEDED";
        case 5:
            return "CANCELED";
        case 6:
            return "ABORTED";
        default:
            return "UNKNOWN";
    }
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

        writer_->write(
            {{"type", "run_start"},
             {"time_sec", rounded(nowSec())},
             {"snapshot_period_sec",
              get_parameter("snapshot_period_sec").as_double()},
             {"nav_cmd_topic", get_parameter("nav_cmd_topic").as_string()},
             {"driver_cmd_topic",
              get_parameter("driver_cmd_topic").as_string()}});

        create_subscription<std_msgs::msg::String>(
            get_parameter("event_topic").as_string(), 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                onEvent(*msg);
            });
        rclcpp::QoS map_qos{rclcpp::KeepLast(1)};
        map_qos.reliable().transient_local();
        create_subscription<nav_msgs::msg::OccupancyGrid>(
            get_parameter("map_topic").as_string(), map_qos,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                latest_map_ = *msg;
                map_time_ = get_clock()->now();
            });
        rclcpp::QoS cmd_qos{rclcpp::KeepLast(10)};
        cmd_qos.best_effort();
        create_subscription<geometry_msgs::msg::Twist>(
            get_parameter("nav_cmd_topic").as_string(), cmd_qos,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                latest_nav_cmd_ = summarizeTwist(*msg);
                nav_cmd_time_ = get_clock()->now();
            });
        create_subscription<geometry_msgs::msg::Twist>(
            get_parameter("driver_cmd_topic").as_string(), cmd_qos,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                latest_driver_cmd_ = summarizeTwist(*msg);
                driver_cmd_time_ = get_clock()->now();
            });
        create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status", 10,
            [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
                detectStatusChanges(*msg);
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
    TFResult lookupTF() {
        try {
            const auto transform = tf_buffer_.lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero);
            const auto& t = transform.transform.translation;
            const auto& r = transform.transform.rotation;
            const double yaw = std::atan2(2.0 * (r.w * r.z + r.x * r.y),
                                          1.0 - 2.0 * (r.y * r.y + r.z * r.z));
            return {{RobotPose{rounded(t.x), rounded(t.y), rounded(yaw)}}, {}};
        }
        catch (const tf2::TransformException& exc) {
            std::string error = exc.what();
            if (error.size() > 160) {
                error.resize(160);
            }
            return {std::nullopt, std::move(error)};
        }
    }

    void onEvent(const std_msgs::msg::String& msg) {
        nlohmann::json payload;
        try {
            payload = nlohmann::json::parse(msg.data);
        }
        catch (const nlohmann::json::parse_error&) {
            payload = nlohmann::json{{"raw", msg.data.substr(0, 500)}};
        }
        const auto tf = lookupTF();
        if (tf.pose) {
            payload[std::string("robot_xy")] = {{"x", tf.pose->x},
                                                {"y", tf.pose->y}};
            payload[std::string("robot_yaw")] = tf.pose->yaw;
        }
        writer_->write({{"type", "planner_event"},
                        {"received_sec", rounded(nowSec())},
                        {"payload", payload}});
    }

    void writeSnapshot() {
        const auto tf = lookupTF();
        if (tf.pose) {
            if (last_robot_xy_) {
                const double dx = tf.pose->x - last_robot_xy_->first;
                const double dy = tf.pose->y - last_robot_xy_->second;
                total_distance_m_ += std::hypot(dx, dy);
            }
            last_robot_xy_ = {tf.pose->x, tf.pose->y};
        }
        writer_->write(buildSnapshot(tf));
    }

    nlohmann::json buildSnapshot(const TFResult& tf) {
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
                {"tf", tfToJson(tf)},
                {"total_distance_m", rounded(total_distance_m_)}};
    }

    nlohmann::json mapSummary(const rclcpp::Time& now) {
        if (!latest_map_) {
            return nullptr;
        }
        const auto counts = countMapCells(*latest_map_, occupied_threshold_);
        auto summary = mapCountsJson(counts);
        summary[std::string("age_sec")] = ageJson(map_time_, now);
        if (last_free_cells_) {
            summary[std::string("free_delta")] =
                counts.free - *last_free_cells_;
        }
        last_free_cells_ = counts.free;
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

    nlohmann::json tfToJson(const TFResult& tf) const {
        if (!tf.pose) {
            return {{"ok", false}, {"error", tf.error}};
        }
        return {{"ok", true},
                {"x", tf.pose->x},
                {"y", tf.pose->y},
                {"yaw", tf.pose->yaw}};
    }

    void detectStatusChanges(const action_msgs::msg::GoalStatusArray& msg) {
        for (const auto& gs : msg.status_list) {
            const auto& uuid = gs.goal_info.goal_id.uuid;
            std::array<uint8_t, 16> key{};
            std::copy(uuid.begin(), uuid.end(), key.begin());

            const auto it = known_nav_statuses_.find(key);
            if (it == known_nav_statuses_.end()) {
                known_nav_statuses_[key] = gs.status;
                continue;
            }
            if (it->second == gs.status) {
                continue;
            }

            const auto tf = lookupTF();
            nlohmann::json record = {
                {"type", "nav_status_change"},
                {"time_sec", rounded(nowSec())},
                {"from_status", static_cast<int>(it->second)},
                {"from_status_name", navStatusName(it->second)},
                {"to_status", static_cast<int>(gs.status)},
                {"to_status_name", navStatusName(gs.status)}};
            if (tf.pose) {
                record[std::string("robot_xy")] = {{"x", tf.pose->x},
                                                   {"y", tf.pose->y}};
                record[std::string("robot_yaw")] = tf.pose->yaw;
            }
            writer_->write(record);
            it->second = gs.status;
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

    std::optional<int> last_free_cells_;
    double total_distance_m_ = 0.0;
    std::optional<std::pair<double, double>> last_robot_xy_;
    std::map<std::array<uint8_t, 16>, int8_t> known_nav_statuses_;
};

}  // namespace yahboom_alc_bringup

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<yahboom_alc_bringup::PlannerRuntimeLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
