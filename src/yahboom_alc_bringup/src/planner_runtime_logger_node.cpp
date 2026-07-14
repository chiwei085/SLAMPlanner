#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "yahboom_alc_bringup/parameter_loader.hpp"
#include "yahboom_alc_bringup/planner_runtime_logger.hpp"
#include "yahboom_alc_bringup/temporal_signal.hpp"
#include "yahboom_alc_bringup/yaw_util.hpp"

namespace yahboom_alc_bringup
{

struct PlannerRuntimeLoggerParams
{
    std::string log_dir = "planner-log";
    double snapshot_period_sec = 2.0;
    std::string event_topic = "/planner/events";
    std::string map_topic = "/map";
    std::string global_costmap_topic = "/global_costmap/costmap";
    std::string wheel_odom_topic = "/odom";
    std::string nav_cmd_topic = "/alc/nav_cmd_vel";
    std::string driver_cmd_topic = "/cmd_vel";
    std::string navigate_status_topic = "/navigate_to_pose/_action/status";
    std::string global_frame = "map";
    std::string robot_frame = "base_footprint";
    int occupied_threshold = 50;
    std::string runtime_metadata_json = "{}";
    std::string nav2_params_file;
    std::string nav2_bt_xml;
};

}  // namespace yahboom_alc_bringup

namespace rik_sd
{

template <>
struct descriptor<yahboom_alc_bringup::PlannerRuntimeLoggerParams>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::PlannerRuntimeLoggerParams;
        return make_descriptor(
            named(&PlannerRuntimeLoggerParams::log_dir, "log_dir"),
            named(&PlannerRuntimeLoggerParams::snapshot_period_sec,
                  "snapshot_period_sec"),
            named(&PlannerRuntimeLoggerParams::event_topic, "event_topic"),
            named(&PlannerRuntimeLoggerParams::map_topic, "map_topic"),
            named(&PlannerRuntimeLoggerParams::global_costmap_topic,
                  "global_costmap_topic"),
            named(&PlannerRuntimeLoggerParams::wheel_odom_topic,
                  "wheel_odom_topic"),
            named(&PlannerRuntimeLoggerParams::nav_cmd_topic, "nav_cmd_topic"),
            named(&PlannerRuntimeLoggerParams::driver_cmd_topic,
                  "driver_cmd_topic"),
            named(&PlannerRuntimeLoggerParams::navigate_status_topic,
                  "navigate_status_topic"),
            named(&PlannerRuntimeLoggerParams::global_frame, "global_frame"),
            named(&PlannerRuntimeLoggerParams::robot_frame, "robot_frame"),
            named(&PlannerRuntimeLoggerParams::occupied_threshold,
                  "occupied_threshold"),
            named(&PlannerRuntimeLoggerParams::runtime_metadata_json,
                  "runtime_metadata_json"),
            named(&PlannerRuntimeLoggerParams::nav2_params_file,
                  "nav2_params_file"),
            named(&PlannerRuntimeLoggerParams::nav2_bt_xml, "nav2_bt_xml"));
    }
};

}  // namespace rik_sd

namespace yahboom_alc_bringup
{

namespace
{

nlohmann::json temporalJson(const TemporalSignalObservation& observation) {
    const auto jsonValue = [](const std::optional<double>& value) {
        return value ? nlohmann::json(rounded(*value))
                     : nlohmann::json(nullptr);
    };
    return {
        {"producer_lag_sec", jsonValue(observation.producer_lag_sec)},
        {"consume_gap_sec", jsonValue(observation.consume_gap_sec)},
        {"receive_silence_sec", jsonValue(observation.receive_silence_sec)}};
}

nlohmann::json nullableJson(const std::optional<double>& value) {
    return value ? nlohmann::json(rounded(*value)) : nlohmann::json(nullptr);
}

std::optional<double> receiveSilence(const std::optional<rclcpp::Time>& stamp,
                                     const rclcpp::Time& now) {
    return stamp ? std::optional<double>((now - *stamp).seconds())
                 : std::nullopt;
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
    std::optional<double> transform_stamp_sec;
    double query_sec = 0.0;
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
        const auto params =
            loadDeclaredParameters<PlannerRuntimeLoggerParams>(*this);
        global_frame_ = params.global_frame;
        robot_frame_ = params.robot_frame;
        occupied_threshold_ = params.occupied_threshold;
        writer_ = std::make_unique<PlannerLogWriter>(params.log_dir);

        nlohmann::json runtime_metadata = nlohmann::json::object();
        try {
            runtime_metadata =
                nlohmann::json::parse(params.runtime_metadata_json);
        }
        catch (const nlohmann::json::parse_error&) {
            runtime_metadata = {{"raw", params.runtime_metadata_json}};
        }
        runtime_metadata[std::string("nav2_params_file")] =
            params.nav2_params_file;
        runtime_metadata[std::string("nav2_bt_xml")] = params.nav2_bt_xml;
        writer_->write(runStartJson(
            nowSec(), params.snapshot_period_sec,
            {params.event_topic, params.map_topic, params.nav_cmd_topic,
             params.driver_cmd_topic, params.navigate_status_topic,
             params.global_costmap_topic, params.wheel_odom_topic},
            runtime_metadata));

        event_sub_ = create_subscription<std_msgs::msg::String>(
            params.event_topic, 10,
            [this](const std_msgs::msg::String& msg) { onEvent(msg); });
        rclcpp::QoS map_qos{rclcpp::KeepLast(1)};
        map_qos.reliable().transient_local();
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            params.map_topic, map_qos,
            [this](const nav_msgs::msg::OccupancyGrid& msg) {
                const double now = nowSec();
                latest_map_ = msg;
                map_time_ = get_clock()->now();
                map_temporal_ = observeTemporalSignal(
                    rclcpp::Time(msg.header.stamp).seconds(), now, map_state_);
                const auto counts = countMapCells(msg, occupied_threshold_);
                map_progressed_ =
                    last_map_counts_ &&
                    (counts.free != last_map_counts_->free ||
                     counts.unknown != last_map_counts_->unknown ||
                     counts.occupied != last_map_counts_->occupied);
                last_map_counts_ = counts;
                map_wheel_delta_m_ = wheel_distance_since_map_;
                wheel_distance_since_map_ = 0.0;
                ++map_message_count_;
            });
        global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            params.global_costmap_topic, map_qos,
            [this](const nav_msgs::msg::OccupancyGrid& msg) {
                latest_global_costmap_ = msg;
                global_costmap_temporal_ = observeTemporalSignal(
                    rclcpp::Time(msg.header.stamp).seconds(), nowSec(),
                    global_costmap_state_);
            });
        wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            params.wheel_odom_topic, rclcpp::SystemDefaultsQoS(),
            [this](const nav_msgs::msg::Odometry& msg) {
                const auto& point = msg.pose.pose.position;
                if (last_wheel_xy_) {
                    wheel_distance_since_map_ +=
                        std::hypot(point.x - last_wheel_xy_->first,
                                   point.y - last_wheel_xy_->second);
                }
                last_wheel_xy_ = {point.x, point.y};
            });
        rclcpp::QoS cmd_qos{rclcpp::KeepLast(10)};
        cmd_qos.best_effort();
        nav_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            params.nav_cmd_topic, cmd_qos,
            [this](const geometry_msgs::msg::Twist& msg) {
                latest_nav_cmd_ = summarizeTwist(msg);
                nav_cmd_time_ = get_clock()->now();
                ++nav_cmd_message_count_;
            });
        driver_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            params.driver_cmd_topic, cmd_qos,
            [this](const geometry_msgs::msg::Twist& msg) {
                latest_driver_cmd_ = summarizeTwist(msg);
                driver_cmd_time_ = get_clock()->now();
                ++driver_cmd_message_count_;
            });
        navigate_status_sub_ =
            create_subscription<action_msgs::msg::GoalStatusArray>(
                params.navigate_status_topic, 10,
                [this](const action_msgs::msg::GoalStatusArray& msg) {
                    detectStatusChanges(msg);
                    latest_status_ = msg;
                    status_time_ = get_clock()->now();
                    ++status_message_count_;
                });

        timer_ = create_wall_timer(
            std::chrono::duration<double>(params.snapshot_period_sec),
            [this]() { writeSnapshot(); });
        RCLCPP_INFO(get_logger(), "Planner runtime log: %s",
                    writer_->path().c_str());
    }

private:
    TFResult lookupTF() {
        const double query_sec = nowSec();
        try {
            const auto transform = tf_buffer_.lookupTransform(
                global_frame_, robot_frame_, tf2::TimePointZero,
                tf2::durationFromSec(0.05));
            const auto& t = transform.transform.translation;
            const double yaw = yawFromQuaternion(transform.transform.rotation);
            return {{RobotPose{rounded(t.x), rounded(t.y), rounded(yaw)}},
                    rclcpp::Time(transform.header.stamp).seconds(),
                    query_sec,
                    {}};
        }
        catch (const tf2::TransformException& exc) {
            std::string error = exc.what();
            if (error.size() > 160) {
                error.resize(160);
            }
            return {std::nullopt, std::nullopt, query_sec, std::move(error)};
        }
    }

    void onEvent(const std_msgs::msg::String& msg) {
        event_time_ = get_clock()->now();
        ++event_message_count_;
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
                {"global_costmap", globalCostmapSummary(now)},
                {"commands",
                 {{"nav",
                   {{"twist", twistSummaryJson(latest_nav_cmd_)},
                    {"receive_silence_sec",
                     nullableJson(receiveSilence(nav_cmd_time_, now))}}},
                  {"driver",
                   {{"twist", twistSummaryJson(latest_driver_cmd_)},
                    {"receive_silence_sec",
                     nullableJson(receiveSilence(driver_cmd_time_, now))}}}}},
                {"navigate_to_pose", statusSummary(now)},
                {"sources", sourceSummary(now)},
                {"tf", tfToJson(tf)},
                {"total_distance_m", rounded(total_distance_m_)}};
    }

    nlohmann::json mapSummary(const rclcpp::Time& now) {
        if (!latest_map_) {
            return nullptr;
        }
        const auto counts = countMapCells(*latest_map_, occupied_threshold_);
        auto summary = mapCountsJson(counts);
        auto temporal = map_temporal_;
        temporal.receive_silence_sec =
            temporalSignalAt(map_state_, now.seconds()).receive_silence_sec;
        summary[std::string("temporal")] = temporalJson(temporal);
        summary[std::string("estimate_progressed")] =
            map_progressed_ ? nlohmann::json(*map_progressed_)
                            : nlohmann::json(nullptr);
        summary[std::string("wheel_odom_delta_m")] =
            map_wheel_delta_m_ ? nlohmann::json(rounded(*map_wheel_delta_m_))
                               : nlohmann::json(nullptr);
        if (last_free_cells_) {
            summary[std::string("free_delta")] =
                counts.free - *last_free_cells_;
        }
        last_free_cells_ = counts.free;
        return summary;
    }

    nlohmann::json globalCostmapSummary(const rclcpp::Time& now) const {
        if (!latest_global_costmap_) {
            return nullptr;
        }
        auto temporal = global_costmap_temporal_;
        temporal.receive_silence_sec =
            temporalSignalAt(global_costmap_state_, now.seconds())
                .receive_silence_sec;
        auto summary = mapCountsJson(
            countMapCells(*latest_global_costmap_, occupied_threshold_));
        summary["temporal"] = temporalJson(temporal);
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
                {"receive_silence_sec",
                 nullableJson(receiveSilence(status_time_, now))}};
    }

    nlohmann::json sourceSummary(const rclcpp::Time& now) const {
        return sourcesJson(
            sourceHealth(event_message_count_, event_time_, now),
            sourceHealth(map_message_count_, map_time_, now),
            sourceHealth(nav_cmd_message_count_, nav_cmd_time_, now),
            sourceHealth(driver_cmd_message_count_, driver_cmd_time_, now),
            sourceHealth(status_message_count_, status_time_, now));
    }

    SourceHealth sourceHealth(const int message_count,
                              const std::optional<rclcpp::Time>& stamp,
                              const rclcpp::Time& now) const {
        return {message_count,
                stamp ? std::optional<double>((now - *stamp).seconds())
                      : std::nullopt};
    }

    nlohmann::json tfToJson(const TFResult& tf) const {
        if (!tf.pose) {
            return {{"query_ok", false},
                    {"query_sec", rounded(tf.query_sec)},
                    {"error", tf.error}};
        }
        // Judge progression against the PREVIOUS query's stamp; with no
        // previous sample the answer is unknown, not false.
        const bool has_previous_stamp = last_tf_stamp_sec_.has_value();
        const bool progressed = has_previous_stamp && tf.transform_stamp_sec &&
                                *tf.transform_stamp_sec > *last_tf_stamp_sec_;
        if (tf.transform_stamp_sec) {
            last_tf_stamp_sec_ = tf.transform_stamp_sec;
        }
        return {{"query_ok", true},
                {"query_sec", rounded(tf.query_sec)},
                {"transform_stamp_sec",
                 tf.transform_stamp_sec
                     ? nlohmann::json(rounded(*tf.transform_stamp_sec))
                     : nlohmann::json(nullptr)},
                {"transform_lag_sec",
                 tf.transform_stamp_sec
                     ? nlohmann::json(
                           rounded(tf.query_sec - *tf.transform_stamp_sec))
                     : nlohmann::json(nullptr)},
                {"transform_progressed",
                 has_previous_stamp && tf.transform_stamp_sec
                     ? nlohmann::json(progressed)
                     : nlohmann::json(nullptr)},
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
    mutable std::optional<double> last_tf_stamp_sec_;
    std::optional<nav_msgs::msg::OccupancyGrid> latest_map_;
    std::optional<nav_msgs::msg::OccupancyGrid> latest_global_costmap_;
    std::optional<TwistSummary> latest_nav_cmd_;
    std::optional<TwistSummary> latest_driver_cmd_;
    std::optional<action_msgs::msg::GoalStatusArray> latest_status_;
    std::optional<rclcpp::Time> map_time_;
    std::optional<rclcpp::Time> nav_cmd_time_;
    std::optional<rclcpp::Time> driver_cmd_time_;
    std::optional<rclcpp::Time> status_time_;
    std::optional<rclcpp::Time> event_time_;
    TemporalSignalState map_state_;
    TemporalSignalState global_costmap_state_;
    TemporalSignalObservation map_temporal_;
    TemporalSignalObservation global_costmap_temporal_;
    std::optional<MapCellCounts> last_map_counts_;
    std::optional<bool> map_progressed_;
    std::optional<double> map_wheel_delta_m_;
    std::optional<std::pair<double, double>> last_wheel_xy_;
    double wheel_distance_since_map_ = 0.0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
        global_costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr driver_cmd_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
        navigate_status_sub_;

    std::optional<int> last_free_cells_;
    double total_distance_m_ = 0.0;
    std::optional<std::pair<double, double>> last_robot_xy_;
    std::map<std::array<uint8_t, 16>, int8_t> known_nav_statuses_;
    int event_message_count_ = 0;
    int map_message_count_ = 0;
    int nav_cmd_message_count_ = 0;
    int driver_cmd_message_count_ = 0;
    int status_message_count_ = 0;
};

}  // namespace yahboom_alc_bringup

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(
            std::make_shared<yahboom_alc_bringup::PlannerRuntimeLoggerNode>());
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& exc) {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        std::cerr << "planner_runtime_logger failed: " << exc.what() << '\n';
        return 1;
    }
}
