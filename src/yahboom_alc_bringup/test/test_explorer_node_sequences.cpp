// Sequence tests for the explorer node's asynchronous action handling. The
// node runs unchanged against fake Nav2 action servers; the tests drive it
// only through ROS interfaces (TF, /map, action results, /planner/events).
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gtest/gtest.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <functional>
#include <optional>
#include <thread>

#include "explorer/cluster_frontier_explorer_node.hpp"

namespace yahboom_alc_bringup
{
namespace
{

using namespace std::chrono_literals;

template <typename ActionT>
struct FakeActionServer
{
    using Handle = rclcpp_action::ServerGoalHandle<ActionT>;

    FakeActionServer(const rclcpp::Node::SharedPtr& node,
                     const std::string& name) {
        server = rclcpp_action::create_server<ActionT>(
            node, name,
            [](const rclcpp_action::GoalUUID&,
               std::shared_ptr<const typename ActionT::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<Handle>& handle) {
                cancel_requested.push_back(handle);
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<Handle>& handle) {
                goals.push_back(handle);
            });
    }

    bool cancelRequested(const std::size_t i) const {
        return std::find(cancel_requested.begin(), cancel_requested.end(),
                         goals.at(i)) != cancel_requested.end();
    }
    void succeed(const std::size_t i) {
        goals.at(i)->succeed(std::make_shared<typename ActionT::Result>());
    }
    void abort(const std::size_t i) {
        goals.at(i)->abort(std::make_shared<typename ActionT::Result>());
    }

    typename rclcpp_action::Server<ActionT>::SharedPtr server;
    std::vector<std::shared_ptr<Handle>> goals;
    std::vector<std::shared_ptr<Handle>> cancel_requested;
};

std::string field(const nlohmann::json& event, const char* key) {
    return event.contains(key) ? event[key].get<std::string>() : "";
}

nav_msgs::msg::OccupancyGrid makeMap(const bool with_frontier) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.width = 40;
    grid.info.height = 20;
    grid.info.resolution = 0.1F;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(40U * 20U, 0);
    if (with_frontier) {
        for (int y = 0; y < 20; ++y) {
            for (int x = 25; x < 40; ++x) {
                grid.data[static_cast<std::size_t>(y * 40 + x)] = -1;
            }
        }
    }
    return grid;
}

class ExplorerSequenceTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override {
        fake_ = std::make_shared<rclcpp::Node>("explorer_test_fake_nav2");
        nav_ = std::make_unique<
            FakeActionServer<nav2_msgs::action::NavigateToPose>>(
            fake_, "/navigate_to_pose");
        spin_ = std::make_unique<FakeActionServer<nav2_msgs::action::Spin>>(
            fake_, "/spin");
        backup_ = std::make_unique<FakeActionServer<nav2_msgs::action::BackUp>>(
            fake_, "/backup");
        for (const char* name :
             {"planner_server", "controller_server", "bt_navigator",
              "behavior_server", "collision_monitor"}) {
            lifecycle_services_.push_back(
                fake_->create_service<lifecycle_msgs::srv::GetState>(
                    std::string("/") + name + "/get_state",
                    [](const std::shared_ptr<
                           lifecycle_msgs::srv::GetState::Request>,
                       std::shared_ptr<lifecycle_msgs::srv::GetState::Response>
                           response) {
                        response->current_state.id =
                            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
                    }));
        }
        rclcpp::QoS map_qos{rclcpp::KeepLast(1)};
        map_qos.reliable().transient_local();
        map_pub_ = fake_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos);
        event_pub_ = fake_->create_publisher<std_msgs::msg::String>(
            "/planner/events", 10);
        event_sub_ = fake_->create_subscription<std_msgs::msg::String>(
            "/planner/events", 100, [this](const std_msgs::msg::String& msg) {
                events_.push_back(nlohmann::json::parse(msg.data));
            });
        tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*fake_);
        map_ = makeMap(true);
    }

    void startExplorer(const std::vector<rclcpp::Parameter>& extra = {}) {
        std::vector<rclcpp::Parameter> params{
            {"cycle_period_sec", 0.1},
            {"nav2_validation_enabled", false},
            {"recovery_reobserve_cycles", 1},
            {"min_clearance_cells", 0},
            {"path_clearance_cells", 0},
            {"goal_publish_period", 0.0},
            {"backtrace_cancel_timeout_sec", 0.2},
            {"backtrace_backup_stall_check_sec", 0.0},
        };
        params.insert(params.end(), extra.begin(), extra.end());
        rclcpp::NodeOptions options;
        options.parameter_overrides(params);
        explorer_ = std::make_shared<ClusterFrontierExplorerNode>(options);
        executor_.add_node(fake_);
        executor_.add_node(explorer_);
    }

    void TearDown() override {
        if (explorer_) {
            executor_.remove_node(explorer_);
        }
        executor_.remove_node(fake_);
    }

    void publishWorld() {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = fake_->now();
        tf.child_frame_id = "base_footprint";
        tf.transform.translation.x = robot_x_;
        tf.transform.translation.y = robot_y_;
        tf.transform.rotation.z = std::sin(robot_yaw_ / 2.0);
        tf.transform.rotation.w = std::cos(robot_yaw_ / 2.0);
        tf.header.frame_id = "odom";
        tf_->sendTransform(tf);
        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = tf.header.stamp;
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";
        map_to_odom.transform.rotation.w = 1.0;
        tf_->sendTransform(map_to_odom);
        const auto now = std::chrono::steady_clock::now();
        if (now - last_map_pub_ > 300ms) {
            map_.header.stamp = fake_->now();
            map_pub_->publish(map_);
            last_map_pub_ = now;
        }
    }

    bool spinUntil(const std::function<bool()>& done,
                   const std::chrono::milliseconds timeout = 8000ms) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            publishWorld();
            executor_.spin_some(10ms);
            if (done()) {
                return true;
            }
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }

    void spinFor(const std::chrono::milliseconds duration) {
        spinUntil([] { return false; }, duration);
    }

    void inject(const nlohmann::json& event) {
        std_msgs::msg::String msg;
        msg.data = event.dump();
        event_pub_->publish(msg);
    }

    std::size_t countEvents(const std::string& name,
                            const std::size_t since = 0) const {
        std::size_t count = 0;
        for (std::size_t i = since; i < events_.size(); ++i) {
            if (field(events_[i], "event") == name) {
                ++count;
            }
        }
        return count;
    }

    std::string eventLog() const {
        std::string log;
        for (const auto& event : events_) {
            log += event.dump() + "\n";
        }
        return log;
    }

    std::optional<nlohmann::json> lastEvent(const std::string& name) const {
        for (auto it = events_.rbegin(); it != events_.rend(); ++it) {
            if (field(*it, "event") == name) {
                return *it;
            }
        }
        return std::nullopt;
    }

    bool waitForEvent(const std::string& name, const std::size_t since = 0) {
        return spinUntil([&] { return countEvents(name, since) > 0; });
    }

    // Active route to a frontier, then a stall that the reobserve stage can
    // not resolve (no frontier left), so recovery reaches the clearance
    // backup that precedes the first recovery spin.
    void driveToClearanceBackup() {
        startExplorer();
        ASSERT_TRUE(spinUntil([&] { return nav_->goals.size() == 1; }))
            << "no frontier goal was sent";
        map_ = makeMap(false);
        last_map_pub_ = {};
        spinFor(500ms);
        inject({{"event", "stall_or_slip"}});
        ASSERT_TRUE(waitForEvent("recovery_tree_started"));
        ASSERT_TRUE(spinUntil([&] { return backup_->goals.size() == 1; }))
            << "clearance backup was not sent";
        ASSERT_TRUE(waitForEvent("recovery_clearance_backup_started"));
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Node::SharedPtr fake_;
    std::shared_ptr<ClusterFrontierExplorerNode> explorer_;
    std::unique_ptr<FakeActionServer<nav2_msgs::action::NavigateToPose>> nav_;
    std::unique_ptr<FakeActionServer<nav2_msgs::action::Spin>> spin_;
    std::unique_ptr<FakeActionServer<nav2_msgs::action::BackUp>> backup_;
    std::vector<rclcpp::ServiceBase::SharedPtr> lifecycle_services_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    nav_msgs::msg::OccupancyGrid map_;
    std::chrono::steady_clock::time_point last_map_pub_{};
    double robot_x_ = 0.5;
    double robot_y_ = 1.0;
    double robot_yaw_ = 0.0;
    std::vector<nlohmann::json> events_;
};

// A stale nav result, arriving after recovery cancelled its goal, is ignored.
TEST_F(ExplorerSequenceTest, LateNavResultAfterRecoveryIsIgnored) {
    driveToClearanceBackup();
    EXPECT_TRUE(nav_->cancelRequested(0));
    const auto mark = events_.size();
    nav_->abort(0);
    spinFor(500ms);
    EXPECT_EQ(countEvents("goal_result", mark), 0U);
    EXPECT_EQ(countEvents("recovery_tree_started", mark), 0U);
}

// reverse_blocked during the optional clearance backup skips it and spins;
// the cancelled backup's late result must not start a second spin.
TEST_F(ExplorerSequenceTest, ClearanceBackupBlockedSkipsToSpinAndIgnoresLate) {
    driveToClearanceBackup();
    inject({{"event", "reverse_blocked"}, {"reason", "rear_blocked"}});
    ASSERT_TRUE(waitForEvent("recovery_clearance_backup_skipped"));
    ASSERT_TRUE(spinUntil([&] { return spin_->goals.size() == 1; }));
    EXPECT_TRUE(backup_->cancelRequested(0));
    EXPECT_EQ(countEvents("backtrace_recovery_failed"), 0U);

    const auto mark = events_.size();
    backup_->succeed(0);
    spinFor(500ms);
    EXPECT_EQ(countEvents("recovery_clearance_backup_finished", mark), 0U);
    EXPECT_EQ(spin_->goals.size(), 1U);

    spin_->succeed(0);
    ASSERT_TRUE(spinUntil([&] {
        const auto e = lastEvent("recovery_stage_finished");
        return e && field(*e, "reason") == "spin_completed";
    }));
}

// Pausing for ALC while a recovery spin runs cancels it; its late result
// does not advance recovery.
TEST_F(ExplorerSequenceTest, PauseDuringSpinCancelsAndIgnoresLateResult) {
    driveToClearanceBackup();
    backup_->succeed(0);
    ASSERT_TRUE(spinUntil([&] { return spin_->goals.size() == 1; }));
    inject({{"event", "alc_started"}});
    ASSERT_TRUE(waitForEvent("frontier_paused_for_alc"));
    ASSERT_TRUE(spinUntil([&] { return spin_->cancelRequested(0); }));

    const auto mark = events_.size();
    spin_->succeed(0);
    spinFor(500ms);
    EXPECT_EQ(countEvents("recovery_stage_finished", mark), 0U);
    EXPECT_EQ(countEvents("recovery_stage_started", mark), 0U);
}

// The same reverse_blocked verdict fails a backtrace backup (it is the
// recovery's own reverse motion) instead of skipping it.
TEST_F(ExplorerSequenceTest, BacktraceBackupBlockedFailsTheBacktrace) {
    // No spin and no room to escape forward: the tree reaches the backtrace.
    startExplorer({{"recovery_small_spin_attempts", 0}});
    ASSERT_TRUE(spinUntil([&] { return nav_->goals.size() == 1; }));
    robot_x_ = 0.8;  // a breadcrumb trail to back along
    spinFor(500ms);
    map_ = makeMap(false);
    std::fill(map_.data.begin(), map_.data.end(), std::int8_t{100});
    last_map_pub_ = {};
    spinFor(500ms);
    inject({{"event", "stall_or_slip"}});
    ASSERT_TRUE(spinUntil([&] { return backup_->goals.size() == 1; }))
        << "backtrace backup was not sent";
    ASSERT_TRUE(waitForEvent("backtrace_recovery_segment"));

    inject({{"event", "reverse_blocked"}, {"reason", "rear_blocked"}});
    ASSERT_TRUE(waitForEvent("backtrace_recovery_failed"));
    EXPECT_EQ(field(*lastEvent("backtrace_recovery_failed"), "reason"),
              "REVERSE_UNSAFE");
    EXPECT_EQ(countEvents("recovery_clearance_backup_skipped"), 0U);
    ASSERT_TRUE(spinUntil([&] { return backup_->cancelRequested(0); }));

    const auto mark = events_.size();
    backup_->succeed(0);
    spinFor(500ms);
    EXPECT_EQ(countEvents("backtrace_recovery_segment", mark), 0U);
}

// Pinned after a forward segment: the turn must undo the driven arc with the
// same wheel angle (reverse speed, opposite yaw rate) back to its start.
TEST_F(ExplorerSequenceTest, ObservationTurnRetracesItsTrackWhenPinned) {
    map_ = makeMap(false);
    geometry_msgs::msg::Twist cmd;
    bool retracing = false;
    double forward_curvature = 0.0;
    auto last = std::chrono::steady_clock::now();
    const auto sub = fake_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_smoothed_raw", 10, [&](const geometry_msgs::msg::Twist& msg) {
            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now - last).count();
            last = now;
            // The retrace opens with a stop, after the last pinned command.
            retracing |= countEvents("observation_turn_retrace_started") > 0 &&
                         cmd.linear.x == 0.0;
            if (cmd.linear.x > 0.0 && !retracing) {
                forward_curvature = cmd.angular.z / cmd.linear.x;
                robot_x_ = std::min(0.65, robot_x_ + cmd.linear.x * dt);
            }
            else if (cmd.linear.x < 0.0 && retracing) {
                EXPECT_NEAR(cmd.angular.z / cmd.linear.x, forward_curvature,
                            1e-6);
                robot_x_ += cmd.linear.x * dt;
            }
            cmd = msg;
        });
    startExplorer();
    ASSERT_TRUE(spinUntil(
        [&] { return countEvents("observation_turn_finished") > 0; }, 25000ms))
        << eventLog();
    const auto finished = lastEvent("observation_turn_finished");
    EXPECT_EQ(field(*finished, "reason"), "retraced_after_contact");
    EXPECT_NE(forward_curvature, 0.0);
    EXPECT_NEAR(robot_x_, 0.5, 0.06);
}

// Nav2 refuses every plan from here: the pass must try the other goals of a
// cluster, end without a 600 s blacklist, and try again in the next pass.
TEST_F(ExplorerSequenceTest, FailedPlansAreRetriedInTheNextPass) {
    FakeActionServer<nav2_msgs::action::ComputePathToPose> planner(
        fake_, "/compute_path_to_pose");
    startExplorer({{"nav2_validation_enabled", true}});
    std::size_t aborted = 0;
    std::optional<std::size_t> first_pass;
    ASSERT_TRUE(spinUntil(
        [&] {
            while (aborted < planner.goals.size()) {
                planner.abort(aborted++);
            }
            const auto passes = countEvents("no_frontier_candidate");
            if (passes >= 1 && !first_pass) {
                first_pass = aborted;
            }
            return passes >= 2;
        },
        15000ms))
        << eventLog();
    EXPECT_GE(*first_pass, 2U);
    // The second pass asks again for a goal that failed in the first.
    const auto position = [&](const std::size_t i) {
        return planner.goals[i]->get_goal()->goal.pose.position;
    };
    bool retried = false;
    for (std::size_t i = *first_pass; i < aborted; ++i) {
        for (std::size_t j = 0; j < *first_pass; ++j) {
            retried |= position(i) == position(j);
        }
    }
    EXPECT_TRUE(retried);
    EXPECT_EQ(countEvents("goal_attempt_updated"), 0U);
    EXPECT_EQ(nav_->goals.size(), 0U);
}

// Nothing left to see from here: go back to an earlier pose, do not stand.
TEST_F(ExplorerSequenceTest, ReturnsAlongItsTrackWhenNoGoalIsLeft) {
    robot_y_ = 0.8;
    robot_yaw_ = 0.4;
    startExplorer({{"min_turning_radius_m", 0.5}});
    ASSERT_TRUE(spinUntil([&] { return nav_->goals.size() == 1; }))
        << eventLog();
    robot_x_ = 2.2;
    robot_y_ = 1.2;
    robot_yaw_ = 0.0;
    map_ = makeMap(false);
    last_map_pub_ = {};
    spinFor(600ms);
    nav_->abort(0);
    ASSERT_TRUE(spinUntil([&] { return nav_->goals.size() >= 2; }, 15000ms))
        << eventLog();
    const auto pose = nav_->goals.back()->get_goal()->pose.pose;
    EXPECT_NEAR(pose.position.x, 0.5, 1e-6);
    EXPECT_NEAR(pose.position.y, 0.8, 1e-6);
    EXPECT_NEAR(2.0 * std::atan2(pose.orientation.z, pose.orientation.w), 0.4,
                1e-6);
}

}  // namespace
}  // namespace yahboom_alc_bringup
