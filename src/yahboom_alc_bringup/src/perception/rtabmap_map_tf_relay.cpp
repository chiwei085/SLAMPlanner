// Decouples map→odom TF publishing from RTAB-Map's internal graph-optimization
// mutex.
//
// Root cause being fixed: RTAB-Map's TF-publishing thread and its graph
// optimizer both compete for CoreWrapper::mapToOdomMutex_. On Jetson under
// load, the optimizer can hold this mutex for 1–2 s, leaving map→odom
// unpublished for the same interval. Nav2's FollowPath controller (and
// bt_navigator) then throw "Lookup would require extrapolation into the future"
// and abort the action.
//
// This node subscribes to /localization_pose (PoseWithCovarianceStamped), which
// RTAB-Map publishes from its SLAM callback — outside the contested mutex. It
// caches the derived map→odom transform and republishes it at 50 Hz with a
// fresh timestamp, so Nav2 always sees a current transform even during
// optimization spikes. Before the first localization pose arrives, it publishes
// an identity map→odom transform so RViz/Nav2 have a complete startup TF tree.
//
// Threading model: MultiThreadedExecutor with two MutuallyExclusive callback
// groups so the 50 Hz timer is never delayed by a slow TF lookup in the
// subscriber callback. A std::shared_mutex guards the cache: the subscriber
// holds unique_lock (write), the timer holds shared_lock (read-only). Multiple
// timer firings could theoretically overlap with shared_lock, but in practice
// the timer callback is microsecond-fast.

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"  // IWYU pragma: keep
#include "geometry_msgs/msg/transform_stamped.hpp"  // IWYU pragma: keep
#include "rclcpp/rclcpp.hpp"                        // IWYU pragma: keep
#include "tf2/LinearMath/Transform.h"               // IWYU pragma: keep
#include "tf2/exceptions.h"                         // IWYU pragma: keep
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // IWYU pragma: keep
#include "tf2_ros/buffer.h"                         // IWYU pragma: keep
#include "tf2_ros/transform_broadcaster.h"          // IWYU pragma: keep
#include "tf2_ros/transform_listener.h"             // IWYU pragma: keep
#include "yahboom_alc_bringup/parameter_loader.hpp"

namespace yahboom_alc_bringup
{

struct MapTfRelayParams
{
    std::string odom_frame = "odom";
    std::string map_frame = "map";
    std::string base_frame = "base_footprint";
    double publish_rate_hz = 50.0;
    double stale_warn_sec = 2.0;
};

}  // namespace yahboom_alc_bringup

namespace rik_sd
{

template <>
struct descriptor<yahboom_alc_bringup::MapTfRelayParams>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::MapTfRelayParams;
        return make_descriptor(
            named(&MapTfRelayParams::odom_frame, "odom_frame"),
            named(&MapTfRelayParams::map_frame, "map_frame"),
            named(&MapTfRelayParams::base_frame, "base_frame"),
            named(&MapTfRelayParams::publish_rate_hz, "publish_rate_hz"),
            named(&MapTfRelayParams::stale_warn_sec, "stale_warn_sec"));
    }
};

}  // namespace rik_sd

class MapTfRelayNode : public rclcpp::Node
{
public:
    explicit MapTfRelayNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("map_tf_relay", options) {
        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        params_ = yahboom_alc_bringup::loadDeclaredParameters<
            yahboom_alc_bringup::MapTfRelayParams>(*this);

        cached_transform_ = makeIdentityMapToOdom();

        // Subscriber and timer in separate MutuallyExclusive callback groups so
        // the MultiThreadedExecutor can run them concurrently — the subscriber
        // callback may block briefly on a TF lookup without delaying the timer.
        sub_cbgroup_ =
            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cbgroup_ =
            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_cbgroup_;

        loc_sub_ =
            create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "localization_pose", rclcpp::SensorDataQoS(),
                [this](geometry_msgs::msg::PoseWithCovarianceStamped::
                           ConstSharedPtr msg) { onLocalizationPose(msg); },
                sub_opts);

        republish_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / params_.publish_rate_hz),
            [this]() { republishTransform(); }, timer_cbgroup_);
    }

private:
    void onLocalizationPose(
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr&
            msg) {
        // map_T_base comes directly from RTAB-Map's SLAM callback output.
        tf2::Transform map_T_base;
        tf2::fromMsg(msg->pose.pose, map_T_base);

        // odom_T_base: try exact stamp first, fall back to latest.
        // EKF publishes odom at 30 Hz so the latest is at most ~33 ms old — at
        // 0.1 m/s that is ~3 mm of drift, well within acceptable limits.
        geometry_msgs::msg::TransformStamped odom_T_base_stamped;
        try {
            odom_T_base_stamped = tf_buffer_->lookupTransform(
                params_.odom_frame, params_.base_frame, msg->header.stamp,
                tf2::durationFromSec(0.2));
        }
        catch (const tf2::TransformException&) {
            try {
                odom_T_base_stamped = tf_buffer_->lookupTransform(
                    params_.odom_frame, params_.base_frame, tf2::TimePointZero);
            }
            catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Cannot lookup %s→%s: %s — skipping update",
                    params_.odom_frame.c_str(), params_.base_frame.c_str(),
                    ex.what());
                return;
            }
        }

        tf2::Transform odom_T_base;
        tf2::fromMsg(odom_T_base_stamped.transform, odom_T_base);

        // map_T_odom = map_T_base × (odom_T_base)^-1
        tf2::Transform map_T_odom = map_T_base * odom_T_base.inverse();

        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp =
            msg->header.stamp;  // source timestamp for staleness tracking
        ts.header.frame_id = msg->header.frame_id;  // "map"
        ts.child_frame_id = params_.odom_frame;
        tf2::toMsg(map_T_odom, ts.transform);

        std::unique_lock<std::shared_mutex> lk(cache_mutex_);
        cached_transform_ = ts;
        cache_from_localization_pose_ = true;
    }

    void republishTransform() {
        std::shared_lock<std::shared_mutex> lk(cache_mutex_);
        if (!cached_transform_.has_value()) {
            return;
        }
        auto ts = *cached_transform_;
        const bool cache_from_localization_pose = cache_from_localization_pose_;
        lk.unlock();

        if (!cache_from_localization_pose) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "No localization_pose received yet; publishing identity %s→%s",
                params_.map_frame.c_str(), params_.odom_frame.c_str());
        }
        auto source_age = (get_clock()->now() - ts.header.stamp).seconds();
        if (cache_from_localization_pose &&
            source_age > params_.stale_warn_sec) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "map→%s transform is %.1f s stale — RTAB-Map may be stalled",
                params_.odom_frame.c_str(), source_age);
        }

        // Republish with current timestamp so Nav2 sees a valid,
        // non-extrapolating transform.
        ts.header.stamp = get_clock()->now();
        tf_broadcaster_->sendTransform(ts);
    }

    geometry_msgs::msg::TransformStamped makeIdentityMapToOdom() {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = get_clock()->now();
        ts.header.frame_id = params_.map_frame;
        ts.child_frame_id = params_.odom_frame;
        ts.transform.rotation.w = 1.0;
        return ts;
    }

    // Cache is written by onLocalizationPose (subscriber, infrequent) and read
    // by republishTransform (timer, 50 Hz). shared_mutex: multiple concurrent
    // readers, exclusive writer. On AArch64, std::atomic<std::shared_ptr<T>> is
    // not lock-free and would add hidden complexity; shared_mutex is explicit,
    // portable, and sufficient.
    mutable std::shared_mutex cache_mutex_;
    std::optional<geometry_msgs::msg::TransformStamped> cached_transform_;
    bool cache_from_localization_pose_{false};

    rclcpp::CallbackGroup::SharedPtr sub_cbgroup_;
    rclcpp::CallbackGroup::SharedPtr timer_cbgroup_;
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr loc_sub_;
    rclcpp::TimerBase::SharedPtr republish_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    yahboom_alc_bringup::MapTfRelayParams params_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<MapTfRelayNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
