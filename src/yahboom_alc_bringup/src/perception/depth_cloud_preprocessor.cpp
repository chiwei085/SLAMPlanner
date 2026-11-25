#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"                        // IWYU pragma: keep
#include "sensor_msgs/msg/point_cloud2.hpp"         // IWYU pragma: keep
#include "tf2/LinearMath/Transform.h"               // IWYU pragma: keep
#include "tf2/exceptions.h"                         // IWYU pragma: keep
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // IWYU pragma: keep
#include "tf2_ros/buffer.h"                         // IWYU pragma: keep
#include "tf2_ros/transform_listener.h"             // IWYU pragma: keep
#include "yahboom_alc_bringup/parameter_loader.hpp"

namespace yahboom_alc_bringup
{

struct DepthCloudPreprocessorParams
{
    std::string input_topic = "/camera/depth/points";
    std::string output_topic = "/camera/depth/points_voxel_base";
    std::string target_frame = "base_footprint";
    double leaf_size_m = 0.05;
    double transform_timeout_sec = 0.2;
    double max_range_m = 6.0;
    // Structured-light depth cameras (Astra Pro Plus here) commonly emit
    // literal (0,0,0) for invalid/no-return pixels instead of NaN,
    // especially in the first frames after power-up while auto-exposure
    // settles. Left unfiltered, such a point transforms to the camera's
    // own mount offset from target_frame -- i.e. a phantom obstacle
    // stuck to the robot. The costmaps have their own obstacle_min_range
    // to catch this, but collision_monitor's pointcloud source has no
    // range filter at all, so it must be caught here, upstream of all
    // three consumers.
    double min_range_m = 0.2;
};

}  // namespace yahboom_alc_bringup

namespace rik_sd
{

template <>
struct descriptor<yahboom_alc_bringup::DepthCloudPreprocessorParams>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::DepthCloudPreprocessorParams;
        return make_descriptor(
            named(&DepthCloudPreprocessorParams::input_topic, "input_topic"),
            named(&DepthCloudPreprocessorParams::output_topic, "output_topic"),
            named(&DepthCloudPreprocessorParams::target_frame, "target_frame"),
            named(&DepthCloudPreprocessorParams::leaf_size_m, "leaf_size_m"),
            named(&DepthCloudPreprocessorParams::transform_timeout_sec,
                  "transform_timeout_sec"),
            named(&DepthCloudPreprocessorParams::max_range_m, "max_range_m"),
            named(&DepthCloudPreprocessorParams::min_range_m, "min_range_m"));
    }
};

}  // namespace rik_sd

class DepthCloudPreprocessorNode : public rclcpp::Node
{
public:
    explicit DepthCloudPreprocessorNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("depth_cloud_preprocessor", options),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
          tf_listener_(
              std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
        params_ = yahboom_alc_bringup::loadDeclaredParameters<
            yahboom_alc_bringup::DepthCloudPreprocessorParams>(*this);

        if (params_.leaf_size_m <= 0.0) {
            RCLCPP_WARN(get_logger(),
                        "leaf_size_m must be positive; using default 0.05 m");
            params_.leaf_size_m = 0.05;
        }
        if (params_.max_range_m <= 0.0) {
            RCLCPP_WARN(get_logger(),
                        "max_range_m must be positive; using default 6.0 m");
            params_.max_range_m = 6.0;
        }
        params_.min_range_m = std::max(0.0, params_.min_range_m);
        params_.transform_timeout_sec =
            std::max(0.0, params_.transform_timeout_sec);
        max_range_sq_m_ = params_.max_range_m * params_.max_range_m;
        min_range_sq_m_ = params_.min_range_m * params_.min_range_m;

        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            params_.output_topic, rclcpp::SensorDataQoS());
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            params_.input_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                onCloud(msg);
            });

        RCLCPP_INFO(
            get_logger(),
            "Depth cloud preprocessor: %s -> %s, target=%s, leaf=%.3f m",
            params_.input_topic.c_str(), params_.output_topic.c_str(),
            params_.target_frame.c_str(), params_.leaf_size_m);
    }

private:
    void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Skipping depth cloud with empty frame_id");
            return;
        }

        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                params_.target_frame, msg->header.frame_id, msg->header.stamp,
                tf2::durationFromSec(params_.transform_timeout_sec));
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Cannot transform depth cloud %s -> %s at stamp %.3f: %s",
                msg->header.frame_id.c_str(), params_.target_frame.c_str(),
                rclcpp::Time(msg->header.stamp).seconds(), ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromROSMsg(*msg, input_cloud);

        // Drop non-finite points and clamp range in one pass. The range
        // clamp isn't just a cost cut: pcl::VoxelGrid derives its grid
        // dimensions from the cloud's bounding box, and a single stray
        // finite-but-huge point (a bad depth return that isn't NaN/Inf)
        // can overflow that computation, causing VoxelGrid to silently
        // emit an empty cloud for the whole frame instead of erroring.
        pcl::PointCloud<pcl::PointXYZ> finite_cloud;
        finite_cloud.reserve(input_cloud.size());
        for (const auto& point : input_cloud.points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z)) {
                continue;
            }
            const double range_sq = static_cast<double>(point.x) * point.x +
                                    static_cast<double>(point.y) * point.y +
                                    static_cast<double>(point.z) * point.z;
            if (range_sq > max_range_sq_m_ || range_sq < min_range_sq_m_) {
                continue;
            }
            finite_cloud.push_back(point);
        }
        finite_cloud.is_dense = true;

        pcl::PointCloud<pcl::PointXYZ> voxel_cloud;
        if (!finite_cloud.empty()) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setLeafSize(static_cast<float>(params_.leaf_size_m),
                                     static_cast<float>(params_.leaf_size_m),
                                     static_cast<float>(params_.leaf_size_m));
            voxel_filter.setInputCloud(finite_cloud.makeShared());
            voxel_filter.filter(voxel_cloud);
        }

        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        transformed_cloud.header = voxel_cloud.header;
        transformed_cloud.reserve(voxel_cloud.size());
        transformed_cloud.is_dense = true;

        tf2::Transform target_from_source;
        tf2::fromMsg(transform_stamped.transform, target_from_source);
        for (const auto& point : voxel_cloud.points) {
            const tf2::Vector3 source_point{point.x, point.y, point.z};
            const tf2::Vector3 target_point = target_from_source * source_point;
            transformed_cloud.emplace_back(
                static_cast<float>(target_point.x()),
                static_cast<float>(target_point.y()),
                static_cast<float>(target_point.z()));
        }
        transformed_cloud.width =
            static_cast<std::uint32_t>(transformed_cloud.size());
        transformed_cloud.height = 1;

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(transformed_cloud, output);
        output.header.stamp = msg->header.stamp;
        output.header.frame_id = params_.target_frame;
        cloud_pub_->publish(output);

        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Depth cloud downsampled %zu finite points to %zu points",
            finite_cloud.size(), transformed_cloud.size());
    }

    double max_range_sq_m_;
    double min_range_sq_m_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    yahboom_alc_bringup::DepthCloudPreprocessorParams params_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthCloudPreprocessorNode>());
    rclcpp::shutdown();
    return 0;
}
