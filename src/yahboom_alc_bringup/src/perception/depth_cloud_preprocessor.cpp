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

class DepthCloudPreprocessorNode : public rclcpp::Node
{
public:
    explicit DepthCloudPreprocessorNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("depth_cloud_preprocessor", options),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
          tf_listener_(
              std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
        input_topic_ = declare_parameter<std::string>("input_topic",
                                                      "/camera/depth/points");
        output_topic_ = declare_parameter<std::string>(
            "output_topic", "/camera/depth/points_voxel_base");
        target_frame_ =
            declare_parameter<std::string>("target_frame", "base_footprint");
        leaf_size_m_ = declare_parameter<double>("leaf_size_m", 0.05);
        transform_timeout_sec_ =
            declare_parameter<double>("transform_timeout_sec", 0.2);
        max_range_m_ = declare_parameter<double>("max_range_m", 6.0);
        // Structured-light depth cameras (Astra Pro Plus here) commonly emit
        // literal (0,0,0) for invalid/no-return pixels instead of NaN,
        // especially in the first frames after power-up while auto-exposure
        // settles. Left unfiltered, such a point transforms to the camera's
        // own mount offset from target_frame_ -- i.e. a phantom obstacle
        // stuck to the robot. The costmaps have their own obstacle_min_range
        // to catch this, but collision_monitor's pointcloud source has no
        // range filter at all, so it must be caught here, upstream of all
        // three consumers.
        min_range_m_ = declare_parameter<double>("min_range_m", 0.2);

        if (leaf_size_m_ <= 0.0) {
            RCLCPP_WARN(get_logger(),
                        "leaf_size_m must be positive; using default 0.05 m");
            leaf_size_m_ = 0.05;
        }
        if (max_range_m_ <= 0.0) {
            RCLCPP_WARN(get_logger(),
                        "max_range_m must be positive; using default 6.0 m");
            max_range_m_ = 6.0;
        }
        min_range_m_ = std::max(0.0, min_range_m_);
        transform_timeout_sec_ = std::max(0.0, transform_timeout_sec_);
        max_range_sq_m_ = max_range_m_ * max_range_m_;
        min_range_sq_m_ = min_range_m_ * min_range_m_;

        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, rclcpp::SensorDataQoS());
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                onCloud(msg);
            });

        RCLCPP_INFO(
            get_logger(),
            "Depth cloud preprocessor: %s -> %s, target=%s, leaf=%.3f m",
            input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(),
            leaf_size_m_);
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
                target_frame_, msg->header.frame_id, msg->header.stamp,
                tf2::durationFromSec(transform_timeout_sec_));
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Cannot transform depth cloud %s -> %s at stamp %.3f: %s",
                msg->header.frame_id.c_str(), target_frame_.c_str(),
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
            voxel_filter.setLeafSize(static_cast<float>(leaf_size_m_),
                                     static_cast<float>(leaf_size_m_),
                                     static_cast<float>(leaf_size_m_));
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
        output.header.frame_id = target_frame_;
        cloud_pub_->publish(output);

        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Depth cloud downsampled %zu finite points to %zu points",
            finite_cloud.size(), transformed_cloud.size());
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
    double leaf_size_m_;
    double transform_timeout_sec_;
    double max_range_m_;
    double max_range_sq_m_;
    double min_range_m_;
    double min_range_sq_m_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthCloudPreprocessorNode>());
    rclcpp::shutdown();
    return 0;
}
