"""Dual RGB-D camera front end for RTAB-Map.

Astra Pro Plus = high camera (walls, features), RealSense D455 = low camera
(floor, low obstacles). Each camera gets its own rgbd_sync; rgbdx_sync then
merges both into /rgbd_images for RTAB-Map (rgbd_cameras:=0). The cameras do
not need overlapping views, but their mount poses below MUST be measured: a
wrong pose shows up as doubled/offset walls in the map.

Mount poses are base_link -> camera body (Astra: camera_depth_frame, i.e. the
old URDF camera_link; D455: d455_link, the driver publishes the rest).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

POSE_AXES = ("x", "y", "z", "roll", "pitch", "yaw")

# Measured 2026-11-18. x: ruler, both bodies centred 2 cm behind the front
# axle (URDF x=0.1192 -> 0.099). pitch and z: floor plane fitted in each
# camera's own depth frame (RANSAC on bottom image rows), which beat the
# ruler/eyeball 5 deg and the uncalibrated D455 IMU (7.6 deg). z = optical
# centre height above floor minus base_link's 0.0215. y: the TF root is not
# the body centre -- Astra depth lens sits 2.5 cm right of its centred RGB
# lens; d455_link is the left IR lens, half the 95 mm baseline left.
# Roll measured < 1 deg, left at 0.
MOUNT_DEFAULTS = {
    "astra": ("0.099", "-0.025", "0.870", "0", "0.165", "0"),
    "d455": ("0.099", "0.0475", "0.1665", "0", "0.239", "0"),
}


def mount_tf(camera, child_frame):
    args = []
    for axis in POSE_AXES:
        args += [f"--{axis}", LaunchConfiguration(f"{camera}_{axis}")]
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"base_link_to_{camera}",
        arguments=args + ["--frame-id", "base_link", "--child-frame-id", child_frame],
    )


def static_tf(name, xyz, quat, parent, child):
    args = []
    for axis, value in zip(("x", "y", "z", "qx", "qy", "qz", "qw"), (*xyz, *quat)):
        args += [f"--{axis}", value]
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=args + ["--frame-id", parent, "--child-frame-id", child],
    )


def rgbd_sync(name, rgb, info, depth, output, max_interval):
    return Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name=name,
        output="screen",
        parameters=[
            {
                "approx_sync": True,
                "approx_sync_max_interval": max_interval,
                "topic_queue_size": 30,
                "sync_queue_size": 30,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "qos": LaunchConfiguration("qos"),
            }
        ],
        remappings=[
            ("rgb/image", rgb),
            ("rgb/camera_info", info),
            ("depth/image", depth),
            ("rgbd_image", output),
        ],
    )


def generate_launch_description():
    astra_share = get_package_share_directory("astra_camera")
    enable_astra = LaunchConfiguration("enable_camera")
    enable_d455 = LaunchConfiguration("enable_realsense")

    astra = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(astra_share, "launch", "astro_pro_plus.launch.xml")
        ),
        condition=IfCondition(enable_astra),
        launch_arguments={
            "camera_name": "camera",
            "enable_color": "true",
            "enable_depth": "true",
            "color_fps": LaunchConfiguration("camera_color_fps"),
            "depth_fps": LaunchConfiguration("camera_depth_fps"),
            "ir_fps": LaunchConfiguration("camera_ir_fps"),
            # Fixed extrinsics go on /tf_static below; publishing them from the
            # camera thread lets a USB stall expire the TF at RGB-D stamps.
            "publish_tf": "false",
            "enable_ir": "false",
            "enable_point_cloud": "true",
            "enable_colored_point_cloud": "false",
        }.items(),
    )

    # Astra Pro Plus factory depth->color extrinsics observed from the driver.
    optical = ("-0.5", "0.5", "-0.5", "0.5")
    astra_tfs = [
        mount_tf("astra", "camera_depth_frame"),
        static_tf(
            "camera_depth_to_color_frame",
            ("-0.002165860891342163", "0.025120487213134767", "0.00009495562314987182"),
            ("-0.0008345809765160083", "-0.00009292588947573677",
             "-0.0009316844516433775", "0.9999992251396179"),
            "camera_depth_frame", "camera_color_frame",
        ),
        static_tf("camera_depth_to_optical_frame", ("0", "0", "0"), optical,
                  "camera_depth_frame", "camera_depth_optical_frame"),
        static_tf("camera_color_to_optical_frame", ("0", "0", "0"), optical,
                  "camera_color_frame", "camera_color_optical_frame"),
    ]

    # D455 runs against the RSUSB-backend librealsense build (the stock V4L
    # backend does not enumerate the device on this Jetson kernel).
    d455 = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="realsense",
        name="d455",
        output="screen",
        condition=IfCondition(enable_d455),
        additional_env={
            "LD_LIBRARY_PATH": [
                LaunchConfiguration("realsense_lib_dir"),
                ":",
                EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
            ]
        },
        parameters=[
            {
                "camera_name": "d455",
                "rgb_camera.color_profile": "640,480,15",
                "depth_module.depth_profile": "640,480,15",
                "enable_color": True,
                "enable_depth": True,
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_gyro": False,
                "enable_accel": False,
                "align_depth.enable": True,
                # ARM builds name the filter pointcloud__neon_. Untextured so
                # the cloud covers the full depth FOV, for the Nav2 costmaps.
                "pointcloud__neon_.enable": True,
                "pointcloud__neon_.stream_filter": 0,
                # Factory d455_link -> optical frames; d455_link is mounted below.
                "publish_tf": True,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    syncs = [
        # Astra color/depth arrive up to ~1.5 frames apart at 30 fps on Jetson.
        rgbd_sync(
            "astra_rgbd_sync",
            "/camera/color/image_raw",
            "/camera/color/camera_info",
            "/camera/depth/image_raw",
            "/astra/rgbd_image",
            0.05,
        ),
        rgbd_sync(
            "d455_rgbd_sync",
            "/realsense/d455/color/image_raw",
            "/realsense/d455/color/camera_info",
            "/realsense/d455/aligned_depth_to_color/image_raw",
            "/d455/rgbd_image",
            0.05,
        ),
        # Output rate follows the slower D455 (~15 Hz). Residual inter-camera
        # offset (<~35 ms) is motion-compensated by rtabmap odom_sensor_sync.
        Node(
            package="rtabmap_sync",
            executable="rgbdx_sync",
            name="rgbdx_sync",
            output="screen",
            parameters=[
                {
                    "rgbd_cameras": 2,
                    "approx_sync": True,
                    "approx_sync_max_interval": 0.05,
                    "topic_queue_size": 10,
                    "sync_queue_size": 10,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "qos": LaunchConfiguration("qos"),
                }
            ],
            remappings=[
                ("rgbd_image0", "/astra/rgbd_image"),
                ("rgbd_image1", "/d455/rgbd_image"),
                ("rgbd_images", "/rgbd_images"),
            ],
        ),
    ]

    mount_args = [
        DeclareLaunchArgument(
            f"{camera}_{axis}",
            default_value=default,
            description=f"{camera} mount pose relative to base_link ({axis}, m/rad).",
        )
        for camera, defaults in MOUNT_DEFAULTS.items()
        for axis, default in zip(POSE_AXES, defaults)
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("qos", default_value="2"),
            DeclareLaunchArgument("enable_camera", default_value="true",
                                  description="Launch the high Astra camera."),
            DeclareLaunchArgument("enable_realsense", default_value="true",
                                  description="Launch the low D455 camera."),
            DeclareLaunchArgument(
                "realsense_lib_dir",
                default_value=os.path.expanduser(
                    "~/realsense-check/rsusb-build/Release"
                ),
                description="Directory with the RSUSB-backend librealsense2.so.",
            ),
            DeclareLaunchArgument("camera_color_fps", default_value="30"),
            DeclareLaunchArgument("camera_depth_fps", default_value="30"),
            DeclareLaunchArgument("camera_ir_fps", default_value="30"),
            DeclareLaunchArgument(
                "sync_start_delay_sec",
                default_value="3.0",
                description="Delay RGB-D sync nodes until the TF tree is up.",
            ),
            *mount_args,
            astra,
            *astra_tfs,
            d455,
            mount_tf("d455", "d455_link"),
            TimerAction(
                period=LaunchConfiguration("sync_start_delay_sec"), actions=syncs
            ),
        ]
    )
