"""RGB-D camera front end for RTAB-Map, in one of two modes.

camera_mode:=dual (default)
    Astra Pro Plus = high camera (walls, features), RealSense D455 = low
    camera (floor, low obstacles). Each camera gets its own rgbd_sync;
    rgbdx_sync merges both into /rgbd_images (RTAB-Map rgbd_cameras:=0).
camera_mode:=astra
    Original single-camera setup: Astra only, at its original low URDF mount,
    publishing /astra/rgbd_image (RTAB-Map rgbd_cameras:=1). rgbdx_sync is
    not used -- it refuses fewer than two cameras.

The cameras do not need overlapping views, but their mount poses MUST be
measured: a wrong pose shows up as doubled walls or a floor that rises with
distance (and then crosses Grid/MaxGroundHeight as phantom obstacles).
Mount poses are base_link -> camera body (Astra: camera_depth_frame, i.e. the
old URDF camera_link; D455: d455_link, the driver publishes the rest).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

CAMERA_MODES = ("dual", "astra")
POSE_AXES = ("x", "y", "z", "roll", "pitch", "yaw")

# Per-mode mount defaults, (x, y, z, roll, pitch, yaw) relative to base_link.
#
# dual -- measured 2026-11-18. x: ruler, both bodies centred 2 cm behind the
# front axle (URDF x=0.1192 -> 0.099). pitch and z: floor plane fitted in each
# camera's own depth frame (RANSAC on bottom image rows), which beat the
# ruler/eyeball 5 deg and the uncalibrated D455 IMU (7.6 deg). z = optical
# centre height above floor minus base_link's 0.0215. y: the TF root is not
# the body centre -- Astra depth lens sits 2.5 cm right of its centred RGB
# lens; d455_link is the left IR lens, half the 95 mm baseline left.
# Roll measured < 1 deg, left at 0 (Astra; D455 see below).
#
# astra -- the original low mount: URDF camera_joint pose.
MOUNT_DEFAULTS = {
    "dual": {
        "astra": ("0.099", "-0.025", "0.870", "0", "0.165", "0"),
        # Re-measured 2026-11-21 after the D455 mount was re-tilted: floor
        # fit in its own depth frame (raw depth + camera_info) gave 0.193 m,
        # 5.7 deg down, 1.7 deg roll; the old 0.239 put the floor 0.14 m low
        # at 1 m. Verified: body-frame floor flat within 0.1 deg, |z| < 1 mm.
        "d455": ("0.099", "0.0475", "0.1712", "-0.030", "0.099", "0"),
    },
    "astra": {
        "astra": ("0.095117", "0", "0.09955", "0", "0.0057171", "0"),
    },
}


def mount_tf(context, mode, camera, child_frame):
    args = []
    for axis, default in zip(POSE_AXES, MOUNT_DEFAULTS[mode][camera]):
        value = LaunchConfiguration(f"{camera}_{axis}").perform(context)
        args += [f"--{axis}", value or default]
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


def astra_nodes(context, mode):
    astra = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yahboom_alc_bringup"),
                "launch",
                "astra_pro_plus_respawn.launch.xml",
            )
        ),
        condition=IfCondition(LaunchConfiguration("enable_camera")),
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
    return [
        astra,
        mount_tf(context, mode, "astra", "camera_depth_frame"),
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


def d455_node():
    # D455 runs against the RSUSB-backend librealsense build (the stock V4L
    # backend does not enumerate the device on this Jetson kernel).
    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="realsense",
        name="d455",
        output="screen",
        # The RSUSB build occasionally segfaults while opening streams under
        # startup load (seen once in the full exploration launch); the low
        # camera feeds the safety layers, so bring it back automatically.
        respawn=True,
        respawn_delay=3.0,
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


def launch_cameras(context, *args, **kwargs):
    mode = LaunchConfiguration("camera_mode").perform(context)
    if mode not in CAMERA_MODES:
        raise ValueError(f"camera_mode must be one of {CAMERA_MODES}; got {mode!r}")

    actions = astra_nodes(context, mode)
    # Astra color/depth arrive up to ~1.5 frames apart at 30 fps on Jetson.
    syncs = [
        rgbd_sync(
            "astra_rgbd_sync",
            "/camera/color/image_raw",
            "/camera/color/camera_info",
            "/camera/depth/image_raw",
            "/astra/rgbd_image",
            0.05,
        )
    ]
    if mode == "dual":
        # Start the D455 only after the Astra has claimed its USB interfaces:
        # concurrent enumeration made the Astra fail with "attempt to claim
        # already-claimed interface" and segfault (and once crashed the D455).
        actions += [
            TimerAction(
                period=LaunchConfiguration("d455_start_delay_sec"),
                actions=[d455_node()],
            ),
            mount_tf(context, mode, "d455", "d455_link"),
        ]
        syncs += [
            rgbd_sync(
                "d455_rgbd_sync",
                "/realsense/d455/color/image_raw",
                "/realsense/d455/color/camera_info",
                "/realsense/d455/aligned_depth_to_color/image_raw",
                "/d455/rgbd_image",
                0.05,
            ),
            # Output rate follows the slower D455 (~15 Hz). Residual
            # inter-camera offset (<~35 ms) is motion-compensated by rtabmap
            # odom_sensor_sync.
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
    actions.append(
        TimerAction(period=LaunchConfiguration("sync_start_delay_sec"), actions=syncs)
    )
    return actions


def generate_launch_description():
    # Empty mount args fall back to the camera_mode's MOUNT_DEFAULTS.
    mount_args = [
        DeclareLaunchArgument(
            f"{camera}_{axis}",
            default_value="",
            description=(
                f"{camera} mount {axis} relative to base_link (m/rad). Empty = "
                + ", ".join(
                    f"{mode}: {cams[camera][i]}"
                    for mode, cams in MOUNT_DEFAULTS.items()
                    if camera in cams
                )
            ),
        )
        for camera in ("astra", "d455")
        for i, axis in enumerate(POSE_AXES)
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("qos", default_value="2"),
            DeclareLaunchArgument(
                "camera_mode",
                default_value="dual",
                description="dual = high Astra + low D455; astra = original single low Astra.",
            ),
            DeclareLaunchArgument("enable_camera", default_value="true",
                                  description="Launch the Astra camera driver."),
            DeclareLaunchArgument(
                "realsense_lib_dir",
                default_value=os.path.expanduser(
                    "~/realsense-check/rsusb-build/Release"
                ),
                description="Directory with the RSUSB-backend librealsense2.so.",
            ),
            DeclareLaunchArgument(
                "d455_start_delay_sec",
                default_value="6.0",
                description="Start the D455 after the Astra has opened its USB streams.",
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
            OpaqueFunction(function=launch_cameras),
        ]
    )
