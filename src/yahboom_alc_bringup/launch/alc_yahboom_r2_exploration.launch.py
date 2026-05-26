import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def launch_lidar(context, *args, **kwargs):
    rplidar_type = LaunchConfiguration("rplidar_type").perform(context)
    if rplidar_type == "none":
        return []

    if rplidar_type in ("a1", "s2"):
        sllidar_share = get_package_share_directory("sllidar_ros2")
        launch_file = (
            "sllidar_launch.py" if rplidar_type == "a1" else "sllidar_s2_launch.py"
        )
        laser_yaw = "3.14"
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar_share, "launch", launch_file)
            )
        )
    elif rplidar_type == "4ROS":
        ydlidar_share = get_package_share_directory("ydlidar_ros2_driver")
        laser_yaw = "0.0"
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ydlidar_share, "launch", "ydlidar_raw_launch.py")
            )
        )
    else:
        raise ValueError(
            "rplidar_type must be one of: none, a1, s2, 4ROS; "
            f"got {rplidar_type!r}"
        )

    tf_base_link_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0435",
            "5.258E-05",
            "0.11",
            laser_yaw,
            "0",
            "0",
            "base_link",
            "laser",
        ],
    )
    return [lidar_launch, tf_base_link_to_laser]


def generate_launch_description():
    yahboom_alc_share = get_package_share_directory("yahboom_alc_bringup")
    yahboom_bringup_share = get_package_share_directory("yahboomcar_bringup")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    astra_camera_share = get_package_share_directory("astra_camera")

    yahboom_bringup_launch = os.path.join(yahboom_bringup_share, "launch")
    default_nav2_params = os.path.join(
        yahboom_alc_share, "params", "nav2_rgbd_params.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    qos = LaunchConfiguration("qos")
    localization = LaunchConfiguration("localization")
    enable_camera = LaunchConfiguration("enable_camera")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav_cmd_vel_topic = LaunchConfiguration("nav_cmd_vel_topic")
    driver_cmd_vel_topic = LaunchConfiguration("driver_cmd_vel_topic")

    r2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboom_bringup_launch, "yahboomcar_bringup_R2_launch.py")
        ),
        launch_arguments={"pub_odom_tf": "false"}.items(),
    )

    astra_camera = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(astra_camera_share, "launch", "astra.launch.xml")
        ),
        condition=IfCondition(enable_camera),
        launch_arguments={
            "camera_name": "camera",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_point_cloud": "true",
            "enable_colored_point_cloud": "false",
        }.items(),
    )

    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        output="screen",
        parameters=[
            {
                "approx_sync": True,
                "approx_sync_max_interval": 0.01,
                "use_sim_time": use_sim_time,
                "qos": qos,
            }
        ],
        remappings=[
            ("rgb/image", "/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("depth/image", "/camera/depth/image_raw"),
        ],
    )

    rtabmap_parameters = {
        "frame_id": "base_footprint",
        "use_sim_time": use_sim_time,
        "subscribe_rgbd": True,
        "subscribe_scan": False,
        "use_action_for_goal": True,
        "qos_image": qos,
        "qos_imu": qos,
        "Reg/Strategy": "2",
        "Reg/Force3DoF": "true",
        "RGBD/NeighborLinkRefining": "True",
        "Optimizer/GravitySigma": "0",
    }

    rtabmap_remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
        ("odom", "/odom"),
    ]

    rtabmap_mapping = Node(
        condition=UnlessCondition(localization),
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remappings,
        arguments=["-d"],
    )

    rtabmap_localization = Node(
        condition=IfCondition(localization),
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        parameters=[
            rtabmap_parameters,
            {
                "Mem/IncrementalMemory": "False",
                "Mem/InitWMWithAllNodes": "True",
            },
        ],
        remappings=rtabmap_remappings,
    )

    nav2 = GroupAction(
        [
            SetRemap(src="cmd_vel", dst=nav_cmd_vel_topic),
            SetRemap(src="/cmd_vel", dst=nav_cmd_vel_topic),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params_file,
                    "autostart": "true",
                    "use_composition": "false",
                }.items(),
            ),
        ]
    )

    alc_planner = Node(
        package="alc_planner",
        executable="alc_planner_node",
        name="alc_planner",
        output="screen",
        parameters=[
            {
                "cE": 8.0,
                "tau_min_revisit": 4,
                "theta_max": 0.25,
                "lambda_decay": 0.03,
                "cl": 6.0,
                "plc_min_revisit": 0.02,
                "map_dist_min_revisit": 0.75,
                "navigation_timeout_sec": 120.0,
                "use_approach_heading": True,
            }
        ],
    )

    r2_bridge = Node(
        package="yahboom_alc_bringup",
        executable="yahboom_r2_cmd_vel_bridge",
        name="yahboom_r2_cmd_vel_bridge",
        output="screen",
        parameters=[
            {
                "input_topic": nav_cmd_vel_topic,
                "output_topic": driver_cmd_vel_topic,
                "min_linear_speed": 0.05,
                "max_linear_speed": 0.26,
                "steering_gain": 1.25,
                "max_steering": 1.25,
                "linear_deadband": 0.001,
                "angular_deadband": 0.001,
                "arc_turn_when_stationary": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "qos",
                default_value="2",
                description="QoS profile used by the RTAB-Map input topics.",
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="false",
                description="Launch RTAB-Map in localization mode.",
            ),
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description="Launch the Astra RGB-D camera driver.",
            ),
            DeclareLaunchArgument(
                "robot_type",
                default_value="r2",
                description="Yahboom robot type. This launch targets r2.",
            ),
            DeclareLaunchArgument(
                "rplidar_type",
                default_value="none",
                description="Optional RPLIDAR type passed to laser bringup: a1, s2, 4ROS, or none.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params,
                description=(
                    "Nav2 parameter YAML. Use this to tune spin behavior timing "
                    "for the R2 arc-turn approximation."
                ),
            ),
            DeclareLaunchArgument(
                "nav_cmd_vel_topic",
                default_value="/alc/nav_cmd_vel",
                description="Topic receiving raw Nav2 controller velocity commands.",
            ),
            DeclareLaunchArgument(
                "driver_cmd_vel_topic",
                default_value="/cmd_vel",
                description="Topic consumed by the Yahboom R2 driver.",
            ),
            r2_bringup,
            OpaqueFunction(function=launch_lidar),
            astra_camera,
            rgbd_sync,
            rtabmap_mapping,
            rtabmap_localization,
            nav2,
            alc_planner,
            r2_bridge,
        ]
    )
