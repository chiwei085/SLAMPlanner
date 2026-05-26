import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    yahboom_alc_share = get_package_share_directory("yahboom_alc_bringup")
    yahboom_bringup_share = get_package_share_directory("yahboomcar_bringup")
    yahboom_nav_share = get_package_share_directory("yahboomcar_nav")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    sllidar_share = get_package_share_directory("sllidar_ros2")
    ydlidar_share = get_package_share_directory("ydlidar_ros2_driver")

    yahboom_bringup_launch = os.path.join(yahboom_bringup_share, "launch")
    default_nav2_params = os.path.join(
        yahboom_alc_share, "params", "nav2_rgbd_params.yaml"
    )
    default_map = os.path.join(yahboom_nav_share, "maps", "yahboomcar.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    qos = LaunchConfiguration("qos")
    localization = LaunchConfiguration("localization")
    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav_cmd_vel_topic = LaunchConfiguration("nav_cmd_vel_topic")
    driver_cmd_vel_topic = LaunchConfiguration("driver_cmd_vel_topic")

    r2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboom_bringup_launch, "yahboomcar_bringup_R2_launch.py")
        ),
        launch_arguments={"pub_odom_tf": "false"}.items(),
    )

    lidar_a1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_share, "launch", "sllidar_launch.py")
        ),
        condition=LaunchConfigurationEquals("rplidar_type", "a1"),
    )

    lidar_s2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_share, "launch", "sllidar_s2_launch.py")
        ),
        condition=LaunchConfigurationEquals("rplidar_type", "s2"),
    )

    lidar_4ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_share, "launch", "ydlidar_raw_launch.py")
        ),
        condition=LaunchConfigurationEquals("rplidar_type", "4ROS"),
    )

    tf_base_link_to_laser_a1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0435",
            "5.258E-05",
            "0.11",
            "3.14",
            "0",
            "0",
            "base_link",
            "laser",
        ],
        condition=LaunchConfigurationEquals("rplidar_type", "a1"),
    )

    tf_base_link_to_laser_s2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0435",
            "5.258E-05",
            "0.11",
            "3.14",
            "0",
            "0",
            "base_link",
            "laser",
        ],
        condition=LaunchConfigurationEquals("rplidar_type", "s2"),
    )

    tf_base_link_to_laser_4ros = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0435",
            "5.258E-05",
            "0.11",
            "0.0",
            "0",
            "0",
            "base_link",
            "laser",
        ],
        condition=LaunchConfigurationEquals("rplidar_type", "4ROS"),
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
        "Reg/Strategy": "1",
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
                    os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params_file,
                }.items(),
            ),
        ]
    )

    alc_planner = Node(
        package="alc_planner",
        executable="alc_planner_node",
        name="alc_planner",
        output="screen",
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
                "map",
                default_value=default_map,
                description="Map YAML passed to Nav2 bringup.",
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
            lidar_a1,
            lidar_s2,
            lidar_4ros,
            tf_base_link_to_laser_a1,
            tf_base_link_to_laser_s2,
            tf_base_link_to_laser_4ros,
            rgbd_sync,
            rtabmap_mapping,
            rtabmap_localization,
            nav2,
            alc_planner,
            r2_bridge,
        ]
    )
