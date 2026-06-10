import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


def launch_frontier_explorer(context, *args, **kwargs):
    enabled = LaunchConfiguration("enable_frontier_explorer").perform(context).lower()
    if enabled not in ("1", "true", "yes", "on"):
        return []

    explorer_type = LaunchConfiguration("frontier_explorer_type").perform(context)
    common_parameters = {
        "enabled": True,
        "map_topic": "/map",
        "robot_frame": "base_footprint",
        "global_frame": "map",
        "planner_event_topic": "/planner/events",
    }
    if explorer_type == "cluster":
        return [
            Node(
                package="yahboom_alc_bringup",
                executable="cluster_frontier_explorer",
                name="cluster_frontier_explorer",
                output="screen",
                parameters=[
                    {
                        **common_parameters,
                        "cycle_period_sec": 3.0,
                        "min_frontier_size": 14,
                        "min_clearance_cells": 2,
                        "path_clearance_cells": 2,
                        "min_frontier_distance": 0.45,
                        "max_frontier_distance": 3.5,
                        "frontier_goal_search_radius": 0.6,
                        "robot_start_search_radius": 2.0,
                        "information_radius": 1.5,
                        "path_weight": 1.0,
                        "information_weight": 0.02,
                        "frontier_size_weight": 0.02,
                        "unreachable_goal_penalty": 3.0,
                        "allow_unreachable_goal": False,
                        "goal_update_distance": 0.5,
                        "goal_publish_period": 3.0,
                        "footprint_front_m": 0.16,
                        "footprint_rear_m": 0.15,
                        "footprint_half_width_m": 0.11,
                        "footprint_padding_m": 0.03,
                        "swept_step_m": 0.05,
                        "unknown_overlap_max_ratio": 0.70,
                        "unknown_overlap_weight": 4.0,
                        "wall_clearance_m": 0.22,
                        "corner_penalty_weight": 1.0,
                        "corridor_progress_weight": 0.4,
                        "heading_penalty_weight": 1.2,
                        "heading_penalty_max_rad": 3.141592653589793,
                        "goal_timeout_sec": 35.0,
                        "stuck_progress_radius_m": 0.08,
                        "stuck_progress_timeout_sec": 9.0,
                        "near_goal_hysteresis_radius_m": 0.52,
                        "near_goal_retry_limit": 1,
                        "near_goal_hysteresis_sec": 12.0,
                        "alignment_grace_enabled": True,
                        "alignment_heading_threshold_rad": 0.35,
                        "alignment_max_grace_sec": 12.0,
                        "alignment_min_rotation_delta_rad": 0.05,
                        "failed_goal_blacklist_radius_m": 0.45,
                        "failed_goal_blacklist_sec": 60.0,
                        "failed_goal_max_retries": 2,
                        "enable_subgoals": True,
                        "max_subgoals": 3,
                        "subgoal_path_tolerance_m": 0.30,
                        "min_subgoal_spacing_m": 0.45,
                        "nav2_validation_enabled": True,
                        "nav2_validation_timeout_sec": 2.0,
                        "nav2_compute_path_action": "/compute_path_to_pose",
                        "marker_topic": "/waypoints",
                        "goal_marker_topic": "/frontier_goal_marker",
                    }
                ],
            )
        ]
    raise ValueError("frontier_explorer_type must be 'cluster'; simple was removed")


def generate_launch_description():
    yahboom_alc_share = get_package_share_directory("yahboom_alc_bringup")
    yahboom_bringup_share = get_package_share_directory("yahboomcar_bringup")
    yahboom_description_share = get_package_share_directory("yahboomcar_description")
    yahboom_nav_share = get_package_share_directory("yahboomcar_nav")
    astra_camera_share = get_package_share_directory("astra_camera")

    default_nav2_params = os.path.join(
        yahboom_alc_share, "params", "nav2_rgbd_params.yaml"
    )
    default_nav2_bt_xml = os.path.join(
        yahboom_alc_share,
        "behavior_trees",
        "navigate_to_pose_rgbd.xml",
    )
    default_model_path = os.path.join(
        yahboom_description_share, "urdf", "yahboomcar_R2.urdf.xacro"
    )
    default_rviz_config = os.path.join(yahboom_nav_share, "rviz", "rtabmap_nav.rviz")
    imu_filter_config = os.path.join(
        yahboom_bringup_share, "param", "imu_filter_param.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    qos = LaunchConfiguration("qos")
    localization = LaunchConfiguration("localization")
    enable_camera = LaunchConfiguration("enable_camera")
    enable_frontier_explorer = LaunchConfiguration("enable_frontier_explorer")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    nav2_start_delay_sec = LaunchConfiguration("nav2_start_delay_sec")
    startup_motion_delay_sec = LaunchConfiguration("startup_motion_delay_sec")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_bt_xml = LaunchConfiguration("nav2_bt_xml")
    planner_log_dir = LaunchConfiguration("planner_log_dir")
    nav_cmd_vel_topic = LaunchConfiguration("nav_cmd_vel_topic")
    driver_cmd_vel_topic = LaunchConfiguration("driver_cmd_vel_topic")
    navigate_status_topic = LaunchConfiguration("navigate_status_topic")

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    ackman_driver_node = Node(
        package="yahboom_alc_bringup",
        executable="r2_safe_driver",
        output="screen",
        parameters=[
            {
                "driver_package": "yahboomcar_bringup",
                "driver_executable": "Ackman_driver_R2",
            }
        ],
    )

    base_node = Node(
        package="yahboomcar_base_node",
        executable="base_node_R2",
        output="screen",
        parameters=[
            {
                "pub_odom_tf": False,
                "linear_scale_x": 1.0,
                "linear_scale_y": 1.0,
            }
        ],
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        output="screen",
        parameters=[imu_filter_config],
    )

    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_localization"),
                "launch",
                "ekf_x1_x3_launch.py",
            )
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        arguments=["-d", rviz_config],
    )

    astra_camera = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(astra_camera_share, "launch", "astro_pro_plus.launch.xml")
        ),
        condition=IfCondition(enable_camera),
        launch_arguments={
            "camera_name": "camera",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_ir": "false",
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

    tf_warmup_delay_sec = LaunchConfiguration("tf_warmup_delay_sec")

    rtabmap_parameters = {
        "frame_id": "base_footprint",
        "use_sim_time": use_sim_time,
        "subscribe_rgbd": True,
        "subscribe_scan": False,
        "use_action_for_goal": True,
        "qos_image": qos,
        "qos_imu": qos,
        "wait_for_transform": 1.0,
        "Reg/Strategy": "0",
        "Reg/Force3DoF": "true",
        "RGBD/CreateOccupancyGrid": "true",
        "RGBD/NeighborLinkRefining": "True",
        "Grid/3D": "false",
        "Grid/NormalsSegmentation": "false",
        "Grid/RayTracing": "true",
        "Grid/RangeMin": "0.25",
        "Grid/RangeMax": "3.0",
        "Grid/MaxGroundHeight": "0.08",
        "Grid/MaxObstacleHeight": "1.2",
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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboom_alc_share, "launch", "r2_navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "bt_xml_filename": nav2_bt_xml,
            "autostart": "true",
            "use_composition": "False",
            "nav_cmd_vel_topic": nav_cmd_vel_topic,
        }.items(),
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
                "planner_event_topic": "/planner/events",
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
                "min_linear_speed": 0.08,
                "max_linear_speed": 0.11,
                "steering_gain": 1.25,
                "max_steering": 1.25,
                "linear_deadband": 0.001,
                "angular_deadband": 0.04,
                "max_linear_accel": 0.12,
                "max_steering_rate": 1.0,
                "assumed_command_period": 0.10,
                "arc_turn_when_stationary": True,
            }
        ],
    )

    planner_runtime_logger = Node(
        package="yahboom_alc_bringup",
        executable="planner_runtime_logger",
        name="planner_runtime_logger",
        output="screen",
        parameters=[
            {
                "log_dir": planner_log_dir,
                "event_topic": "/planner/events",
                "map_topic": "/map",
                "nav_cmd_topic": nav_cmd_vel_topic,
                "driver_cmd_topic": driver_cmd_vel_topic,
                "navigate_status_topic": navigate_status_topic,
                "global_frame": "map",
                "robot_frame": "base_footprint",
                "snapshot_period_sec": 2.0,
            }
        ],
    )

    r2_safety_stop = Node(
        package="yahboom_alc_bringup",
        executable="r2_safety_stop",
        name="r2_safety_stop",
        output="screen",
        parameters=[
            {
                "cmd_vel_topic": driver_cmd_vel_topic,
                "publish_rate_hz": 30.0,
                "stop_duration_sec": 4.0,
            }
        ],
    )

    stop_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[
                        os.path.join(
                            get_package_prefix("yahboom_alc_bringup"),
                            "lib",
                            "yahboom_alc_bringup",
                            "r2_emergency_stop",
                        )
                    ],
                    output="screen",
                ),
            ]
        )
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
                "gui",
                default_value="false",
                description="Use joint_state_publisher_gui if true.",
            ),
            DeclareLaunchArgument(
                "model",
                default_value=default_model_path,
                description="Absolute path to the Yahboom R2 URDF xacro.",
            ),
            DeclareLaunchArgument(
                "enable_frontier_explorer",
                default_value="true",
                description="Automatically send frontier goals to Nav2.",
            ),
            DeclareLaunchArgument(
                "frontier_explorer_type",
                default_value="cluster",
                description="Frontier explorer implementation. Only cluster is supported.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz2 for remote visualization.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz2 config file.",
            ),
            DeclareLaunchArgument(
                "tf_warmup_delay_sec",
                default_value="3.0",
                description="Delay SLAM startup to let the TF tree stabilize on Jetson.",
            ),
            DeclareLaunchArgument(
                "nav2_start_delay_sec",
                default_value="8.0",
                description="Delay Nav2 startup until RTAB-Map has time to publish map and TF.",
            ),
            DeclareLaunchArgument(
                "startup_motion_delay_sec",
                default_value="12.0",
                description="Delay goal-producing nodes so the robot remains stopped after launch.",
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
                "nav2_bt_xml",
                default_value=default_nav2_bt_xml,
                description="Nav2 NavigateToPose BT XML used for RGB-D exploration.",
            ),
            DeclareLaunchArgument(
                "planner_log_dir",
                default_value="/home/jetson/SLAMPlanner/planner-log",
                description="Directory for sparse planner runtime JSONL logs.",
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
            DeclareLaunchArgument(
                "navigate_status_topic",
                default_value="/navigate_to_pose/_action/status",
                description="NavigateToPose action status topic observed by the runtime logger.",
            ),
            joint_state_publisher,
            robot_state_publisher,
            ackman_driver_node,
            base_node,
            imu_filter_node,
            ekf_node,
            rviz_node,
            OpaqueFunction(function=launch_lidar),
            astra_camera,
            TimerAction(
                period=tf_warmup_delay_sec,
                actions=[
                    rgbd_sync,
                    rtabmap_mapping,
                    rtabmap_localization,
                ],
            ),
            TimerAction(period=nav2_start_delay_sec, actions=[nav2]),
            r2_bridge,
            r2_safety_stop,
            planner_runtime_logger,
            TimerAction(
                period=startup_motion_delay_sec,
                actions=[
                    alc_planner,
                    OpaqueFunction(function=launch_frontier_explorer),
                ],
            ),
            stop_on_shutdown,
        ]
    )
