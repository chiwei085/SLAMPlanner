import json
import os
import sys

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtabmap_common  # noqa: E402  (sibling module, see its docstring)

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


def launch_planner_runtime_logger(context, *args, **kwargs):
    # Record the parameters this run really uses.
    params_files = {
        name: LaunchConfiguration(name).perform(context)
        for name in ("exploration_params_file", "nav2_params_file")
    }
    metadata = {}
    for name, path in params_files.items():
        with open(path) as stream:
            metadata[name.removesuffix("_file")] = yaml.safe_load(stream)
    return [
        Node(
            package="yahboom_alc_bringup",
            executable="planner_runtime_logger",
            name="planner_runtime_logger",
            output="screen",
            parameters=[
                params_files["exploration_params_file"],
                {
                    "log_dir": LaunchConfiguration("planner_log_dir"),
                    "nav_cmd_topic": LaunchConfiguration("nav_cmd_vel_topic"),
                    "driver_cmd_topic": LaunchConfiguration("driver_cmd_vel_topic"),
                    "navigate_status_topic": LaunchConfiguration(
                        "navigate_status_topic"
                    ),
                    "runtime_metadata_json": json.dumps(metadata),
                    "nav2_params_file": params_files["nav2_params_file"],
                    "nav2_bt_xml": LaunchConfiguration("nav2_bt_xml"),
                },
            ],
        )
    ]


def generate_launch_description():
    yahboom_alc_share = get_package_share_directory("yahboom_alc_bringup")
    yahboom_bringup_share = get_package_share_directory("yahboomcar_bringup")
    yahboom_description_share = get_package_share_directory("yahboomcar_description")

    default_nav2_params = os.path.join(
        yahboom_alc_share, "params", "nav2_rgbd_params.yaml"
    )
    default_nav2_bt_xml = os.path.join(
        yahboom_alc_share,
        "behavior_trees",
        "navigate_to_pose_rgbd.xml",
    )
    # ALC and frontier goals arrive facing their view (straight tail).
    view_bt_xml = os.path.join(
        yahboom_alc_share,
        "behavior_trees",
        "navigate_to_view_rgbd.xml",
    )
    default_model_path = os.path.join(
        yahboom_description_share, "urdf", "yahboomcar_R2.urdf.xacro"
    )
    default_rviz_config = os.path.join(
        yahboom_alc_share, "rviz", "alc_r2_exploration.rviz"
    )
    imu_filter_config = os.path.join(
        yahboom_bringup_share, "param", "imu_filter_param.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    qos = LaunchConfiguration("qos")
    localization = LaunchConfiguration("localization")
    enable_camera = LaunchConfiguration("enable_camera")
    camera_mode = LaunchConfiguration("camera_mode")
    enable_frontier_explorer = LaunchConfiguration("enable_frontier_explorer")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    nav2_start_delay_sec = LaunchConfiguration("nav2_start_delay_sec")
    startup_motion_delay_sec = LaunchConfiguration("startup_motion_delay_sec")
    camera_pointcloud_start_delay_sec = LaunchConfiguration(
        "camera_pointcloud_start_delay_sec"
    )
    exploration_params_file = LaunchConfiguration("exploration_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_bt_xml = LaunchConfiguration("nav2_bt_xml")
    nav_cmd_vel_topic = LaunchConfiguration("nav_cmd_vel_topic")
    driver_cmd_vel_topic = LaunchConfiguration("driver_cmd_vel_topic")
    ekf_launch_prefix = LaunchConfiguration("ekf_launch_prefix")

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

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        prefix=ekf_launch_prefix,
        parameters=[rtabmap_common.ekf_params_file()],
        remappings=[("/odometry/filtered", "/odom")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        arguments=["-d", rviz_config],
    )

    # camera_mode:=dual -> high Astra + low D455 merged on /rgbd_images;
    # camera_mode:=astra -> original low Astra only, on /astra/rgbd_image.
    # Inherits this file's launch configurations (fps, qos, mount poses).
    dual_rgbd_cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboom_alc_share, "launch", "dual_rgbd_cameras.launch.py")
        ),
        launch_arguments={
            "sync_start_delay_sec": LaunchConfiguration("tf_warmup_delay_sec"),
        }.items(),
    )

    depth_cloud_preprocessor = Node(
        package="yahboom_alc_bringup",
        executable="depth_cloud_preprocessor",
        name="depth_cloud_preprocessor",
        output="screen",
        condition=IfCondition(enable_camera),
        parameters=[exploration_params_file, {"use_sim_time": use_sim_time}],
    )

    # Low D455 cloud for the costmaps / collision monitor: the high Astra can
    # miss floor-level obstacles right in front of the robot.
    d455_cloud_preprocessor = Node(
        package="yahboom_alc_bringup",
        executable="depth_cloud_preprocessor",
        name="d455_cloud_preprocessor",
        output="screen",
        condition=IfCondition(PythonExpression(["'", camera_mode, "' == 'dual'"])),
        parameters=[exploration_params_file, {"use_sim_time": use_sim_time}],
    )

    tf_warmup_delay_sec = LaunchConfiguration("tf_warmup_delay_sec")

    # Shared with teleop mapping so both modes build maps the same way.
    rtabmap_mapping, rtabmap_localization = rtabmap_common.rtabmap_nodes(
        rtabmap_common.rtabmap_parameters(
            camera_mode, qos, use_sim_time, publish_tf=False
        ),
        localization,
    )

    # Publishes map→odom TF at 50 Hz, decoupled from RTAB-Map's graph-optimization
    # mutex. It publishes identity map→odom until /localization_pose first arrives,
    # then caches the derived transform. During an optimization spike the relay keeps
    # republishing the last known transform with a fresh timestamp, so Nav2 never
    # sees a TF extrapolation gap.
    rtabmap_map_tf_relay = Node(
        package="yahboom_alc_bringup",
        executable="rtabmap_map_tf_relay",
        name="rtabmap_map_tf_relay",
        output="screen",
        parameters=[exploration_params_file, {"use_sim_time": use_sim_time}],
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
        parameters=[exploration_params_file, {"nav_behavior_tree": view_bt_xml}],
    )

    frontier_explorer = Node(
        package="yahboom_alc_bringup",
        executable="cluster_frontier_explorer",
        name="cluster_frontier_explorer",
        output="screen",
        condition=IfCondition(enable_frontier_explorer),
        parameters=[
            exploration_params_file,
            {
                "nav2_view_behavior_tree": view_bt_xml,
            },
        ],
    )

    r2_bridge = Node(
        package="yahboom_alc_bringup",
        executable="yahboom_r2_cmd_vel_bridge",
        name="yahboom_r2_cmd_vel_bridge",
        output="screen",
        parameters=[
            exploration_params_file,
            {
                "input_topic": nav_cmd_vel_topic,
                "output_topic": driver_cmd_vel_topic,
            },
        ],
    )

    r2_stall_detector = Node(
        package="yahboom_alc_bringup",
        executable="r2_stall_detector",
        name="r2_stall_detector",
        output="screen",
        parameters=[
            exploration_params_file,
            {
                "driver_cmd_topic": driver_cmd_vel_topic,
            },
        ],
    )

    r2_safety_stop = Node(
        package="yahboom_alc_bringup",
        executable="r2_safety_stop",
        name="r2_safety_stop",
        output="screen",
        parameters=[exploration_params_file, {"cmd_vel_topic": driver_cmd_vel_topic}],
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
            # Fast DDS shared-memory transport is vulnerable to stale
            # /dev/shm fastrtps_port locks after an interrupted launch. A
            # failed SHM port can make local graph discovery intermittent,
            # leaving RTAB-Map alive but unsubscribed from rgbd_sync. Use the
            # loopback UDP transport for this all-local robot graph instead.
            SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
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
                "camera_mode",
                default_value="dual",
                description=(
                    "dual = high Astra + low D455; astra = original single low "
                    "Astra. Mount poses: astra_{x,y,z,roll,pitch,yaw} / "
                    "d455_{...} (dual_rgbd_cameras.launch.py)."
                ),
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
                "camera_pointcloud_start_delay_sec",
                default_value="6.0",
                description=(
                    "Delay raw depth point-cloud consumers until the Astra "
                    "driver has finished opening depth/color streams."
                ),
            ),
            DeclareLaunchArgument(
                "startup_motion_delay_sec",
                default_value="12.0",
                description="Delay goal-producing nodes so the robot remains stopped after launch.",
            ),
            DeclareLaunchArgument(
                "rplidar_type",
                default_value="none",
                description="Optional RPLIDAR type passed to laser bringup: a1, s2, 4ROS, or none.",
            ),
            DeclareLaunchArgument(
                "exploration_params_file",
                default_value=os.path.join(
                    yahboom_alc_share, "params", "exploration.yaml"
                ),
                description="Parameter YAML of the explorer, bridge, and ALC planner.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params,
                description=(
                    "Nav2 parameter YAML. Use this to tune the R2 planner, "
                    "controller, and behavior server."
                ),
            ),
            DeclareLaunchArgument(
                "nav2_bt_xml",
                default_value=default_nav2_bt_xml,
                description="Nav2 NavigateToPose BT XML used for RGB-D exploration.",
            ),
            DeclareLaunchArgument(
                "planner_log_dir",
                default_value=os.path.expanduser("~/SLAMPlanner/planner-log"),
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
            DeclareLaunchArgument(
                "ekf_launch_prefix",
                default_value=rtabmap_common.EKF_LAUNCH_PREFIX,
                description=(
                    "Prefix for ekf_filter_node scheduling priority. Systems "
                    "with CAP_SYS_NICE may override with 'chrt -f 80'."
                ),
            ),
            DeclareLaunchArgument(
                "camera_color_fps",
                default_value="30",
                description=(
                    "Astra color stream FPS. Keep 30 by default because this "
                    "camera's UVC color mode rejects 640x480@15 and can crash "
                    "the driver."
                ),
            ),
            DeclareLaunchArgument(
                "camera_depth_fps",
                default_value="30",
                description=(
                    "Astra depth stream FPS. Keep 30 by default because "
                    "640x480@15 is not advertised by the observed depth mode."
                ),
            ),
            DeclareLaunchArgument(
                "camera_ir_fps",
                default_value="30",
                description=(
                    "Astra IR stream FPS. IR is disabled here, but keep the "
                    "argument available for deployments that enable it."
                ),
            ),
            joint_state_publisher,
            robot_state_publisher,
            ackman_driver_node,
            base_node,
            imu_filter_node,
            ekf_node,
            rviz_node,
            OpaqueFunction(function=launch_lidar),
            dual_rgbd_cameras,
            TimerAction(
                period=camera_pointcloud_start_delay_sec,
                actions=[depth_cloud_preprocessor, d455_cloud_preprocessor],
            ),
            TimerAction(
                period=tf_warmup_delay_sec,
                actions=[
                    rtabmap_mapping,
                    rtabmap_localization,
                    rtabmap_map_tf_relay,
                ],
            ),
            TimerAction(period=nav2_start_delay_sec, actions=[nav2]),
            r2_bridge,
            r2_stall_detector,
            r2_safety_stop,
            OpaqueFunction(function=launch_planner_runtime_logger),
            TimerAction(
                period=startup_motion_delay_sec,
                actions=[
                    alc_planner,
                    frontier_explorer,
                ],
            ),
            stop_on_shutdown,
        ]
    )
