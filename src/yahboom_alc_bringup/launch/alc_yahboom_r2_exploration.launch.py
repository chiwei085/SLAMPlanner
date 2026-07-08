import json
import os

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
from launch.substitutions import Command, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Single source for the R2 chassis footprint used by both the frontier
# explorer and the cmd_vel bridge. Keep in sync with the footprint string in
# params/nav2_rgbd_params.yaml.
ROBOT_FOOTPRINT_PARAMS = {
    "footprint_front_m": 0.19,
    "footprint_rear_m": 0.19,
    "footprint_half_width_m": 0.115,
    "footprint_padding_m": 0.02,
}

NAV2_RUNTIME_METADATA = {
    "planner_plugin": "nav2_smac_planner/SmacPlannerHybrid",
    "planner_motion_model": "REEDS_SHEPP",
    "planner_minimum_turning_radius": 0.50,
    "planner_reverse_penalty": 1.8,
    "planner_change_penalty": 0.0,
    "controller_plugin": (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    ),
    "controller_allow_reversing": True,
    "controller_use_rotate_to_heading": False,
    "local_inflation_radius": 0.29,
    "global_inflation_radius": 0.16,
    "global_depth_voxel_layer": True,
    "depth_cloud_preprocessor_leaf_size_m": 0.05,
    "depth_cloud_preprocessor_target_frame": "base_footprint",
    "depth_cloud_preprocessor_min_range_m": 0.2,
    "depth_cloud_preprocessor_max_range_m": 6.0,
    "depth_camera_fps": 30,
    "local_depth_min_obstacle_height": 0.01,
    "local_depth_observation_persistence": 0.4,
    "local_footprint_padding": 0.02,
    "global_footprint_padding": 0.02,
    "costmap_transform_tolerance": 1.0,
    "controller_transform_tolerance": 1.0,
    "bt_navigator_transform_tolerance": 1.0,
    "recovery_stagnation_escalation_trees": 3,
    "bridge_reverse": {
        "enabled": True,
        "allowed_cost_max": 98,
        "unknown_is_blocked": True,
        "check_distance_m": 0.12,
        "safety_margin_m": 0.0,
        "stationary_arc_requires_forward_clearance": True,
        "stationary_arc_check_distance_m": 0.12,
    },
}


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

    common_parameters = {
        "enabled": True,
        "map_topic": "/map",
        "robot_frame": "base_footprint",
        "global_frame": "map",
        "planner_event_topic": "/planner/events",
        "nav2_success_xy_tolerance_m": 0.55,
    }
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
                    "exploratory_path_clearance_cells": 1,
                    "min_frontier_distance": 0.45,
                    "max_frontier_distance": 5.5,
                    "frontier_goal_search_radius": 0.6,
                    "robot_start_search_radius": 2.0,
                    "information_radius": 1.5,
                    "path_weight": 0.65,
                    "information_weight": 0.15,
                    "frontier_size_weight": 0.02,
                    "unreachable_goal_penalty": 3.0,
                    "soft_unreachable_penalty": 1.5,
                    "allow_unreachable_goal": False,
                    "nav2_validation_required_for_final_goal": True,
                    "approach_offset_m": 0.4,
                    "goal_update_distance": 0.5,
                    "goal_publish_period": 3.0,
                    **ROBOT_FOOTPRINT_PARAMS,
                    "swept_step_m": 0.05,
                    "unknown_overlap_max_ratio": 0.70,
                    "unknown_overlap_weight": 1.5,
                    "wall_clearance_m": 0.28,
                    "min_wall_clearance_m": 0.0,
                    # Hard goal-clearance floor tracking the Nav2 global costmap
                    # inflation_radius (0.16). Goals closer than this to an
                    # obstacle sit in Nav2's inflated band and get aborted with
                    # global_path_blocked; filtering them here stops the explorer
                    # from recommending unplannable near-wall goals and
                    # poisoning the blacklist. Kept in force under relaxed search.
                    "nav2_goal_clearance_m": 0.22,
                    # Stage 2 viewpoint / next-best-view scoring. Enabled at 0.1
                    # after run_20260624_210848 validation: that run logged 10
                    # best_candidates with visible_unknown=0 despite information
                    # >200 (goals the camera physically cannot see past walls),
                    # and weight ~0.05 already flips the worst blind cases, so 0.1
                    # demotes them with margin while leaving information a say.
                    # Revert by setting back to 0.0. range/fov match the Astra
                    # depth camera (HFOV ~58 deg -> half ~0.5 rad).
                    "viewpoint_information_weight": 0.1,
                    "viewpoint_range_m": 3.0,
                    "viewpoint_half_fov_rad": 0.5,
                    # Stage 3 coverage revisit. With RangeMax 5.0 the map is
                    # painted from far away, so distant rooms become known before
                    # the robot drives near them and exploration stops caring.
                    # This sends the robot to the nearest reachable known-free
                    # region it has never approached within coverage_proximity_m,
                    # but ONLY when frontier search (incl. relaxed) finds nothing
                    # - a pure fallback that cannot affect active exploration, so
                    # enabling it is regression-safe. Disable by setting
                    # coverage_revisit_enabled False.
                    "coverage_revisit_enabled": True,
                    "coverage_proximity_m": 1.2,
                    "coverage_sample_spacing_m": 0.4,
                    "coverage_max_targets": 12,
                    # Coverage blend (F1/F3). When > 0, unvisited known-free
                    # regions compete directly with frontier candidates instead
                    # of being a pure fallback. A value around 1.0-2.0 offsets
                    # the typical information_weight * unknown_count advantage
                    # that frontier candidates have. 0 = fallback-only (safe
                    # default, identical to previous behaviour).
                    "coverage_blend_weight": 1.5,
                    # Intention commitment (F9). When
                    # intention_distance_threshold_m > 0, selecting a goal
                    # farther than this distance locks in a strategic intention.
                    # Candidates within intention_clear_radius_m * 2 of that
                    # goal receive an intention_continuation_bonus score
                    # reduction in subsequent cycles, preventing nearby
                    # opportunities from perpetually outbidding the committed
                    # long-range target. intention_timeout_sec clears stale
                    # intentions (0 = never expire). All disabled by default.
                    "intention_distance_threshold_m": 3.0,
                    "intention_continuation_bonus": 2.0,
                    "intention_clear_radius_m": 1.5,
                    "intention_timeout_sec": 35.0,
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
                    "alignment_heading_threshold_rad": 0.55,
                    "alignment_max_grace_sec": 8.0,
                    "alignment_min_rotation_delta_rad": 0.12,
                    "alignment_spin_enabled": False,
                    "alignment_spin_max_attempts": 1,
                    "alignment_spin_time_allowance_sec": 8.0,
                    "no_frontier_relax_sec": 30.0,
                    "relaxed_min_frontier_size": 8,
                    "relaxed_max_frontier_distance": 5.0,
                    "failed_goal_blacklist_radius_m": 0.35,
                    "failed_goal_blacklist_sec": 600.0,
                    "failed_goal_max_retries": 2,
                    "enable_subgoals": True,
                    "max_subgoals": 3,
                    "subgoal_path_tolerance_m": 0.30,
                    "min_subgoal_spacing_m": 0.45,
                    "nav2_validation_enabled": True,
                    "nav2_validation_timeout_sec": 2.0,
                    "nav2_compute_path_action": "/compute_path_to_pose",
                    "nav2_validation_blacklist_enabled": True,
                    "backtrace_recovery_enabled": True,
                    "backtrace_breadcrumb_max_distance_m": 2.0,
                    "backtrace_breadcrumb_max_age_sec": 90.0,
                    "backtrace_backup_step_m": 0.10,
                    "backtrace_backup_speed_mps": 0.08,
                    "backtrace_backup_time_allowance_sec": 5.0,
                    "backtrace_max_segments": 4,
                    "backtrace_min_actual_displacement_m": 0.03,
                    "backtrace_cancel_timeout_sec": 1.0,
                    "backtrace_backup_stall_check_sec": 1.0,
                    "backtrace_reverse_motion_ratio_min": 0.6,
                    "backtrace_backup_action": "/backup",
                    "recovery_reobserve_cycles": 1,
                    "recovery_small_spin_yaw_rad": 0.349066,
                    # Reduced from 2 → 1: each stationary spin generates fake
                    # occupied cells via yaw odometry error + depth misalignment.
                    # One attempt preserves basic reobservation; the backtrace
                    # recovery (沿軌跡撤退) is the preferred escape mechanism.
                    "recovery_small_spin_attempts": 1,
                    "recovery_forward_escape_distance_m": 1.5,
                    "rear_blocked_min_latch_sec": 15.0,
                    "rear_blocked_clear_forward_displacement_m": 0.25,
                    "recovery_clearance_backup_m": 0.30,
                    # Cross-tree stagnation escalation: the explorer re-sends a
                    # goal after every reobserve, which resets the recovery
                    # tree (and its spin counter), so the backtrace stage was
                    # unreachable — 4 runs on 2026-07-07 show backtrace=0
                    # despite 18-47 recovery trees each.  When 3 consecutive
                    # trees start with the robot inside a 0.25 m circle, jump
                    # straight to the reverse-pulse/backtrace stage.
                    "recovery_stagnation_escalation_trees": 3,
                    "recovery_stagnation_radius_m": 0.25,
                    "marker_topic": "/waypoints",
                    "goal_marker_topic": "/frontier_goal_marker",
                }
            ],
        )
    ]


def generate_launch_description():
    yahboom_alc_share = get_package_share_directory("yahboom_alc_bringup")
    yahboom_bringup_share = get_package_share_directory("yahboomcar_bringup")
    yahboom_description_share = get_package_share_directory("yahboomcar_description")
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
    enable_frontier_explorer = LaunchConfiguration("enable_frontier_explorer")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    nav2_start_delay_sec = LaunchConfiguration("nav2_start_delay_sec")
    startup_motion_delay_sec = LaunchConfiguration("startup_motion_delay_sec")
    camera_pointcloud_start_delay_sec = LaunchConfiguration(
        "camera_pointcloud_start_delay_sec"
    )
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_bt_xml = LaunchConfiguration("nav2_bt_xml")
    planner_log_dir = LaunchConfiguration("planner_log_dir")
    nav_cmd_vel_topic = LaunchConfiguration("nav_cmd_vel_topic")
    driver_cmd_vel_topic = LaunchConfiguration("driver_cmd_vel_topic")
    navigate_status_topic = LaunchConfiguration("navigate_status_topic")
    ekf_launch_prefix = LaunchConfiguration("ekf_launch_prefix")
    camera_color_fps = LaunchConfiguration("camera_color_fps")
    camera_depth_fps = LaunchConfiguration("camera_depth_fps")
    camera_ir_fps = LaunchConfiguration("camera_ir_fps")

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
        parameters=[
            os.path.join(yahboom_alc_share, "params", "ekf_r2.yaml")
        ],
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

    astra_camera = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(astra_camera_share, "launch", "astro_pro_plus.launch.xml")
        ),
        condition=IfCondition(enable_camera),
        launch_arguments={
            "camera_name": "camera",
            "enable_color": "true",
            "enable_depth": "true",
            "color_fps": camera_color_fps,
            "depth_fps": camera_depth_fps,
            "ir_fps": camera_ir_fps,
            # These extrinsics are fixed. Publishing them on /tf from the
            # camera's streaming thread lets a USB scheduling stall make the
            # transform expire at an RGB-D timestamp. They are published
            # below on /tf_static instead.
            "publish_tf": "false",
            "enable_ir": "false",
            "enable_point_cloud": "true",
            "enable_colored_point_cloud": "false",
        }.items(),
    )

    # Astra Pro Plus factory extrinsics observed from the camera driver. Keep
    # this immutable chain on /tf_static: camera_link is attached to the R2
    # URDF, while these links describe fixed depth/color sensor geometry.
    camera_static_transforms = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_link_to_depth_frame",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "camera_link",
                "--child-frame-id", "camera_depth_frame",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_depth_to_color_frame",
            arguments=[
                "--x", "-0.002165860891342163", "--y", "0.025120487213134767",
                "--z", "0.00009495562314987182",
                "--qx", "-0.0008345809765160083", "--qy", "-0.00009292588947573677",
                "--qz", "-0.0009316844516433775", "--qw", "0.9999992251396179",
                "--frame-id", "camera_depth_frame",
                "--child-frame-id", "camera_color_frame",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_depth_to_optical_frame",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5",
                "--frame-id", "camera_depth_frame",
                "--child-frame-id", "camera_depth_optical_frame",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_color_to_optical_frame",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5",
                "--frame-id", "camera_color_frame",
                "--child-frame-id", "camera_color_optical_frame",
            ],
        ),
    ]

    depth_cloud_preprocessor = Node(
        package="yahboom_alc_bringup",
        executable="depth_cloud_preprocessor",
        name="depth_cloud_preprocessor",
        output="screen",
        condition=IfCondition(enable_camera),
        parameters=[
            {
                "input_topic": "/camera/depth/points",
                "output_topic": "/camera/depth/points_voxel_base",
                "target_frame": "base_footprint",
                "leaf_size_m": 0.05,
                "transform_timeout_sec": 0.2,
                # Matches the costmaps' obstacle_min_range (nav2_rgbd_params.yaml):
                # rejects invalid near-zero depth returns (common at camera
                # startup) here so collision_monitor's pointcloud source --
                # which has no range filter of its own -- is protected too.
                "min_range_m": 0.2,
                "max_range_m": 6.0,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        output="screen",
        parameters=[
            {
                "approx_sync": True,
                # Astra color/depth frames can arrive more than 10 ms apart on
                # Jetson, so a 10 ms cap dropped valid RGB-D pairs and caused
                # RTAB-Map conversion failures. But a fully unbounded interval
                # (0.0) lets the approximate-time policy pair frames that are
                # far apart, which trips RTAB-Map's "time difference between rgb
                # and depth frames is high" warning and feeds it badly registered
                # pairs. 50 ms (~1.5 frame periods at 30 fps) is generous enough
                # to keep genuine pairs while rejecting those spurious far
                # associations; the queue of 30 still supplies a better-aligned
                # pair when one is dropped.
                "approx_sync_max_interval": 0.05,
                "topic_queue_size": 30,
                "sync_queue_size": 30,
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
        # publish_tf is disabled: rtabmap_map_tf_relay handles map→odom TF at 50 Hz
        # using localization_pose (published from RTAB-Map's SLAM callback, outside
        # the graph-optimization mutex). This breaks the 1–2 s TF stall caused by
        # CoreWrapper::mapToOdomMutex_ contention under Jetson load.
        "publish_tf": False,
        "Reg/Strategy": "0",
        "Reg/Force3DoF": "true",
        "RGBD/CreateOccupancyGrid": "true",
        "RGBD/NeighborLinkRefining": "True",
        "Grid/3D": "false",
        "Grid/NormalsSegmentation": "false",
        "Grid/RayTracing": "true",
        "Grid/RangeMin": "0.25",
        "Grid/RangeMax": "5.0",
        "Grid/MaxGroundHeight": "0.08",
        "Grid/MaxObstacleHeight": "1.2",
        # Robot footprint radius used to force-clear cells the robot physically
        # occupies. Any pose-graph node whose footprint overlaps a cell marks it
        # as free unconditionally — the strongest possible clearing evidence since
        # it requires no sensor. Diagonal half-size of R2 body (0.19 front +
        # 0.115 half-width ≈ 0.22 m diagonal) plus a small margin.
        "Grid/FootprintRadius": "0.25",
        "subscribe_imu": True,
        "Optimizer/GravitySigma": "0.3",
    }

    rtabmap_remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
        ("odom", "/odom"),
        ("imu", "/imu/data"),
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
        parameters=[
            {
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "publish_rate_hz": 50.0,
                "stale_warn_sec": 2.0,
                "use_sim_time": use_sim_time,
            }
        ],
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
                "cE": 4.0,
                "tau_min_revisit": 4,
                "theta_max": 0.25,
                "lambda_decay": 0.03,
                "cl": 6.0,
                "plc_min_revisit": 0.02,
                "map_dist_min_revisit": 0.75,
                "navigation_timeout_sec": 120.0,
                "degraded_exploration_alc_bonus": 0.35,
                "degraded_exploration_window_sec": 60.0,
                "degraded_exploration_min_events": 3,
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
                "planner_event_topic": "/planner/events",
                "min_linear_speed": 0.08,
                "enforce_min_linear_speed": True,
                "safety_stop_topic": "/collision_safety_stop_active",
                "max_linear_speed": 0.11,
                "steering_gain": 1.25,
                "max_steering": 1.25,
                "wheelbase_m": 0.16,
                "min_turning_radius_m": 0.50,
                "max_curvature": 2.0,
                "max_steering_angle_rad": 1.25,
                "linear_deadband": 0.001,
                "angular_deadband": 0.005,
                "max_linear_accel": 0.12,
                "max_steering_rate": 1.0,
                "assumed_command_period": 0.10,
                "arc_turn_when_stationary": True,
                "stationary_arc_requires_forward_clearance": True,
                "stationary_arc_check_distance_m": 0.12,
                "reverse_safety_enabled": True,
                "reverse_costmap_topic": "/local_costmap/costmap",
                "reverse_robot_frame": "base_footprint",
                "reverse_costmap_max_age_sec": 1.0,
                "reverse_check_distance_m": 0.12,
                "reverse_safety_margin_m": 0.0,
                "reverse_allowed_cost_max": 98,
                "reverse_unknown_is_blocked": False,
                **ROBOT_FOOTPRINT_PARAMS,
            }
        ],
    )

    # IMU-impact reflex only. Smooth depth braking is now handled by the Nav2
    # collision monitor (slowdown + approach polygons in nav2_rgbd_params.yaml);
    # this node watches the IMU and raises /collision_safety_stop_active + a
    # stall_or_slip event when it detects a forward impact. It is no longer in
    # the cmd_vel path, so the bridge reads /alc/nav_cmd_vel directly.
    r2_collision_safety_gate = Node(
        package="yahboom_alc_bringup",
        executable="r2_collision_safety_gate",
        name="r2_collision_safety_gate",
        output="screen",
        parameters=[
            {
                "input_topic": nav_cmd_vel_topic,
                "imu_topic": "/imu/data",
                "planner_event_topic": "/planner/events",
                "safety_stop_topic": "/collision_safety_stop_active",
                "command_vx_threshold_mps": 0.01,
                "imu_stale_timeout_sec": 0.5,
                "imu_baseline_samples": 20,
                "imu_impact_decel_threshold_mps2": 1.5,
                "impact_stop_hold_sec": 0.5,
                "impact_cooldown_sec": 3.0,
            }
        ],
    )

    r2_stall_detector = Node(
        package="yahboom_alc_bringup",
        executable="r2_stall_detector",
        name="r2_stall_detector",
        output="screen",
        parameters=[
            {
                "nav_cmd_topic": nav_cmd_vel_topic,
                "driver_cmd_topic": driver_cmd_vel_topic,
                "wheel_odom_topic": "/odom",
                "imu_topic": "/imu/data",
                "depth_image_topic": "/camera/depth/image_raw",
                "planner_event_topic": "/planner/events",
                "slam_global_frame": "map",
                "robot_frame": "base_footprint",
                "forward_command_min_duration_sec": 0.5,
                "forward_no_motion_check_sec": 0.8,
                "command_vx_threshold_mps": 0.08,
                "wheel_odom_delta_threshold_m": 0.05,
                "slam_delta_max_m": 0.02,
                "imu_yaw_rate_threshold_radps": 0.08,
                "depth_obstacle_enabled": True,
                "depth_roi_center_width_ratio": 0.20,
                "depth_roi_center_height_ratio": 0.50,
                "depth_min_valid_m": 0.20,
                "depth_forward_obstacle_range_m": 0.60,
                "depth_min_obstacle_pixels": 5,
                "depth_stale_timeout_sec": 0.5,
                "stale_observation_timeout_sec": 0.5,
                "zero_burst_duration_sec": 0.5,
                "zero_burst_rate_hz": 30.0,
                "retrigger_cooldown_sec": 3.0,
                "depth_blind_slam_override_m": 0.010,
                "camera_front_offset_m": 0.10,
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
                "runtime_metadata_json": json.dumps(
                    {
                        **NAV2_RUNTIME_METADATA,
                        "bridge": {
                            "reverse_safety_enabled": True,
                            "reverse_allowed_cost_max": 98,
                            "reverse_check_distance_m": 0.12,
                            "reverse_safety_margin_m": 0.0,
                            "reverse_unknown_is_blocked": False,
                            "min_turning_radius_m": 0.50,
                            "max_curvature": 2.0,
                        },
                        "stall_detector": {
                            "enabled": True,
                            "forward_command_min_duration_sec": 0.5,
                            "command_vx_threshold_mps": 0.08,
                            "wheel_odom_delta_threshold_m": 0.05,
                            "slam_delta_max_m": 0.02,
                            "depth_blind_slam_override_m": 0.010,
                            "camera_front_offset_m": 0.10,
                        },
                        "recovery": {
                            "clearance_backup_m": 0.30,
                        },
                        "slam": {
                            "subscribe_imu": True,
                            "optimizer_gravity_sigma": 0.3,
                            "publish_tf": False,
                            "map_tf_relay_rate_hz": 50.0,
                        },
                        "footprint": ROBOT_FOOTPRINT_PARAMS,
                    }
                ),
                "nav2_params_file": nav2_params_file,
                "nav2_bt_xml": nav2_bt_xml,
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
            DeclareLaunchArgument(
                "ekf_launch_prefix",
                default_value="nice -n -10",
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
            astra_camera,
            *camera_static_transforms,
            TimerAction(
                period=camera_pointcloud_start_delay_sec,
                actions=[depth_cloud_preprocessor],
            ),
            TimerAction(
                period=tf_warmup_delay_sec,
                actions=[
                    rgbd_sync,
                    rtabmap_mapping,
                    rtabmap_localization,
                    rtabmap_map_tf_relay,
                ],
            ),
            TimerAction(period=nav2_start_delay_sec, actions=[nav2]),
            r2_collision_safety_gate,
            r2_bridge,
            r2_stall_detector,
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
