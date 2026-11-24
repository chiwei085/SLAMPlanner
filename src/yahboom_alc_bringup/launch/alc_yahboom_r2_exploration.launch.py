import json
import os
import sys

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
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rtabmap_common  # noqa: E402  (sibling module, see its docstring)

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
    "local_inflation_radius": 0.45,
    "global_inflation_radius": 0.55,
    "local_inflation_cost_scaling_factor": 3.0,
    "global_inflation_cost_scaling_factor": 3.5,
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
        "footprint_free_cost_max": 99,
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
                    # Same radius as Smac minimum_turning_radius and the
                    # bridge: goals inside a turning circle need a reverse cusp.
                    "min_turning_radius_m": 0.50,
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
                    "goal_update_distance": 0.5,
                    "goal_publish_period": 3.0,
                    **ROBOT_FOOTPRINT_PARAMS,
                    "swept_step_m": 0.05,
                    "unknown_overlap_max_ratio": 0.70,
                    "unknown_overlap_weight": 1.5,
                    "wall_clearance_m": 0.28,
                    "min_wall_clearance_m": 0.0,
                    # Hard goal-clearance floor. The global costmap now uses a
                    # weighted inflation gradient (radius 0.55, scaling 3.5):
                    # only the inscribed band (~0.135 m) is lethal, and beyond
                    # that cost decays exponentially. 0.22 m keeps goals below
                    # the planner's analytic_expansion_max_cost band (~200 at
                    # d<0.2 m) while not rejecting reachable near-wall goals in
                    # the non-lethal gradient. Kept in force under relaxed search.
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
                    # Replaced at startup by atan(w / 2 fx) from the Astra
                    # depth camera_info (the narrower camera, so the view
                    # holds for both).
                    "viewpoint_half_fov_rad": 0.5,
                    "view_camera_info_topic": "/camera/depth/camera_info",
                    # Frontier goals are approached along a straight tail
                    # (ViewApproach planner, navigate_to_view_rgbd.xml). The
                    # length must equal ViewApproach tail_length; the stop
                    # offset is the goal checker xy_goal_tolerance.
                    "view_tail_length_m": 1.40,
                    "view_stop_offset_m": 0.35,
                    "nav2_view_behavior_tree": os.path.join(
                        get_package_share_directory("yahboom_alc_bringup"),
                        "behavior_trees",
                        "navigate_to_view_rgbd.xml",
                    ),
                    "nav2_view_planner_id": "ViewApproach",
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
                    "hazard_region_enabled": True,
                    "hazard_region_severity1_radius_m": 0.6,
                    "hazard_region_severity2_radius_m": 1.0,
                    "hazard_region_ttl_sec": 600.0,
                    "hazard_region_path_penalty": 100.0,
                    "hazard_region_safe_traversal_clear_enabled": True,
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
    camera_mode = LaunchConfiguration("camera_mode")
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

    # Low D455 cloud for the costmaps / collision monitor: the high Astra can
    # miss floor-level obstacles right in front of the robot.
    d455_cloud_preprocessor = Node(
        package="yahboom_alc_bringup",
        executable="depth_cloud_preprocessor",
        name="d455_cloud_preprocessor",
        output="screen",
        condition=IfCondition(PythonExpression(["'", camera_mode, "' == 'dual'"])),
        parameters=[
            {
                "input_topic": "/realsense/d455/depth/color/points",
                "output_topic": "/realsense/d455/depth/points_voxel_base",
                "target_frame": "base_footprint",
                "leaf_size_m": 0.05,
                "transform_timeout_sec": 0.2,
                "min_range_m": 0.2,
                "max_range_m": 6.0,
                "use_sim_time": use_sim_time,
            }
        ],
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
                "rotation_risk_enabled": True,
                "rotation_risk_weight": 0.1,
                "rotation_risk_reference_det": 1.0e-6,
                "rotation_risk_max_lambda": 2.0,
                "rotation_risk_max_yaw_rad": 3.141592653589793,
                # rtabmap runs without a namespace here.
                "map_data_topic": "/mapData",
                "info_topic": "/info",
                "map_topic": "/map",
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
                "feedback_imu_topic": "/imu/data_raw",
                "feedback_frozen_timeout_sec": 0.5,
                "max_linear_speed": 0.11,
                # linear.y is the front wheel angle in degrees / 1000 (R2
                # firmware); the bridge converts curvature with these.
                "wheelbase_m": 0.2353,
                "min_turning_radius_m": 0.50,
                "max_curvature": 2.0,
                "max_steering_angle_rad": 0.6,
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
                # Was False (8d610ce, "suppress zombie frontier zones") to
                # stop unknown cells near frontiers from blocking recovery
                # backups entirely. That traded blocked-recovery stalls for
                # blind reversing into never-observed space, which is what
                # let BackUp goals reverse straight into unseen obstacles at
                # the frontier edge. Restored to True now that the
                # depth_voxel_layer sensor_frame fix (nav2_rgbd_params.yaml)
                # gives accurate raytrace clearing again, so "unknown" behind
                # the robot means genuinely unobserved, not just stale.
                "reverse_unknown_is_blocked": True,
                # A cell that flips to unknown the instant it falls out of the
                # rolling costmap's observation window (camera FOV / raytrace
                # persistence), but was seen clearly free within this many
                # seconds, is treated as free rather than blocked -- this is
                # what lets a spin-in-place recovery actually clear a
                # reverse_blocked latch instead of re-triggering it forever.
                "reverse_recent_free_grace_sec": 2.0,
                # Deliberately the opposite default of reverse_unknown_is_blocked:
                # this gates the stationary in-place turn that runs at the start
                # of nearly every frontier goal, and a frontier goal always
                # borders unknown space by definition. Blocking on unknown ahead
                # here stalled the robot at (0,0) for entire runs instead of just guarding the rare
                # blind-reverse case reverse_unknown_is_blocked targets.
                "stationary_arc_unknown_is_blocked": False,
                **ROBOT_FOOTPRINT_PARAMS,
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
                # Commanded forward (driver output) but the EKF odom did not
                # move: no camera input, see r2_stall_detector.hpp.
                "driver_cmd_topic": driver_cmd_vel_topic,
                "odom_topic": "/odom",
                "imu_topic": "/imu/data",
                "planner_event_topic": "/planner/events",
                "command_vx_threshold_mps": 0.05,
                # Judge a stall only after the driver commands add up to 6 cm
                # (3x the ~2 cm pose noise); moving < 30% of that is stuck.
                # From rest the chassis itself takes up to ~1.45 s to move
                # 1 cm (20 bridge-less free-floor starts, 2026-11-21), so the
                # first 1.5 s of commands are not counted.
                "min_expected_motion_m": 0.06,
                "stuck_motion_ratio": 0.3,
                "start_response_delay_sec": 1.5,
                "imu_yaw_rate_threshold_radps": 0.08,
                "stale_observation_timeout_sec": 0.5,
                "zero_burst_duration_sec": 0.5,
                "retrigger_cooldown_sec": 3.0,
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
                "global_costmap_topic": "/global_costmap/costmap",
                "wheel_odom_topic": "/odom",
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
                            "footprint_free_cost_max": 99,
                            "reverse_check_distance_m": 0.12,
                            "reverse_safety_margin_m": 0.0,
                            "reverse_unknown_is_blocked": True,
                            "reverse_recent_free_grace_sec": 2.0,
                            "stationary_arc_unknown_is_blocked": False,
                            "min_turning_radius_m": 0.50,
                            "max_curvature": 2.0,
                        },
                        "stall_detector": {
                            "enabled": True,
                            "input": "driver cmd vs EKF odom + gyro, no camera",
                            "command_vx_threshold_mps": 0.05,
                            "min_expected_motion_m": 0.06,
                            "stuck_motion_ratio": 0.3,
                            "start_response_delay_sec": 1.5,
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
