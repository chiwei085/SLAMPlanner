import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("yahboom_r2_teleop")
    bringup_share = get_package_share_directory("yahboomcar_bringup")
    description_share = get_package_share_directory("yahboomcar_description")
    alc_share = get_package_share_directory("yahboom_alc_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")
    rviz = LaunchConfiguration("rviz")
    qos = LaunchConfiguration("qos")
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    common_rtabmap_parameters = {
        "frame_id": "base_footprint",
        "use_sim_time": use_sim_time,
        "subscribe_rgbd": True,
        # dual: 0 = /rgbd_images (rgbdx_sync); astra: 1 = /astra/rgbd_image.
        "rgbd_cameras": ParameterValue(
            PythonExpression(["1 if '", LaunchConfiguration("camera_mode"), "' == 'astra' else 0"]),
            value_type=int,
        ),
        "odom_sensor_sync": True,
        "subscribe_scan": False,
        "subscribe_imu": True,
        "qos_image": qos,
        "qos_imu": qos,
        "wait_for_transform": 1.0,
        "publish_tf": True,
        "Reg/Strategy": "0",
        # Multi-camera PnP needs OpenGV (not in this RTAB-Map build): dual uses 3D-3D.
        "Vis/EstimationType": PythonExpression(["'1' if '", LaunchConfiguration("camera_mode"), "' == 'astra' else '0'"]),
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
        "Grid/FootprintRadius": "0.25",
        "Optimizer/GravitySigma": "0.3",
    }
    rtabmap_remappings = [
        ("rgbd_images", "/rgbd_images"),  # camera_mode:=dual
        ("rgbd_image", "/astra/rgbd_image"),  # camera_mode:=astra
        ("odom", "/odom"),
        ("imu", "/imu/data"),
    ]

    # camera_mode:=dual (high Astra + low D455) or astra (original low Astra).
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(alc_share, "launch", "dual_rgbd_cameras.launch.py")
        ),
        launch_arguments={
            "sync_start_delay_sec": LaunchConfiguration("tf_warmup_delay_sec"),
        }.items(),
    )
    rtabmap_mapping = Node(
        package="rtabmap_slam", executable="rtabmap", output="screen",
        condition=UnlessCondition(localization), parameters=[common_rtabmap_parameters],
        remappings=rtabmap_remappings, arguments=["-d"],
    )
    rtabmap_localization = Node(
        package="rtabmap_slam", executable="rtabmap", output="screen",
        condition=IfCondition(localization),
        parameters=[common_rtabmap_parameters, {"Mem/IncrementalMemory": "False", "Mem/InitWMWithAllNodes": "True"}],
        remappings=rtabmap_remappings,
    )

    return LaunchDescription([
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("camera_mode", default_value="dual", description="dual = high Astra + low D455; astra = original single low Astra."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("qos", default_value="2"),
        DeclareLaunchArgument("joy_device_id", default_value="0"),
        DeclareLaunchArgument("throttle_axis", default_value="1"),
        DeclareLaunchArgument("steering_axis", default_value="2"),
        DeclareLaunchArgument("enable_button", default_value="11"),
        DeclareLaunchArgument("tf_warmup_delay_sec", default_value="3.0"),
        DeclareLaunchArgument("model", default_value=os.path.join(description_share, "urdf", "yahboomcar_R2.urdf.xacro")),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(share, "rviz", "r2_teleop_mapping.rviz")),
        Node(package="joint_state_publisher", executable="joint_state_publisher"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": robot_description}]),
        Node(package="yahboomcar_bringup", executable="Ackman_driver_R2", output="screen"),
        Node(package="yahboomcar_base_node", executable="base_node_R2", output="screen", parameters=[{"pub_odom_tf": False, "linear_scale_x": 1.0, "linear_scale_y": 1.0}]),
        Node(package="imu_filter_madgwick", executable="imu_filter_madgwick_node", output="screen", parameters=[os.path.join(bringup_share, "param", "imu_filter_param.yaml")]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen", parameters=[os.path.join(share, "params", "ekf_r2.yaml")], remappings=[("/odometry/filtered", "/odom")]),
        Node(package="joy", executable="joy_node", output="screen", parameters=[{"device_id": ParameterValue(LaunchConfiguration("joy_device_id"), value_type=int), "deadzone": 0.1, "autorepeat_rate": 30.0}]),
        Node(package="yahboom_r2_teleop", executable="r2_joy_teleop", output="screen", parameters=[{"throttle_axis": ParameterValue(LaunchConfiguration("throttle_axis"), value_type=int), "steering_axis": ParameterValue(LaunchConfiguration("steering_axis"), value_type=int), "enable_button": ParameterValue(LaunchConfiguration("enable_button"), value_type=int)}]),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(rviz), arguments=["-d", LaunchConfiguration("rviz_config")], output="screen"),
        cameras,
        TimerAction(period=LaunchConfiguration("tf_warmup_delay_sec"), actions=[rtabmap_mapping, rtabmap_localization]),
    ])
