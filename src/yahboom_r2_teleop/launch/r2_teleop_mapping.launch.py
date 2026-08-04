import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("yahboom_r2_teleop")
    bringup_share = get_package_share_directory("yahboomcar_bringup")
    description_share = get_package_share_directory("yahboomcar_description")
    camera_share = get_package_share_directory("astra_camera")

    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")
    enable_camera = LaunchConfiguration("enable_camera")
    rviz = LaunchConfiguration("rviz")
    qos = LaunchConfiguration("qos")
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    common_rtabmap_parameters = {
        "frame_id": "base_footprint",
        "use_sim_time": use_sim_time,
        "subscribe_rgbd": True,
        "subscribe_scan": False,
        "subscribe_imu": True,
        "qos_image": qos,
        "qos_imu": qos,
        "wait_for_transform": 1.0,
        "publish_tf": True,
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
        "Grid/FootprintRadius": "0.25",
        "Optimizer/GravitySigma": "0.3",
    }
    rtabmap_remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
        ("odom", "/odom"),
        ("imu", "/imu/data"),
    ]

    camera = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(camera_share, "launch", "astro_pro_plus.launch.xml")
        ),
        condition=IfCondition(enable_camera),
        launch_arguments={
            "camera_name": "camera",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_ir": "false",
            "enable_point_cloud": "true",
            "enable_colored_point_cloud": "false",
            "color_fps": "30",
            "depth_fps": "30",
            "ir_fps": "30",
            "publish_tf": "false",
        }.items(),
    )

    camera_transforms = [
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--x", "0", "--y", "0", "--z", "0", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", "--frame-id", "camera_link", "--child-frame-id", "camera_depth_frame"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--x", "-0.002165860891342163", "--y", "0.025120487213134767", "--z", "0.00009495562314987182", "--qx", "-0.0008345809765160083", "--qy", "-0.00009292588947573677", "--qz", "-0.0009316844516433775", "--qw", "0.9999992251396179", "--frame-id", "camera_depth_frame", "--child-frame-id", "camera_color_frame"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--x", "0", "--y", "0", "--z", "0", "--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5", "--frame-id", "camera_depth_frame", "--child-frame-id", "camera_depth_optical_frame"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--x", "0", "--y", "0", "--z", "0", "--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5", "--frame-id", "camera_color_frame", "--child-frame-id", "camera_color_optical_frame"]),
    ]

    rgbd_sync = Node(
        package="rtabmap_sync", executable="rgbd_sync", output="screen",
        parameters=[{"approx_sync": True, "approx_sync_max_interval": 0.05,
                     "topic_queue_size": 30, "sync_queue_size": 30,
                     "use_sim_time": use_sim_time, "qos": qos}],
        remappings=rtabmap_remappings[:3],
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
        camera,
        *camera_transforms,
        TimerAction(period=LaunchConfiguration("tf_warmup_delay_sec"), actions=[rgbd_sync, rtabmap_mapping, rtabmap_localization]),
    ])
