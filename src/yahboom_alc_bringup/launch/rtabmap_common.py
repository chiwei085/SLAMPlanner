"""RTAB-Map and odometry setup shared by autonomous exploration and teleop
mapping, so both modes build maps from the same settings.

Not a launch file: alc_yahboom_r2_exploration.launch.py and
yahboom_r2_teleop/r2_teleop_mapping.launch.py import it from this package's
installed launch directory.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Raises the EKF's scheduling priority so odom stays on time under load.
EKF_LAUNCH_PREFIX = "nice -n -10"


def ekf_params_file():
    return os.path.join(
        get_package_share_directory("yahboom_alc_bringup"), "params", "ekf_r2.yaml"
    )


def rtabmap_parameters(camera_mode, qos, use_sim_time, publish_tf):
    """publish_tf: exploration passes False (rtabmap_map_tf_relay publishes
    map->odom for Nav2); teleop passes True. Map building is unaffected."""
    return {
        "frame_id": "base_footprint",
        "use_sim_time": use_sim_time,
        "subscribe_rgbd": True,
        # dual: 0 = one RGBDImages topic (/rgbd_images from rgbdx_sync).
        # astra: 1 = a single RGBDImage topic (/astra/rgbd_image).
        "rgbd_cameras": ParameterValue(
            PythonExpression(["1 if '", camera_mode, "' == 'astra' else 0"]),
            value_type=int,
        ),
        # Motion-compensate each camera's frame by odom between its stamp and
        # the node stamp: Astra (30 Hz) and D455 (15 Hz) are not hw-synced.
        "odom_sensor_sync": True,
        "subscribe_scan": False,
        "subscribe_imu": True,
        "use_action_for_goal": True,
        "qos_image": qos,
        "qos_imu": qos,
        "wait_for_transform": 1.0,
        "publish_tf": publish_tf,
        "Reg/Strategy": "0",
        # Multi-camera PnP (1, the default) needs RTAB-Map built with OpenGV;
        # this build is not, so in dual mode every visual registration failed
        # (113 errors in 194 s, 0 refined links, 0 loop closures: map == odom).
        # 3D-3D works for multi-camera; single-camera astra keeps PnP.
        "Vis/EstimationType": PythonExpression(
            ["'1' if '", camera_mode, "' == 'astra' else '0'"]
        ),
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
        # occupies. Any pose-graph node whose footprint overlaps a cell marks
        # it as free unconditionally -- the strongest clearing evidence since
        # it requires no sensor. Diagonal half-size of R2 body (0.19 front +
        # 0.115 half-width = 0.22 m diagonal) plus a small margin.
        "Grid/FootprintRadius": "0.25",
        "Optimizer/GravitySigma": "0.3",
    }


RTABMAP_REMAPPINGS = [
    ("rgbd_images", "/rgbd_images"),  # camera_mode:=dual
    ("rgbd_image", "/astra/rgbd_image"),  # camera_mode:=astra
    ("odom", "/odom"),
    ("imu", "/imu/data"),
]


def rtabmap_nodes(parameters, localization):
    """Mapping (fresh database, -d) or localization against the saved one."""
    mapping = Node(
        condition=UnlessCondition(localization),
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        parameters=[parameters],
        remappings=RTABMAP_REMAPPINGS,
        arguments=["-d"],
    )
    localization_node = Node(
        condition=IfCondition(localization),
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        parameters=[
            parameters,
            {
                "Mem/IncrementalMemory": "False",
                "Mem/InitWMWithAllNodes": "True",
            },
        ],
        remappings=RTABMAP_REMAPPINGS,
    )
    return mapping, localization_node
