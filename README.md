# SLAMPlanner

ROS2 overlay workspace for Yahboom/Rosmaster R2 autonomous exploration with
RTAB-Map, Nav2, and the ALC planner.

The hardware-specific integration lives in `src/yahboom_alc_bringup`.
`ThirdParty/ALC-Planner` is kept as the planner package and is not used for
Yahboom-specific bridge code.

## Build

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
colcon build --packages-select alc_planner yahboom_alc_bringup
source install/setup.bash
```

## Launch MVP Pipeline

```bash
ros2 launch yahboom_alc_bringup alc_yahboom_r2_exploration.launch.py
```

The launch wires Yahboom R2 bringup, RTAB-Map mapping, Nav2, ALC-Planner, and
the R2 cmd_vel bridge. It also starts the Astra RGB-D camera by default, so
RTAB-Map receives `/camera/color/image_raw`, `/camera/color/camera_info`, and
`/camera/depth/image_raw` without a separate camera launch.

Nav2 is launched as the navigation stack only. AMCL and Nav2 map_server are not
started; RTAB-Map is the sole `map -> odom` TF authority, and Nav2 consumes the
RTAB-Map map and TF outputs.

The default integration profile is tuned for indoor MVP testing: ALC planner
parameters are supplied through ROS parameters, RTAB-Map uses visual+ICP
registration, and Nav2 goal checking is relaxed so the R2 only needs to satisfy
the revisit position closely rather than a precise final heading.

Nav2 velocity commands are remapped to `/alc/nav_cmd_vel`; only the bridge
publishes to the driver's `/cmd_vel`. Source `software/library_ws` before
`yahboomcar_ws`; Astra camera, optional LiDAR drivers, and the R2 bringup's
`robot_localization` EKF are installed there.
