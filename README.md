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
the R2 cmd_vel bridge. Nav2 velocity commands are remapped to
`/alc/nav_cmd_vel`; only the bridge publishes to the driver's `/cmd_vel`.
Source `software/library_ws` before `yahboomcar_ws`; the R2 bringup's EKF launch
depends on the `robot_localization` package installed there.
