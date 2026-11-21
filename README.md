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
colcon build --packages-select alc_planner yahboom_alc_bringup yahboom_r2_teleop
source install/setup.bash
```

## Launch

Autonomous exploration and joystick mapping both default to the dual-camera
setup; pass `camera_mode:=astra` for the original single-camera setup.

```bash
# Autonomous exploration (RTAB-Map + Nav2 + ALC-Planner)
ros2 launch yahboom_alc_bringup alc_yahboom_r2_exploration.launch.py
ros2 launch yahboom_alc_bringup alc_yahboom_r2_exploration.launch.py camera_mode:=astra

# Joystick mapping (RTAB-Map only, no Nav2)
ros2 launch yahboom_r2_teleop r2_teleop_mapping.launch.py
ros2 launch yahboom_r2_teleop r2_teleop_mapping.launch.py camera_mode:=astra
```

The exploration launch wires Yahboom R2 bringup, RTAB-Map mapping, Nav2,
ALC-Planner, and the R2 cmd_vel bridge. Both launches start the cameras
themselves through `yahboom_alc_bringup/launch/dual_rgbd_cameras.launch.py`.

## Cameras

| Mode | Cameras | RTAB-Map input |
| --- | --- | --- |
| `dual` (default) | High Astra Pro Plus (~0.89 m) + low D455 (~0.19 m) | `/rgbd_images` |
| `astra` | Astra Pro Plus at the original low URDF mount | `/astra/rgbd_image` |

In dual mode, the Astra sees walls and features; the D455 covers the near floor
(from ~0.3 m, versus ~1.7 m for the high Astra) and supplies the stall-detector
depth reflex. Both feed the costmaps. Views need not overlap; RTAB-Map uses
`odom_sensor_sync` to compensate their unsynchronized timestamps.

The D455 uses RealSense ROS **4.58.3** with librealsense **2.58.3**, built with
`FORCE_RSUSB_BACKEND=ON`: the packaged V4L2 backend could not enumerate it on
this Jetson. See [RealSense D455 setup](src/yahboom_alc_bringup/README.md#realsense-d455-setup-rsusb)
for the verified versions, SDK build, runtime library selection, and checks.

Mount overrides are `astra_{x,y,z,roll,pitch,yaw}` and
`d455_{x,y,z,roll,pitch,yaw}`, relative to `base_link` (metres/radians, positive
pitch looks down). Empty values use the selected mode's defaults in
[dual_rgbd_cameras.launch.py](src/yahboom_alc_bringup/launch/dual_rgbd_cameras.launch.py).
Measured dual-mode pitches: Astra 0.165 rad, D455 0.239 rad.

After re-mounting, measure height and pitch with a floor-plane fit in each
camera's depth frame; avoid estimates by eye or the uncalibrated D455 IMU.
Incorrect pitch can turn the floor into phantom obstacles above
`Grid/MaxGroundHeight` (0.08 m); incorrect position can double walls.

## Integration

Nav2 is launched as the navigation stack only. AMCL and Nav2 map_server are not
started; RTAB-Map is the sole `map -> odom` TF authority, and Nav2 consumes the
RTAB-Map map and TF outputs.

The default integration profile is tuned for indoor MVP testing: ALC planner
parameters are supplied through ROS parameters, RTAB-Map uses visual+ICP
registration, and Nav2 goal checking is relaxed so the R2 only needs to satisfy
the revisit position closely rather than a precise final heading.

Nav2 velocity commands are remapped to `/alc/nav_cmd_vel`; only the bridge
publishes to the driver's `/cmd_vel`. Depth-based obstacle braking is owned by
Nav2: the local costmap voxel layer and the Nav2 collision monitor both consume
the voxel-filtered depth clouds (`/camera/depth/points_voxel_base`, plus
`/realsense/d455/depth/points_voxel_base` in dual mode). The collision monitor
runs a three-stage reflex on those clouds — a time-to-collision `approach` zone for smooth proportional slowdown, a
`slowdown` zone, and an inner `stop` polygon whose front edge is matched to the
camera's RangeMin so the robot halts just before an obstacle enters the near
blind zone (see `params/nav2_rgbd_params.yaml`).

A robot blocked by something the cameras cannot see is caught by
`r2_stall_detector`, which compares the motion the executed command should
produce with the motion SLAM observes. There is no IMU impact reflex: at R2
speeds a real contact decelerates less than normal driving vibration, so a
single-sample IMU threshold cannot tell the two apart.

Before enabling higher speeds, measure the camera's useful return band at
0.30 m, 0.45 m, and 0.55 m with targets at several heights. Use those results
to tune the collision monitor's polygon distances, height band, and the gate's
IMU impact threshold. Test any 5–10 degree camera tilt change separately;
this repository does not alter the physical mount. Source `software/library_ws` before
`yahboomcar_ws`; Astra camera, optional LiDAR drivers, and the R2 bringup's
`robot_localization` EKF are installed there.
