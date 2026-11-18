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

## Camera Modes

| | `camera_mode:=dual` (default) | `camera_mode:=astra` |
| --- | --- | --- |
| Cameras | Astra Pro Plus high (~0.89 m) + RealSense D455 low (~0.19 m) | Astra Pro Plus at the original low URDF mount |
| RTAB-Map input | `/rgbd_images` (`rgbdx_sync` of `/astra/rgbd_image` + `/d455/rgbd_image`), `rgbd_cameras: 0` | `/astra/rgbd_image`, `rgbd_cameras: 1` |
| Costmaps / collision monitor | Astra + D455 voxel clouds | Astra voxel cloud |
| Stall-detector depth reflex | D455 `/realsense/d455/depth/image_rect_raw` | Astra `/camera/depth/image_raw` |

In dual mode the high Astra sees walls and features but no floor closer than
about 1.7 m; the low D455 covers the floor from about 0.3 m. The views need not
overlap. RTAB-Map runs with `odom_sensor_sync` to compensate the unsynchronized
30 Hz Astra / 15 Hz D455 stamps.

The D455 needs the RSUSB-backend librealsense build (the stock backend does not
enumerate it on this Jetson); its directory is the `realsense_lib_dir` launch
argument, default `~/realsense-check/rsusb-build/Release`. The D455 starts
`d455_start_delay_sec` (6 s) after the Astra: starting both at once made one
driver fail to claim its USB interface and segfault. The D455 node also
respawns if it crashes, since it feeds the costmaps and the depth reflex.

### Camera mount poses

Each camera's pose relative to `base_link` is a launch argument:
`astra_{x,y,z,roll,pitch,yaw}` and `d455_{x,y,z,roll,pitch,yaw}` (metres,
radians, pitch positive = looking down). Leave them empty to use the defaults
for the selected mode (`--show-args` lists both). The dual-mode defaults were
measured on 2026-11-18: Astra pitch 0.165 rad (9.5 deg), D455 pitch 0.239 rad
(13.7 deg).

After re-mounting a camera, measure pitch and height from a floor-plane fit in
the camera's own depth frame, not by eye or from the D455 IMU (uncalibrated; it
read 7.6 deg against a true 13.7 deg). A pitch error makes the floor rise with
distance until it crosses RTAB-Map `Grid/MaxGroundHeight` (0.08 m) and shows up
as phantom obstacles; a position error shows up as doubled walls.

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

`r2_collision_safety_gate` is **not** in the cmd_vel path. It only watches the
IMU for a forward-deceleration impact — the one hazard the depth/costmap layers
cannot see (sub-RangeMin near field, pure rotation) — and on impact it raises
`/collision_safety_stop_active` (the bridge halts forward motion while keeping
reverse for recovery) and emits a `stall_or_slip` event for the recovery state
machine. The stall detector remains the wheel/SLAM motion-fault fallback.

Before enabling higher speeds, measure the camera's useful return band at
0.30 m, 0.45 m, and 0.55 m with targets at several heights. Use those results
to tune the collision monitor's polygon distances, height band, and the gate's
IMU impact threshold. Test any 5–10 degree camera tilt change separately;
this repository does not alter the physical mount. Source `software/library_ws` before
`yahboomcar_ws`; Astra camera, optional LiDAR drivers, and the R2 bringup's
`robot_localization` EKF are installed there.
