# SLAMPlanner

ROS2 overlay workspace for Yahboom/Rosmaster R2 autonomous exploration with
RTAB-Map, Nav2, and the ALC planner.

The hardware-specific integration lives in `src/yahboom_alc_bringup`.
`ThirdParty/ALC-Planner` is kept as the planner package and is not used for
Yahboom-specific bridge code.

## Build

Initialize the planner submodule before the first build:

```bash
git submodule update --init --recursive
```

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
colcon build --packages-select alc_planner yahboom_alc_bringup yahboom_r2_teleop
source install/setup.bash
```

Run from the workspace root so `colcon_defaults.yaml` applies: Ninja, at most
two concurrent C++ compilations, sequential packages, and no test binaries.
Explicit `--cmake-args` replace the defaults, so omit them for daily builds.

To build and run tests explicitly in the same sourced shell:

```bash
colcon build --packages-select alc_planner yahboom_alc_bringup yahboom_r2_teleop \
  --cmake-args -GNinja -DBUILD_TESTING=ON \
  -DCMAKE_JOB_POOLS=compile=2 -DCMAKE_JOB_POOL_COMPILE=compile
source install/setup.bash
colcon test --packages-select alc_planner yahboom_alc_bringup yahboom_r2_teleop
colcon test-result --verbose
```

The next daily build disables test compilation again. `colcon test` alone
does not build test binaries.

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
ALC-Planner, the cluster frontier explorer, and the R2 cmd_vel bridge. Both
launches start the cameras themselves through
`yahboom_alc_bringup/launch/dual_rgbd_cameras.launch.py`.

Both launches default to mapping with a fresh RTAB-Map database (`-d`), which
deletes the previous database at startup. Copy `~/.ros/rtabmap.db` before
starting if it must be kept; `localization:=true` uses the saved database
instead. Exploration defaults to `rviz:=false`; joystick mapping defaults to
`rviz:=true`. LiDAR is disabled by default (`rplidar_type:=none`).

## Cameras

| Mode | Cameras | RTAB-Map input |
| --- | --- | --- |
| `dual` (default) | High Astra Pro Plus (~0.89 m) + low D455 (~0.19 m) | `/rgbd_images` |
| `astra` | Astra Pro Plus at the original low URDF mount | `/astra/rgbd_image` |

In dual mode, the Astra sees walls and features; the D455 covers the near floor
and low obstacles. Both feed the costmaps and collision monitor. Views need
not overlap; RTAB-Map uses `odom_sensor_sync` to compensate their
unsynchronized timestamps.

The D455 uses RealSense ROS **4.58.3** with librealsense **2.58.3**, built with
`FORCE_RSUSB_BACKEND=ON`: the packaged V4L2 backend could not enumerate it on
this Jetson. See [RealSense D455 setup](src/yahboom_alc_bringup/README.md#realsense-d455-setup-rsusb)
for the verified versions, SDK build, runtime library selection, and checks.

Mount overrides are `astra_{x,y,z,roll,pitch,yaw}` and
`d455_{x,y,z,roll,pitch,yaw}`, relative to `base_link` (metres/radians, positive
pitch looks down). Empty values use the selected mode's defaults in
[dual_rgbd_cameras.launch.py](src/yahboom_alc_bringup/launch/dual_rgbd_cameras.launch.py).
Current dual-mode pitches: Astra 0.165 rad, D455 0.099 rad; D455 roll is
-0.030 rad. The D455 defaults were remeasured on 2026-11-21 after re-tilting
its mount.

After re-mounting, measure height and pitch with a floor-plane fit in each
camera's depth frame; avoid estimates by eye or the uncalibrated D455 IMU.
Incorrect pitch can turn the floor into phantom obstacles above
`Grid/MaxGroundHeight` (0.08 m); incorrect position can double walls.

## Integration

Nav2 is launched as the navigation stack only. AMCL and Nav2 map_server are not
started. During exploration, `rtabmap_map_tf_relay` is the sole `map -> odom`
TF publisher at 50 Hz, deriving and caching the transform from RTAB-Map's
`/localization_pose` and odometry. Joystick mapping uses RTAB-Map's own TF
publisher. Both modes share the RTAB-Map and EKF settings in
`src/yahboom_alc_bringup/launch/rtabmap_common.py`.

The default integration profile is tuned for indoor MVP testing: ALC planner
parameters are supplied through ROS parameters. RTAB-Map uses visual-only
registration (`Reg/Strategy=0`), with 3D-3D estimation in dual mode and PnP in
astra mode. Nav2 uses Smac Hybrid-A* with Reeds-Shepp motion and Regulated
Pure Pursuit; goal checking accepts 0.35 m position error and a full-turn yaw
tolerance. The frontier explorer validates routes through Nav2 and also
scores unvisited known-free regions alongside frontiers.

Nav2 path-following commands pass through the velocity smoother and collision
monitor to `/alc/nav_cmd_vel`, then through the bridge to the driver's
`/cmd_vel`. Nav2 recovery behaviors and explorer observation turns also send
commands to `/alc/nav_cmd_vel`; stop nodes can publish zero directly to
`/cmd_vel`. Depth-based obstacle braking is owned by Nav2: the local and global
costmap voxel layers and the Nav2 collision monitor consume
the voxel-filtered depth clouds (`/camera/depth/points_voxel_base`, plus
`/realsense/d455/depth/points_voxel_base` in dual mode). The collision monitor
runs a three-stage reflex on those clouds — a time-to-collision `approach` zone for smooth proportional slowdown, a
`slowdown` zone, and an inner `stop` polygon whose front edge is matched to the
camera's RangeMin so the robot halts just before an obstacle enters the near
blind zone (see [nav2_rgbd_params.yaml](src/yahboom_alc_bringup/params/nav2_rgbd_params.yaml)).

`r2_stall_detector` compares expected forward motion from the executed
`/cmd_vel` with EKF `/odom` displacement, using `/imu/data` yaw rate to avoid
declaring a stall while turning. The launch allows 1.5 s for startup response,
then requires at least 0.06 m of expected motion before judging displacement
below 30% as a stall. It emits `stall_or_slip` and a zero-command burst.
It has no depth or SLAM-pose input and cannot independently detect wheel slip.
There is no IMU impact reflex.

The bridge limits speed to 0.11 m/s and streams zero commands for 1 s when
its input has been silent for 0.5 s. Reverse motion is checked against the
local costmap, including a 2 s grace for recently free cells. When no candidate
is available, the explorer can turn toward unknown space using forward/reverse
arcs. During this observation turn, the bridge permits reverse into unknown
cells for up to 30 s, while still rejecting known obstacles. With no rear
sensor, this carries a blind-reverse risk.

Before enabling higher speeds, measure the camera's useful return band at
0.30 m, 0.45 m, and 0.55 m with targets at several heights. Use those results
to tune the collision monitor's polygon distances and height band. Test any
5–10 degree camera tilt change separately and update the measured mount TF.
Source `software/library_ws` before
`yahboomcar_ws`; Astra camera, optional LiDAR drivers, and the R2 bringup's
`robot_localization` EKF are installed there.

## Recorded real-car runs

Only start a run when the car is available and someone is watching it:

```bash
scripts/real_car_run.sh --duration 600 --idle-stop 60
```

The script refuses to start while robot processes are already running. It
records costmaps, TF, odometry, map, commands, and planner events under
`/media/jetson/ADATA/realcar_<timestamp>/`, then copies the planner log and
RTAB-Map database with its SHA-256. It stops when the duration ends, no frontier
or a perception-stall pause persists for the idle interval, the launch exits,
`<output>/STOP` is created, or Ctrl-C is pressed.

Shutdown streams zero `/cmd_vel` before stopping processes: the R2 driver
latches its last command and has no timeout. Do not stop it by killing
processes alone. Check any shutdown `WARNING` for surviving processes. Keep
run data on ADATA; `/tmp` is cleared on reboot. Planner JSONL logs default to
`/home/jetson/SLAMPlanner/planner-log` (`planner_log_dir` overrides this).
