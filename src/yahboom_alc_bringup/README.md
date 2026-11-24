# yahboom_alc_bringup

Overlay bringup package for running the ALC planner on a Yahboom R2.

This package intentionally owns the Yahboom-specific integration code outside
`ThirdParty/ALC-Planner`. The ALC planner remains a hardware-independent
planner package; this overlay provides the launch wiring and the velocity
bridge needed by the R2 driver.

`alc_yahboom_r2_exploration.launch.py` explicitly starts the Yahboom R2 base
bringup, including `Ackman_driver_R2`, and the RGB-D cameras before launching
RTAB-Map, Nav2, ALC-Planner, and the bridge.

Cameras come from `dual_rgbd_cameras.launch.py`, also used by
`yahboom_r2_teleop`. `camera_mode:=dual` (default) runs a high Astra Pro Plus
plus a low RealSense D455 merged by `rgbdx_sync` into `/rgbd_images`;
`camera_mode:=astra` runs the original single low Astra on `/astra/rgbd_image`.
Mount poses are the `astra_*` / `d455_*` launch arguments; see the workspace
README for the mode table and how the defaults were measured.

The launch disables RTAB-Map's `map -> odom` TF publisher and runs
`rtabmap_map_tf_relay` as the only `map -> odom` TF publisher. Nav2 is started
with this package's `r2_navigation_launch.py` only, so AMCL and Nav2 map_server
do not compete with RTAB-Map localization.

Parameters of this package's nodes and of `alc_planner` are in
[params/exploration.yaml](params/exploration.yaml). RTAB-Map uses visual-only registration
(`Reg/Strategy=0`, 3D-3D estimation for dual cameras, PnP for astra), and
Nav2's goal checker is configured with a wide yaw tolerance to avoid R2 arc-turn
loops around ALC revisit targets.

## Velocity Bridge

`yahboom_r2_cmd_vel_bridge` converts Nav2-style velocity commands from
`/alc/nav_cmd_vel` (path following) and `/alc/behavior_cmd_vel` (behaviors,
which win while they run) to the Yahboom R2 driver's `/cmd_vel` convention:

- `linear.x` remains the forward velocity.
- `angular.z` is converted to R2 steering in `linear.y`: the curvature
  `angular.z / linear.x` gives the front wheel angle `atan(wheelbase * curvature)`,
  sent as degrees / 1000, which is the unit the R2 firmware reads (±0.045 is its
  ±45° limit).
- output `angular.z` is set to zero by default.

`exploration.yaml` sets `min_linear_speed=0.08`,
`max_linear_speed=0.11`, `wheelbase_m=0.2353` (Yahboom R2 URDF axles), and
`max_steering_angle_rad=0.6` (URDF steer-joint limit).
After 0.5 s without input, the bridge streams centered zero commands for 1 s
because the R2 driver has no command timeout.

## Reeds-Shepp Reverse Safety

Nav2 uses `SmacPlannerHybrid` with `REEDS_SHEPP`, so global plans may include
forward and reverse segments. The local and global costmaps, frontier
pre-checks, and command bridge share a conservative R2 footprint of about
`0.38 x 0.23 m`, plus inflation in Nav2.

The bridge is the final gate for reverse motion. Every negative `linear.x`
command is checked against the local costmap: stale costmap, missing TF,
unknown cells, out-of-bounds swept footprint, or any footprint cell above the
configured cost threshold publishes zero `/cmd_vel` and emits
`reverse_blocked`. The check covers the full short swept footprint volume
behind the robot, not only the rear centerline.

There are two exceptions for unknown cells: cells seen free within the last
2 s are accepted, and observation turns toward unknown space permit blind
reverse for up to 30 s per turn. Known occupied cells still block motion.
There is no rear sensor, so the observation-turn exception carries a
blind-reverse risk.

Because this profile assumes front-facing RGB-D only (one or two cameras), Nav2 `BackUp` recovery is kept
short and slow. Repeated reverse safety failures mark the current frontier as
`REVERSE_UNSAFE` and force frontier reselection instead of retrying the same
viewpoint.

## Stall Detection

`r2_stall_detector` compares expected forward motion from executed `/cmd_vel`
with EKF `/odom` displacement. It also reads `/imu/data` yaw rate to avoid
declaring a stall while turning; it has no depth or SLAM-pose input. The launch
allows 1.5 s for the chassis to respond from rest, then accumulates at least
0.06 m of expected motion. Displacement below 30% of that distance, without
IMU yaw motion, triggers a 0.5 s zero-command burst and `stall_or_slip`.

The frontier explorer feeds this event into its recovery state machine and
hazard/goal-failure tracking. Nav2 `SimpleProgressChecker` also remains enabled.
Neither EKF odometry nor this detector independently measures body translation,
so wheel slip cannot be reliably distinguished from actual progress. There is
no IMU impact reflex.

## Spin Compatibility

The R2 cannot turn on the spot, so the bridge turns a stationary spin command
into low-speed arcs. ALC targets are not spun at (`spin_at_target: false`);
the car arrives facing the keyframe's view instead.

## RealSense D455 setup (RSUSB)

The packaged librealsense V4L2 backend reported `No RealSense devices were
found!` on this Jetson. Rebuilding **the same SDK version** with the RSUSB
backend and loading that library into the packaged ROS driver enabled the
D455. Reducing FPS alone did not fix discovery. RSUSB uses libusb rather than
the kernel V4L2 capture backend.

Verified on 2026-11-18 from the installed packages, SDK CMake cache, and
successful driver/RGB-D check logs in `~/realsense-check/`:

| Component | Verified configuration |
| --- | --- |
| Host | Ubuntu 22.04.5, aarch64; Jetson Linux L4T 36.4.3; kernel `5.15.148-tegra` |
| ROS | Humble |
| ROS driver | `realsense2_camera` **4.58.3** (`ros-humble-realsense2-camera` package `4.58.3-1jammy.20260804.190440`) |
| Driver messages | `ros-humble-realsense2-camera-msgs` package `4.58.3-1jammy.20260720.173829` |
| Packaged SDK | `ros-humble-librealsense2` **2.58.3** (package `2.58.3-1jammy.20260720.173911`); retained as the ROS dependency |
| Runtime SDK | librealsense source tag **`v2.58.3`**, shared library built with **`FORCE_RSUSB_BACKEND=ON`** |
| Camera | D455, firmware **5.16.0.1**, USB **3.2** as reported by the driver |

### Build the replacement SDK

Keep the ROS driver and SDK versions above together when reproducing this
setup. On a fresh host, install the matching ROS packages and the SDK build
prerequisites (`git`, `curl`, `build-essential`, `cmake`, `pkg-config`, `libusb-1.0-0-dev`,
`libudev-dev`, `libssl-dev`), then build the SDK separately from this workspace:

```bash
mkdir -p ~/realsense-check
cd ~/realsense-check
git clone --branch v2.58.3 --depth 1 https://github.com/realsenseai/librealsense.git librealsense-src
cmake -S librealsense-src -B rsusb-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON -DBUILD_SHARED_LIBS=ON \
  -DBUILD_WITH_NEON=ON -DBUILD_WITH_CUDA=OFF -DHWM_OVER_XU=ON \
  -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_GLSL_EXTENSIONS=OFF -DBUILD_TOOLS=OFF \
  -DBUILD_UNIT_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_WITH_DDS=OFF -DCHECK_FOR_UPDATES=OFF
cmake --build rsusb-build -j2
sudo install -m 644 librealsense-src/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Reconnect the camera after installing the USB rules. The resulting
`rsusb-build/Release/` must contain `librealsense2.so.2.58.3` and its
`librealsense2.so.2.58` / `librealsense2.so` symlinks. The launch loads this
build directly; no system-wide SDK install or ROS driver rebuild is needed.

The recorded local source has one build-dependency workaround:
`CMake/external_json.cmake` skips its downloader, and nlohmann/json **3.12.0**'s
single header is supplied at
`rsusb-build/third-party/json/include/nlohmann/json.hpp`. A clean upstream
checkout uses its own downloader; if that fails, reproduce this workaround
before configuring. From `~/realsense-check`, the equivalent steps are:

```bash
mkdir -p rsusb-build/third-party/json/include/nlohmann
curl -fL https://raw.githubusercontent.com/nlohmann/json/v3.12.0/single_include/nlohmann/json.hpp \
  -o rsusb-build/third-party/json/include/nlohmann/json.hpp
printf '%s\n' 'message(STATUS "Using separately downloaded nlohmann/json v3.12.0 single header")' \
  > librealsense-src/CMake/external_json.cmake
```

Then rerun the CMake configure/build commands above. This dependency-download
workaround does not select the camera backend.

### Runtime configuration and verification

`dual_rgbd_cameras.launch.py` prepends `realsense_lib_dir` (default
`~/realsense-check/rsusb-build/Release`) to the D455 process's `LD_LIBRARY_PATH`.
A non-default SDK location can be passed to either exploration or teleop launch
as `realsense_lib_dir:=/absolute/path/to/Release`.

The D455 configuration in that launch uses color and depth **640x480 at 15 Hz**,
`align_depth.enable=true`, `pointcloud__neon_.enable=true`,
`pointcloud__neon_.stream_filter=0`, and `publish_tf=true`; infrared, gyro, and
accel streams are disabled. Keep `d455_start_delay_sec=6.0` so Astra opens its
USB streams first. Simultaneous startup caused USB interface-claim failures
and crashes; the D455 node also respawns after 3 s if it exits.

Check library resolution without opening the camera:

```bash
source /opt/ros/humble/setup.bash
LD_LIBRARY_PATH="$HOME/realsense-check/rsusb-build/Release:${LD_LIBRARY_PATH:-}" \
  ldd /opt/ros/humble/lib/librealsense2_camera.so | grep 'librealsense2.so'
```

The resolved path must point into `rsusb-build/Release/`, rather than
`/opt/ros/humble/lib/aarch64-linux-gnu/`. Driver startup should report
`RealSense ROS v4.58.3`, `Built with LibRealSense v2.58.3`, and
`Running with LibRealSense v2.58.3`. **The version log alone cannot distinguish
the two backends**, because both SDK builds have the same version.

After starting the normal launch, check that all three image streams arrive:

```bash
ros2 topic hz /realsense/d455/color/image_raw
ros2 topic hz /realsense/d455/depth/image_rect_raw
ros2 topic hz /realsense/d455/aligned_depth_to_color/image_raw
```

Run each check separately. The original successful check received about
14.3–14.5 FPS on all three streams with nonzero depth values (`result.json`).
RSUSB still logged occasional libusb control-transfer warnings, so device
discovery alone is insufficient: confirm continuing RGB-D delivery.

## Run

This package requires C++20 and a standard library with `std::format` support
(verified on this Jetson with GCC 16.2). The ALC planner keeps its own
language standard.

Build this overlay together with the ALC planner:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
colcon build --packages-select alc_planner yahboom_alc_bringup --cmake-args -GNinja
source install/setup.bash
```

Launch the pipeline (dual camera by default):

```bash
ros2 launch yahboom_alc_bringup alc_yahboom_r2_exploration.launch.py
# original single-camera setup
ros2 launch yahboom_alc_bringup alc_yahboom_r2_exploration.launch.py camera_mode:=astra
```

The launch attempts to run `ekf_filter_node` with
`ekf_launch_prefix="nice -n -10"` so odom TF publication has scheduling
priority during point-cloud spikes. On deployments without `CAP_SYS_NICE`, this
prints a permission warning and continues at normal priority; systems with that
capability can use `ekf_launch_prefix="chrt -f 80"` for stronger scheduling
isolation.

Depth cloud load is reduced by publishing shared downsampled clouds on
`/camera/depth/points_voxel_base` (Astra) and, in dual mode,
`/realsense/d455/depth/points_voxel_base` (D455, 640x480@15). Astra FPS defaults stay at 30 because this
R2 camera rejected 640x480@15 for both depth and UVC color during launch; only
override `camera_color_fps`, `camera_depth_fps`, or `camera_ir_fps` after
confirming the mode is supported by the hardware.

Keep `software/library_ws` before `yahboomcar_ws` in the shell environment.
Astra camera, optional LiDAR drivers, and the R2 bringup's EKF dependency
`robot_localization` are installed in `software/library_ws`.

Smoke-check the bridge:

```bash
ros2 topic pub --once /alc/nav_cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.02}, angular: {z: 0.4}}"
ros2 topic echo /cmd_vel
```
