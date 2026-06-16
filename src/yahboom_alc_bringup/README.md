# yahboom_alc_bringup

Overlay bringup package for running the ALC planner on a Yahboom R2.

This package intentionally owns the Yahboom-specific integration code outside
`ThirdParty/ALC-Planner`. The ALC planner remains a hardware-independent
planner package; this overlay provides the launch wiring and the velocity
bridge needed by the R2 driver.

`alc_yahboom_r2_exploration.launch.py` explicitly starts the Yahboom R2 base
bringup, including `Ackman_driver_R2`, and the Astra RGB-D camera before
launching RTAB-Map, Nav2, ALC-Planner, and the bridge.

The launch uses RTAB-Map as the only `map -> odom` TF publisher. Nav2 is started
with `navigation_launch.py` only, so AMCL and Nav2 map_server do not compete
with RTAB-Map localization.

The integration launch passes an indoor MVP parameter profile to `alc_planner`
so candidate radius, revisit floors, trigger threshold, and navigation timeout
can be tuned without recompiling. RTAB-Map uses visual+ICP registration, and
Nav2's goal checker is configured with a wide yaw tolerance to avoid R2 arc-turn
loops around ALC revisit targets.

## Velocity Bridge

`yahboom_r2_cmd_vel_bridge` converts Nav2-style velocity commands from
`/alc/nav_cmd_vel` to the Yahboom R2 driver's `/cmd_vel` convention:

- `linear.x` remains the forward velocity.
- `angular.z` is converted to R2 steering in `linear.y`.
- output `angular.z` is set to zero by default.

Default limits follow the lowest gear in
`yahboom_joy_R2_v2.py`: `min_linear_speed=0.05`,
`steering_gain=1.25`, and `max_steering=1.25`.

## Reeds-Shepp Reverse Safety

Nav2 uses `SmacPlannerHybrid` with `REEDS_SHEPP`, so global plans may include
forward and reverse segments. The local and global costmaps, frontier
pre-checks, and command bridge share a conservative R2 footprint of about
`0.38 x 0.23 m`, plus inflation in Nav2.

The bridge is the final gate for reverse motion. Every negative `linear.x`
command is fail-closed against the local costmap: stale costmap, missing TF,
unknown cells, out-of-bounds swept footprint, or any footprint cell above the
configured cost threshold publishes zero `/cmd_vel` and emits
`reverse_blocked`. The check covers the full short swept footprint volume
behind the robot, not only the rear centerline.

Because this profile assumes front RGB-D only, Nav2 `BackUp` recovery is kept
short and slow. Repeated reverse safety failures mark the current frontier as
`REVERSE_UNSAFE` and force frontier reselection instead of retrying the same
viewpoint.

## Stall Detection

`r2_stall_detector` handles the case where forward commands and wheel odometry
show progress but independent observations do not. It compares `/alc/nav_cmd_vel`,
wheel `/odom`, the SLAM `map -> base_footprint` TF, and `/imu/data`. When a
forward command persists, wheel odom advances, but SLAM/IMU motion does not
match, it publishes a zero command burst and emits `stall_or_slip`.

The frontier explorer treats `stall_or_slip` as a front blockage signal, cancels
the active Nav2 goal, marks the viewpoint `STALL_OR_SLIP`, and selects a new
frontier. Nav2 `SimpleProgressChecker` remains enabled as a Nav2-level timeout,
but it is not the only stuck detector because wheel odometry can be fooled by
slip.

## Spin Compatibility

ALC-Planner sends Nav2 `/spin` goals with `target_yaw = 2*pi`. Yahboom R2
cannot perform a true stationary differential-drive spin through this driver,
so the bridge approximates stationary spin commands as low-speed arc turns.
Nav2 spin completion still depends on TF yaw accumulation, so spin actions may
complete slowly or timeout depending on odometry and behavior-server timing.

For MVP this is acceptable because Nav2 still returns an action result and the
ALC state machine should not remain stuck. If loop-closure attempts are often
reported as failed, provide a tuned Nav2 YAML through the launch
`nav2_params_file` argument.

## Run

Build this overlay together with the ALC planner:

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
colcon build --packages-select alc_planner yahboom_alc_bringup
source install/setup.bash
```

Launch the MVP pipeline:

```bash
ros2 launch yahboom_alc_bringup alc_yahboom_r2_exploration.launch.py
```

Keep `software/library_ws` before `yahboomcar_ws` in the shell environment.
Astra camera, optional LiDAR drivers, and the R2 bringup's EKF dependency
`robot_localization` are installed in `software/library_ws`.

Smoke-check the bridge:

```bash
ros2 topic pub --once /alc/nav_cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.02}, angular: {z: 0.4}}"
ros2 topic echo /cmd_vel
```
