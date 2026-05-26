# yahboom_alc_bringup

Overlay bringup package for running the ALC planner on a Yahboom R2.

This package intentionally owns the Yahboom-specific integration code outside
`ThirdParty/ALC-Planner`. The ALC planner remains a hardware-independent
planner package; this overlay provides the launch wiring and the velocity
bridge needed by the R2 driver.

`alc_yahboom_r2_exploration.launch.py` explicitly starts the Yahboom R2 base
bringup, including `Ackman_driver_R2`, before launching RTAB-Map, Nav2,
ALC-Planner, and the bridge.

## Velocity Bridge

`yahboom_r2_cmd_vel_bridge` converts Nav2-style velocity commands from
`/alc/nav_cmd_vel` to the Yahboom R2 driver's `/cmd_vel` convention:

- `linear.x` remains the forward velocity.
- `angular.z` is converted to R2 steering in `linear.y`.
- output `angular.z` is set to zero by default.

Default limits follow the lowest gear in
`yahboom_joy_R2_v2.py`: `min_linear_speed=0.05`,
`steering_gain=1.25`, and `max_steering=1.25`.

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

Keep `software/library_ws` before `yahboomcar_ws` in the shell environment. The
R2 bringup includes the EKF launch from `robot_localization`, which is installed
in `software/library_ws`.

Smoke-check the bridge:

```bash
ros2 topic pub --once /alc/nav_cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.02}, angular: {z: 0.4}}"
ros2 topic echo /cmd_vel
```
