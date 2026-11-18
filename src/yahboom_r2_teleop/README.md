# Yahboom R2 Teleop Mapping

Joystick-only chassis control with the same RGB-D RTAB-Map mapping pipeline and
an RViz profile derived from `yahboom_alc_bringup`. This package does not start
Nav2, ALC-Planner, frontier exploration, costmaps, or autonomous safety/recovery
nodes.

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
source ~/SLAMPlanner/install/setup.bash
ros2 launch yahboom_r2_teleop r2_teleop_mapping.launch.py
# original single-camera setup
ros2 launch yahboom_r2_teleop r2_teleop_mapping.launch.py camera_mode:=astra
```

Like the exploration launch, this defaults to `camera_mode:=dual` (high Astra +
low RealSense D455); `camera_mode:=astra` is the original single low Astra.
Cameras, mount poses (`astra_*` / `d455_*`), and RGB-D sync come from
`yahboom_alc_bringup/launch/dual_rgbd_cameras.launch.py`. The RViz profile shows
both cameras' images; the D455 panels stay empty in astra mode.

Press START (Linux joystick button 11) once to enable driving and once again to
stop/disable. A one-second debounce prevents the controller's Turbo mode from
toggling drive repeatedly.

The chassis buzzer provides three distinct state cues without blocking joystick
processing: two short beeps when teleop starts, one long beep when drive is
enabled, and three short beeps when drive is disabled. Set the
`buzzer_feedback` node parameter to `false` to silence these cues.
The left stick vertical axis controls speed and the right stick horizontal axis
controls the R2 front steering command, matching Yahboom's supplied
`yahboom_joy_R2_v2.py`. The node continuously publishes zero velocity whenever
control is disabled or joystick messages have stopped for 0.5 seconds.

Drive limits match `yahboom_alc_bringup`: 0.08 m/s motor breakaway floor,
0.11 m/s maximum speed, 0.12 m/s² acceleration, 1.25 maximum steering command,
and 1.0 steering-command unit/s slew rate.

Before launching ROS, open `web/gamepad_mapping.html` in Chrome/Chromium and
press any controller button. It uses the browser Gamepad API directly and needs
no server. The live view identifies every raw axis/button and highlights the
configured throttle, steering, and enable inputs. Browser indices can differ
from Linux `joy_node`; the page therefore also shows the exact ROS command used
to confirm the final indices.

Useful overrides:

```bash
ros2 launch yahboom_r2_teleop r2_teleop_mapping.launch.py \
  joy_device_id:=1 throttle_axis:=1 steering_axis:=2 enable_button:=9 rviz:=false \
  camera_mode:=astra
```
