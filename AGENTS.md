# Agent Instructions

## Commit Style

Use the `[type] short description` format: lowercase, imperative mood, no period.

[type]: feat, fix, test, chore, build, docs

Committing does not require re-running verification; follow the style above.

`ThirdParty/ALC-Planner` is a git submodule (`alc_planner`). Commit changes
there inside the submodule first, then commit the updated pointer here.

## Verification

Run from the repo root in a shell that has sourced
`/opt/ros/humble/setup.bash`. After a build, also source `install/setup.bash`.

```bash
# Format C++ (skip vendored include/rik_sd)
find src -name '*.cpp' -o -name '*.hpp' | grep -v include/rik_sd | xargs clang-format -i

# Build (slow on the Jetson; never run two colcon builds in this workspace at once)
colcon build --packages-select alc_planner yahboom_alc_bringup yahboom_r2_teleop --cmake-args -GNinja

# Test (test-result exits nonzero on failures)
colcon test --packages-select alc_planner yahboom_alc_bringup yahboom_r2_teleop
colcon test-result --verbose

# Launch files: parse only, does not start the robot
ros2 launch <package> <file> --show-args
```

| Changed | Run |
| --- | --- |
| C++ sources/headers/tests | format → build → test |
| CMake, `package.xml`, `setup.py`, dependencies | build → test |
| Launch files or launch arguments | build → `--show-args` on each changed launch file |

`build/compile_commands.json` is not regenerated automatically. clangd
diagnostics can be stale, and the unity-built library sources have no entries
of their own. Trust the colcon build output over clangd.

## Real-car validation runs

Only drive the robot when the user has said the car is available in this
session, and the user is watching it. The user's view of the real car outranks
every log: odometry, SLAM, and the IMU have all reported motion or impacts
that did not happen. When an event looks physical (stall, impact, blocked),
ask the user what the car actually did before drawing a conclusion.

```bash
scripts/real_car_run.sh --duration 600 --idle-stop 60   # out: /media/jetson/ADATA/realcar_<ts>/
```

The script runs in the foreground (give the shell a timeout above
`--duration` + 90 s). It refuses to start if ROS or robot processes are
already running. While it runs it records the costmaps, TF, odom, `/map`, and
the planner events, and prints one line per relevant event. It stops on any of
these: the duration ends, `no_frontier_candidate` or a perception-stall pause persists for `--idle-stop`
seconds, the launch exits, `touch <out>/STOP`, or Ctrl-C. To stop, it streams
zero `/cmd_vel` before sending SIGINT: the R2 driver latches its last command
and has no timeout, so never stop the car by killing processes alone.
Afterwards it copies the planner log and `~/.ros/rtabmap.db` (with its SHA-256)
into the output folder and prints a summary. A WARNING line means processes
survived the shutdown, so tell the user. Keep run data on the ADATA drive,
because `/tmp` is wiped on reboot.

### Checking a conclusion against the recorded data

Change code only for a mechanism you have broken down to its smallest units
and checked against data that could have proven it wrong. A fix that is
reverse-fitted to numbers is not acceptable. Prefer removing a wrong mechanism
over adding a guard. In the report, say what the evidence does not prove.

- The bag's `/global_costmap/costmap` is what the planner actually saw,
  including the live voxel layer. To reproduce a Nav2 decision, take its lethal
  (100) cells as a map and run the stock `map_server` and `planner_server`
  offline with the global costmap and planner parameters from
  `nav2_rgbd_params.yaml`. Use static_layer + inflation_layer only, the logged
  start pose and yaw, and the goal yaw that was actually sent. Use an isolated
  `ROS_DOMAIN_ID` with `ROS_LOCALHOST_ONLY=1`. Run the binaries directly, not
  through `ros2 run`, which leaves its children alive. Clean up with
  `pkill -f "[l]ib/nav2_"`; a plain pattern also kills your own shell.
- `rtabmap.db` holds the saved RGB-D keyframes, their poses, and the 2D grid.
  Copy it before you read it. Replay the stored local grids node by node to
  get the `/map` from before an event. Check obstacles against the RGB image
  and depth of the nearest keyframes. Keyframes are sparse: state how many
  seconds lie between the keyframe and the event.
- To ask what the explorer would have chosen, compile
  `frontier_explorer_core.cpp` directly into a probe (the unity library pulls
  in rclcpp). Load the map from that moment, then compare candidate counts
  with and without one specific state, such as a blacklist entry or a hazard
  region.
- A validation failure is a fact only about the exact request: start, goal
  pose including yaw, and costmap. Before blaming a goal, check `waypoint_xy`
  against `goal_xy`.

Write findings as plain text in the local `dev-logs/` (gitignored).
