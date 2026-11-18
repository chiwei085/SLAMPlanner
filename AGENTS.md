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
