# Agent Instructions

This repo uses local Python hooks under `.agent-hooks/` for repeatable
formatting and verification. The hook directory is intentionally ignored by
Git, but it is the preferred local automation when present.

## Commit Style

Use the `[type] short desciption` format. lowercase, imperative mood, no period:

[type]:

- feat
- fix
- test
- chore
- build
- docs

Build and tests must pass before committing.

## When To Run Hooks

- After editing C++ headers/sources/tests: run `python3 .agent-hooks/format.py`,
  then `python3 .agent-hooks/build.py`, then `python3 .agent-hooks/test.py`.
- After changing CMake, package manifests, dependencies, or compile database
  behavior: run `python3 .agent-hooks/build.py`, then
  `python3 .agent-hooks/clangd_check.py`.
- After changing bridge logic or tests: run `python3 .agent-hooks/test.py` and
  `python3 .agent-hooks/clang_tidy.py`.
- After changing launch files or launch arguments: run
  `python3 .agent-hooks/launch_check.py`.
- Before reporting a complete implementation, prefer the full sequence:
  `python3 .agent-hooks/run_all.py`.

## Hook Expectations

- `format.py` runs clang-format on repo-owned C++ files and excludes vendored
  `include/rik_sd`.
- `build.py` builds `alc_planner` and `yahboom_alc_bringup` with colcon and
  force-configures CMake so stale build dependency files do not poison checks.
- `test.py` runs the `yahboom_alc_bringup` gtests and prints test results.
- `clangd_check.py` verifies `.clangd` and `build/compile_commands.json`.
  `clangd --check` may return nonzero on template/dependent-type tweak checks;
  compile database validation is the hard failure condition.
- `clang_tidy.py` runs the current analyzer set:
  `clang-analyzer-*`, `bugprone-*`, and `performance-*`.
- `launch_check.py` runs the integrated launch file with `--show-args`; it does
  not start the robot pipeline.

If `.agent-hooks/` is absent, reproduce the equivalent commands manually rather
than adding tracked copies of agent-local hook scripts.
