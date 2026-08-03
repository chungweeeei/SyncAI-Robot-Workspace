# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Status

**SyncAI-ROS-MCP** is an MCP (Model Context Protocol) server that runs as a
ROS 2 node, exposing ROS 2 introspection and control as MCP tools. This is a
standard `ament_python` ROS 2 package (ROS 2 Humble); see `README.md` for the
package layout and usage.

## Environment

A standard ROS 2 Humble host (or container). Key facts that affect how commands
behave:

- **ROS distro:** Humble. Source `/opt/ros/humble/setup.bash` (plus
  `install/setup.bash` after a build) before running any `ros2` / `colcon`
  command in a non-interactive shell.
- **FastMCP** is a pip-only dependency (not in rosdep). Install it into the same
  Python interpreter that runs the node (the system `python3` used by
  `ros2 run`): `python3 -m pip install "fastmcp>=3.4.4"`.

## Common commands

Because setup files are typically sourced only in *interactive* shells, a
non-interactive shell must source ROS explicitly first.

```bash
# Build the whole workspace (creates build/, install/, log/)
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Build a single package
colcon build --symlink-install --packages-select <pkg_name>

# After building, source the overlay before running/testing
source install/setup.bash

# Run all tests, then inspect results
colcon test
colcon test-result --verbose

# Test a single package
colcon test --packages-select <pkg_name>

# Run a node
ros2 run <pkg_name> <executable>

# Resolve declared dependencies from package.xml manifests
rosdep install --from-paths src --ignore-src -y
```

Lint/style are enforced by the ament test suite (`ament_flake8`, `ament_pep257`,
`ament_copyright`) under `test/`.
