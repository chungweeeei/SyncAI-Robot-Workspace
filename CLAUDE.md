# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository status

This workspace is currently a scaffold. The only contents are:
- `.devcontainer/` — Docker-based ROS2 Humble development environment
- `README.md` — empty

There is no source code, build system, or test suite yet. When adding the first ROS2 packages, place them under a `src/` directory at the repo root so `colcon` can discover them from `/workspace`.

## Development environment

Development is intended to happen inside the devcontainer (`.devcontainer/devcontainer.json` + `Dockerfile`):
- Base image: `ros:humble`
- Pre-installed: `git`, `vim`, `python3-pip`, `python3-rosdep`, `python3-colcon-common-extensions`
- Runs as user `ros` (uid 1000), workspace mounted at `/workspace`
- Container uses `--network=host` so ROS2 DDS discovery works with hosts on the LAN
- ROS2 environment is sourced automatically via `~/.bashrc` (`source /opt/ros/humble/setup.bash`)

Open the folder in VS Code and "Reopen in Container" to get the configured ROS, Python, and C++ tooling.

## Expected workflow once packages exist

Standard ROS2 + colcon commands, run from `/workspace` inside the container:

```bash
# Install package dependencies declared in package.xml files
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build --symlink-install

# Build a single package
colcon build --packages-select <package_name>

# Source the overlay before running nodes
source install/setup.bash

# Run package tests
colcon test --packages-select <package_name>
colcon test-result --verbose
```
