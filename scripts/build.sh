#!/usr/bin/env bash
# =============================================================================
# Build every ROS 2 package in the workspace with colcon.
#
# Runs INSIDE the robot image, either as the entrypoint of the one-shot
# service in docker-compose.build.yaml
#
#   docker compose -f docker-compose.build.yaml run --rm build [colcon args...]
#
# or by hand in the live robot container
#
#   docker compose exec robot01 scripts/build.sh [colcon args...]
#
# Every argument is appended to `colcon build --symlink-install`, so
# `--packages-select syncai_planner` or `--parallel-workers 2` work as they
# would on a bare colcon invocation.
#
# Steps, each switchable through the environment (defaults in parentheses):
#   BUILD_ROSDEP    check|install|off  (check)   see below
#   BUILD_COLCON    1|0                (1)       off = run only the checks above it
#
# `rosdep check` rather than `rosdep install` by default: when this runs in the
# throwaway build container, anything apt installs there is gone at exit and
# never reaches robot01 — the build would pass here and the node would die at
# runtime on a missing .so. A missing dependency is a Dockerfile change, and
# the check prints the keys to add. `install` exists to get a build through
# while that change is being made. The keys rosdep reports as "cannot locate"
# (GTSAM, livox_sdk2, libgraphicsmagick++1-dev, python3-assertpy-pip,
# python3-structlog) are
# satisfied by the image's deps-builder stage / apt lines under names rosdep
# does not know; that output is noise, not a failure.
# =============================================================================
set -euo pipefail

# Everything below is relative to the workspace root: colcon.meta is only
# found there (colcon's --metas defaults to ./colcon.meta), and the livox
# flags in it are what make livox_ros_driver2 configure at all.
cd "$(dirname "${BASH_SOURCE[0]}")/.."
WS="$(pwd)"

BUILD_ROSDEP="${BUILD_ROSDEP:-check}"
BUILD_COLCON="${BUILD_COLCON:-1}"

step() { printf '\n==> %s\n' "$*"; }
die()  { printf 'build.sh: %s\n' "$*" >&2; exit 1; }

[ -f /opt/ros/humble/setup.bash ] || \
    die "no ROS 2 Humble at /opt/ros/humble — run this inside the robot image" \
        "(docker compose -f docker-compose.build.yaml run --rm build), not on the host."

# --- 1. submodules -----------------------------------------------------------
# A fresh clone without `git submodule update --init --recursive` leaves the
# six src/third-party dirs empty; colcon then silently builds the in-tree
# packages and fails on the first `find_package` that needed one of them.
# Cloning is left to the host on purpose: FASTLIO2_ROS2 is an SSH remote and
# the container has no key.
step "checking submodules"
missing=0
for sub in behaviortree_cpp_v3 FASTLIO2_ROS2 Livox-SDK2 livox_ros_driver2 small_gicp vizionsdk-ros2; do
    if [ -z "$(ls -A "src/third-party/${sub}" 2>/dev/null)" ]; then
        echo "  MISSING src/third-party/${sub}"
        missing=1
    fi
done
[ "$missing" -eq 0 ] || \
    die "empty submodule(s) — on the HOST run: git submodule update --init --recursive"

# livox_ros_driver2 ships package_ROS2.xml and gitignores package.xml (its
# build.sh does the copy). Every submodule update therefore deletes the copy
# and every ament package in the workspace fails to configure. Restore it
# when absent; never overwrite one that is there (it may be a ROS1 copy
# someone made on purpose — unlikely, but the copy costs nothing to skip).
livox=src/third-party/livox_ros_driver2
if [ ! -f "${livox}/package.xml" ]; then
    echo "  restoring ${livox}/package.xml from package_ROS2.xml"
    cp -f "${livox}/package_ROS2.xml" "${livox}/package.xml"
fi

# --- 2. ROS environment ------------------------------------------------------
# Underlay only. Sourcing install/setup.bash before building would overlay
# the workspace on itself, which colcon warns about and which can pin stale
# paths into the new build.
# ROS's setup scripts read variables they never set (AMENT_TRACE_SETUP_FILES,
# COLCON_TRACE, ...) and die under `set -u`, so nounset is lifted just around
# the source.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

# --- 3. rosdep ---------------------------------------------------------------
case "$BUILD_ROSDEP" in
    check)
        step "rosdep check (report only — unmet apt keys belong in the Dockerfile)"
        # Exit status deliberately ignored: the unknown-key errors listed in
        # the header make it non-zero on a perfectly buildable image.
        rosdep check --from-paths src --ignore-src || true
        ;;
    install)
        step "rosdep install (into THIS container only)"
        rosdep install --from-paths src --ignore-src -r -y
        ;;
    off) ;;
    *) die "BUILD_ROSDEP must be check, install or off (got '${BUILD_ROSDEP}')" ;;
esac

# --- 4. colcon ---------------------------------------------------------------
if [ "$BUILD_COLCON" = "1" ]; then
    step "colcon build --symlink-install $*"
    colcon build --symlink-install "$@"
fi

step "done: ${WS}"
if [ "$BUILD_COLCON" = "1" ]; then
    echo "  robot01 picks the new install/ up on the next session (re)build:"
    echo "  ros2 service call /<robot_id>/switch_mode syncai_common/srv/SwitchMode \"{mode: 2}\""
fi
