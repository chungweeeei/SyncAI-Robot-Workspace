#!/usr/bin/env bash
# =============================================================================
# Build the workspace with colcon. Everything under src/ is a package colcon
# knows how to build on its own — ament_cmake, plain CMake, ament_python, and
# since 2026-10 four ament_cargo packages (syncai_driver_manager,
# syncai_robot_state, syncai_lio_bridge and syncai_camera_feeder, through the
# colcon-cargo / colcon-ros-cargo the image installs) — so "every ROS 2 package" and "everything colcon finds" are the
# same set. The two exceptions this header
# used to carve out (syncai_frontend, built by npm; syncai_backend, whose deps
# came from pip) both left the workspace in 2026-09 for their own repositories.
#
# Runs INSIDE the robot image, either as the entrypoint of the one-shot
# service in docker-compose.build.yml
#
#   docker compose -f docker-compose.build.yml run --rm build [colcon args...]
#
# or by hand in the live robot container
#
#   docker compose exec robot01 scripts/build.sh [colcon args...]
#
# Every argument is appended to `colcon build --symlink-install` plus the
# build-type arguments below, so `--packages-select syncai_global_planner` or
# `--parallel-workers 2` work as they would on a bare colcon invocation.
#
# Steps, each switchable through the environment (defaults in parentheses):
#   BUILD_ROSDEP    check|install|off  (check)   see below
#   BUILD_COLCON    1|0                (1)       off = run only the checks above it
#                                                (and skip the Rust-source prune
#                                                that follows the build)
#   BUILD_TYPE      Release|RelWithDebInfo|Debug|MinSizeRel|off  (Release)
#
# BUILD_TYPE is a release build by default. Without it every ament_cmake
# package configures with an empty CMAKE_BUILD_TYPE -- no -O at all -- and only
# the three packages that guard it in their own CMakeLists (syncai_pointlio,
# syncai_mapping, syncai_localizer) came out optimised; the nav stack (costmap,
# planner, MPPI, which samples thousands of trajectories per cycle) did not.
# The same goes for the ament_cargo packages, which colcon-ros-cargo builds as
# cargo's dev profile unless --release is passed. So the value becomes both
# `--cmake-args -DCMAKE_BUILD_TYPE=<type>` and, for anything but Debug,
# `--cargo-args --release`; colcon extends colcon.meta's per-package
# cmake-args with the command line's, so livox_ros_driver2 keeps its flags.
# `off` passes neither, which is a plain `colcon build`.
#
# Passing your own `--cmake-args` / `--cargo-args` among the arguments REPLACES
# these (argparse keeps the last occurrence of an option), so spell the build
# type out in them too, or set BUILD_TYPE=off and own them entirely.
# Switching type on an existing build/ needs no clean: -D overrides the cached
# value and cargo keeps one target directory per profile.
#
# `rosdep check` rather than `rosdep install` by default: when this runs in the
# throwaway build container, anything apt installs there is gone at exit and
# never reaches robot01 — the build would pass here and the node would die at
# runtime on a missing .so. A missing dependency is a Dockerfile change, and
# the check prints the keys to add. `install` exists to get a build through
# while that change is being made. The keys rosdep reports as "cannot locate"
# (libgraphicsmagick++1-dev, python3-assertpy-pip) are satisfied by the image's
# apt lines under names rosdep does not know, and the two it reports as "not
# satisfied" (libomp-dev from small_gicp's manifest -- OpenMP comes from gcc's
# libgomp; python3-pytest-mock, a test_depend) do not stop a build; that
# output is noise, not a failure. GTSAM and livox_sdk2 used to be on the list
# and left it with the fork's manifests in 2026-09 -- the workspace packages
# that need them deliberately do not declare a key (see their package.xml).
# =============================================================================
set -euo pipefail

# Everything below is relative to the workspace root: colcon.meta is only
# found there (colcon's --metas defaults to ./colcon.meta), and the livox
# flags in it are what make livox_ros_driver2 configure at all.
cd "$(dirname "${BASH_SOURCE[0]}")/.."
WS="$(pwd)"

BUILD_ROSDEP="${BUILD_ROSDEP:-check}"
BUILD_COLCON="${BUILD_COLCON:-1}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

step() { printf '\n==> %s\n' "$*"; }
die()  { printf 'build.sh: %s\n' "$*" >&2; exit 1; }

[ -f /opt/ros/humble/setup.bash ] || \
    die "no ROS 2 Humble at /opt/ros/humble — run this inside the robot image" \
        "(docker compose -f docker-compose.build.yml run --rm build), not on the host."

# --- 1. vcs checkouts --------------------------------------------------------
# Two groups of directories in src/ are materialised by vcstool rather than
# tracked here -- src/third-party/ by third-party.repos, our own packages that
# live in their own repos by dependencies.repos -- and all of them are empty in
# a fresh clone. Importing is left to
# the host on purpose: the checkouts are the host's working tree bind-mounted
# in, and a build must not mutate what git sees on the host (the one SSH
# remote, the FAST-LIO2 fork, left in 2026-09 with the localizer port, so
# credentials are no longer the reason).
#
# Checking them up front rather than letting colcon do it, because none of the
# three failure modes says "you forgot to import":
#   - an empty src/third-party dir makes colcon silently build the in-tree
#     packages and fail on the first `find_package` that needed one of them;
#   - an absent src/syncai_common fails every package at once with a message
#     about a missing ament package;
#   - an absent first-party package (syncai_mapping, syncai_driver_manager,
#     syncai_robot_state, syncai_lio_bridge, syncai_camera_feeder) fails
#     NOTHING. colcon happily builds a workspace with nothing to save a map
#     with, no bridge to the gait controller, nothing publishing RobotState, no
#     odometry source and no camera feed, so the mapping session's lio window
#     dies on a missing package, the robot stands still at the first cmd_vel,
#     the console's telemetry never arrives, and the nav session's costmaps
#     wait forever on an odom -> base_link nobody broadcasts.
# This does. The dir:repos table is the one place to add the next package that
# moves out to its own repository (and dependencies.repos the place to pin it).
step "checking vcs checkouts"
missing=""
for entry in third-party/behaviortree_cpp_v3:third-party.repos \
             third-party/Livox-SDK2:third-party.repos \
             third-party/livox_ros_driver2:third-party.repos \
             third-party/small_gicp:third-party.repos \
             third-party/vizionsdk-ros2:third-party.repos \
             third-party/direct_visual_lidar_calibration:third-party.repos \
             syncai_common:dependencies.repos \
             syncai_mapping:dependencies.repos \
             syncai_driver_manager:dependencies.repos \
             syncai_robot_state:dependencies.repos \
             syncai_lio_bridge:dependencies.repos \
             syncai_camera_feeder:dependencies.repos; do
    dir="${entry%%:*}"
    repos="${entry##*:}"
    if [ -z "$(ls -A "src/${dir}" 2>/dev/null)" ]; then
        echo "  MISSING src/${dir}"
        missing="${missing} ${repos}"
    fi
done
if [ -n "${missing}" ]; then
    # Deduplicate: five empty third-party dirs are still one missing import, and
    # so are six empty first-party ones.
    lists="$(printf '%s\n' ${missing} | sort -u | tr '\n' ' ')"
    die "empty vcs checkout(s) — on the HOST run, from the workspace root:" \
        "$(for l in ${lists}; do printf '\n  vcs import --recursive < %s' "$l"; done)"
fi

# direct_visual_lidar_calibration keeps Sophus, nlohmann-json and nanoflann as
# git submodules, and a plain `vcs import` leaves their directories empty. Its
# configure then fails on a missing header deep in the compile, which does not
# say "import with --recursive"; this does. One submodule stands for all three.
calib=src/third-party/direct_visual_lidar_calibration
if [ -z "$(ls -A "${calib}/thirdparty/json" 2>/dev/null)" ]; then
    die "${calib} has no submodules — on the HOST run, from the workspace root:" \
        "$(printf '\n  vcs import --recursive < third-party.repos')"
fi

# livox_ros_driver2 ships package_ROS2.xml and gitignores package.xml (its
# build.sh does the copy). Every re-import therefore deletes the copy and every
# ament package in the workspace fails to configure. Restore it
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
#
# "Underlay" is two layers since 2026-10: /opt/ros/humble, then the image's
# ros2-rust underlay (rclrs + rosidl_generator_rs + the rebuilt standard
# interfaces, see the Dockerfile). Without the second, syncai_common gets no
# Rust bindings and both rclrs packages fail deep in cargo on a crate that
# "cannot be found" -- so its absence is an image that predates it, and is
# reported as that rather than left to cargo.
[ -f "${ROS2_RUST_UNDERLAY:-/opt/ros2_rust_underlay}/install/setup.bash" ] || \
    die "no ros2-rust underlay at ${ROS2_RUST_UNDERLAY:-/opt/ros2_rust_underlay} —" \
        "the image predates it; rebuild it (docker compose build robot01)."
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROS2_RUST_UNDERLAY:-/opt/ros2_rust_underlay}/install/setup.bash"
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
# Validated here rather than left to CMake: a misspelt type ("release") is
# accepted by CMake as a custom configuration with no flags, which is the
# unoptimised build this exists to prevent, with no error anywhere.
type_args=()
case "$BUILD_TYPE" in
    Debug)
        type_args=(--cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}") ;;
    Release|RelWithDebInfo|MinSizeRel)
        type_args=(--cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
                   --cargo-args --release) ;;
    off) ;;
    *) die "BUILD_TYPE must be Release, RelWithDebInfo, Debug, MinSizeRel or off" \
           "(got '${BUILD_TYPE}')" ;;
esac

if [ "$BUILD_COLCON" = "1" ]; then
    step "colcon build --symlink-install ${type_args[*]} $*"
    colcon build --symlink-install "${type_args[@]}" "$@"

    # --- 5. prune the rclrs nodes' installed source ------------------------
    # cargo-ament-build copies every ament_cargo package's Cargo.toml,
    # Cargo.lock and src/ into install/<pkg>/share/<pkg>/rust and drops a
    # `rust_packages` marker in the ament index, unconditionally (it has no
    # flag to skip it). That copy exists so a downstream Rust package can depend
    # on the crate: colcon-ros-cargo turns every marker on AMENT_PREFIX_PATH into
    # a `[patch.crates-io]` entry pointing at that directory. These four are
    # binary-only nodes nothing depends on, and at runtime they need lib/,
    # launch/ and params/ only -- so the copy is dead weight, and source we
    # would rather not ship in an install/ that goes to the customer IPC.
    #
    # The marker goes with the directory, never the directory alone: a marker
    # whose rust/ is gone becomes a patch to a missing path, and cargo fails
    # every later Rust build on it ("failed to read .../rust/Cargo.toml") even
    # though no package uses the patch -- checked 2026-10-12. That bites as
    # soon as install/setup.bash is sourced before a build, which puts all four
    # prefixes on AMENT_PREFIX_PATH.
    #
    # An explicit list, not "every rust_packages marker": syncai_common's
    # share/syncai_common/rust is the rosidl_generator_rs crate every Rust node
    # builds against, and pruning it breaks all four. A plain `colcon build`
    # (not through this script) still leaves the copy; the next run here
    # removes it.
    step "pruning installed Rust source of the rclrs nodes"
    for pkg in syncai_driver_manager syncai_robot_state syncai_lio_bridge \
               syncai_camera_feeder; do
        prefix="install/${pkg}"
        [ -d "${prefix}" ] || continue
        rm -rf "${prefix}/share/${pkg}/rust" \
               "${prefix}/share/ament_index/resource_index/rust_packages/${pkg}"
        echo "  ${pkg}"
    done
fi

step "done: ${WS}"
if [ "$BUILD_COLCON" = "1" ]; then
    echo "  robot01 picks the new install/ up on the next session (re)build:"
    echo "  ros2 service call /<robot_id>/switch_mode syncai_common/srv/SwitchMode \"{mode: 2}\""
fi
