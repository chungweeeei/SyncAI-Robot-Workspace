#!/usr/bin/env bash
# =============================================================================
# Fail unless every package in build/ was compiled as a release build. Run by
# the Dockerfile's `builder` stage right after scripts/build.sh, from the
# workspace root.
#
# scripts/build.sh builds Release by default, so this normally passes without
# anyone noticing it. It exists for the day that default is not what ran: a
# `--cmake-args` added to the builder's command line replaces the build type
# rather than adding to it (build.sh's header has why), BUILD_TYPE can be set
# in the environment, and a package can pin its own. None of those fails a
# build or a smoke test -- an unoptimised planner and MPPI start fine and are
# merely several times slower -- so the first sign would be a robot that
# drives badly at a customer's site.
#
# Two kinds of package have a build type:
#   - CMake (ament_cmake and plain cmake): CMAKE_BUILD_TYPE in CMakeCache.txt
#     must be exactly `Release`. Empty is a failure too: that is the
#     configuration with no -O at all.
#   - cargo (ament_cargo): colcon-ros-cargo gives each package its own target
#     directory, build/<pkg>, holding release/ or debug/ by profile. A debug/
#     there means a pass without `--cargo-args --release`.
# ament_python packages (syncai_sys_manager) have none and are not looked at.
# =============================================================================
set -euo pipefail

build="${1:-build}"
[ -d "${build}" ] || { echo "check_build.sh: no ${build}/ here -- run from the workspace root" >&2; exit 2; }

bad=0
cmake_n=0
cargo_n=0

for cache in "${build}"/*/CMakeCache.txt; do
    [ -f "${cache}" ] || continue
    cmake_n=$((cmake_n + 1))
    pkg="$(basename "$(dirname "${cache}")")"
    type="$(sed -n 's/^CMAKE_BUILD_TYPE:[A-Z]*=//p' "${cache}" | sed -n "1,1p")"
    if [ "${type}" != "Release" ]; then
        echo "  NOT RELEASE  ${pkg}: CMAKE_BUILD_TYPE='${type}'"
        bad=$((bad + 1))
    fi
done

# .cargo-lock is what cargo drops in a profile directory it has built into, so
# an empty debug/ left by something else does not count.
for dir in "${build}"/*/; do
    pkg="$(basename "${dir}")"
    if [ -f "${dir}debug/.cargo-lock" ]; then
        echo "  NOT RELEASE  ${pkg}: cargo built the dev profile (${dir}debug)"
        bad=$((bad + 1))
        cargo_n=$((cargo_n + 1))
    elif [ -f "${dir}release/.cargo-lock" ]; then
        cargo_n=$((cargo_n + 1))
    fi
done

# A build/ this finds nothing in is a build that did not happen, or a layout
# this script no longer understands. Either way it has checked nothing.
if [ "${cmake_n}" -eq 0 ] || [ "${cargo_n}" -eq 0 ]; then
    echo "check_build.sh: found ${cmake_n} CMake and ${cargo_n} cargo package(s) in ${build}/ --" \
         "expected both kinds; nothing was verified." >&2
    exit 2
fi

if [ "${bad}" -gt 0 ]; then
    echo "check_build.sh: ${bad} package(s) are not release builds (of ${cmake_n} CMake, ${cargo_n} cargo)." >&2
    exit 1
fi
echo "check_build.sh: ${cmake_n} CMake and ${cargo_n} cargo packages, all release builds."
