#!/usr/bin/env bash
# =============================================================================
# Fail if any binary under an install/ links a shared library this image does
# not have. Run by the Dockerfile's `release` stage against the install/ it
# just copied in, and by hand in any image to see what a new dependency needs:
#
#   scripts/release/check_libs.sh [install-dir]     # default: install
#
# Why it exists: the release image is runtime-base plus install/, while the
# install/ was compiled in dev, which has every -dev package and the library
# behind each. A library that only dev has makes a build that passes and a
# node that dies at start with "error while loading shared libraries" -- on
# the robot, since nothing before it runs the binary. This is the build-time
# version of that failure, with the file and the library named.
#
# Every ELF file is checked, not only lib/ and bin/: pluginlib plugins,
# typesupport libraries and Python extension modules are loaded by path at run
# time and sit all over the tree.
#
# What it cannot see: anything opened by name while running -- a GStreamer
# element, an rmw implementation, a program a node shells out to (byobu,
# multilog, nmcli). The CI smoke test covers the nodes it can start; the camera
# feeder it cannot, and that one is checked on a robot.
# =============================================================================
set -eo pipefail

prefix="${1:-install}"
[ -f "${prefix}/setup.bash" ] || { echo "check_libs.sh: no ${prefix}/setup.bash" >&2; exit 2; }

# The workspace's own libraries resolve through LD_LIBRARY_PATH, which the
# three setup files build up -- the same three, in the same order, as the
# image's CMD. (No `set -u`: ROS's setup scripts read variables they never set.)
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROS2_RUST_UNDERLAY:-/opt/ros2_rust_underlay}/install/setup.bash"
# shellcheck disable=SC1090
source "${prefix}/setup.bash"

checked=0
missing=0
while IFS= read -r -d '' file; do
    # ELF magic, read with od so that a binary's NUL bytes never pass through
    # a shell variable.
    [ "$(od -An -tx1 -N4 "${file}" 2>/dev/null | tr -d ' \n')" = "7f454c46" ] || continue
    checked=$((checked + 1))
    # ldd exits non-zero on a static executable; only its "not found" lines
    # are of interest.
    while IFS= read -r line; do
        echo "  MISSING  ${file}: ${line}"
        missing=$((missing + 1))
    done < <(ldd "${file}" 2>/dev/null | grep 'not found' | sed 's/^[[:space:]]*//' || true)
done < <(find -L "${prefix}" -type f -print0)

if [ "${checked}" -eq 0 ]; then
    echo "check_libs.sh: no ELF file under ${prefix}/ -- nothing was verified." >&2
    exit 2
fi
if [ "${missing}" -gt 0 ]; then
    echo "check_libs.sh: ${missing} unresolved librar(y/ies) across ${checked} binaries." \
         "Add the package that ships each one to the Dockerfile's runtime-base stage." >&2
    exit 1
fi
echo "check_libs.sh: ${checked} binaries, every shared library resolves."
