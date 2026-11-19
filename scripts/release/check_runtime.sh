#!/usr/bin/env bash
# =============================================================================
# The checks of a release image that `ldd` cannot make. Run by the Dockerfile's
# `release` stage after check_libs.sh, from the workspace root inside the
# image; a failure fails the image build.
#
# check_libs.sh proves every binary can be loaded. This is for what the stack
# reaches by NAME while it runs, each of which is installed by a line in the
# Dockerfile's runtime-base stage and, if that line is lost, takes down
# something that neither a build nor the CI smoke test would notice:
#
#   1. programs it shells out to. No byobu: switch_mode builds no session and
#      the robot comes up doing nothing. No multilog: every pane's pipe dies
#      and there are no logs. No nmcli / avahi-publish: the wifi and mDNS
#      managers fail. The smoke test starts the panes itself, without
#      sys_manager, so it goes through none of them.
#   2. the GStreamer elements of syncai_camera_feeder's pipeline. They are
#      resolved when the pipeline string is parsed, the feeder needs a camera
#      to get that far, and CI has none (the smoke test skips its pane). The
#      Tegra elements pipeline.rs also names (nvv4l2decoder, nvjpegenc) are
#      not checked: no image has them, the nvidia runtime injects them on the
#      robot.
#   3. syncai_sys_manager, the container's main process, imports. Its install
#      strips the .py sources and keeps bytecode (InstallNoSource), which only
#      a build without --symlink-install ever exercises -- so only this image.
#   4. every `ros2 launch <pkg> <file>` in config/sessions/*.yaml exists in
#      install/. The release skips packages (the Dockerfile's builder stage);
#      a spec that names one of them would start an empty pane.
#   5. install/ is what a release should be: no link into src/ or build/ (a
#      --symlink-install tree that got in), no Python source of sys_manager,
#      no Rust source of the rclrs nodes.
#
# It still starts no node and opens no device. What is left for a robot: the
# camera actually streaming, the charger's marker actually detected.
# =============================================================================
set -eo pipefail

prefix="${1:-install}"
[ -f "${prefix}/setup.bash" ] || { echo "check_runtime.sh: no ${prefix}/setup.bash -- run from the workspace root" >&2; exit 2; }

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROS2_RUST_UNDERLAY:-/opt/ros2_rust_underlay}/install/setup.bash"
# shellcheck disable=SC1090
source "${prefix}/setup.bash"

bad=0
fail() { echo "  FAIL  $*"; bad=$((bad + 1)); }

echo "-- programs the stack runs"
for tool in byobu tmux multilog gzip nice nmcli avahi-publish sudo ros2 gst-inspect-1.0; do
    command -v "${tool}" >/dev/null || fail "${tool} is not on PATH"
done

echo "-- GStreamer elements of the camera feeder"
if command -v gst-inspect-1.0 >/dev/null; then
    for element in v4l2src identity tee queue appsink jpegdec jpegenc rtpjpegpay udpsink multiudpsink; do
        gst-inspect-1.0 --exists "${element}" || fail "no GStreamer element '${element}'"
    done
fi

echo "-- syncai_sys_manager imports"
python3 -c 'import syncai_sys_manager.main, syncai_sys_manager.managers.node_manager' || \
    fail "python3 cannot import syncai_sys_manager (see the traceback above)"

echo "-- every launch the session specs name"
specs=(config/sessions/*.yaml)
if [ ! -f "${specs[0]}" ]; then
    fail "no config/sessions/*.yaml in $(pwd)"
else
    while IFS=$'\t' read -r spec pkg file; do
        [ -n "${pkg}" ] || continue
        share="$(ros2 pkg prefix --share "${pkg}" 2>/dev/null)" || { fail "${spec}: package '${pkg}' is not in ${prefix}/"; continue; }
        [ -n "$(find -L "${share}" -name "${file}" -type f 2>/dev/null | sed -n "1,1p")" ] || \
            fail "${spec}: ${pkg} has no ${file}"
    done < <(python3 - "${specs[@]}" <<'EOF'
import re, sys, yaml
for path in sys.argv[1:]:
    spec = yaml.safe_load(open(path))
    for window in spec["windows"]:
        for pane in window["panes"]:
            m = re.search(r"ros2 launch (\S+) (\S+)", pane["cmd"])
            if m:
                print(f"{path}\t{m.group(1)}\t{m.group(2)}")
EOF
)
fi

echo "-- install/ carries no source and no link out of itself"
# The first five of each, through `sed -n "1,5p"` and not `head -n 5`: under
# pipefail, head closing the pipe early kills find with SIGPIPE and `set -e`
# then ends the script without a word -- on exactly the tree this has the most
# to say about (a --symlink-install one has over a thousand such links).
# A link whose target is under a src/ or build/ directory is --symlink-install's.
links="$(find "${prefix}" -type l -lname '*/src/*' -o -type l -lname '*/build/*' | sed -n "1,5p")"
[ -z "${links}" ] || fail "links into src/ or build/ (a --symlink-install tree?):"$'\n'"${links}"
dangling="$(find "${prefix}" -xtype l | sed -n "1,5p")"
[ -z "${dangling}" ] || fail "dangling links:"$'\n'"${dangling}"
py="$(find "${prefix}/syncai_sys_manager" -path '*site-packages/syncai_sys_manager/*' -name '*.py' | sed -n "1,5p")"
[ -z "${py}" ] || fail "syncai_sys_manager's Python source was installed (InstallNoSource did not run):"$'\n'"${py}"
for pkg in syncai_driver_manager syncai_robot_state syncai_lio_bridge syncai_camera_feeder; do
    [ -d "${prefix}/${pkg}" ] || { fail "${pkg} is not in ${prefix}/"; continue; }
    [ ! -d "${prefix}/${pkg}/share/${pkg}/rust" ] || fail "${pkg}: its Rust source is in share/${pkg}/rust (build.sh's prune did not run)"
done

if [ "${bad}" -gt 0 ]; then
    echo "check_runtime.sh: ${bad} check(s) failed." >&2
    exit 1
fi
echo "check_runtime.sh: all checks passed."
