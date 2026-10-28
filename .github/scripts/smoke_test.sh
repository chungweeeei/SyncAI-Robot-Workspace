#!/usr/bin/env bash
# =============================================================================
# CI smoke test: bring up each session spec's panes and check that every node
# comes up and stays up. Run by the `smoke` job of .github/workflows/ci.yml,
# inside the robot image, after `build` has produced install/.
#
# What it proves: every launch in config/sessions/start_nav.yaml,
# start_mapping.yaml and calib_lidar_camera.yaml starts, its plugins load, its parameters pass their own
# checks, and nothing dies in the first SETTLE seconds. That is the class of
# bug a green colcon build does not catch -- an undefined symbol that only
# surfaces when pluginlib dlopen()s a Release .so (PR #32), a stale plugin
# name, MPPI throwing from initialize() on a bad model_dt.
#
# What it does not prove: anything that needs data. There is no lidar, gait
# controller or camera here, so nothing localizes, plans or moves.
#
# The nav session is brought up on a 3D-only map (octomap.bt + map.pcd, no
# gridmap.yaml), the kind OctomapPlanner3D navigates on. From 2026-10-19 it
# was brought up twice, on a 2D map too; the 2D pass went with the nav
# session's map_server window on 2026-10-28, the first step of deprecating 2D
# navigation. Besides "nothing died", the pass checks that Octo3D built its
# surface map (EXPECT_3D). The map is switched for it by rewriting the INI's
# `name:` line in place, as the backend's map switch does.
#
# The panes are read from the specs themselves, not from a list kept here, so
# a window added to a spec is tested without touching this file. Each pane's
# `sleep` is honoured the way NodeManager does it (`sleep N && cmd`, every pane
# timed from session start), since startup order is all there is in place of a
# lifecycle manager. SKIP_LOGS names the panes that cannot start without
# hardware.
#
# NEVER run this on a robot: it starts the whole stack, driver_manager
# included, in whatever DDS domain it finds. It refuses unless CI=true and the
# INI's robot_id is ci_smoke, which the workflow mounts over config/system.ini.
# =============================================================================
set -uo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."

SETTLE="${SETTLE:-45}"
# camera_feeder opens /dev/syncai/camera0, which a runner does not have; its
# launch respawns it every 2 s, so it would neither stay down nor come up.
SKIP_LOGS="${SKIP_LOGS:-camera_feeder}"
MAP_NAME=ci_smoke
MAP_NAME_3D=ci_smoke_3d
OUT=log/smoke
# What the 3D-only pass must also show, as "<log> <TAB> <extended regex>":
# Octo3D's surface map built from the fixture's octomap.bt, and the
# task_runner having heard Octo3D's map_available and switched its goals to
# the *_3d.xml trees (the per-goal tree choice, wired end to end), and Octo3D
# having read the fixture's keepout.json (the 3D zones: the derivation of
# the file beside the tree and the summary line, which the gtests never
# touch since they parse strings).
EXPECT_3D="$(printf '%s\t%s\n' \
    planner     'surface map: [0-9]+ x [0-9]+ columns' \
    planner     'keepout zones from .*keepout\.json: 1 zones, [0-9]+ cells blocked \([0-9]+ core\), 0 stair links cut' \
    task_runner '3D map \(.*map_available\) -- goals run the \*_3d\.xml trees')"

# A node that dies in ros2 launch is reported as "process has died"; the rest
# are what a crash in a constructor or a dlopen() looks like before it does.
CRASH_RE='process has died|Segmentation fault|core dumped|undefined symbol|Failed to load library|terminate called|std::bad_alloc|Traceback \(most recent call last\)'

die() { printf 'smoke_test.sh: %s\n' "$*" >&2; exit 2; }

[ "${CI:-}" = "true" ] || die "CI is not true -- this starts the whole stack; it is not for a robot."
grep -Eq '^robot_id:[[:space:]]*ci_smoke[[:space:]]*$' config/system.ini || \
    die "config/system.ini is not the ci_smoke INI (the workflow mounts it)."
[ -w config/system.ini ] || \
    die "config/system.ini is not writable -- the 3D pass switches its [map] name (mount it rw)."

set +u
source /opt/ros/humble/setup.bash
source "${ROS2_RUST_UNDERLAY:-/opt/ros2_rust_underlay}/install/setup.bash"
source install/setup.bash
set -u

# --- fixture map -------------------------------------------------------------
# syncai_localizer opens its map in the constructor and dies without one (and
# its launch starts nothing when the pcd is missing), so the nav session needs
# a map on disk. A 5 x 5 m walled room, 2 m high, with intensity because the
# localizer loads PointXYZI; the mapping and calibration passes run on it too,
# though neither reads it. Generated rather than committed: map/ is gitignored
# and these are throwaway. It has no gridmap since 2026-10-28: nothing in the
# nav session serves one any more.
mkdir -p "map/${MAP_NAME}"
python3 - "map/${MAP_NAME}" <<'EOF'
import sys
d = sys.argv[1]
pts = []
steps = [i * 0.05 - 2.5 for i in range(101)]
for z in [i * 0.05 for i in range(41)]:
    for s in steps:
        pts += [(s, -2.5, z), (s, 2.5, z), (-2.5, s, z), (2.5, s, z)]
with open(f"{d}/map.pcd", "w") as f:
    f.write("VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
            f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA ascii\n")
    f.writelines(f"{x:.3f} {y:.3f} {z:.3f} 0\n" for x, y, z in pts)
with open(f"{d}/poses.txt", "w") as f:
    pass
EOF

# The 3D-only fixture: the same walls as map.pcd for the localizer, an
# octomap.bt for Octo3D and no gridmap.yaml / keepout.yaml (the 2D mask) at
# all -- but a keepout.json, the 3D zones, one box on the ground floor. The
# tree is the planner CLI's two_floors scene (a 12 x 8 m floor and a second
# one 5 m above it), written by the CLI that was just built, so the fixture
# is the same OcTree format clean_map writes. poses.txt is empty: no stair
# links, which Octo3D logs as a warning and boots without.
mkdir -p "map/${MAP_NAME_3D}"
cp "map/${MAP_NAME}/map.pcd" "map/${MAP_NAME_3D}/map.pcd"
: > "map/${MAP_NAME_3D}/poses.txt"
rm -f "map/${MAP_NAME_3D}/gridmap.yaml" "map/${MAP_NAME_3D}/keepout.yaml"
ros2 run syncai_global_planner octomap_planner_3d_cli --synthetic two_floors \
    --write-bt "map/${MAP_NAME_3D}/octomap.bt" > /dev/null || \
    die "cannot write the 3D fixture's octomap.bt"
cat > "map/${MAP_NAME_3D}/keepout.json" <<'JSON'
{ "version": 1,
  "zones": [ { "id": "smoke", "name": "a box on the ground floor",
               "points": [{"x": 3.0, "y": 3.0}, {"x": 4.0, "y": 3.0},
                          {"x": 4.0, "y": 4.0}, {"x": 3.0, "y": 4.0}],
               "z_min": -0.5, "z_max": 0.5 } ] }
JSON

# Point the INI's [map] name at another map, in place: config/system.ini is a
# single-file bind mount, and a rename (sed -i) cannot replace one.
set_map_name() {
    python3 - config/system.ini "$1" <<'PY'
import re, sys
path, name = sys.argv[1], sys.argv[2]
with open(path, "r+") as f:
    text = f.read()
    new, n = re.subn(r"(?m)^name:.*$", f"name: {name}", text)
    if n != 1:
        sys.exit(f"expected one 'name:' line in {path}, found {n}")
    f.seek(0)
    f.write(new)
    f.truncate()
PY
}

# --- one spec ----------------------------------------------------------------
failures=0

# run_spec <spec> [<label>] [<expectations>]: <label> names the log directory
# (default: the spec's basename); <expectations> are "<log>\t<regex>" lines,
# each of which that pane's log must match.
run_spec() {
    local spec="$1"
    local name="${2:-$(basename "${spec}" .yaml)}"
    local expect="${3:-}"
    local dir="${OUT}/${name}"
    rm -rf "${dir}"; mkdir -p "${dir}"
    printf '\n==> %s (%s)\n' "${spec}" "${name}"

    # sleep <TAB> log <TAB> cmd, one line per pane.
    local panes
    panes="$(python3 - "${spec}" <<'EOF'
import sys, yaml
spec = yaml.safe_load(open(sys.argv[1]))
for w in spec["windows"]:
    for p in w["panes"]:
        if p.get("enter", True) is False:
            continue
        log = p.get("log") or w["name"]
        print(f"{p.get('sleep', 0)}\t{log}\t{p['cmd']}")
EOF
)" || die "cannot parse ${spec}"

    local -a pids=() logs=()
    local sleep log cmd base
    while IFS=$'\t' read -r sleep log cmd; do
        base="${log##*/}"
        if [[ " ${SKIP_LOGS} " == *" ${base} "* ]]; then
            echo "  skip  ${log}  (${cmd})"
            continue
        fi
        echo "  start ${log}  (+${sleep}s) ${cmd}"
        # Own session per pane, so the cleanup below takes the launch and every
        # node under it down as one process group.
        setsid bash -c "sleep ${sleep} && exec ${cmd}" > "${dir}/${base}.log" 2>&1 &
        pids+=("$!"); logs+=("${base}")
    done <<< "${panes}"

    sleep "${SETTLE}"

    echo "  -- nodes"
    # grep: config/cyclonedds.xml traces to stdout, so the CLI's own
    # participant prints its whole config ahead of the node names.
    ros2 node list 2>/dev/null | grep '^/' | sort | tee "${dir}/nodes.txt" | sed 's/^/     /'

    local i ok=1
    for i in "${!pids[@]}"; do
        local f="${dir}/${logs[$i]}.log" bad=""
        kill -0 "${pids[$i]}" 2>/dev/null || bad="launch exited"
        if grep -Eq "${CRASH_RE}" "${f}"; then bad="${bad:+${bad}; }crash in log"; fi
        if [ -n "${bad}" ]; then
            ok=0
            echo "::error title=smoke ${name}::${logs[$i]}: ${bad}"
            echo "  ---- ${f} (last 60 lines) ----"
            tail -n 60 "${f}" | sed 's/^/     /'
        else
            echo "  ok    ${logs[$i]}"
        fi
    done

    local elog eregex
    while IFS=$'\t' read -r elog eregex; do
        [ -n "${elog}" ] || continue
        if grep -Eq "${eregex}" "${dir}/${elog}.log" 2>/dev/null; then
            echo "  ok    ${elog}: /${eregex}/"
        else
            ok=0
            echo "::error title=smoke ${name}::${elog}: log does not show /${eregex}/"
            echo "  ---- ${dir}/${elog}.log (last 40 lines) ----"
            tail -n 40 "${dir}/${elog}.log" 2>/dev/null | sed 's/^/     /'
        fi
    done <<< "${expect}"

    # SIGINT first, as byobu's kill does, then make sure.
    for i in "${pids[@]}"; do kill -INT -- "-${i}" 2>/dev/null; done
    sleep 5
    for i in "${pids[@]}"; do kill -KILL -- "-${i}" 2>/dev/null; done
    wait 2>/dev/null
    ros2 daemon stop >/dev/null 2>&1

    [ "${ok}" = 1 ] || failures=$((failures + 1))
}

set_map_name "${MAP_NAME}" || die "cannot set the INI's [map] name"
run_spec config/sessions/start_mapping.yaml
set_map_name "${MAP_NAME_3D}" || die "cannot set the INI's [map] name"
run_spec config/sessions/start_nav.yaml start_nav_3d "${EXPECT_3D}"
set_map_name "${MAP_NAME}"
# Only bringup is left of it once camera_feeder is skipped, but that is the
# pane its livox_xfer_format:=0 argument goes to.
run_spec config/sessions/calib_lidar_camera.yaml

if [ "${failures}" -gt 0 ]; then
    echo "smoke test: ${failures} session pass(es) had a node that did not come up."
    exit 1
fi
echo "smoke test: every node came up in all three passes (mapping, nav 3D-only, calibration)."
