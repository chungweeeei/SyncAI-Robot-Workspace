#!/usr/bin/env bash
# =============================================================================
# CI smoke test: bring up each session spec's panes and check that every node
# comes up and stays up. Run by the `smoke` job of .github/workflows/ci.yml,
# inside the robot image, after `build` has produced install/.
#
# What it proves: every launch in config/sessions/start_nav.yaml and
# start_mapping.yaml starts, its plugins load, its parameters pass their own
# checks, and nothing dies in the first SETTLE seconds. That is the class of
# bug a green colcon build does not catch -- an undefined symbol that only
# surfaces when pluginlib dlopen()s a Release .so (PR #32), a stale plugin
# name, MPPI throwing from initialize() on a bad model_dt.
#
# What it does not prove: anything that needs data. There is no lidar, gait
# controller or camera here, so nothing localizes, plans or moves.
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
OUT=log/smoke

# A node that dies in ros2 launch is reported as "process has died"; the rest
# are what a crash in a constructor or a dlopen() looks like before it does.
CRASH_RE='process has died|Segmentation fault|core dumped|undefined symbol|Failed to load library|terminate called|std::bad_alloc|Traceback \(most recent call last\)'

die() { printf 'smoke_test.sh: %s\n' "$*" >&2; exit 2; }

[ "${CI:-}" = "true" ] || die "CI is not true -- this starts the whole stack; it is not for a robot."
grep -Eq '^robot_id:[[:space:]]*ci_smoke[[:space:]]*$' config/system.ini || \
    die "config/system.ini is not the ci_smoke INI (the workflow mounts it)."

set +u
source /opt/ros/humble/setup.bash
source "${ROS2_RUST_UNDERLAY:-/opt/ros2_rust_underlay}/install/setup.bash"
source install/setup.bash
set -u

# --- fixture map -------------------------------------------------------------
# map_server and syncai_localizer open their map in the constructor and die
# without one (and the localizer launch starts nothing when the pcd is
# missing), so the nav session needs a map on disk. A 5 x 5 m walled room:
# the gridmap is free inside with a lethal border, the pcd is the same walls
# 2 m high, with intensity because the localizer loads PointXYZI. Generated
# rather than committed: map/ is gitignored and these are throwaway.
# keepout.yaml is left out on purpose -- costmap_filter_info writes a blank one,
# and that path is part of what is being tested.
mkdir -p "map/${MAP_NAME}"
python3 - "map/${MAP_NAME}" <<'EOF'
import sys
d = sys.argv[1]
n, res = 100, 0.05
rows = []
for y in range(n):
    rows.append(bytes(0 if x in (0, n - 1) or y in (0, n - 1) else 254 for x in range(n)))
with open(f"{d}/gridmap.pgm", "wb") as f:
    f.write(b"P5\n%d %d\n255\n" % (n, n) + b"".join(rows))
with open(f"{d}/gridmap.yaml", "w") as f:
    f.write("image: gridmap.pgm\nmode: trinary\nresolution: 0.05\n"
            "origin: [-2.5, -2.5, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
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

# --- one spec ----------------------------------------------------------------
failures=0

run_spec() {
    local spec="$1"
    local name; name="$(basename "${spec}" .yaml)"
    local dir="${OUT}/${name}"
    rm -rf "${dir}"; mkdir -p "${dir}"
    printf '\n==> %s\n' "${spec}"

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

    # SIGINT first, as byobu's kill does, then make sure.
    for i in "${pids[@]}"; do kill -INT -- "-${i}" 2>/dev/null; done
    sleep 5
    for i in "${pids[@]}"; do kill -KILL -- "-${i}" 2>/dev/null; done
    wait 2>/dev/null
    ros2 daemon stop >/dev/null 2>&1

    [ "${ok}" = 1 ] || failures=$((failures + 1))
}

run_spec config/sessions/start_nav.yaml
run_spec config/sessions/start_mapping.yaml

if [ "${failures}" -gt 0 ]; then
    echo "smoke test: ${failures} session(s) had a node that did not come up."
    exit 1
fi
echo "smoke test: every node came up in both sessions."
