#!/bin/bash
# =============================================================================
# SyncAI Robot Byobu Session
# Usage: bash scripts/byobu_session.sh   (inside the robot container)
#
# Brings up the whole nav stack, one launch per pane. Localization comes from
# the FAST-LIO2 chain; AMCL is not used:
#   bringup (robot_state_publisher: base_link -> lidar_top TF for the LIO
#            bridge; plus the MID360 driver)
#   localizer (includes pointlio; frames <robot_id>/lio_odom|lio_body)
#   lio_bridge (map -> <robot_id>/odom, the AMCL replacement)
#   planner / controller run on their default params files — the separate
#   *_3d_params.yaml variants were merged back into them, so there is nothing
#   to override here any more. No keepout filter yet, hence no
#   costmap_filter_info window.
#
# This replaces the old 2D/AMCL session of the same name, which started
# syncai_amcl plus the bringup_2d.launch.py scan merger — both retired.
#
# Startup order matters (no lifecycle manager): map_server / LIO ->
# bridge -> planner/controller -> task_runner, so later windows prefix a
# sleep before launching.
#
# LOCALIZATION: the localizer applies [initial_pose] from the INI as its boot
# guess on the first odom sample, so a robot standing at that pose localizes
# itself with no further action. The relocalize call pre-typed in the
# "localization" window (pane 2) is the fallback for when it is standing
# somewhere else — edit x/y/yaw to its coarse pose and hit Enter. (The console's
# "Set initial pose" tool publishes on the same path.)
#
# ROS env comes from ~/.bashrc (ros humble + install/setup.bash); every pane
# starts in the workspace root so the relative config/system.ini resolves.
# =============================================================================

SESSION_NAME="syncai"

# Workspace root = parent of this script's directory
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Values the pre-typed relocalize call needs, read from the same INI every
# launch file reads. Via python3/configparser rather than awk because [map] pcd
# uses configparser interpolation ("map/%(name)s/map.pcd") — awk would hand the
# service call a path with a literal %(name)s in it.
SYSTEM_INI="$WS_DIR/config/system.ini"
read_ini() {  # $1 = section, $2 = key
  python3 - "$SYSTEM_INI" "$1" "$2" <<'PY' 2>/dev/null
import configparser, sys
cfg = configparser.ConfigParser()
cfg.read(sys.argv[1])
print(cfg.get(sys.argv[2], sys.argv[3], fallback="").strip())
PY
}

ROBOT_ID="$(read_ini system robot_id)"
ROBOT_ID="${ROBOT_ID:-default_robot}"

# [map] pcd is relative to the workspace root, same convention as the localizer
# launch. It is also a hard requirement over there: the localizer loads the map
# during construction, so a missing file means the localizer pane starts nothing
# at all.
MAP_PCD="$(read_ini map pcd)"
case "$MAP_PCD" in
  "") MAP_PCD="$WS_DIR/map/map.pcd" ;;  # no [map] pcd: leave something editable
  /*) ;;
  *) MAP_PCD="$WS_DIR/$MAP_PCD" ;;
esac

# Persistent, size-capped log capture. Each pane's rendered output is tapped
# (copied, so the live pane view is untouched) into its own multilog dir:
#   log/stack/<robot_id>/<name>/{current,@<tai64n>.s.gz}
# 16 MiB/file x 10 rotated files, gzipped (~a week of typical logs). Read back
# with scripts/tailog.sh. log/ is gitignored.
LOG_ROOT="$WS_DIR/log/stack/$ROBOT_ID"
pipe_log() {  # $1 = pane target (window[.pane]), $2 = log name
  local dir="$LOG_ROOT/$2"
  byobu pipe-pane -o -t "$SESSION_NAME:$1" \
    "mkdir -p '$dir' && exec multilog t s16777215 n10 '!gzip' '$dir'"
}

# Kill existing session if any
byobu kill-session -t "$SESSION_NAME" 2>/dev/null

# ---------- Window 0: bringup (static TFs for the LIO bridge) ----------
byobu new-session -d -s "$SESSION_NAME" -n "bringup" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:bringup" \
  "ros2 launch syncai_bringup bringup.launch.py" Enter
pipe_log "bringup" "bringup"

# ---------- Window 1: map_server / LIO localizer / relocalize ----------
byobu new-window -t "$SESSION_NAME" -n "localization" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:localization" \
  "ros2 launch syncai_map_server map_server.launch.py" Enter
pipe_log "localization.0" "map_server"
byobu split-window -v -t "$SESSION_NAME:localization" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:localization.1" \
  "sleep 2 && ros2 launch localizer localizer_launch.py" Enter
pipe_log "localization.1" "localizer"
# Pre-typed (NOT executed): hit Enter here to (re)localize. Only needed when the
# robot is NOT at the INI's [initial_pose] — that one is applied automatically.
# Adjust x/y/yaw to its coarse pose in the map first.
byobu split-window -v -t "$SESSION_NAME:localization" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:localization.2" \
  "ros2 service call /$ROBOT_ID/localizer/relocalize interface/srv/Relocalize \"{pcd_path: '$MAP_PCD', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}\""

# ---------- Window 2: lio_bridge (map -> <robot_id>/odom, replaces amcl) ----------
byobu new-window -t "$SESSION_NAME" -n "lio_bridge" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:lio_bridge" \
  "sleep 4 && ros2 launch syncai_lio_bridge lio_bridge.launch.py" Enter
pipe_log "lio_bridge" "lio_bridge"

# ---------- Window 3: planner / controller ----------
byobu new-window -t "$SESSION_NAME" -n "plan_ctrl" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:plan_ctrl" \
  "sleep 4 && ros2 launch syncai_planner planner_server.launch.py" Enter
pipe_log "plan_ctrl.0" "planner"
byobu split-window -v -t "$SESSION_NAME:plan_ctrl" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:plan_ctrl.1" \
  "sleep 4 && ros2 launch syncai_controller controller_server.launch.py" Enter
pipe_log "plan_ctrl.1" "controller"

# ---------- Window 4: task_runner (needs planner/controller action servers) ----------
byobu new-window -t "$SESSION_NAME" -n "task_runner" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:task_runner" \
  "sleep 10 && ros2 launch syncai_task_runner task_runner.launch.py" Enter
pipe_log "task_runner" "task_runner"

# ---------- Window 5: driver_manager / system_manager ----------
byobu new-window -t "$SESSION_NAME" -n "managers" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:managers" \
  "ros2 launch syncai_driver_manager driver_manager.launch.py" Enter
pipe_log "managers.0" "driver_manager"
byobu split-window -v -t "$SESSION_NAME:managers" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:managers.1" \
  "ros2 launch syncai_system_manager system_manager.launch.py" Enter
pipe_log "managers.1" "system_manager"

# ---------- Window 6: robot_state / backend ----------
byobu new-window -t "$SESSION_NAME" -n "state_backend" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:state_backend" \
  "ros2 launch syncai_robot_state robot_state.launch.py" Enter
pipe_log "state_backend.0" "robot_state"
byobu split-window -v -t "$SESSION_NAME:state_backend" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:state_backend.1" \
  "ros2 launch syncai_backend backend.launch.py" Enter
pipe_log "state_backend.1" "backend"

# ---------- Window 7: rviz (pre-typed, hit Enter to start) / spare shell ----------
byobu new-window -t "$SESSION_NAME" -n "rviz" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:rviz" \
  "rviz2 -d config/rviz2/${ROBOT_ID}.rviz"
byobu split-window -v -t "$SESSION_NAME:rviz" -c "$WS_DIR"

# Go back to window 0 and attach
byobu select-window -t "$SESSION_NAME:bringup"
byobu attach-session -t "$SESSION_NAME"
