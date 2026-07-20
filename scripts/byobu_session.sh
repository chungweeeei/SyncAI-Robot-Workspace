#!/bin/bash
# =============================================================================
# SyncAI Robot Byobu Session
# Usage: bash scripts/byobu_session.sh   (inside the robot container)
#
# Brings up the whole nav stack, one launch per pane. Startup order matters
# (no lifecycle manager): map_server -> amcl -> planner/controller ->
# task_runner, so the later windows prefix a sleep before launching.
# ROS env comes from ~/.bashrc (ros humble + install/setup.bash); every pane
# starts in the workspace root so the relative config/system.ini resolves.
# =============================================================================

SESSION_NAME="syncai"

# Workspace root = parent of this script's directory
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# robot_id from the system INI (namespaces the per-pane log dirs so sim robots
# sharing the same workspace bind mount don't clobber each other's logs).
# INI style is "robot_id: robot06" (configparser accepts ':' or '=').
ROBOT_ID="$(awk -F'[:=]' '/^robot_id/ {gsub(/[ \t]/, "", $2); print $2}' "$WS_DIR/config/system.ini" 2>/dev/null)"
ROBOT_ID="${ROBOT_ID:-default_robot}"

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

# ---------- Window 0: bringup (scan merger + static TFs) ----------
byobu new-session -d -s "$SESSION_NAME" -n "bringup" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:bringup" \
  "ros2 launch syncai_bringup bringup_2d.launch.py" Enter
pipe_log "bringup" "bringup"

# ---------- Window 1: map_server / amcl ----------
byobu new-window -t "$SESSION_NAME" -n "localization" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:localization" \
  "ros2 launch syncai_map_server map_server.launch.py" Enter
pipe_log "localization.0" "map_server"
byobu split-window -v -t "$SESSION_NAME:localization" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:localization.1" \
  "sleep 2 && ros2 launch syncai_amcl amcl.launch.py" Enter
pipe_log "localization.1" "amcl"

# ---------- Window 2: costmap filter info + keepout mask servers ----------
byobu new-window -t "$SESSION_NAME" -n "filter_info" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:filter_info" \
  "ros2 launch syncai_map_server costmap_filter_info.launch.py" Enter
pipe_log "filter_info" "filter_info"

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
byobu send-keys -t "$SESSION_NAME:rviz" "rviz2"
byobu split-window -v -t "$SESSION_NAME:rviz" -c "$WS_DIR"

# Go back to window 0 and attach
byobu select-window -t "$SESSION_NAME:bringup"
byobu attach-session -t "$SESSION_NAME"
