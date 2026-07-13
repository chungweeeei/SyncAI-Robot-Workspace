#!/bin/bash
# =============================================================================
# SyncAI Robot Byobu Session — 3D localization variant (FAST-LIO2)
# Usage: bash scripts/byobu_session_3d.sh   (inside the robot container)
#
# Same layout as byobu_session.sh, but localization comes from the LIO stack
# instead of AMCL:
#   bringup_2d (scan merger, local costmap still uses /<robot_id>/scan)
#   bringup_3d (static base_link -> lidar_top TF for the LIO bridge)
#   localizer_isaac (lio_node + localizer, frames <robot_id>/lio_odom|lio_body)
#   lio_bridge (map -> <robot_id>/odom, the AMCL replacement)
#   planner uses planner_server_3d_params.yml (no keepout filter yet, so the
#   costmap_filter_info window from the 2D session is omitted).
#
# Startup order matters (no lifecycle manager): map_server / LIO ->
# bridge -> planner/controller -> task_runner, so later windows prefix a
# sleep before launching. NOTE: localization is NOT active until you run the
# pre-typed relocalize service call in the "localization" window (pane 2) —
# hit Enter there once the robot is at its known start pose.
# ROS env comes from ~/.bashrc (ros humble + install/setup.bash); every pane
# starts in the workspace root so the relative config/system.ini resolves.
# =============================================================================

SESSION_NAME="syncai3d"

# Workspace root = parent of this script's directory
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# robot_id from the system INI (for the pre-typed relocalize service path).
# INI style is "robot_id: robot06" (configparser accepts ':' or '=').
ROBOT_ID="$(awk -F'[:=]' '/^robot_id/ {gsub(/[ \t]/, "", $2); print $2}' "$WS_DIR/config/system.ini" 2>/dev/null)"
ROBOT_ID="${ROBOT_ID:-default_robot}"

MAP_PCD="$WS_DIR/map/lio_map/map.pcd"

# Kill existing session if any
byobu kill-session -t "$SESSION_NAME" 2>/dev/null

# ---------- Window 0: bringup (scan merger + static TFs, 2D & 3D) ----------
byobu split-window -v -t "$SESSION_NAME:bringup" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:bringup.1" \
  "ros2 launch syncai_bringup bringup_3d.launch.py" Enter

# ---------- Window 1: map_server / LIO localizer / relocalize ----------
byobu new-window -t "$SESSION_NAME" -n "localization" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:localization" \
  "ros2 launch syncai_map_server map_server.launch.py" Enter
byobu split-window -v -t "$SESSION_NAME:localization" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:localization.1" \
  "sleep 2 && ros2 launch localizer localizer_isaac_launch.py rviz:=false" Enter
# Pre-typed (NOT executed): hit Enter here to (re)localize once the stack is
# up. Adjust x/y/yaw to the robot's coarse pose in the map if it is not at
# the mapping start pose.
byobu split-window -v -t "$SESSION_NAME:localization" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:localization.2" \
  "ros2 service call /$ROBOT_ID/localizer/relocalize interface/srv/Relocalize \"{pcd_path: '$MAP_PCD', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}\""

# ---------- Window 2: lio_bridge (map -> <robot_id>/odom, replaces amcl) ----------
byobu new-window -t "$SESSION_NAME" -n "lio_bridge" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:lio_bridge" \
  "sleep 4 && ros2 launch syncai_lio_bridge lio_bridge.launch.py" Enter

# ---------- Window 3: planner / controller ----------
byobu new-window -t "$SESSION_NAME" -n "plan_ctrl" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:plan_ctrl" \
  "sleep 4 && ros2 launch syncai_planner planner_server.launch.py params_file:=$WS_DIR/src/syncai_planner/params/planner_server_3d_params.yml" Enter
byobu split-window -v -t "$SESSION_NAME:plan_ctrl" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:plan_ctrl.1" \
  "sleep 4 && ros2 launch syncai_controller controller_server.launch.py" Enter

# ---------- Window 4: task_runner (needs planner/controller action servers) ----------
byobu new-window -t "$SESSION_NAME" -n "task_runner" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:task_runner" \
  "sleep 10 && ros2 launch syncai_task_runner task_runner.launch.py" Enter

# ---------- Window 5: driver_manager / system_manager ----------
byobu new-window -t "$SESSION_NAME" -n "managers" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:managers" \
  "ros2 launch syncai_driver_manager driver_manager.launch.py" Enter
byobu split-window -v -t "$SESSION_NAME:managers" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:managers.1" \
  "ros2 launch syncai_system_manager system_manager.launch.py" Enter

# ---------- Window 6: robot_state / backend ----------
byobu new-window -t "$SESSION_NAME" -n "state_backend" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:state_backend" \
  "ros2 launch syncai_robot_state robot_state.launch.py" Enter
byobu split-window -v -t "$SESSION_NAME:state_backend" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:state_backend.1" \
  "ros2 launch syncai_backend backend.launch.py" Enter

# ---------- Window 7: rviz (pre-typed, hit Enter to start) / spare shell ----------
byobu new-window -t "$SESSION_NAME" -n "rviz" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:rviz" \
  "rviz2 -d config/rviz2/${ROBOT_ID}.rviz"
byobu split-window -v -t "$SESSION_NAME:rviz" -c "$WS_DIR"

# Go back to window 0 and attach
byobu select-window -t "$SESSION_NAME:bringup"
byobu attach-session -t "$SESSION_NAME"
