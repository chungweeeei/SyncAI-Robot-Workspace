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

# Kill existing session if any
byobu kill-session -t "$SESSION_NAME" 2>/dev/null

# ---------- Window 0: bringup (scan merger + static TFs) ----------
byobu new-session -d -s "$SESSION_NAME" -n "bringup" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:bringup" \
  "ros2 launch syncai_bringup bringup_2d.launch.py" Enter

# ---------- Window 1: map_server / amcl ----------
byobu new-window -t "$SESSION_NAME" -n "localization" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:localization" \
  "ros2 launch syncai_map_server map_server.launch.py" Enter
byobu split-window -v -t "$SESSION_NAME:localization" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:localization.1" \
  "sleep 2 && ros2 launch syncai_amcl amcl.launch.py" Enter

# ---------- Window 2: costmap filter info + keepout mask servers ----------
byobu new-window -t "$SESSION_NAME" -n "filter_info" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:filter_info" \
  "ros2 launch syncai_map_server costmap_filter_info.launch.py" Enter

# ---------- Window 3: planner / controller ----------
byobu new-window -t "$SESSION_NAME" -n "plan_ctrl" -c "$WS_DIR"
byobu send-keys -t "$SESSION_NAME:plan_ctrl" \
  "sleep 4 && ros2 launch syncai_planner planner_server.launch.py" Enter
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
byobu send-keys -t "$SESSION_NAME:rviz" "rviz2"
byobu split-window -v -t "$SESSION_NAME:rviz" -c "$WS_DIR"

# Go back to window 0 and attach
byobu select-window -t "$SESSION_NAME:bringup"
byobu attach-session -t "$SESSION_NAME"
