#!/usr/bin/env bash
# Attach to the robot stack's live byobu session from the HOST.
#
#   scripts/attach.sh [container] [session]
#
# Replaces typing `docker exec -it robot01 byobu attach -t syncai-dev` by hand,
# which was both long and wrong half the time: the session name follows the
# operating mode (syncai-dev in AUTO, syncai-mapping in MANUAL — see
# config/sessions/*.yaml), so a hardcoded alias fails as soon as someone
# switches to mapping. NodeManager never stores the mode either; it derives it
# from which session byobu actually has, and this script does the same.
#
# The lookup uses `tmux list-sessions`, not `byobu list-sessions`. byobu is a
# wrapper over tmux talking to the same server, but calling tmux directly skips
# byobu's profile initialisation and stays silent when run without a TTY (this
# `docker exec` has no -it). The attach itself still goes through byobu so the
# status bar and F-keys come up as the operator expects.
#
# Intended to sit behind a shell alias on the host, e.g. in ~/.bashrc:
#   alias robot='~/SyncAI-Robot-Workspace/scripts/attach.sh'
set -euo pipefail

container="${1:-robot01}"
session="${2:-}"

if ! docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null | grep -q true; then
  echo "attach.sh: container '$container' is not running (docker compose up -d?)" >&2
  exit 1
fi

if [[ -z "$session" ]]; then
  # Normally exactly one session exists. Two means someone built both by hand,
  # which get_mode reports as ambiguous and switch_mode cleans up — here we
  # just take the first and let the operator sort it out from inside.
  session="$(docker exec "$container" tmux list-sessions -F '#S' 2>/dev/null | head -n1 || true)"
fi

if [[ -z "$session" ]]; then
  # No session is what MAINTENANCE looks like (or sys_manager is still coming
  # up). A plain shell is more useful than byobu's "can't find session" error.
  echo "attach.sh: no byobu session in '$container' (MAINTENANCE?), opening a shell" >&2
  exec docker exec -it "$container" bash
fi

exec docker exec -it "$container" byobu attach -t "$session"
