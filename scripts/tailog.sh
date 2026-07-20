#!/bin/bash
# =============================================================================
# Read back a byobu-pane log captured by scripts/byobu_session*.sh.
#
# Each pane's output is tapped into a multilog dir at
#   log/stack/<robot_id>/<name>/{current,@<tai64n>.s}
# where `current` is the live, uncompressed file and the rotated `@<tai64n>.s`
# files hold gzipped history (the `!gzip` processor keeps the `.s` name but the
# contents are gzip). Lines are prefixed with a TAI64N timestamp; this script
# decodes them with tai64nlocal.
#
# Usage:
#   scripts/tailog.sh <name> [robot_id]        # follow the live log (tail -F)
#   scripts/tailog.sh -a <name> [robot_id]     # dump full history + current
#
# <name> is a pane log name: bringup, map_server, amcl, localizer, filter_info,
#   lio_bridge, planner, controller, task_runner, driver_manager,
#   system_manager, robot_state, backend.
# robot_id defaults to the one in config/system.ini.
# =============================================================================
set -euo pipefail

ALL=0
if [ "${1:-}" = "-a" ]; then ALL=1; shift; fi

NAME="${1:?usage: tailog.sh [-a] <name> [robot_id]}"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROBOT_ID="${2:-}"
if [ -z "$ROBOT_ID" ]; then
  ROBOT_ID="$(awk -F'[:=]' '/^robot_id/ {gsub(/[ \t]/, "", $2); print $2}' \
    "$WS_DIR/config/system.ini" 2>/dev/null)"
fi
ROBOT_ID="${ROBOT_ID:-default_robot}"

DIR="$WS_DIR/log/stack/$ROBOT_ID/$NAME"
[ -d "$DIR" ] || { echo "tailog: no log dir: $DIR" >&2; exit 1; }

if [ "$ALL" -eq 1 ]; then
  # Rotated files first (tai64n names sort lexically = chronologically), then
  # the live current file, all decoded in one stream. zcat -f passes plain
  # (non-gzip) data through unchanged, so it is safe on both.
  shopt -s nullglob
  rotated=("$DIR"/@*.[su])
  {
    for f in $(printf '%s\n' "${rotated[@]}" | sort); do zcat -f < "$f"; done
    [ -f "$DIR/current" ] && cat "$DIR/current"
  } | tai64nlocal
else
  tail -F "$DIR/current" | tai64nlocal
fi
