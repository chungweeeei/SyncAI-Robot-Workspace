#!/usr/bin/env bash
# Ring-buffer recorder for Point-LIO's raw input, saved when the localizer
# reports LOST -- the data docs/pointlio-divergence-2026-11-03.md §3 says is
# missing: what Point-LIO was fed in the lap where it slid at P3.
#
# Records livox/lidar + livox/imu (Point-LIO's input) plus the references the
# 2026-11-03 analysis leaned on (the body IMU, cmd_vel, lio_odom) continuously
# into 60 s split files, deletes split files older than RING_MIN minutes, and
# when the localizer's log says "-> LOST" (or someone touches $RING/SAVE),
# hard-links the last KEEP_MIN minutes into record/lio_saved/<time>/ once
# POST_SEC have passed, so the save also holds what came after.
#
# Why a whole lap and not a few seconds around P3: Point-LIO replayed from a
# cold start needs the robot standing still for its IMU init, and the robot
# moves through P3. KEEP_MIN 20 holds the lap that went wrong from the dock
# before it (standing) plus the lap before that, which did not. Replaying it
# from cold also answers whether the trigger is in that lap's data at all, or
# in the state Point-LIO had built up over the night.
#
# Run it in its own container (docker-compose.recorder.yml), not in a session
# pane: switch_mode kills every session. Workspace root as cwd, like
# everything else here. ~14 GB/h of lidar uncompressed; RING_MIN 30 keeps ~7 GB
# on disk, each save adds ~4.6 GB of links that become real data once the
# ring deletes its copy.
set -uo pipefail
# Job control, for its one side effect here: without it bash starts every
# background command with SIGINT ignored, so the recorder never saw the INT
# that makes it close its bag, and `down` ended it with SIGKILL and its
# unflushed cache lost. With it each background job also gets its own process
# group, which stop_all signals whole.
set -m

RING_MIN=${RING_MIN:-30}          # minutes kept in the ring
KEEP_MIN=${KEEP_MIN:-20}          # minutes before the trigger kept in a save
POST_SEC=${POST_SEC:-120}         # seconds after the trigger before saving
SPLIT_SEC=${SPLIT_SEC:-60}        # split file length
MAX_SAVES=${MAX_SAVES:-6}         # saves before triggers are only logged
MIN_FREE_GB=${MIN_FREE_GB:-20}    # stop recording below this much free disk
COOLDOWN_SEC=${COOLDOWN_SEC:-600} # a trigger within this of the last save is only logged

ROOT=record
RING=$ROOT/lio_ring
SAVED=$ROOT/lio_saved

log() { echo "[$(date '+%F %T')] $*"; }

# ROS environment, in the order robot01's command: sources it. install/ is
# needed for the message types: livox_ros_driver2/CustomMsg and
# syncai_common's IMUState cannot be recorded without their type support.
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /opt/ros2_rust_underlay/install/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

ROBOT_ID=$(python3 -c "
import configparser
c = configparser.ConfigParser()
c.read('config/system.ini')
print(c.get('system', 'robot_id', fallback='').strip())")
if [ -z "$ROBOT_ID" ]; then
  log "no [system] robot_id in config/system.ini (is the instance INI mounted?); not recording"
  exit 1
fi
LOC_LOG=log/stack/$ROBOT_ID/localizer/current

TOPICS=(
  "/$ROBOT_ID/livox/lidar"
  "/$ROBOT_ID/livox/imu"
  "/$ROBOT_ID/imu"
  "/$ROBOT_ID/cmd_vel"
  "/$ROBOT_ID/pointlio/lio_odom"
)

if ! ros2 bag reindex -h >/dev/null 2>&1; then
  # A save is split files out of a bag whose metadata.yaml is written only
  # when the recorder exits, and lists files the ring has since deleted;
  # reindex is what makes a save playable.
  log "this rosbag2 has no 'ros2 bag reindex'; saves would not be playable. Not recording"
  exit 1
fi

free_gb() { df -P -BG "$ROOT" | awk 'NR == 2 { sub("G", "", $4); print $4 }'; }

mkdir -p "$RING" "$SAVED"
rm -f "$RING/SAVE" "$RING/TRIGGER"

if [ "$(free_gb)" -lt "$MIN_FREE_GB" ]; then
  log "only $(free_gb) GB free on $ROOT (< MIN_FREE_GB $MIN_FREE_GB); not recording"
  exit 1
fi

REC_PID=""
TAIL_PID=""
start_recorder() {
  local name
  name=ring_$(date +%Y%m%d_%H%M%S)
  # nice / ionice: on a saturated CPU Point-LIO falls behind and skips scans
  # (syncai_pointlio's max_lidar_backlog), and a recorder that caused that
  # would be recording a different run from the one it is meant to catch.
  nice -n 15 ionice -c2 -n7 ros2 bag record -o "$RING/$name" \
    --max-bag-duration "$SPLIT_SEC" "${TOPICS[@]}" &
  REC_PID=$!
  log "recording ${TOPICS[*]} into $RING/$name (pid $REC_PID)"
}

stop_all() {
  log "stopping"
  # The group: tail and grep, not only the subshell around them.
  [ -n "$TAIL_PID" ] && kill -- -"$TAIL_PID" 2>/dev/null
  if [ -n "$REC_PID" ] && kill -0 "$REC_PID" 2>/dev/null; then
    kill -INT "$REC_PID"
    wait "$REC_PID"
  fi
  exit 0
}
trap stop_all INT TERM

# The trigger: the localizer's health line when it goes LOST
# ("loc health: OK -> LOST (...)"). multilog renames current on rotation;
# tail -F follows the new file.
(
  tail -n0 -F "$LOC_LOG" 2>/dev/null | grep --line-buffered -- '-> LOST' | while IFS= read -r line; do
    echo "$line" >>"$RING/triggers.log"
    touch "$RING/TRIGGER"
  done
) &
TAIL_PID=$!
log "watching $LOC_LOG for '-> LOST'; 'touch $RING/SAVE' saves by hand"

# The split being written is the highest-numbered .db3 of the newest ring bag;
# a save waits POST_SEC, longer than a split, so every file it takes is closed.
active_file() {
  local dir
  dir=$(ls -1dt "$RING"/ring_*/ 2>/dev/null | head -n1)
  # -v: <bag>_9.db3 before <bag>_10.db3.
  [ -n "$dir" ] && ls -1v "$dir"*.db3 2>/dev/null | tail -n1
}

save() {
  local why=$1 trigger_epoch=$2
  local stamp out active f n=0
  stamp=$(date -d "@$trigger_epoch" +%Y%m%d_%H%M%S)
  out=$SAVED/$stamp
  active=$(active_file)
  # Kept under the ring bag's own directory name, so the reindexed bag has the
  # name its split files were written under.
  while IFS= read -r f; do
    [ "$f" = "$active" ] && continue
    local bag
    bag=$(basename "$(dirname "$f")")
    mkdir -p "$out/$bag"
    ln "$f" "$out/$bag/" && n=$((n + 1))
  done < <(find "$RING" -name '*.db3' -newermt "@$((trigger_epoch - KEEP_MIN * 60))")
  if [ "$n" -eq 0 ]; then
    log "save $stamp ($why): no split files in the window; nothing saved"
    return
  fi
  local bagdir
  for bagdir in "$out"/ring_*/; do
    ros2 bag reindex "$bagdir" >/dev/null 2>&1 || ros2 bag reindex "$bagdir" sqlite3 >/dev/null 2>&1 ||
      log "save $stamp: reindex of $bagdir failed; the .db3 files are there, reindex by hand"
  done
  # The logs rotate by size (16 MiB x 100); keep the ones the analysis reads
  # beside the data.
  mkdir -p "$out/logs"
  local name
  for name in localizer pointlio controller task_runner; do
    cp "log/stack/$ROBOT_ID/$name/current" "$out/logs/$name.current" 2>/dev/null
  done
  cp "$RING/triggers.log" "$out/" 2>/dev/null
  echo "$why" >"$out/WHY"
  log "save $stamp ($why): $n split files -> $out ($(du -sh "$out" | cut -f1))"
}

start_recorder
saves=0
pending_epoch=""
pending_why=""
last_save=0
while true; do
  sleep 10 &
  wait $!

  if [ "$(free_gb)" -lt "$MIN_FREE_GB" ]; then
    log "only $(free_gb) GB free (< MIN_FREE_GB $MIN_FREE_GB); stopping the recorder, saves kept"
    kill -INT "$REC_PID" 2>/dev/null
    wait "$REC_PID"
    REC_PID=""
    while true; do sleep 3600 & wait $!; done
  fi

  if ! kill -0 "$REC_PID" 2>/dev/null; then
    log "the recorder exited; starting a new ring bag"
    start_recorder
  fi

  find "$RING" -name '*.db3' -mmin +"$RING_MIN" -delete

  for t in TRIGGER SAVE; do
    [ -e "$RING/$t" ] || continue
    rm -f "$RING/$t"
    now=$(date +%s)
    why=$([ "$t" = SAVE ] && echo "by hand" || echo "localizer LOST: $(tail -n1 "$RING/triggers.log")")
    if [ -n "$pending_epoch" ]; then
      log "trigger ($why) while a save is pending; covered by it"
    elif [ "$saves" -ge "$MAX_SAVES" ]; then
      log "trigger ($why) after MAX_SAVES $MAX_SAVES saves; not saved"
    elif [ "$t" = TRIGGER ] && [ $((now - last_save)) -lt "$COOLDOWN_SEC" ]; then
      log "trigger ($why) within COOLDOWN_SEC of the last save; not saved"
    else
      pending_epoch=$now
      pending_why=$why
      log "trigger ($why): saving the last $KEEP_MIN min in $POST_SEC s"
    fi
  done

  if [ -n "$pending_epoch" ] && [ $(($(date +%s) - pending_epoch)) -ge "$POST_SEC" ]; then
    save "$pending_why" "$pending_epoch"
    saves=$((saves + 1))
    last_save=$(date +%s)
    pending_epoch=""
  fi
done
