#!/bin/bash
# =============================================================================
# Publish the USB camera to the MediaMTX `camera` path over RTSP.
#
# Usage:
#   bash scripts/publish_camera.sh
#
# Run it inside the robot container (the Jetson nv* plugins and the camera node
# are both available there):
#   docker exec -d robot01 bash /home/syncrobotic/robot_ws/scripts/publish_camera.sh
#
# Overridable with env vars:
#   DEVICE=/dev/video1  WIDTH=1280  HEIGHT=720  FRAMERATE=60
#   BITRATE=4000000     RTSP_URL=rtsp://127.0.0.1:8554/camera
#   AUTO_EXPOSURE=0     EXPOSURE=330   (EXPOSURE only read when AUTO_EXPOSURE=1)
#   AUTO_WB=1           WB_TEMP=5000   (WB_TEMP only read when AUTO_WB=0)
#   GAIN=1  BRIGHTNESS=16  POWER_LINE_FREQ=1
#
# The camera has no 30 fps mode (MJPG at 1280x720 offers 60 and 120 only), and
# nvjpegdec emits NVMM I420, so nvvidconv is required to reach the NV12 the
# encoder wants. Measured at ~3.75 Mbit/s with an IDR every half second. That
# short IDR interval matters because every WHEP viewer joins mid-stream and
# cannot decode until the next one arrives. See config/mediamtx.yml.
#
# NOTE: the camera is a V4L2 capture device, so exactly one process may hold
# it. A second instance dies with "Device or resource busy" at S_FMT — the
# pre-flight check below turns that into a readable message.
#
# Read the stream back at:
#   WHEP    http://<robot>:8889/camera/whep
#   Player  http://<robot>:8889/camera
#   RTSP    rtsp://<robot>:8554/camera
# =============================================================================
set -euo pipefail

# Default matches CAMERA_DEV in docker-compose.robots.yml — the usb-3.1
# VCS-AR0234-C. This is NOT /dev/video0: the Jetson enumerates two of these
# cameras across four video4linux nodes and none of them is 0. `ls -l
# /dev/v4l/by-path/` maps a physical USB port back to a node when the numbering
# moves; override DEVICE (and CAMERA_DEV in compose) together when it does.
DEVICE="${DEVICE:-/dev/video1}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FRAMERATE="${FRAMERATE:-60}"
BITRATE="${BITRATE:-4000000}"
# Loopback, not the host's LAN address, and not by accident: mediamtx restricts
# the publish action to 127.0.0.1/::1 (authInternalUsers in config/mediamtx.yml),
# because the publisher is always same-host. Under network_mode: host a
# connection to the LAN IP arrives with that IP as its source, fails the ip
# match, and rtspclientsink dies at RECORD with "Not authorized to access
# resource" — after the encoder has already started, so it reads like an
# encoding failure rather than an auth one. A hardcoded LAN IP is also wrong for
# a second reason: it is DHCP-assigned and has already moved once.
RTSP_URL="${RTSP_URL:-rtsp://192.168.8.160:8554/robot01/camera}"

# One IDR every half second, whatever the framerate.
IDR_INTERVAL=$(( FRAMERATE / 2 ))

# -----------------------------------------------------------------------------
# V4L2 sensor controls.
#
# These are NOT cosmetic defaults — the script used to set none of them, and
# that was a bug. UVC controls live in the camera and persist across open/close
# for as long as it stays powered, so the stream inherited whatever state the
# last process (or a stray v4l2-ctl) happened to leave behind. It was found
# parked at auto_exposure=1 (Manual Mode) with exposure_time_absolute=40 — 4 ms
# against a default of 33 ms — which reads as "the encoder is broken" rather
# than "someone pinned the exposure", plus white_balance_automatic=0 with the
# temperature frozen at 4000 K, which tints everything. Setting them here makes
# a publish reproducible instead of dependent on device history.
#
# They are applied through v4l2src's `extra-controls` rather than a v4l2-ctl
# call, for two reasons. v4l2-ctl (v4l-utils) is NOT installed in the robot
# image, and the header above tells you to run this script inside the
# container — a v4l2-ctl-based fix would silently no-op exactly where it is
# meant to run. Second, /dev/video0 admits one opener, so a separate v4l2-ctl
# process would have to race the pipeline for the handle; extra-controls is
# applied by v4l2src on the handle it already owns, before streaming starts.
#
# AUTO_EXPOSURE is the menu control, and its polarity is the inverse of what
# the name suggests: 0 = Auto Mode, 1 = Manual Mode. Auto is the default here.
# exposure_time_absolute is only passed in manual mode — in auto the driver
# rejects the write and v4l2src logs a warning for a value it is going to
# ignore anyway. Its unit is 100 us, and it is bounded by the frame period:
# at 60 fps nothing longer than ~166 (16.6 ms) can be honoured, which is the
# real reason this camera looks dim indoors. 1280x720 offers only 60 and 120
# fps, so if auto-exposure still bottoms out, the fix is 1920x1080 at 30 fps
# (double the exposure headroom), not a larger EXPOSURE.
#
# POWER_LINE_FREQ defaults to 1 (50 Hz) to cancel mains flicker banding; the
# camera ships with it Disabled. Use 2 on 60 Hz mains.
AUTO_EXPOSURE="${AUTO_EXPOSURE:-0}"
EXPOSURE="${EXPOSURE:-330}"
AUTO_WB="${AUTO_WB:-1}"
WB_TEMP="${WB_TEMP:-5000}"
GAIN="${GAIN:-1}"
BRIGHTNESS="${BRIGHTNESS:-16}"
POWER_LINE_FREQ="${POWER_LINE_FREQ:-1}"

# Order matters: the auto_* switches must precede the manual values they gate,
# or the driver rejects the manual write against a mode that is still auto.
CONTROLS="c,auto_exposure=${AUTO_EXPOSURE}"
[[ "$AUTO_EXPOSURE" == "1" ]] && CONTROLS+=",exposure_time_absolute=${EXPOSURE}"
CONTROLS+=",white_balance_automatic=${AUTO_WB}"
[[ "$AUTO_WB" == "0" ]] && CONTROLS+=",white_balance_temperature=${WB_TEMP}"
CONTROLS+=",gain=${GAIN},brightness=${BRIGHTNESS}"
CONTROLS+=",power_line_frequency=${POWER_LINE_FREQ}"

if [[ ! -e "$DEVICE" ]]; then
  echo "ERROR: $DEVICE does not exist. Is the camera plugged in, and is the" >&2
  echo "       device passed through to this container?" >&2
  exit 1
fi

# Pre-flight: report the current holder instead of failing later at S_FMT.
for pid_dir in /proc/[0-9]*; do
  for fd in "$pid_dir"/fd/*; do
    if [[ "$(readlink "$fd" 2>/dev/null)" == "$DEVICE" ]]; then
      pid="$(basename "$pid_dir")"
      echo "ERROR: $DEVICE is already held by PID $pid ($(tr -d '\0' < "$pid_dir/comm" 2>/dev/null))." >&2
      echo "       Stop it first:  kill -INT $pid" >&2
      exit 1
    fi
  done
done

echo "==> Publishing ${DEVICE} ${WIDTH}x${HEIGHT}@${FRAMERATE} to ${RTSP_URL}"
echo "==> Sensor controls: ${CONTROLS}"

# -e so Ctrl-C / SIGINT sends EOS downstream rather than cutting mid-frame.
exec gst-launch-1.0 -e v4l2src device="$DEVICE" io-mode=2 extra-controls="$CONTROLS" \
  ! image/jpeg,width="$WIDTH",height="$HEIGHT",framerate="$FRAMERATE"/1 \
  ! nvjpegdec ! 'video/x-raw(memory:NVMM)' \
  ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' \
  ! nvv4l2h264enc bitrate="$BITRATE" profile=0 insert-sps-pps=true \
      insert-vui=true iframeinterval="$IDR_INTERVAL" idrinterval="$IDR_INTERVAL" \
      control-rate=1 maxperf-enable=true \
  ! h264parse config-interval=-1 \
  ! rtspclientsink location="$RTSP_URL" protocols=tcp
