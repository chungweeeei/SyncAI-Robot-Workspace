#!/bin/bash
# =============================================================================
# Publish /dev/video0 to the MediaMTX `camera` path over RTSP.
#
# Usage:
#   bash scripts/publish_camera.sh
#
# Run it inside the robot container (the Jetson nv* plugins and /dev/video0 are
# both available there):
#   docker exec -d robot01 bash /home/syncrobotic/robot_ws/scripts/publish_camera.sh
#
# Overridable with env vars:
#   DEVICE=/dev/video0  WIDTH=1280  HEIGHT=720  FRAMERATE=60
#   BITRATE=4000000     RTSP_URL=rtsp://127.0.0.1:8554/camera
#
# The camera has no 30 fps mode (MJPG at 1280x720 offers 60 and 120 only), and
# nvjpegdec emits NVMM I420, so nvvidconv is required to reach the NV12 the
# encoder wants. Measured at ~3.75 Mbit/s with an IDR every half second. That
# short IDR interval matters because every WHEP viewer joins mid-stream and
# cannot decode until the next one arrives. See config/mediamtx.yml.
#
# NOTE: /dev/video0 is a V4L2 capture device, so exactly one process may hold
# it. A second instance dies with "Device or resource busy" at S_FMT — the
# pre-flight check below turns that into a readable message.
#
# Read the stream back at:
#   WHEP    http://<robot>:8889/camera/whep
#   Player  http://<robot>:8889/camera
#   RTSP    rtsp://<robot>:8554/camera
# =============================================================================
set -euo pipefail

DEVICE="${DEVICE:-/dev/video0}"
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

# -e so Ctrl-C / SIGINT sends EOS downstream rather than cutting mid-frame.
exec gst-launch-1.0 -e v4l2src device="$DEVICE" io-mode=2 \
  ! image/jpeg,width="$WIDTH",height="$HEIGHT",framerate="$FRAMERATE"/1 \
  ! nvjpegdec ! 'video/x-raw(memory:NVMM)' \
  ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' \
  ! nvv4l2h264enc bitrate="$BITRATE" profile=0 insert-sps-pps=true \
      insert-vui=true iframeinterval="$IDR_INTERVAL" idrinterval="$IDR_INTERVAL" \
      control-rate=1 maxperf-enable=true \
  ! h264parse config-interval=-1 \
  ! rtspclientsink location="$RTSP_URL" protocols=tcp
