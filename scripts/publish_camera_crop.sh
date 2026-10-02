#!/bin/bash
# =============================================================================
# Camera publisher (crop + scale + push to a remote MediaMTX)
#
# Differences from the original `scripts/publish_camera.sh` (since removed):
#   1. Supports cropping off the black borders and scaling back to the requested
#      output size (done in hardware by nvvidconv, zero CPU cost)
#   2. Fixes the original pre-flight check that broke on symlinks (that check
#      never caught anything)
#   3. Has start / stop / status / restart subcommands and runs in the background
#
# This was added as a new file; the original scripts/publish_camera.sh was left
# untouched.
#
# -----------------------------------------------------------------------------
# MediaMTX is not managed by this script
#
# This machine is only a publisher; the server is an existing external service,
# pointed at via MEDIAMTX_RTSP_HOST. So this machine needs neither a mediamtx
# image nor a container, nor a config/mediamtx.yml -- that config belongs to the
# machine running the server.
#
# Early versions would `docker run` their own mediamtx (error out if the image
# was missing, recreate it if it had been rm'd, tear down and rebuild on a
# version mismatch); that whole section is gone. The reason is not the hassle
# but the topology: the stream aggregation point is one service shared by the
# whole fleet, and having every machine start its own local broker only leaves
# viewers with no idea which one to connect to. To revisit the container
# management logic, dig through git history.
#
# -----------------------------------------------------------------------------
# Usage (run on the Jetson host, not inside the container)
#
#   bash scripts/publish_camera_crop.sh            # start (background), no arguments needed
#   bash scripts/publish_camera_crop.sh stop       # stop publishing
#   bash scripts/publish_camera_crop.sh restart    # restart
#   bash scripts/publish_camera_crop.sh status     # show current state and viewing URLs
#   bash scripts/publish_camera_crop.sh logs       # follow the publisher's log
#   bash scripts/publish_camera_crop.sh foreground # run in the foreground, for debugging
#
# All of this script's settings live in publish_camera_crop.env in the same
# directory (optional): black-border widths, capture/output sizes, and which
# MediaMTX to publish to. Example:
#
#   # publish_camera_crop.env
#   CROP_LEFT="${CROP_LEFT:-80}"
#   CROP_RIGHT="${CROP_RIGHT:-45}"
#   MEDIAMTX_RTSP_HOST="${MEDIAMTX_RTSP_HOST:-192.168.8.160}"
#   MEDIAMTX_RTSP_PORT="${MEDIAMTX_RTSP_PORT:-8554}"
#   MEDIAMTX_STREAM_PATH="${MEDIAMTX_STREAM_PATH:-camera}"
#
# The ${VAR:-value} form is deliberate: an environment variable passed on the
# command line still wins, so trying a new value does not require editing the
# file.
#
# The repo-root .env is deliberately NOT read: that is docker compose's file, it
# holds API keys and database passwords, and this script hands its entire
# environment to the gst-launch child process. The publish target is a
# host-side concern with no need to be shared with the compose services; next
# to the script is the right place for it.
#
# -----------------------------------------------------------------------------
# Environment variables
#   DEVICE=/dev/syncai/camera0
#   SRC_WIDTH=1920   SRC_HEIGHT=1200   FRAMERATE=60     <- capture size from the camera
#   OUT_WIDTH=1920   OUT_HEIGHT=1080                    <- encoded output size
#   CROP_LEFT=0  CROP_RIGHT=0  CROP_TOP=0  CROP_BOTTOM=0  <- pixels to crop off each edge
#   FIT_OUTPUT_ASPECT=1                                 <- auto-fit to the output aspect ratio
#   BITRATE=4000000
#   MEDIAMTX_RTSP_HOST=<remote server>  MEDIAMTX_RTSP_PORT=8554   <- publish target
#   MEDIAMTX_STREAM_PATH=camera   (same as STREAM_PATH; the config file uses the prefixed name)
#   MEDIAMTX_PUBLISH_USER=  MEDIAMTX_PUBLISH_PASS=   <- only if the server requires auth
#   RTSP_URL=rtsp://<host>:<port>/<path>  <- overrides the whole URL, highest priority
#   MEDIAMTX_WEBRTC_PORT=8889  MEDIAMTX_API_PORT=9997  <- only affect the printed URLs and the status query
#   AUTO_EXPOSURE=0  EXPOSURE=330   (EXPOSURE only takes effect when AUTO_EXPOSURE=1)
#   AUTO_WB=1        WB_TEMP=5000   (WB_TEMP only takes effect when AUTO_WB=0)
#   GAIN=1  BRIGHTNESS=16  POWER_LINE_FREQ=1
#   LOG_LEVEL=INFO   (DEBUG|INFO|WARN|ERROR)
#
# -----------------------------------------------------------------------------
# Camera hardware characteristics (AR0234, measured on Jetson AGX Orin / L4T R36.4.4)
#
# The sensor is natively 1920x1200 (16:10). Both MJPG and UYVY offer 1920x1200 /
# 1920x1080 / 1280x720 / 640x480, and each is a genuine crop/scale with no black
# padding.
#
# MJPG is chosen over UYVY because of USB bandwidth:
#   UYVY 1920x1200@60 = 1920*1200*2*60 = 2.2 Gbit/s   -> close to the practical USB 3.0 ceiling
#   MJPG at the same spec (roughly 10:1 compression) ~= 90 Mbit/s     -> comfortable
# The extra decode cost is absorbed by the Jetson's dedicated NVJPG hardware
# engine; the CPU barely moves.
#
# nvjpegdec outputs NVMM I420 while the encoder only accepts NV12, so nvvidconv
# is required. Both are YUV 4:2:0; the only difference is the chroma plane
# layout (I420 three planes / NV12 interleaved chroma).
#
# The IDR interval is fixed at half a second, derived from the framerate. Every
# WHEP viewer joins mid-stream and cannot decode anything until the next IDR
# (Instantaneous Decoder Refresh) arrives -- the picture is solid black until
# then. A half-second IDR = a new viewer waits at most half a second, at the
# cost of a higher bitrate (an IDR is a full frame, far fatter than a P-frame;
# measured at about 3.75 Mbit/s).
#
# Note: the camera is a V4L2 capture device and only one process may hold it at
# a time. The second one in does not fail at open(); it drags on until S_FMT and
# then blows up with "Device or resource busy".
# =============================================================================
set -euo pipefail

# Absolute path of this script. Absolute is required: the background start
# re-invokes the script itself, and a relative path is no longer resolvable
# after the working directory has changed.
#
# The repo root is no longer needed: it only mattered when config/mediamtx.yml
# had to be bind-mounted; since MediaMTX moved to a remote host this script only
# reads the publish_camera_crop.env beside it.
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
SCRIPT_DIRECTORY="$(dirname "$SCRIPT_PATH")"

# -----------------------------------------------------------------------------
# Load settings. Precedence: command-line environment > publish_camera_crop.env > script defaults
#
# Black-border widths, sizes and the publish target all live in the file next
# door; they must not be hardcoded in the shared script. A missing file is the
# normal case, not an error.
# -----------------------------------------------------------------------------
CONFIG_ENV_FILE="${CONFIG_ENV_FILE:-${SCRIPT_DIRECTORY}/publish_camera_crop.env}"
if [[ -f "$CONFIG_ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$CONFIG_ENV_FILE"
fi

# -----------------------------------------------------------------------------
# Logging
#
# Everything goes to stderr to keep stdout clean -- several of the calculation
# functions return their values via stdout, and log lines mixed in would
# corrupt them. The severity threshold is controlled by LOG_LEVEL.
# -----------------------------------------------------------------------------
LOG_LEVEL="${LOG_LEVEL:-INFO}"

# Map a level name to a number so levels can be compared
log_level_to_number() {
  case "$1" in
    DEBUG) echo 10 ;;
    INFO)  echo 20 ;;
    WARN)  echo 30 ;;
    ERROR) echo 40 ;;
    *)     echo 20 ;;   # unknown levels are treated as INFO; the logging config itself must never blow up the script
  esac
}

LOG_LEVEL_THRESHOLD="$(log_level_to_number "$LOG_LEVEL")"

# Uniform log output: <time> [<level>] <message>
write_log() {
  local severity="$1"; shift
  local severity_number
  severity_number="$(log_level_to_number "$severity")"
  [[ "$severity_number" -lt "$LOG_LEVEL_THRESHOLD" ]] && return 0
  printf '%s [%-5s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$severity" "$*" >&2
}

log_debug() { write_log DEBUG "$@"; }
log_info()  { write_log INFO  "$@"; }
log_warn()  { write_log WARN  "$@"; }
log_error() { write_log ERROR "$@"; }

# -----------------------------------------------------------------------------
# Parameters
# -----------------------------------------------------------------------------
DEVICE="${DEVICE:-/dev/syncai/camera0}"

# Capture size: default to the sensor's native full frame so the crop has the most material to work with
SRC_WIDTH="${SRC_WIDTH:-1920}"
SRC_HEIGHT="${SRC_HEIGHT:-1200}"
FRAMERATE="${FRAMERATE:-60}"

# Output size: the final encoded and published size
OUT_WIDTH="${OUT_WIDTH:-1920}"
OUT_HEIGHT="${OUT_HEIGHT:-1080}"

# -----------------------------------------------------------------------------
# Crop amounts: how many pixels to cut off each edge
#
# These four are "fill in however wide the black border you see is" -- enter
# whatever you measured, no coordinate arithmetic needed. Left and right are
# usually asymmetric (the lens module is not perfectly centred), which is why
# CROP_LEFT and CROP_RIGHT are separate variables; do not assume they are equal.
# -----------------------------------------------------------------------------
CROP_LEFT="${CROP_LEFT:-0}"
CROP_RIGHT="${CROP_RIGHT:-0}"
CROP_TOP="${CROP_TOP:-0}"
CROP_BOTTOM="${CROP_BOTTOM:-0}"

# -----------------------------------------------------------------------------
# Whether to auto-fit the output aspect ratio
#
# On:  within the usable area left after removing the black borders, find the
#      largest rectangle matching OUT_WIDTH:OUT_HEIGHT and centre it vertically.
#      The output is then never stretched non-uniformly.
# Off: the usable area, whatever its shape, is scaled straight to the output
#      size; if the ratios differ the picture is distorted.
# -----------------------------------------------------------------------------
FIT_OUTPUT_ASPECT="${FIT_OUTPUT_ASPECT:-1}"

BITRATE="${BITRATE:-4000000}"

# -----------------------------------------------------------------------------
# Publish target (remote MediaMTX)
#
# There is no default host to fall back on: which server it is is a deployment
# decision, and a hardcoded IP would only turn the one typo into "published to
# some address belonging to who knows whom". So it is left blank, and a missing
# value is explicitly rejected in the pre-flight check with a pointer to the
# config file.
#
# All three ports are on the same host. RTSP is the one actually used for
# publishing; WEBRTC and API only affect the printed viewing URLs and the status
# query. These values must match rtspAddress / webrtcAddress / apiAddress in the
# server's mediamtx.yml -- that file is on the server, not in this repo (the
# config/mediamtx.yml in the repo is left over from the old same-machine
# deployment), so whether the values here are right or wrong cannot be told
# locally; verify them on the server.
#
# Publishing only works if the server's authInternalUsers allows this machine to
# publish. The config/mediamtx.yml shipped in this repo restricts publish to
# 127.0.0.1/::1 (written for the old "server and publisher on the same machine"
# deployment), so remote publishing is outside the allowed set: rtspclientsink
# dies at the RECORD stage reporting "Not authorized to access resource" -- and
# by then the encoder has long been running, so it looks like an encoder
# failure rather than an auth failure. The server side must either add this
# machine's IP to ips or issue credentials (see MEDIAMTX_PUBLISH_USER /
# MEDIAMTX_PUBLISH_PASS below).
#
# STREAM_PATH takes MEDIAMTX_STREAM_PATH as its second-tier source so that the
# three publish-target keys in the config file share a prefix and visibly form
# one group (host / port / path). Internally and on the command line the script
# still uses STREAM_PATH -- the PID file and log file names both hang off it.
#
# When several machines publish to the same server, the path must carry the
# machine name as a prefix (e.g. dogA/camera); otherwise the second machine
# fights the first over the same path and the server keeps only one publisher.
# -----------------------------------------------------------------------------
MEDIAMTX_RTSP_HOST="${MEDIAMTX_RTSP_HOST:-}"
MEDIAMTX_RTSP_PORT="${MEDIAMTX_RTSP_PORT:-8554}"
MEDIAMTX_WEBRTC_PORT="${MEDIAMTX_WEBRTC_PORT:-8889}"
MEDIAMTX_API_PORT="${MEDIAMTX_API_PORT:-9997}"

STREAM_PATH="${STREAM_PATH:-${MEDIAMTX_STREAM_PATH:-camera}}"

# Publish credentials are optional: not needed when the server admits this
# machine via the ips allowlist instead.
#
# Only build them into the URL when a username is actually set. When empty, do
# not leave rtsp://:@host -- that is an authentication attempt with an empty
# username, which mediamtx treats as a failed login rather than "anonymous",
# and is not the same as sending none. Characters such as @ : / in the password
# must be percent-encoded by hand; the URL syntax will not escape them for you.
MEDIAMTX_PUBLISH_USER="${MEDIAMTX_PUBLISH_USER:-}"
MEDIAMTX_PUBLISH_PASS="${MEDIAMTX_PUBLISH_PASS:-}"

RTSP_CREDENTIALS=""
if [[ -n "$MEDIAMTX_PUBLISH_USER" ]]; then
  RTSP_CREDENTIALS="${MEDIAMTX_PUBLISH_USER}:${MEDIAMTX_PUBLISH_PASS}@"
fi

RTSP_URL="${RTSP_URL:-rtsp://${RTSP_CREDENTIALS}${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT}/${STREAM_PATH}}"

# Version for logs and status, with the password masked.
#
# The publisher's log is a file left in /tmp and status is printed to the
# terminal (and frequently pasted into chat); neither should retain the
# password. What is actually handed to gst-launch is always the unmasked
# RTSP_URL.
redact_rtsp_url() {
  sed -E 's#(rtsp://[^:/@]+):[^@/]*@#\1:***@#' <<< "$1"
}
RTSP_URL_DISPLAY="$(redact_rtsp_url "$RTSP_URL")"

# PID file and log file for background execution. Named after STREAM_PATH so
# different paths can coexist.
#
# Slashes in the file name must be replaced. When publishing to a shared server
# the path usually carries a machine-name prefix (dogB/camera), and embedding it
# verbatim would yield /tmp/publish_camera_dogB/camera.log -- that directory
# does not exist, so the background start cannot even open its log, and start
# only reports "exited right after starting" with no hint that the file name is
# the problem.
RUNTIME_NAME="${STREAM_PATH//\//_}"
RUNTIME_PID_FILE="${RUNTIME_PID_FILE:-/tmp/publish_camera_${RUNTIME_NAME}.pid}"
RUNTIME_LOG_FILE="${RUNTIME_LOG_FILE:-/tmp/publish_camera_${RUNTIME_NAME}.log}"

# Keep one IDR every half second regardless of the configured framerate
IDR_INTERVAL=$(( FRAMERATE / 2 ))

# -----------------------------------------------------------------------------
# V4L2 sensor controls
#
# These are not decorative defaults. UVC control state lives in the camera
# firmware and persists across open/close for as long as the camera stays
# powered, so the stream inherits whatever state the previous process (or some
# manual v4l2-ctl run) left behind.
#
# Real incident: the camera was stuck at auto_exposure=1 (Manual Mode) with
# exposure_time_absolute=40 (4 ms; the default is 33 ms) -- the picture was so
# dark it looked like a broken encoder; on top of that white_balance_automatic=0
# with the colour temperature frozen at 4000 K turned the whole image blue.
#
# Polarity trap: a value of 1 means the opposite thing on these two controls:
#   auto_exposure:           0 = Auto Mode,  1 = Manual Mode  (menu type, inherits the UVC numbering)
#   white_balance_automatic: 0 = manual,     1 = automatic    (a plain boolean switch)
# So AUTO_EXPOSURE=0 and AUTO_WB=1 look contradictory but both mean "automatic".
#
# Both manual values are gated: they can only be written when the corresponding
# auto switch is off. In auto mode the driver rejects the write and v4l2src just
# prints a warning for a value that would have been ignored anyway. The reverse
# also matters: in auto mode the exposure_time_absolute and
# white_balance_temperature read back are stale; the values the firmware is
# actually using are never written back, so they cannot be used to tell whether
# AE/AWB is working.
#
# exposure_time_absolute is in units of 100 us, and its ceiling is pinned by the
# frame period: at 60 fps no value above ~166 (16.6 ms) can ever be applied,
# which is the real reason this camera is dim indoors. If auto exposure is
# already at the ceiling and the picture is still dark, the correct fix is to
# drop to 30 fps (doubling the exposure headroom), not to raise EXPOSURE -- a
# higher value gets truncated by the framerate and does nothing.
#
# white_balance_temperature means "tell the camera how many K the ambient light
# is", and the camera compensates in the opposite direction: a low setting
# (2300-3500K, warm light) -> compensates toward blue; a high setting
# (5000-6500K, daylight) -> compensates toward yellow.
#
# POWER_LINE_FREQ defaults to 1 (50 Hz) to remove the flicker banding caused by
# mains lighting; the camera ships with it Disabled. Use 2 on a 60 Hz grid.
# -----------------------------------------------------------------------------
AUTO_EXPOSURE="${AUTO_EXPOSURE:-0}"
EXPOSURE="${EXPOSURE:-330}"
AUTO_WB="${AUTO_WB:-1}"
WB_TEMP="${WB_TEMP:-5000}"
GAIN="${GAIN:-1}"
BRIGHTNESS="${BRIGHTNESS:-16}"
POWER_LINE_FREQ="${POWER_LINE_FREQ:-1}"

# Assemble the control string that v4l2src extra-controls expects
#
# Order matters: each auto_* switch must precede the manual value it gates,
# otherwise the driver rejects the manual write on the grounds that "the mode is
# still automatic".
build_sensor_controls() {
  local controls="c,auto_exposure=${AUTO_EXPOSURE}"
  [[ "$AUTO_EXPOSURE" == "1" ]] && controls+=",exposure_time_absolute=${EXPOSURE}"
  controls+=",white_balance_automatic=${AUTO_WB}"
  [[ "$AUTO_WB" == "0" ]] && controls+=",white_balance_temperature=${WB_TEMP}"
  controls+=",gain=${GAIN},brightness=${BRIGHTNESS}"
  controls+=",power_line_frequency=${POWER_LINE_FREQ}"
  echo "$controls"
}

# =============================================================================
# Crop and scale
#
# A single nvvidconv element does crop, scale and colour-format conversion at
# once, all on the VIC hardware, so "cut off the black borders and scale back up
# to the original size" costs no extra CPU and does not break the NVMM
# zero-copy path.
#
# Beware that nvvidconv's left/right/top/bottom are "coordinates of the source
# rectangle", not "number of pixels to crop". gst-inspect's description says
# "Pixels to crop at left"; that is wrong. Measured (1920x1200 input, left=480
# right=1440 top=300 bottom=900), the extracted region is the central 960x600,
# i.e. right-left by bottom-top.
# =============================================================================

# Round up to an even number
#
# Used specifically on "crop amounts". The chroma planes of YUV 4:2:0 are half
# the luma size in each dimension, so an odd crop boundary misaligns the chroma
# samples, which shows up as a thin line of wrong colour along the edge.
#
# The direction is deliberately up (crop more) rather than down (crop less):
# cropping one pixel too few leaves a black line, which is a genuine error;
# cropping one pixel too many only loses a sliver of field of view.
align_up_to_even() {
  echo $(( ($1 + 1) / 2 * 2 ))
}

# Round down to an even number, used on the centring offset
align_down_to_even() {
  echo $(( $1 / 2 * 2 ))
}

# Euclid's algorithm for the greatest common divisor, used to reduce an aspect ratio to its simplest integer ratio
greatest_common_divisor() {
  local first="$1" second="$2" remainder
  while [[ "$second" -ne 0 ]]; do
    remainder=$(( first % second ))
    first="$second"
    second="$remainder"
  done
  echo "$first"
}

# Find the largest rectangle with the target aspect ratio inside the usable area; returns "width height"
#
# The method reduces the target ratio to its simplest integer form (1920x1080 ->
# 16:9), then finds the largest multiple k such that (ratio_width*k,
# ratio_height*k) still fits inside the usable area.
#
# Integer ratio multiplication is used instead of floating-point multiply and
# round in order to avoid rounding error entirely -- being off by a pixel or two
# is not a small matter here, since it makes nvvidconv scale non-uniformly and
# the picture ends up slightly stretched.
fit_rectangle_to_aspect() {
  local available_width="$1" available_height="$2"
  local target_width="$3" target_height="$4"

  local divisor ratio_width ratio_height
  divisor="$(greatest_common_divisor "$target_width" "$target_height")"
  ratio_width=$(( target_width / divisor ))
  ratio_height=$(( target_height / divisor ))

  # Compute the largest multiple bounded by width and by height separately, and take the smaller
  local max_multiple_by_width=$(( available_width / ratio_width ))
  local max_multiple_by_height=$(( available_height / ratio_height ))
  local multiple=$(( max_multiple_by_width < max_multiple_by_height \
                     ? max_multiple_by_width : max_multiple_by_height ))

  # Then step down to a multiple that makes both width and height even
  while [[ "$multiple" -gt 0 ]]; do
    if [[ $(( (ratio_width * multiple) % 2 )) -eq 0 && \
          $(( (ratio_height * multiple) % 2 )) -eq 0 ]]; then
      break
    fi
    multiple=$(( multiple - 1 ))
  done

  if [[ "$multiple" -le 0 ]]; then
    log_error "usable area ${available_width}x${available_height} cannot fit any ${ratio_width}:${ratio_height} rectangle"
    return 1
  fi

  log_debug "aspect ratio ${target_width}:${target_height} reduces to ${ratio_width}:${ratio_height}, multiple ${multiple}"
  echo "$(( ratio_width * multiple )) $(( ratio_height * multiple ))"
}

# Compute the nvvidconv argument string from the crop amounts and output ratio; returns an empty string when no crop is needed
build_crop_arguments() {
  # ---- Step 1: round the user-specified crop amounts to even ----
  local crop_left crop_right crop_top crop_bottom
  crop_left="$(align_up_to_even "$CROP_LEFT")"
  crop_right="$(align_up_to_even "$CROP_RIGHT")"
  crop_top="$(align_up_to_even "$CROP_TOP")"
  crop_bottom="$(align_up_to_even "$CROP_BOTTOM")"

  # ---- Step 2: compute the usable area left after removing the black borders ----
  local available_left="$crop_left"
  local available_top="$crop_top"
  local available_width=$(( SRC_WIDTH - crop_left - crop_right ))
  local available_height=$(( SRC_HEIGHT - crop_top - crop_bottom ))

  if [[ "$available_width" -le 0 || "$available_height" -le 0 ]]; then
    log_error "crop amounts too large: usable area is ${available_width}x${available_height} (must be positive)"
    log_error "  source ${SRC_WIDTH}x${SRC_HEIGHT}, left+right crop $(( crop_left + crop_right )), top+bottom crop $(( crop_top + crop_bottom ))"
    return 1
  fi

  # ---- Step 3: fit the output aspect ratio to avoid non-uniform stretching ----
  local final_width="$available_width"
  local final_height="$available_height"

  if [[ "$FIT_OUTPUT_ASPECT" == "1" ]]; then
    local fitted_rectangle
    # Failure must be caught explicitly with an if. fit_rectangle_to_aspect runs
    # in the $() subshell, so its internal return does not terminate the main
    # flow; it only becomes the exit code of this command substitution.
    if ! fitted_rectangle="$(fit_rectangle_to_aspect \
          "$available_width" "$available_height" "$OUT_WIDTH" "$OUT_HEIGHT")"; then
      return 1
    fi
    read -r final_width final_height <<< "$fitted_rectangle"
  fi

  # ---- Step 4: distribute the slack and convert to source-rectangle coordinates ----
  #
  # Horizontally, the user-specified left edge is respected as-is: the black
  # border width was measured, and it must not be under-cropped to make the
  # ratio work. Any width surplus after fitting the ratio is always trimmed off
  # the right. Vertically, the region is centred, because top/bottom cropping
  # usually only exists to make the ratio work.
  local rect_left="$available_left"
  local rect_right=$(( rect_left + final_width ))

  local vertical_slack=$(( available_height - final_height ))
  local vertical_offset
  vertical_offset="$(align_down_to_even $(( vertical_slack / 2 )))"
  local rect_top=$(( available_top + vertical_offset ))
  local rect_bottom=$(( rect_top + final_height ))

  # ---- Step 5: if the final rectangle equals the whole frame, add no arguments ----
  if [[ "$rect_left" -eq 0 && "$rect_top" -eq 0 && \
        "$rect_right" -eq "$SRC_WIDTH" && "$rect_bottom" -eq "$SRC_HEIGHT" ]]; then
    log_debug "final rectangle equals the whole frame; no crop arguments for nvvidconv"
    echo ""
    return 0
  fi

  # ---- Step 6: spell out the result ----
  log_info "crop amounts (rounded to even): left ${crop_left} right ${crop_right} top ${crop_top} bottom ${crop_bottom}"
  log_info "  usable area ${available_width}x${available_height}"
  log_info "  source rectangle (${rect_left},${rect_top})-(${rect_right},${rect_bottom}) = ${final_width}x${final_height}"
  log_info "  scaled output ${OUT_WIDTH}x${OUT_HEIGHT}"

  local effective_crop_right=$(( SRC_WIDTH - rect_right ))
  local effective_crop_bottom=$(( SRC_HEIGHT - rect_bottom ))
  if [[ "$effective_crop_right" -ne "$crop_right" || "$effective_crop_bottom" -ne "$crop_bottom" ]]; then
    log_info "  effective crop after aspect-ratio adjustment: left ${rect_left} right ${effective_crop_right} top ${rect_top} bottom ${effective_crop_bottom}"
  fi

  local final_aspect output_aspect
  final_aspect=$(( final_width * 10000 / final_height ))
  output_aspect=$(( OUT_WIDTH * 10000 / OUT_HEIGHT ))
  if [[ "$final_aspect" -ne "$output_aspect" ]]; then
    log_warn "cropped aspect ratio (${final_width}:${final_height}) differs from the output (${OUT_WIDTH}:${OUT_HEIGHT}); the picture will be stretched non-uniformly"
    log_warn "  set FIT_OUTPUT_ASPECT=1 to avoid the distortion"
  fi

  echo "left=${rect_left} right=${rect_right} top=${rect_top} bottom=${rect_bottom}"
}

# =============================================================================
# Remote MediaMTX
# =============================================================================

# Confirm the remote MediaMTX's RTSP port accepts connections
#
# This step cannot be skipped, and the reason differs from the local-deployment
# days: back then it was waiting for a freshly `docker run` container to finish
# binding; now the server is on another machine and it may not be running, the
# network may be down, or the address may simply be a typo. Without this check
# rtspclientsink gets connection refused and dies immediately, and since start
# runs in the background the user only sees "exited right after starting" with
# no hint that it was a network problem.
#
# The probed host/port must be derived from the same variables as RTSP_URL; do
# not hardcode 8554. Changing the port while probing the old one turns into
# either "give up after the timeout" or "probe some other service and charge
# ahead publishing".
#
# 8 retries (about 2 seconds) instead of the former 10 seconds. A remote service
# is either running or it is not, unlike a freshly started local container that
# needs time to bind; the few retries left are only there to tolerate the
# occasional wifi packet loss.
#
# Only TCP connectivity is verified, not authentication. Auth is only rejected
# at the RECORD stage, by which point we are inside the publisher process -- so
# for auth-failure symptoms look at the log (logs); they are not caught here.
check_mediamtx_reachable() {
  if [[ -z "$MEDIAMTX_RTSP_HOST" ]]; then
    log_error "MEDIAMTX_RTSP_HOST is not set; no idea which MediaMTX to publish to"
    log_error "  config file:  $CONFIG_ENV_FILE"
    log_error "  or pass it ad hoc:  MEDIAMTX_RTSP_HOST=<server address> $(basename "$SCRIPT_PATH")"
    exit 1
  fi

  local max_attempts=8      # 8 x 0.25s = wait at most 2 seconds
  local attempt=0

  while [[ "$attempt" -lt "$max_attempts" ]]; do
    if timeout 1 bash -c "cat < /dev/null > /dev/tcp/${MEDIAMTX_RTSP_HOST}/${MEDIAMTX_RTSP_PORT}" 2>/dev/null; then
      log_info "remote MediaMTX reachable (${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT})"
      return 0
    fi
    attempt=$(( attempt + 1 ))
    sleep 0.25
  done

  log_error "cannot reach remote MediaMTX: ${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT}"
  log_error "  check that mediamtx is still running on the server and that rtspAddress is this port"
  log_error "  check network reachability:  ping ${MEDIAMTX_RTSP_HOST}"
  log_error "  the address is configured in:  $CONFIG_ENV_FILE (MEDIAMTX_RTSP_HOST / MEDIAMTX_RTSP_PORT)"
  exit 1
}

# List the URLs the stream can be viewed at
#
# Only the server's address is printed. This used to list the IP of every local
# network interface -- an approach that only held while MediaMTX ran locally;
# with the server moved remote, printing local IPs would hand out a set of
# unreachable URLs, which is worse than printing nothing.
print_viewing_urls() {
  log_info "   RTSP   rtsp://${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT}/${STREAM_PATH}"
  log_info "   WHEP   http://${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_WEBRTC_PORT}/${STREAM_PATH}"
}

# =============================================================================
# Publisher process lifecycle
# =============================================================================

# Read the PID file and return a PID that is still alive; return an empty string otherwise
#
# Also cleans up a stale PID file (process already dead but the file remains);
# otherwise every subsequent start would wrongly believe it is already running.
read_live_pipeline_pid() {
  [[ -f "$RUNTIME_PID_FILE" ]] || { echo ""; return 0; }

  local recorded_pid
  recorded_pid="$(cat "$RUNTIME_PID_FILE" 2>/dev/null || echo "")"

  if [[ -z "$recorded_pid" ]] || ! kill -0 "$recorded_pid" 2>/dev/null; then
    log_debug "PID file contents stale ($recorded_pid), removing"
    rm -f "$RUNTIME_PID_FILE"
    echo ""
    return 0
  fi

  echo "$recorded_pid"
}

# Stop the publisher process
#
# SIGINT first rather than SIGKILL: gst-launch's -e flag turns SIGINT into an
# EOS sent downstream, giving rtspclientsink a chance to say goodbye to
# MediaMTX properly instead of being cut off mid-frame and leaving half a
# session behind. Escalate to SIGKILL only if it does not finish.
stop_pipeline() {
  local live_pid
  live_pid="$(read_live_pipeline_pid)"

  if [[ -z "$live_pid" ]]; then
    log_info "publisher process is not running"
    return 0
  fi

  log_info "stopping publisher process (PID $live_pid), sending SIGINT for a clean shutdown"
  kill -INT "$live_pid" 2>/dev/null || true

  # Give it up to 5 seconds to wind down on its own
  local attempt=0
  while [[ "$attempt" -lt 50 ]]; do
    if ! kill -0 "$live_pid" 2>/dev/null; then
      log_info "publisher process has exited"
      rm -f "$RUNTIME_PID_FILE"
      return 0
    fi
    attempt=$(( attempt + 1 ))
    sleep 0.1
  done

  log_warn "still running 5 seconds after SIGINT, sending SIGKILL"
  kill -KILL "$live_pid" 2>/dev/null || true
  rm -f "$RUNTIME_PID_FILE"
}

# =============================================================================
# Pre-flight checks
# =============================================================================

# Confirm the device node exists
check_device_exists() {
  if [[ ! -e "$DEVICE" ]]; then
    log_error "$DEVICE does not exist. Is the camera plugged in?"
    exit 1
  fi
  log_debug "device node exists: $DEVICE"
}

# Confirm no other process is holding the camera, and name it if one is
#
# DEVICE must be canonicalised first. What the kernel records under
# /proc/<pid>/fd/ is always the real path with symlinks resolved, and the
# default DEVICE /dev/syncai/camera0 is precisely a symlink (to ../video0).
# A string comparison against the symlink path can never match -- that is how
# the original version was written, and that check never caught anything.
check_device_not_busy() {
  local device_real_path
  device_real_path="$(readlink -f "$DEVICE")"
  log_debug "device canonical path: $DEVICE -> $device_real_path"

  local pid_directory file_descriptor link_target holder_pid holder_command
  for pid_directory in /proc/[0-9]*; do
    for file_descriptor in "$pid_directory"/fd/*; do
      link_target="$(readlink "$file_descriptor" 2>/dev/null)" || continue
      if [[ "$link_target" == "$device_real_path" ]]; then
        holder_pid="$(basename "$pid_directory")"
        holder_command="$(tr -d '\0' < "$pid_directory/comm" 2>/dev/null)"
        log_error "$DEVICE ($device_real_path) is held by PID $holder_pid ($holder_command)."
        log_error "  stop it first:  kill -INT $holder_pid"
        exit 1
      fi
    done
  done
  log_debug "no other process is holding the camera"
}

# Confirm GStreamer and the Tegra hardware elements are present
#
# nvjpegdec / nvvidconv / nvv4l2h264enc come from nvidia-l4t-gstreamer and only
# exist on an L4T host. Running where they are absent yields the error
# no element "nvvidconv"; catching it up front is easier to understand.
check_gstreamer_elements() {
  local element
  for element in v4l2src nvjpegdec nvvidconv nvv4l2h264enc h264parse rtspclientsink; do
    if ! gst-inspect-1.0 "$element" >/dev/null 2>&1; then
      log_error "missing GStreamer element: $element"
      log_error "  Tegra elements (nv*) require running on the L4T host, or in a container with the nvidia runtime"
      exit 1
    fi
  done
  log_debug "all GStreamer elements present"
}

# =============================================================================
# Subcommands
# =============================================================================

# Internal: actually run the GStreamer pipeline; never returns
#
# Entered when cmd_start re-invokes the script in the background. exec replaces
# the shell process here, so the PID recorded by the background layer is
# gst-launch's own PID and stop hits the right target.
cmd_run_pipeline() {
  local sensor_controls crop_arguments
  sensor_controls="$(build_sensor_controls)"

  # Catch failure explicitly: build_crop_arguments runs in a subshell, so its
  # return 1 does not terminate the main flow on its own.
  if ! crop_arguments="$(build_crop_arguments)"; then
    log_error "crop argument computation failed, aborting publish"
    exit 1
  fi

  log_info "publishing ${DEVICE} capture ${SRC_WIDTH}x${SRC_HEIGHT}@${FRAMERATE} -> output ${OUT_WIDTH}x${OUT_HEIGHT} -> ${RTSP_URL_DISPLAY}"
  log_info "sensor controls: ${sensor_controls}"
  log_info "bitrate ${BITRATE} bps, IDR interval ${IDR_INTERVAL} frames (about 0.5 s)"

  # -e makes SIGINT send EOS downstream instead of cutting off mid-frame
  #
  # crop_arguments is deliberately unquoted: it may be an empty string (no
  # crop) or several whitespace-separated arguments, and the shell needs to
  # word-split it.
  # shellcheck disable=SC2086
  exec gst-launch-1.0 -e \
    v4l2src device="$DEVICE" io-mode=2 extra-controls="$sensor_controls" \
    ! image/jpeg,width="$SRC_WIDTH",height="$SRC_HEIGHT",framerate="$FRAMERATE"/1 \
    ! nvjpegdec ! 'video/x-raw(memory:NVMM)' \
    ! nvvidconv $crop_arguments \
    ! "video/x-raw(memory:NVMM),format=NV12,width=${OUT_WIDTH},height=${OUT_HEIGHT}" \
    ! nvv4l2h264enc bitrate="$BITRATE" profile=0 insert-sps-pps=true \
        insert-vui=true iframeinterval="$IDR_INTERVAL" idrinterval="$IDR_INTERVAL" \
        control-rate=1 maxperf-enable=true \
    ! h264parse config-interval=-1 \
    ! rtspclientsink location="$RTSP_URL" protocols=tcp
}

# Run in the foreground, Ctrl-C to quit. For debugging; error messages are visible directly.
cmd_foreground() {
  check_device_exists
  check_device_not_busy
  check_gstreamer_elements

  check_mediamtx_reachable

  log_info "──────────────────────────────────────────────"
  log_info " The picture can be viewed at:"
  print_viewing_urls
  log_info "──────────────────────────────────────────────"

  cmd_run_pipeline
}

# Start in the background
cmd_start() {
  local live_pid
  live_pid="$(read_live_pipeline_pid)"
  if [[ -n "$live_pid" ]]; then
    log_error "publisher is already running (PID $live_pid)"
    log_error "  to restart use:  $(basename "$SCRIPT_PATH") restart"
    exit 1
  fi

  check_device_exists
  check_device_not_busy
  check_gstreamer_elements

  # Confirm the server is reachable before opening the camera: a publisher that
  # cannot find the server dies outright, and by then the camera has been opened
  # once, so the reopen right after a restart hits Device or resource busy.
  check_mediamtx_reachable

  log_info "starting publisher in the background, log goes to $RUNTIME_LOG_FILE"

  # setsid detaches it from the current session so an ssh disconnect or a closed terminal does not take it down
  setsid nohup bash "$SCRIPT_PATH" __run_pipeline >> "$RUNTIME_LOG_FILE" 2>&1 &
  local started_pid=$!
  echo "$started_pid" > "$RUNTIME_PID_FILE"

  # Give it a moment: if it is going to fail it usually blows up within a
  # second, and pasting the log tail is better than making the user dig through
  # the file themselves.
  sleep 2

  if ! kill -0 "$started_pid" 2>/dev/null; then
    log_error "publisher exited right after starting; log tail:"
    tail -20 "$RUNTIME_LOG_FILE" >&2
    rm -f "$RUNTIME_PID_FILE"
    exit 1
  fi

  log_info "publisher running (PID $started_pid)"
  log_info "──────────────────────────────────────────────"
  log_info " The picture can be viewed at:"
  print_viewing_urls
  log_info "──────────────────────────────────────────────"
  log_info " stop:  $(basename "$SCRIPT_PATH") stop"
  log_info " logs:  $(basename "$SCRIPT_PATH") logs"
}

# Stop publishing
#
# Only stops the local publisher process. The server belongs to someone else
# and other machines may still be publishing to it; this must not, and does
# not, touch it -- this script used to `docker stop` the local container on the
# way out, which under a remote deployment would amount to "stop my camera and,
# incidentally, shut down the whole fleet's aggregation server".
cmd_stop() {
  stop_pipeline
  log_info "stopped"
}

cmd_restart() {
  cmd_stop
  # The camera takes a moment to release; reopening too quickly hits Device or resource busy
  sleep 1
  cmd_start
}

# Show current status
cmd_status() {
  local live_pid
  live_pid="$(read_live_pipeline_pid)"

  echo "MediaMTX (remote, not managed by this script)"
  echo "  server    : ${MEDIAMTX_RTSP_HOST:-<not set>}"
  echo "  RTSP port : $MEDIAMTX_RTSP_PORT"
  if [[ -z "$MEDIAMTX_RTSP_HOST" ]]; then
    echo "  reachable : -"
  elif timeout 1 bash -c "cat < /dev/null > /dev/tcp/${MEDIAMTX_RTSP_HOST}/${MEDIAMTX_RTSP_PORT}" 2>/dev/null; then
    echo "  reachable : yes"
  else
    echo "  reachable : no"
  fi
  echo ""
  echo "Publisher"
  if [[ -n "$live_pid" ]]; then
    echo "  state     : running (PID $live_pid)"
  else
    echo "  state     : not running"
  fi
  echo "  device    : $DEVICE"
  echo "  capture   : ${SRC_WIDTH}x${SRC_HEIGHT}@${FRAMERATE}"
  echo "  output    : ${OUT_WIDTH}x${OUT_HEIGHT}"
  echo "  crop      : left ${CROP_LEFT} right ${CROP_RIGHT} top ${CROP_TOP} bottom ${CROP_BOTTOM}"
  echo "  target    : $RTSP_URL_DISPLAY"
  echo "  log       : $RUNTIME_LOG_FILE"
  echo "  config    : $CONFIG_ENV_FILE"
  echo ""

  # Ask the server's control API whether this path is actually receiving a stream.
  # This is the only way to confirm "the stream is really alive" -- a live local
  # process does not mean the data is getting through.
  #
  # A failed query does not mean the stream is in trouble: mediamtx's apiAddress
  # is usually bound only to the server's own loopback (this repo's
  # config/mediamtx.yml does exactly that), so asking from this machine cannot
  # connect in the first place. A failure therefore prints a single hint line,
  # is not treated as an error, and does not affect the exit code.
  if [[ -n "$MEDIAMTX_RTSP_HOST" ]]; then
    echo "Path status reported by MediaMTX"
    if command -v curl >/dev/null 2>&1; then
      curl -s --max-time 3 "http://${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_API_PORT}/v3/paths/list" 2>/dev/null \
        | sed 's/,/,\n/g' | grep -E '"name"|"ready"|"tracks"|"bytesReceived"' | sed 's/^/  /' \
        || echo "  (query failed; the control API is usually only open on the server's own loopback, which is normal)"
    else
      echo "  (curl not available, skipped)"
    fi
    echo ""

    echo "Viewing URLs"
    echo "  RTSP   rtsp://${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT}/${STREAM_PATH}"
    echo "  WHEP   http://${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_WEBRTC_PORT}/${STREAM_PATH}"
  fi
}

# Follow the log
cmd_logs() {
  if [[ ! -f "$RUNTIME_LOG_FILE" ]]; then
    log_error "log file does not exist: $RUNTIME_LOG_FILE"
    exit 1
  fi
  tail -f "$RUNTIME_LOG_FILE"
}

cmd_usage() {
  cat <<USAGE
Usage: $(basename "$SCRIPT_PATH") [subcommand]

  (none)      same as start
  start       start publishing in the background (MediaMTX is an existing remote service; it is not started here)
  stop        stop publishing (does not touch the remote MediaMTX)
  restart     restart
  status      show state, whether the stream is really being received, and viewing URLs
  logs        follow the publisher log (tail -f)
  foreground  run in the foreground, for debugging
  help        show this help

Config file: $CONFIG_ENV_FILE
  crop and sizes      CROP_* / SRC_* / OUT_*
  publish target      MEDIAMTX_RTSP_HOST / MEDIAMTX_RTSP_PORT / MEDIAMTX_STREAM_PATH
                      (or pass a complete RTSP_URL to override the whole thing)
  publish credentials MEDIAMTX_PUBLISH_USER / MEDIAMTX_PUBLISH_PASS (only if the server requires them)
Environment variables passed ad hoc on the command line take precedence over the config file.
USAGE
}

# =============================================================================
# Entry point
# =============================================================================
main() {
  local subcommand="${1:-start}"

  case "$subcommand" in
    start)          cmd_start ;;
    stop)           cmd_stop ;;
    restart)        cmd_restart ;;
    status)         cmd_status ;;
    logs)           cmd_logs ;;
    foreground|fg)  cmd_foreground ;;
    help|-h|--help) cmd_usage ;;
    # Internal subcommand, invoked in the background by cmd_start; not for direct use
    __run_pipeline) cmd_run_pipeline ;;
    *)
      log_error "unknown subcommand: $subcommand"
      cmd_usage >&2
      exit 1
      ;;
  esac
}

# Only run the main flow when executed directly.
# When sourced (e.g. by unit tests) only the function definitions are loaded; nothing is started.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  main "$@"
fi
