#!/usr/bin/env bash
# Interactive motion test for the quadruped: stand, lie down, walk forward /
# backward, stop, and a random loop — sent through the backend
# (chungweeeei/SyncAI-Robot-Backend), the same two endpoints the console uses,
# so it runs from the host or a laptop with no ROS installed:
#
#   motion keys  POST /api/v1/robot/set_motion_key   {"key": "0"}
#   velocity     WS   /api/v1/robot/teleop           {"vx": .., "vy": .., "wz": ..} at ~10 Hz
#
#   scripts/motion_test.sh
#   BACKEND_URL=http://robot01.local:3000 MOVE_SPEED=0.4 MOVE_DURATION=3 scripts/motion_test.sh
#
# Needs curl and python3 (standard library only: the WebSocket client below is
# hand-rolled so the host needs no pip package).
#
# Motion keys (the backend's MotionKey, syncai_driver_manager protocol.rs):
# 0 = stand, 1 = locomotion / RL, 2 = lie down, 3 = damping, 5 = MPC. ESTOP (4)
# is not forwarded by that endpoint, so this script has none. "Up" sends 0 and,
# after STAND_SETTLE seconds, LOCOMOTION_KEY (1): the console sends these from
# two separate buttons, and cmd_vel walks the robot only under the RL policy.
# LOCOMOTION_KEY= (empty) sends the stand key alone.
#
# Stopping is the backend's teleop contract, not this script's: a zero frame,
# 0.5 s without a frame (its watchdog) or the socket closing all publish a
# repeated zero cmd_vel. Every move here ends with a zero frame and a close,
# and a killed script closes the socket too, so Ctrl+C mid-move still brakes.
# The backend refuses teleop while a MOVE goal is executing; that refusal is
# printed, not retried.
set -uo pipefail

BACKEND_URL="${BACKEND_URL:-http://127.0.0.1:3000}"
MOVE_SPEED="${MOVE_SPEED:-0.3}"         # m/s; MPPI's walk_vx_min — RL stands still well below it
MOVE_DURATION="${MOVE_DURATION:-2}"     # seconds per forward / backward move
STAND_SETTLE="${STAND_SETTLE:-3}"       # seconds between the stand key and LOCOMOTION_KEY
LOCOMOTION_KEY="${LOCOMOTION_KEY-1}"
LOOP_PAUSE="${LOOP_PAUSE:-3}"           # seconds between actions in the random loop

for cmd in curl python3; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "motion_test.sh: ${cmd} not found" >&2
    exit 1
  fi
done

send_key() {
  local out code
  out=$(curl -sS -m 5 -w $'\n%{http_code}' -X POST "${BACKEND_URL}/api/v1/robot/set_motion_key" \
    -H 'Content-Type: application/json' -d "{\"key\": \"$1\"}" 2>&1)
  code="${out##*$'\n'}"
  out="${out%$'\n'*}"
  if [[ "$code" == "200" ]]; then
    echo "  key $1 -> ${out}"
  else
    echo "  [錯誤] key $1 失敗 (HTTP ${code}): ${out}"
  fi
}

# drive VX SECONDS: one teleop socket, frames at 10 Hz, then a zero frame and
# a close. Server frames are only refusals ({"error": ...}); they are printed.
drive() {
  python3 - "$BACKEND_URL" "$1" "$2" <<'PY'
import base64, json, os, select, socket, struct, sys, time
from urllib.parse import urlparse

url, vx, duration = urlparse(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
if url.scheme == "https":
    sys.exit("  [錯誤] https is not supported; use the backend's http:// URL")
host, port = url.hostname, url.port or 80
path = "/api/v1/robot/teleop"

try:
    sock = socket.create_connection((host, port), timeout=5)
except OSError as e:
    sys.exit(f"  [錯誤] cannot reach backend at {host}:{port}: {e}")
key = base64.b64encode(os.urandom(16)).decode()
sock.sendall((
    f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
    f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
).encode())
resp = b""
while b"\r\n\r\n" not in resp:
    chunk = sock.recv(4096)
    if not chunk:
        break
    resp += chunk
status = resp.split(b"\r\n", 1)[0].decode(errors="replace")
if " 101 " not in status + " ":
    sys.exit(f"  [錯誤] teleop WebSocket handshake failed: {status}")
buf = resp.split(b"\r\n\r\n", 1)[1]

def frame(opcode, payload=b""):
    # Client frames must be masked (RFC 6455 5.3).
    mask = os.urandom(4)
    n = len(payload)
    head = bytes([0x80 | opcode])
    if n < 126:
        head += bytes([0x80 | n])
    else:
        head += bytes([0x80 | 126]) + struct.pack("!H", n)
    return head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload))

def send(vx_):
    sock.sendall(frame(0x1, json.dumps({"vx": vx_, "vy": 0.0, "wz": 0.0}).encode()))

def drain():
    # Print any refusal the server sent; unmasked, short text frames only.
    global buf
    while select.select([sock], [], [], 0)[0]:
        chunk = sock.recv(4096)
        if not chunk:
            return
        buf += chunk
    while len(buf) >= 2:
        n = buf[1] & 0x7F
        hdr = 2 if n < 126 else 4
        if n == 126 and len(buf) >= 4:
            n = struct.unpack("!H", buf[2:4])[0]
        if len(buf) < hdr + n:
            return
        if buf[0] & 0x0F == 0x1:
            print(f"  [後端] {buf[hdr:hdr + n].decode(errors='replace')}")
        buf = buf[hdr + n:]

try:
    end = time.monotonic() + duration
    while time.monotonic() < end:
        send(vx)
        drain()
        time.sleep(0.1)
finally:
    # The zero is the client's release; the close is the backend's own
    # stop-on-disconnect, which also covers a lost zero.
    try:
        send(0.0)
        time.sleep(0.2)
        drain()
        sock.sendall(frame(0x8, struct.pack("!H", 1000)))
    except OSError:
        pass
    sock.close()
PY
}

do_up() {
  echo "[執行] 站立 (up)"
  send_key 0
  if [[ -n "$LOCOMOTION_KEY" ]]; then
    sleep "$STAND_SETTLE"
    send_key "$LOCOMOTION_KEY"
  fi
}

do_down() {
  echo "[執行] 趴下 (down)"
  send_key 2
}

do_forward() {
  echo "[執行] 移動: forward (${MOVE_SPEED} m/s, ${MOVE_DURATION} s)"
  drive "$MOVE_SPEED" "$MOVE_DURATION"
}

do_backward() {
  echo "[執行] 移動: backward (-${MOVE_SPEED} m/s, ${MOVE_DURATION} s)"
  drive "-${MOVE_SPEED}" "$MOVE_DURATION"
}

do_stop() {
  echo "[執行] 停止動作 (stop) - 煞車"
  # The backend publishes a stop only for the socket that set the robot
  # moving, so a fresh socket's zero would be skipped: drive one frame,
  # then zero. A 0.001 m/s frame does not move the robot (far below the
  # RL walking floor) and makes this socket the one that stops it.
  drive 0.001 0.1
}

do_loop() {
  echo "[執行] 進入循環隨機測試 (loop) — 按 q 再 Enter 離開，Ctrl+C 結束程式"
  local actions=(do_up do_down do_forward do_backward)
  local round=0 key
  while true; do
    round=$((round + 1))
    echo "--- 第 ${round} 輪 ---"
    "${actions[RANDOM % ${#actions[@]}]}"
    # read -t doubles as the pause between actions and the way out of the loop
    if read -r -t "$LOOP_PAUSE" key && [[ "$key" == "q" ]]; then
      echo "離開循環測試。"
      return
    fi
  done
}

show_menu() {
  cat <<EOF

========== 機器狗測試腳本 ==========
 backend: ${BACKEND_URL}
 1. 站立 (up)
 2. 趴下 (down)
 3. 向前移動 (move forward)
 4. 向後移動 (move backward)
 9. 停止動作 (stop)
 0. 進入循環隨機測試 (loop)
 q. 退出程式 (exit)
=====================================
EOF
}

while true; do
  show_menu
  read -r -p "請輸入數字選擇動作: " choice || break
  echo
  case "$choice" in
    1) do_up ;;
    2) do_down ;;
    3) do_forward ;;
    4) do_backward ;;
    9) do_stop ;;
    0) do_loop ;;
    q | Q) echo "退出測試腳本。"; break ;;
    *) echo "[錯誤] 無效選項: ${choice}" ;;
  esac
done
