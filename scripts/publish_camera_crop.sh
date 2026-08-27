#!/bin/bash
# =============================================================================
# 相機推流（裁切 + 縮放 + MediaMTX 自動管理）
#
# 跟原本的 scripts/publish_camera.sh 的差別：
#   1. 自己把 MediaMTX container 管好（不存在就建、被刪掉會重建、設定變了會重建）
#   2. 支援裁切黑邊 + 縮放回指定輸出尺寸（nvvidconv 硬體完成，零 CPU 成本）
#   3. 修正原本前置檢查對 symlink 失效的問題（那個檢查從來沒攔到過）
#   4. 推流目標改回 loopback，符合 config/mediamtx.yml 的權限設定
#   5. 有 start / stop / status / restart 子命令，背景執行
#
# 這是新增的檔案，原本的 scripts/publish_camera.sh 完全不動。
#
# -----------------------------------------------------------------------------
# 用法（在 Jetson 的 host 上跑，不是在 container 裡）
#
#   bash scripts/publish_camera_crop.sh            # 啟動（背景），什麼都不用帶
#   bash scripts/publish_camera_crop.sh stop       # 停止推流與 MediaMTX
#   bash scripts/publish_camera_crop.sh restart    # 重啟
#   bash scripts/publish_camera_crop.sh status     # 看目前狀態與觀看網址
#   bash scripts/publish_camera_crop.sh logs       # 追推流的日誌
#   bash scripts/publish_camera_crop.sh foreground # 前景執行，除錯用
#
# 這支腳本的設定全部放在同目錄的 publish_camera_crop.env（選用）：黑邊寬度、
# 擷取/輸出尺寸，以及推流要送到哪個 MediaMTX。範例:
#
#   # publish_camera_crop.env
#   CROP_LEFT="${CROP_LEFT:-80}"
#   CROP_RIGHT="${CROP_RIGHT:-45}"
#   MEDIAMTX_RTSP_HOST="${MEDIAMTX_RTSP_HOST:-127.0.0.1}"
#   MEDIAMTX_RTSP_PORT="${MEDIAMTX_RTSP_PORT:-8554}"
#   MEDIAMTX_STREAM_PATH="${MEDIAMTX_STREAM_PATH:-camera}"
#
# 寫成 ${VAR:-value} 的形式是刻意的：這樣命令列上臨時帶的環境變數仍然優先，
# 用來試新數值時不必改檔案。
#
# 刻意不讀 repo 根目錄的 .env：那份是 docker compose 的檔案，裡面有 API key 與
# 資料庫密碼，而這支腳本會把自己的環境整份交給 gst-launch 的子行程。推流目標是
# host 端的事，跟 compose 的服務沒有共用的必要，放在腳本旁邊就好。
#
# -----------------------------------------------------------------------------
# 可用環境變數
#   DEVICE=/dev/syncai/camera0
#   SRC_WIDTH=1920   SRC_HEIGHT=1200   FRAMERATE=60     ← 從相機擷取的尺寸
#   OUT_WIDTH=1920   OUT_HEIGHT=1080                    ← 編碼輸出的尺寸
#   CROP_LEFT=0  CROP_RIGHT=0  CROP_TOP=0  CROP_BOTTOM=0  ← 各邊要裁掉的像素數
#   FIT_OUTPUT_ASPECT=1                                 ← 自動貼合輸出長寬比
#   BITRATE=4000000
#   MEDIAMTX_RTSP_HOST=127.0.0.1  MEDIAMTX_RTSP_PORT=8554   ← 推流目標
#   MEDIAMTX_STREAM_PATH=camera   (等同 STREAM_PATH，設定檔裡用前綴那個名字)
#   RTSP_URL=rtsp://<host>:<port>/<path>  ← 直接蓋掉整條 URL，最高優先
#   MEDIAMTX_WEBRTC_PORT=8889  MEDIAMTX_API_PORT=9997  ← 只影響印出的網址與查詢
#   MEDIAMTX_IMAGE=bluenviron/mediamtx:1.20.0  ← 版本與 config/mediamtx.yml 綁定
#   MEDIAMTX_CONTAINER=mediamtx
#   MEDIAMTX_CONFIG=<repo>/config/mediamtx.yml
#   SKIP_MEDIAMTX=0   ← 設 1 則完全不碰 MediaMTX，假設外部已備妥
#   AUTO_EXPOSURE=0  EXPOSURE=330   (EXPOSURE 只在 AUTO_EXPOSURE=1 時生效)
#   AUTO_WB=1        WB_TEMP=5000   (WB_TEMP 只在 AUTO_WB=0 時生效)
#   GAIN=1  BRIGHTNESS=16  POWER_LINE_FREQ=1
#   LOG_LEVEL=INFO   (DEBUG|INFO|WARN|ERROR)
#
# -----------------------------------------------------------------------------
# 相機硬體特性（AR0234，實測於 Jetson AGX Orin / L4T R36.4.4）
#
# 感測器原生 1920x1200 (16:10)。MJPG 與 UYVY 都提供 1920x1200 / 1920x1080 /
# 1280x720 / 640x480，每一種都是真正的 crop/scale，不會補黑邊。
#
# 選 MJPG 而非 UYVY 是 USB 頻寬問題：
#   UYVY 1920x1200@60 = 1920*1200*2*60 = 2.2 Gbit/s   → 逼近 USB 3.0 實際上限
#   MJPG 同規格（壓縮比約 10:1）      ≈ 90 Mbit/s     → 輕鬆
# 多付出的解碼成本由 Jetson 的專用 NVJPG 硬體引擎吸收，CPU 幾乎不動。
#
# nvjpegdec 輸出 NVMM I420，而編碼器只吃 NV12，所以 nvvidconv 是必要的。
# 兩者都是 YUV 4:2:0，差別只在色度平面排列（I420 三平面 / NV12 色度交錯）。
#
# IDR 間隔固定為半秒，跟著幀率算。每個 WHEP 觀眾都是中途加入 (mid-stream)，
# 在收到下一個 IDR (Instantaneous Decoder Refresh) 之前完全無法解碼，畫面是
# 全黑的。半秒 IDR = 新觀眾最多等半秒，代價是位元率偏高（IDR 是完整幀，遠比
# P-frame 肥，實測約 3.75 Mbit/s）。
#
# 注意：相機是 V4L2 擷取裝置，同一時間只准一個 process 持有。第二個進來的
# 不會在 open() 失敗，而是拖到 S_FMT 才爆 "Device or resource busy"。
# =============================================================================
set -euo pipefail

# 腳本自身的絕對路徑與 repo 根目錄。用絕對路徑是必要的：背景啟動時會重新
# 呼叫自己，相對路徑在換過工作目錄之後就找不到了。
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
SCRIPT_DIRECTORY="$(dirname "$SCRIPT_PATH")"
REPOSITORY_ROOT="$(dirname "$SCRIPT_DIRECTORY")"

# -----------------------------------------------------------------------------
# 載入設定，優先序：命令列環境變數 > publish_camera_crop.env > 腳本預設
#
# 黑邊寬度、尺寸、推流目標都放在旁邊那個檔案，不該寫死在共用腳本裡。
# 檔案不存在是正常情況，不是錯誤。
# -----------------------------------------------------------------------------
CONFIG_ENV_FILE="${CONFIG_ENV_FILE:-${SCRIPT_DIRECTORY}/publish_camera_crop.env}"
if [[ -f "$CONFIG_ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$CONFIG_ENV_FILE"
fi

# -----------------------------------------------------------------------------
# 日誌
#
# 全部寫到 stderr，避免污染 stdout —— 幾個計算用的函式是靠 stdout 回傳值的，
# 日誌混進去會把回傳值弄髒。等級門檻由 LOG_LEVEL 控制。
# -----------------------------------------------------------------------------
LOG_LEVEL="${LOG_LEVEL:-INFO}"

# 把等級名稱換成數值，方便比大小
log_level_to_number() {
  case "$1" in
    DEBUG) echo 10 ;;
    INFO)  echo 20 ;;
    WARN)  echo 30 ;;
    ERROR) echo 40 ;;
    *)     echo 20 ;;   # 不認得的等級一律當 INFO，不讓日誌設定本身弄爆腳本
  esac
}

LOG_LEVEL_THRESHOLD="$(log_level_to_number "$LOG_LEVEL")"

# 統一的日誌輸出：<時間> [<等級>] <訊息>
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
# 參數
# -----------------------------------------------------------------------------
DEVICE="${DEVICE:-/dev/syncai/camera0}"

# 擷取尺寸：預設用感測器原生全幅，讓裁切有最多素材可用
SRC_WIDTH="${SRC_WIDTH:-1920}"
SRC_HEIGHT="${SRC_HEIGHT:-1200}"
FRAMERATE="${FRAMERATE:-60}"

# 輸出尺寸：最終編碼與推流的尺寸
OUT_WIDTH="${OUT_WIDTH:-1920}"
OUT_HEIGHT="${OUT_HEIGHT:-1080}"

# -----------------------------------------------------------------------------
# 裁切量：每一邊要切掉幾個像素
#
# 這四個是「看到多寬的黑邊就填多少」，量出來多少就填多少，不需要自己換算座標。
# 左右兩邊通常不對稱（鏡頭模組沒有完美置中），所以 CROP_LEFT 與 CROP_RIGHT
# 是分開的兩個變數，不要假設它們相等。
# -----------------------------------------------------------------------------
CROP_LEFT="${CROP_LEFT:-0}"
CROP_RIGHT="${CROP_RIGHT:-0}"
CROP_TOP="${CROP_TOP:-0}"
CROP_BOTTOM="${CROP_BOTTOM:-0}"

# -----------------------------------------------------------------------------
# 是否自動貼合輸出長寬比
#
# 開啟時：在扣掉黑邊之後的可用區域裡，找出符合 OUT_WIDTH:OUT_HEIGHT 的最大矩形，
#         並且垂直置中。這樣輸出畫面不會被非等比拉伸。
# 關閉時：可用區域是多少就直接縮放到輸出尺寸，比例不同就會變形。
# -----------------------------------------------------------------------------
FIT_OUTPUT_ASPECT="${FIT_OUTPUT_ASPECT:-1}"

BITRATE="${BITRATE:-4000000}"

# -----------------------------------------------------------------------------
# MediaMTX
#
# 這支腳本自己把 MediaMTX container 管起來。原因是這個 repo 裡雖然有
# config/mediamtx.yml，但 docker-compose.yml 並沒有對應的服務定義（CLAUDE.md
# 說有，實際檔案裡只有 postgres / pgadmin / temporal / temporal_ui），整台機器
# 上也找不到 mediamtx 執行檔。推流總得有地方去，所以在這裡補上。
#
# network host 是必要的，有兩個理由：
#   1. config/mediamtx.yml 的 authInternalUsers 把 publish 限制在 127.0.0.1，
#      走 bridge 網路的話推流端的來源位址會是 172.x，過不了認證。
#   2. WebRTC 的 ICE candidate 必須是真實網路介面，bridge 會讓瀏覽器連不上。
#
# 版本刻意 pin 死，不用 latest：config/mediamtx.yml 的設定鍵是對照 v1.20.0 的
# 預設值逐項校對過的，而 MediaMTX 遇到不認識的鍵會直接拒絕啟動（不是警告，
# 是啟動失敗）。設定檔裡明確關掉的 moq 就是例子 —— 它在 v1.20.0 預設為 ON，
# 還會偷偷佔用 8892/8893 三個 port。這類鍵只要在新版改名或移除，容器就再也
# 起不來，而且會是在某次無關的 docker pull 之後才爆，很難聯想到原因。
# 設定檔與 image tag 必須一起改。
# -----------------------------------------------------------------------------
MEDIAMTX_IMAGE="${MEDIAMTX_IMAGE:-bluenviron/mediamtx:1.20.0}"
MEDIAMTX_CONTAINER="${MEDIAMTX_CONTAINER:-mediamtx}"
MEDIAMTX_CONFIG="${MEDIAMTX_CONFIG:-${REPOSITORY_ROOT}/config/mediamtx.yml}"
SKIP_MEDIAMTX="${SKIP_MEDIAMTX:-0}"

# -----------------------------------------------------------------------------
# 推流目標
#
# 預設 host 是 loopback 而不是 LAN 位址，這是認證問題不是偏好：mediamtx 把
# publish 限制在 127.0.0.1/::1（config/mediamtx.yml 的 authInternalUsers），
# 因為推流端永遠同機。連到 LAN IP 的連線會以那個 LAN IP 當來源位址，過不了 ip
# 比對，rtspclientsink 會在 RECORD 階段死掉並回報 "Not authorized to access
# resource" —— 而那時編碼器早就跑起來了，所以看起來像編碼失敗而不是認證失敗。
#
# 三個 port 的預設值必須跟 config/mediamtx.yml 的 rtspAddress / webrtcAddress /
# apiAddress 對上；那個檔案是唯一的真相來源，這裡只是它的鏡像。改一邊要改兩邊。
#
# STREAM_PATH 收 MEDIAMTX_STREAM_PATH 當第二層來源，是為了讓設定檔裡的推流目標
# 三個鍵同前綴、擺在一起看得出是一組（host / port / path）。腳本內部與命令列
# 仍然用 STREAM_PATH —— PID 檔與日誌檔的命名都掛在它身上。
# -----------------------------------------------------------------------------
MEDIAMTX_RTSP_HOST="${MEDIAMTX_RTSP_HOST:-127.0.0.1}"
MEDIAMTX_RTSP_PORT="${MEDIAMTX_RTSP_PORT:-8554}"
MEDIAMTX_WEBRTC_PORT="${MEDIAMTX_WEBRTC_PORT:-8889}"
MEDIAMTX_API_PORT="${MEDIAMTX_API_PORT:-9997}"

STREAM_PATH="${STREAM_PATH:-${MEDIAMTX_STREAM_PATH:-camera}}"
RTSP_URL="${RTSP_URL:-rtsp://${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT}/${STREAM_PATH}}"

# 背景執行用的 PID 檔與日誌檔。用 STREAM_PATH 命名，讓不同路徑可以並存。
RUNTIME_PID_FILE="${RUNTIME_PID_FILE:-/tmp/publish_camera_${STREAM_PATH}.pid}"
RUNTIME_LOG_FILE="${RUNTIME_LOG_FILE:-/tmp/publish_camera_${STREAM_PATH}.log}"

# 不管幀率設多少，都維持每半秒一個 IDR
IDR_INTERVAL=$(( FRAMERATE / 2 ))

# -----------------------------------------------------------------------------
# V4L2 感測器控制
#
# 這些不是裝飾性的預設值。UVC 控制項的狀態存在相機韌體裡，只要相機不斷電就會
# 跨 open/close 一路留給下一個程式，所以串流會繼承上一個 process（或某次手動
# v4l2-ctl）留下的任何狀態。
#
# 實際遇過的災情：相機停在 auto_exposure=1 (Manual Mode) 配
# exposure_time_absolute=40（4 ms，預設是 33 ms）—— 畫面暗到讓人以為編碼器壞
# 了；再加上 white_balance_automatic=0 且色溫凍在 4000 K，整個畫面泛藍。
#
# 極性陷阱，這兩個控制項的 1 代表相反的意思:
#   auto_exposure:           0 = Auto Mode,  1 = Manual Mode  （選單型，沿用 UVC 編號）
#   white_balance_automatic: 0 = 手動,       1 = 自動          （單純布林開關）
# 所以 AUTO_EXPOSURE=0 與 AUTO_WB=1 看似矛盾，其實兩個都是「自動」。
#
# 兩個手動值都是被閘控 (gated) 的：只有在對應的自動開關關掉時才寫得進去，
# 自動模式下驅動會拒絕寫入，v4l2src 只會為一個反正會被忽略的值印出警告。
# 反過來也要注意：自動模式下讀回來的 exposure_time_absolute 與
# white_balance_temperature 是陳舊值，韌體實際在用的值不會回寫，不能拿來判斷
# AE/AWB 有沒有在運作。
#
# exposure_time_absolute 的單位是 100 us，而且上限被幀週期卡死：60 fps 之下任何
# 超過 ~166（16.6 ms）的值都不可能被採納，這才是這顆相機室內偏暗的真正原因。
# 如果自動曝光已經頂到天花板還是暗，正解是降到 30 fps（曝光餘裕翻倍），而不是
# 把 EXPOSURE 調大 —— 調大會被幀率截斷，沒有用。
#
# white_balance_temperature 的語意是「告訴相機環境光是幾 K」，相機再反向補償：
# 設低（2300-3500K，暖光）→ 往藍補；設高（5000-6500K，日光）→ 往黃補。
#
# POWER_LINE_FREQ 預設 1（50 Hz）用來消除市電造成的閃爍橫紋；相機出廠是
# Disabled。60 Hz 電網請用 2。
# -----------------------------------------------------------------------------
AUTO_EXPOSURE="${AUTO_EXPOSURE:-0}"
EXPOSURE="${EXPOSURE:-330}"
AUTO_WB="${AUTO_WB:-1}"
WB_TEMP="${WB_TEMP:-5000}"
GAIN="${GAIN:-1}"
BRIGHTNESS="${BRIGHTNESS:-16}"
POWER_LINE_FREQ="${POWER_LINE_FREQ:-1}"

# 組出 v4l2src extra-controls 需要的控制字串
#
# 順序有意義：auto_* 開關必須排在它所閘控的手動值前面，否則驅動會拿「模式還是
# 自動」為由拒絕手動寫入。
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
# 裁切與縮放
#
# nvvidconv 一個元素就同時做裁切、縮放與色彩格式轉換，全部在 VIC 硬體上完成，
# 所以「切掉黑邊再放大回原始尺寸」不會多花任何 CPU，也不會破壞 NVMM 的零拷貝
# (zero-copy) 路徑。
#
# 要注意 nvvidconv 的 left/right/top/bottom 是「來源矩形的座標」，不是「要裁掉
# 幾個像素」。gst-inspect 的說明文字寫 "Pixels to crop at left"，那是錯的。
# 實測（1920x1200 輸入，left=480 right=1440 top=300 bottom=900）取出的是正中央
# 960x600 的區域，也就是 right-left 與 bottom-top。
# =============================================================================

# 向上對齊到偶數
#
# 專門用在「裁切量」上。YUV 4:2:0 的色度平面長寬各是亮度的一半，裁切邊界若是
# 奇數，色度取樣會錯位，表現為邊緣一條顏色不對的細線。
#
# 方向刻意選向上（多裁）而不是向下（少裁）：少裁一個像素會留下一條黑邊，那是
# 真正的錯誤；多裁一個像素只是損失一點點視野。
align_up_to_even() {
  echo $(( ($1 + 1) / 2 * 2 ))
}

# 向下對齊到偶數，用在置中偏移量上
align_down_to_even() {
  echo $(( $1 / 2 * 2 ))
}

# 歐幾里得演算法求最大公因數，用來把長寬比約分成最簡整數比
greatest_common_divisor() {
  local first="$1" second="$2" remainder
  while [[ "$second" -ne 0 ]]; do
    remainder=$(( first % second ))
    first="$second"
    second="$remainder"
  done
  echo "$first"
}

# 在可用區域內找出符合目標長寬比的最大矩形，以 "寬 高" 形式回傳
#
# 做法是把目標比例約分成最簡整數比（1920x1080 → 16:9），再找最大的倍數 k，
# 使得 (ratio_width*k, ratio_height*k) 仍然塞得進可用區域。
#
# 用整數比連乘而不是浮點相乘再四捨五入，是為了完全避開捨入誤差 —— 差一兩個
# 像素在這裡不是小事，它會讓 nvvidconv 做非等比縮放，畫面就被輕微拉伸了。
fit_rectangle_to_aspect() {
  local available_width="$1" available_height="$2"
  local target_width="$3" target_height="$4"

  local divisor ratio_width ratio_height
  divisor="$(greatest_common_divisor "$target_width" "$target_height")"
  ratio_width=$(( target_width / divisor ))
  ratio_height=$(( target_height / divisor ))

  # 分別算出受寬、受高限制的最大倍數，取較小的那個
  local max_multiple_by_width=$(( available_width / ratio_width ))
  local max_multiple_by_height=$(( available_height / ratio_height ))
  local multiple=$(( max_multiple_by_width < max_multiple_by_height \
                     ? max_multiple_by_width : max_multiple_by_height ))

  # 再往下找到能讓寬高都是偶數的倍數
  while [[ "$multiple" -gt 0 ]]; do
    if [[ $(( (ratio_width * multiple) % 2 )) -eq 0 && \
          $(( (ratio_height * multiple) % 2 )) -eq 0 ]]; then
      break
    fi
    multiple=$(( multiple - 1 ))
  done

  if [[ "$multiple" -le 0 ]]; then
    log_error "可用區域 ${available_width}x${available_height} 放不下任何 ${ratio_width}:${ratio_height} 的矩形"
    return 1
  fi

  log_debug "長寬比 ${target_width}:${target_height} 約分為 ${ratio_width}:${ratio_height}，倍數 ${multiple}"
  echo "$(( ratio_width * multiple )) $(( ratio_height * multiple ))"
}

# 依裁切量與輸出比例算出 nvvidconv 的參數字串；不需要裁切時回傳空字串
build_crop_arguments() {
  # ---- 步驟 1：把使用者指定的裁切量對齊到偶數 ----
  local crop_left crop_right crop_top crop_bottom
  crop_left="$(align_up_to_even "$CROP_LEFT")"
  crop_right="$(align_up_to_even "$CROP_RIGHT")"
  crop_top="$(align_up_to_even "$CROP_TOP")"
  crop_bottom="$(align_up_to_even "$CROP_BOTTOM")"

  # ---- 步驟 2：算出扣掉黑邊之後的可用區域 ----
  local available_left="$crop_left"
  local available_top="$crop_top"
  local available_width=$(( SRC_WIDTH - crop_left - crop_right ))
  local available_height=$(( SRC_HEIGHT - crop_top - crop_bottom ))

  if [[ "$available_width" -le 0 || "$available_height" -le 0 ]]; then
    log_error "裁切量過大：可用區域為 ${available_width}x${available_height}（必須為正）"
    log_error "  來源 ${SRC_WIDTH}x${SRC_HEIGHT}，左右共裁 $(( crop_left + crop_right ))，上下共裁 $(( crop_top + crop_bottom ))"
    return 1
  fi

  # ---- 步驟 3：貼合輸出長寬比，避免非等比拉伸 ----
  local final_width="$available_width"
  local final_height="$available_height"

  if [[ "$FIT_OUTPUT_ASPECT" == "1" ]]; then
    local fitted_rectangle
    # 必須用 if 明確接住失敗。fit_rectangle_to_aspect 是在 $() 的子 shell 裡
    # 執行的，它內部的 return 不會終止主流程，只會變成這個指令替換的離開碼。
    if ! fitted_rectangle="$(fit_rectangle_to_aspect \
          "$available_width" "$available_height" "$OUT_WIDTH" "$OUT_HEIGHT")"; then
      return 1
    fi
    read -r final_width final_height <<< "$fitted_rectangle"
  fi

  # ---- 步驟 4：分配剩餘空間，換算成來源矩形座標 ----
  #
  # 水平方向尊重使用者指定的左邊界不動：黑邊寬度是量出來的，不能為了湊比例
  # 而少裁。貼合比例後多出來的寬度一律從右邊再多切一點。
  # 垂直方向則置中，因為上下的裁切量通常是為了湊比例才產生的。
  local rect_left="$available_left"
  local rect_right=$(( rect_left + final_width ))

  local vertical_slack=$(( available_height - final_height ))
  local vertical_offset
  vertical_offset="$(align_down_to_even $(( vertical_slack / 2 )))"
  local rect_top=$(( available_top + vertical_offset ))
  local rect_bottom=$(( rect_top + final_height ))

  # ---- 步驟 5：最終矩形若等於整張畫面，就不加參數 ----
  if [[ "$rect_left" -eq 0 && "$rect_top" -eq 0 && \
        "$rect_right" -eq "$SRC_WIDTH" && "$rect_bottom" -eq "$SRC_HEIGHT" ]]; then
    log_debug "最終矩形等於整張畫面，nvvidconv 不加 crop 參數"
    echo ""
    return 0
  fi

  # ---- 步驟 6：把算出來的結果講清楚 ----
  log_info "裁切量（對齊偶數後）: 左${crop_left} 右${crop_right} 上${crop_top} 下${crop_bottom}"
  log_info "  可用區域 ${available_width}x${available_height}"
  log_info "  來源矩形 (${rect_left},${rect_top})-(${rect_right},${rect_bottom}) = ${final_width}x${final_height}"
  log_info "  縮放輸出 ${OUT_WIDTH}x${OUT_HEIGHT}"

  local effective_crop_right=$(( SRC_WIDTH - rect_right ))
  local effective_crop_bottom=$(( SRC_HEIGHT - rect_bottom ))
  if [[ "$effective_crop_right" -ne "$crop_right" || "$effective_crop_bottom" -ne "$crop_bottom" ]]; then
    log_info "  為湊比例調整後的實際裁切量: 左${rect_left} 右${effective_crop_right} 上${rect_top} 下${effective_crop_bottom}"
  fi

  local final_aspect output_aspect
  final_aspect=$(( final_width * 10000 / final_height ))
  output_aspect=$(( OUT_WIDTH * 10000 / OUT_HEIGHT ))
  if [[ "$final_aspect" -ne "$output_aspect" ]]; then
    log_warn "裁切後長寬比 (${final_width}:${final_height}) 與輸出 (${OUT_WIDTH}:${OUT_HEIGHT}) 不同，畫面會被非等比拉伸"
    log_warn "  要避免變形請設 FIT_OUTPUT_ASPECT=1"
  fi

  echo "left=${rect_left} right=${rect_right} top=${rect_top} bottom=${rect_bottom}"
}

# =============================================================================
# MediaMTX 管理
# =============================================================================

# 確認 image 在本機
#
# 刻意不在這裡自動 docker pull。推流是即時性的操作，中間插一段不知道要跑多久
# 的下載（還可能因為沒網路而卡住）會讓這支腳本的行為變得不可預測。image 屬於
# 環境準備的一部分，事先備妥，這裡只負責明確地說出缺了什麼。
ensure_mediamtx_image() {
  if docker image inspect "$MEDIAMTX_IMAGE" >/dev/null 2>&1; then
    log_debug "MediaMTX image 已存在: $MEDIAMTX_IMAGE"
    return 0
  fi

  log_error "本機沒有 image: $MEDIAMTX_IMAGE"
  log_error "  先下載好再跑:  docker pull $MEDIAMTX_IMAGE"
  log_error "  沒有外網的話從別台機器搬:  docker save $MEDIAMTX_IMAGE | ssh <host> docker load"
  exit 1
}

# 用指定的參數建立並啟動 MediaMTX container
#
# 抽成一個函式是因為底下有兩個地方要建：完全沒有的時候，以及現有的那個不符
# 預期而被砍掉重建的時候。參數只寫一份，改的時候不會漏掉其中一處。
create_mediamtx_container() {
  docker run -d \
    --name "$MEDIAMTX_CONTAINER" \
    --network host \
    --restart unless-stopped \
    -v "${MEDIAMTX_CONFIG}:/mediamtx.yml:ro" \
    "$MEDIAMTX_IMAGE" >/dev/null
}

# 取得 MediaMTX container 的狀態；不存在時回傳 absent
#
# 不能直接寫成 `docker inspect ... || echo absent`：container 不存在時
# docker inspect 會先往 stdout 吐一個空行才以非 0 離開，那樣會得到「空行 +
# absent」兩行，後面的 case 比對就對不上 absent，會誤走到「嘗試 start」的分支。
# 取第一行、空的才補 absent。
get_mediamtx_container_state() {
  local state
  state="$(docker inspect -f '{{.State.Status}}' "$MEDIAMTX_CONTAINER" 2>/dev/null | head -1)"
  echo "${state:-absent}"
}

# 檢查現有 container 是不是用我們要的 image 跑的
#
# 這個檢查是為了版本切換：改了 MEDIAMTX_IMAGE 卻沿用舊 container，跑的還是舊
# 版本，而設定檔已經換成新版的鍵 —— 那種「明明改了卻沒生效」最難查。
mediamtx_container_matches_image() {
  local running_image
  running_image="$(docker inspect -f '{{.Config.Image}}' "$MEDIAMTX_CONTAINER" 2>/dev/null | head -1)"
  [[ "$running_image" == "$MEDIAMTX_IMAGE" ]]
}

# 把 MediaMTX container 拉到「正在執行，而且吃的是正確 image 與最新設定」的狀態
#
# 每一種可能的狀態都要處理，因為這個 container 不歸 compose 管，隨時可能被人
# 手動 docker rm 掉、被 docker system prune 掃掉，或是上一次啟動失敗留下一個
# created / dead 的殼：
#   image 不符      → 砍掉重建（版本切換）
#   running         → restart，確保重新讀取 bind mount 進去的設定檔
#   不存在          → 直接建
#   其他 (exited/created/paused/dead) → 先試 start，起不來就砍掉重建
ensure_mediamtx_running() {
  if [[ ! -f "$MEDIAMTX_CONFIG" ]]; then
    log_error "找不到 MediaMTX 設定檔: $MEDIAMTX_CONFIG"
    exit 1
  fi
  log_debug "MediaMTX 設定檔: $MEDIAMTX_CONFIG"

  ensure_mediamtx_image

  # 不存在時 docker inspect 會失敗，統一表示成 absent
  local container_state
  container_state="$(get_mediamtx_container_state)"
  log_debug "MediaMTX container 目前狀態: $container_state"

  # image 對不上就直接重建，不管它現在是什麼狀態
  if [[ "$container_state" != "absent" ]] && ! mediamtx_container_matches_image; then
    local previous_image
    previous_image="$(docker inspect -f '{{.Config.Image}}' "$MEDIAMTX_CONTAINER" 2>/dev/null | head -1)"
    log_warn "現有 container 用的是 $previous_image，與指定的 $MEDIAMTX_IMAGE 不符，重建"
    docker rm -f "$MEDIAMTX_CONTAINER" >/dev/null 2>&1 || true
    container_state="absent"
  fi

  case "$container_state" in
    running)
      log_info "MediaMTX ($MEDIAMTX_CONTAINER) 正在執行，重啟以重新載入設定"
      docker restart "$MEDIAMTX_CONTAINER" >/dev/null
      ;;
    absent)
      log_info "建立並啟動 MediaMTX container ($MEDIAMTX_CONTAINER, $MEDIAMTX_IMAGE)"
      create_mediamtx_container
      ;;
    *)
      log_info "MediaMTX container 狀態為 $container_state，嘗試啟動"
      if ! docker start "$MEDIAMTX_CONTAINER" >/dev/null 2>&1; then
        log_warn "啟動失敗，移除後重建"
        docker rm -f "$MEDIAMTX_CONTAINER" >/dev/null 2>&1 || true
        create_mediamtx_container
      fi
      ;;
  esac
}

# 等 MediaMTX 的 RTSP port 真的開始接受連線
#
# 這一步不能省。docker run 回來只代表 container 建立了，MediaMTX 進程還要幾百
# 毫秒才會 bind 到 RTSP port。太早開始推流的話 rtspclientsink 會 connection
# refused 而死，而那個錯誤看起來會像設定寫錯，不像時序問題。
#
# 探測的 host/port 一定要跟 RTSP_URL 同一組變數算出來，不能寫死 8554：改了
# port 卻探測舊 port，會變成「等到逾時就放棄」或「探到別的服務就衝出去推流」。
wait_for_rtsp_ready() {
  local max_attempts=40      # 40 x 0.25s = 最多等 10 秒
  local attempt=0

  while [[ "$attempt" -lt "$max_attempts" ]]; do
    if timeout 1 bash -c "cat < /dev/null > /dev/tcp/${MEDIAMTX_RTSP_HOST}/${MEDIAMTX_RTSP_PORT}" 2>/dev/null; then
      log_info "MediaMTX 已就緒（RTSP ${MEDIAMTX_RTSP_PORT} 接受連線）"
      return 0
    fi
    attempt=$(( attempt + 1 ))
    sleep 0.25
  done

  log_error "等待 MediaMTX 就緒逾時（10 秒內 ${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT} 沒有回應）"
  log_error "  看它的日誌:  docker logs $MEDIAMTX_CONTAINER"
  exit 1
}

# 停掉 MediaMTX container（保留它，下次 start 直接 start 起來就好）
stop_mediamtx() {
  local container_state
  container_state="$(get_mediamtx_container_state)"

  case "$container_state" in
    absent)
      log_info "MediaMTX container 不存在，不需要停止"
      ;;
    running)
      log_info "停止 MediaMTX container ($MEDIAMTX_CONTAINER)"
      docker stop "$MEDIAMTX_CONTAINER" >/dev/null
      ;;
    *)
      log_info "MediaMTX container 已經不在執行中（狀態 $container_state）"
      ;;
  esac
}

# 列出所有可以看串流的網址
#
# scope global 會濾掉 loopback；一台機器可能有多張網卡，全部列出來省得自己查。
print_viewing_urls() {
  local address
  while read -r address; do
    [[ -z "$address" ]] && continue
    log_info "   RTSP   rtsp://${address}:${MEDIAMTX_RTSP_PORT}/${STREAM_PATH}"
    log_info "   WHEP   http://${address}:${MEDIAMTX_WEBRTC_PORT}/${STREAM_PATH}"
  done < <(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1)
}

# =============================================================================
# 推流 process 的生命週期
# =============================================================================

# 讀 PID 檔，回傳一個「還活著」的 PID；沒有就回傳空字串
#
# 會順手清掉失效的 PID 檔（process 已經死了但檔案還在），否則之後每次 start
# 都會誤以為已經在跑。
read_live_pipeline_pid() {
  [[ -f "$RUNTIME_PID_FILE" ]] || { echo ""; return 0; }

  local recorded_pid
  recorded_pid="$(cat "$RUNTIME_PID_FILE" 2>/dev/null || echo "")"

  if [[ -z "$recorded_pid" ]] || ! kill -0 "$recorded_pid" 2>/dev/null; then
    log_debug "PID 檔內容失效（$recorded_pid），清除"
    rm -f "$RUNTIME_PID_FILE"
    echo ""
    return 0
  fi

  echo "$recorded_pid"
}

# 停掉推流 process
#
# 先送 SIGINT 而不是 SIGKILL：gst-launch 的 -e 參數會把 SIGINT 轉成往下游送
# EOS，讓 rtspclientsink 有機會正常地跟 MediaMTX 說再見，而不是從畫面中間硬
# 切斷留下半個 session。等不到才升級成 SIGKILL。
stop_pipeline() {
  local live_pid
  live_pid="$(read_live_pipeline_pid)"

  if [[ -z "$live_pid" ]]; then
    log_info "推流 process 沒有在執行"
    return 0
  fi

  log_info "停止推流 process (PID $live_pid)，送 SIGINT 讓它正常結束"
  kill -INT "$live_pid" 2>/dev/null || true

  # 最多等 5 秒讓它自己收尾
  local attempt=0
  while [[ "$attempt" -lt 50 ]]; do
    if ! kill -0 "$live_pid" 2>/dev/null; then
      log_info "推流 process 已結束"
      rm -f "$RUNTIME_PID_FILE"
      return 0
    fi
    attempt=$(( attempt + 1 ))
    sleep 0.1
  done

  log_warn "SIGINT 之後 5 秒仍未結束，改送 SIGKILL"
  kill -KILL "$live_pid" 2>/dev/null || true
  rm -f "$RUNTIME_PID_FILE"
}

# =============================================================================
# 前置檢查
# =============================================================================

# 確認裝置節點存在
check_device_exists() {
  if [[ ! -e "$DEVICE" ]]; then
    log_error "$DEVICE 不存在。相機插上了嗎？"
    exit 1
  fi
  log_debug "裝置節點存在: $DEVICE"
}

# 確認沒有別的 process 佔著相機，有的話直接指名道姓
#
# 這裡必須先把 DEVICE 正規化。kernel 在 /proc/<pid>/fd/ 底下記錄的永遠是解析
# 過 symlink 的真實路徑，而 DEVICE 預設值 /dev/syncai/camera0 正是一個 symlink
# （指向 ../video0）。直接拿 symlink 路徑去做字串比對永遠不會相等 —— 原本的
# 版本就是這樣寫的，那個檢查一次都沒攔到過。
check_device_not_busy() {
  local device_real_path
  device_real_path="$(readlink -f "$DEVICE")"
  log_debug "裝置正規化路徑: $DEVICE -> $device_real_path"

  local pid_directory file_descriptor link_target holder_pid holder_command
  for pid_directory in /proc/[0-9]*; do
    for file_descriptor in "$pid_directory"/fd/*; do
      link_target="$(readlink "$file_descriptor" 2>/dev/null)" || continue
      if [[ "$link_target" == "$device_real_path" ]]; then
        holder_pid="$(basename "$pid_directory")"
        holder_command="$(tr -d '\0' < "$pid_directory/comm" 2>/dev/null)"
        log_error "$DEVICE ($device_real_path) 已被 PID $holder_pid ($holder_command) 佔用。"
        log_error "  先停掉它:  kill -INT $holder_pid"
        exit 1
      fi
    done
  done
  log_debug "沒有其他 process 佔用相機"
}

# 確認 GStreamer 與 Tegra 硬體元件都在
#
# nvjpegdec / nvvidconv / nvv4l2h264enc 來自 nvidia-l4t-gstreamer，只有 L4T 的
# host 上才有。在沒有這些元件的環境跑，錯誤訊息是 no element "nvvidconv"，
# 先擋下來比較好懂。
check_gstreamer_elements() {
  local element
  for element in v4l2src nvjpegdec nvvidconv nvv4l2h264enc h264parse rtspclientsink; do
    if ! gst-inspect-1.0 "$element" >/dev/null 2>&1; then
      log_error "缺少 GStreamer 元件: $element"
      log_error "  Tegra 元件 (nv*) 需要在 L4T host 上執行，或在有 nvidia runtime 的 container 裡"
      exit 1
    fi
  done
  log_debug "GStreamer 元件齊全"
}

# =============================================================================
# 子命令
# =============================================================================

# 內部使用：真正執行 GStreamer pipeline，永遠不會返回
#
# 被 cmd_start 以背景方式重新呼叫自己而進入。這裡用 exec 取代 shell process，
# 所以背景那層記錄下來的 PID 就是 gst-launch 本身的 PID，stop 才殺得準。
cmd_run_pipeline() {
  local sensor_controls crop_arguments
  sensor_controls="$(build_sensor_controls)"

  # 明確接住失敗：build_crop_arguments 在子 shell 裡跑，它的 return 1 不會
  # 自己終止主流程。
  if ! crop_arguments="$(build_crop_arguments)"; then
    log_error "裁切參數計算失敗，中止推流"
    exit 1
  fi

  log_info "推流 ${DEVICE} 擷取 ${SRC_WIDTH}x${SRC_HEIGHT}@${FRAMERATE} → 輸出 ${OUT_WIDTH}x${OUT_HEIGHT} → ${RTSP_URL}"
  log_info "感測器控制: ${sensor_controls}"
  log_info "位元率 ${BITRATE} bps，IDR 間隔 ${IDR_INTERVAL} 幀（約 0.5 秒）"

  # -e 讓 SIGINT 往下游送 EOS，而不是從畫面中間硬切斷
  #
  # crop_arguments 刻意不加引號：它可能是空字串（不裁切）或多個以空白分隔的
  # 參數，需要讓 shell 做字詞切分。
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

# 前景執行，Ctrl-C 結束。除錯時用，錯誤訊息直接看得到。
cmd_foreground() {
  check_device_exists
  check_device_not_busy
  check_gstreamer_elements

  if [[ "$SKIP_MEDIAMTX" == "1" ]]; then
    log_info "SKIP_MEDIAMTX=1，跳過 MediaMTX 的啟動"
  else
    ensure_mediamtx_running
    wait_for_rtsp_ready
  fi

  log_info "──────────────────────────────────────────────"
  log_info " 可以從這些網址看到畫面:"
  print_viewing_urls
  log_info "──────────────────────────────────────────────"

  cmd_run_pipeline
}

# 背景啟動
cmd_start() {
  local live_pid
  live_pid="$(read_live_pipeline_pid)"
  if [[ -n "$live_pid" ]]; then
    log_error "推流已經在執行中 (PID $live_pid)"
    log_error "  要重啟請用:  $(basename "$SCRIPT_PATH") restart"
    exit 1
  fi

  check_device_exists
  check_device_not_busy
  check_gstreamer_elements

  # 順序不能反：推流端找不到伺服器會直接死掉，所以 MediaMTX 必須先起來並且
  # 確認接受連線，才輪到推流。
  if [[ "$SKIP_MEDIAMTX" == "1" ]]; then
    log_info "SKIP_MEDIAMTX=1，跳過 MediaMTX 的啟動"
  else
    ensure_mediamtx_running
    wait_for_rtsp_ready
  fi

  log_info "背景啟動推流，日誌寫到 $RUNTIME_LOG_FILE"

  # setsid 讓它脫離目前的 session，ssh 斷線或關掉終端機都不會把它帶走
  setsid nohup bash "$SCRIPT_PATH" __run_pipeline >> "$RUNTIME_LOG_FILE" 2>&1 &
  local started_pid=$!
  echo "$started_pid" > "$RUNTIME_PID_FILE"

  # 給它一點時間，如果會失敗通常一秒內就爆了，直接把日誌尾巴貼出來比讓使用者
  # 自己去翻檔案好。
  sleep 2

  if ! kill -0 "$started_pid" 2>/dev/null; then
    log_error "推流啟動後隨即結束，日誌尾巴："
    tail -20 "$RUNTIME_LOG_FILE" >&2
    rm -f "$RUNTIME_PID_FILE"
    exit 1
  fi

  log_info "推流執行中 (PID $started_pid)"
  log_info "──────────────────────────────────────────────"
  log_info " 可以從這些網址看到畫面:"
  print_viewing_urls
  log_info "──────────────────────────────────────────────"
  log_info " 停止:  $(basename "$SCRIPT_PATH") stop"
  log_info " 日誌:  $(basename "$SCRIPT_PATH") logs"
}

# 停止推流與 MediaMTX
cmd_stop() {
  stop_pipeline
  if [[ "$SKIP_MEDIAMTX" == "1" ]]; then
    log_info "SKIP_MEDIAMTX=1，不動 MediaMTX"
  else
    stop_mediamtx
  fi
  log_info "已停止"
}

cmd_restart() {
  cmd_stop
  # 相機釋放需要一點時間，太快接著開會撞到 Device or resource busy
  sleep 1
  cmd_start
}

# 顯示目前狀態
cmd_status() {
  local live_pid container_state
  live_pid="$(read_live_pipeline_pid)"
  container_state="$(get_mediamtx_container_state)"

  echo "MediaMTX"
  echo "  container : $MEDIAMTX_CONTAINER"
  echo "  image     : $MEDIAMTX_IMAGE"
  echo "  狀態      : $container_state"
  echo "  設定檔    : $MEDIAMTX_CONFIG"
  echo ""
  echo "推流"
  if [[ -n "$live_pid" ]]; then
    echo "  狀態      : 執行中 (PID $live_pid)"
  else
    echo "  狀態      : 未執行"
  fi
  echo "  裝置      : $DEVICE"
  echo "  擷取      : ${SRC_WIDTH}x${SRC_HEIGHT}@${FRAMERATE}"
  echo "  輸出      : ${OUT_WIDTH}x${OUT_HEIGHT}"
  echo "  裁切      : 左${CROP_LEFT} 右${CROP_RIGHT} 上${CROP_TOP} 下${CROP_BOTTOM}"
  echo "  目標      : $RTSP_URL"
  echo "  日誌      : $RUNTIME_LOG_FILE"
  echo "  設定來源  : $CONFIG_ENV_FILE"
  echo ""

  # 問 MediaMTX 的控制 API 這條 path 到底有沒有真的在收流。
  # 這是唯一能確認「串流真的活著」的方法 —— process 活著不代表資料有進去。
  if [[ "$container_state" == "running" ]]; then
    echo "MediaMTX 回報的 path 狀態"
    if command -v curl >/dev/null 2>&1; then
      curl -s --max-time 3 "http://127.0.0.1:${MEDIAMTX_API_PORT}/v3/paths/list" 2>/dev/null \
        | sed 's/,/,\n/g' | grep -E '"name"|"ready"|"tracks"|"bytesReceived"' | sed 's/^/  /' \
        || echo "  (查詢失敗)"
    else
      echo "  (沒有 curl，跳過)"
    fi
    echo ""
  fi

  if [[ "$container_state" == "running" ]]; then
    echo "觀看網址"
    local address
    while read -r address; do
      [[ -z "$address" ]] && continue
      echo "  RTSP   rtsp://${address}:${MEDIAMTX_RTSP_PORT}/${STREAM_PATH}"
      echo "  WHEP   http://${address}:${MEDIAMTX_WEBRTC_PORT}/${STREAM_PATH}"
    done < <(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1)
  fi
}

# 追日誌
cmd_logs() {
  if [[ ! -f "$RUNTIME_LOG_FILE" ]]; then
    log_error "日誌檔不存在: $RUNTIME_LOG_FILE"
    exit 1
  fi
  tail -f "$RUNTIME_LOG_FILE"
}

cmd_usage() {
  cat <<USAGE
用法: $(basename "$SCRIPT_PATH") [子命令]

  (無)        等同 start
  start       背景啟動 MediaMTX 與推流
  stop        停止推流與 MediaMTX
  restart     重啟
  status      顯示狀態、串流是否真的在收流、觀看網址
  logs        追推流日誌 (tail -f)
  foreground  前景執行，除錯用
  help        顯示這段說明

設定檔: $CONFIG_ENV_FILE
  裁切與尺寸    CROP_* / SRC_* / OUT_*
  推流目標      MEDIAMTX_RTSP_HOST / MEDIAMTX_RTSP_PORT / MEDIAMTX_STREAM_PATH
                （或直接帶完整的 RTSP_URL 蓋掉整條）
命令列上臨時帶的環境變數優先於設定檔。
USAGE
}

# =============================================================================
# 進入點
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
    # 內部子命令，由 cmd_start 背景呼叫，使用者不該直接用
    __run_pipeline) cmd_run_pipeline ;;
    *)
      log_error "不認得的子命令: $subcommand"
      cmd_usage >&2
      exit 1
      ;;
  esac
}

# 只有被直接執行時才跑主流程。
# 被 source 進來時（例如單元測試）僅載入函式定義，不會啟動任何東西。
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  main "$@"
fi
