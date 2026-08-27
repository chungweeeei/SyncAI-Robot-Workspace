#!/bin/bash
# =============================================================================
# 相機推流（裁切 + 縮放 + 推到遠端 MediaMTX）
#
# 跟原本的 scripts/publish_camera.sh 的差別：
#   1. 支援裁切黑邊 + 縮放回指定輸出尺寸（nvvidconv 硬體完成，零 CPU 成本）
#   2. 修正原本前置檢查對 symlink 失效的問題（那個檢查從來沒攔到過）
#   3. 有 start / stop / status / restart 子命令，背景執行
#
# 這是新增的檔案，原本的 scripts/publish_camera.sh 完全不動。
#
# -----------------------------------------------------------------------------
# MediaMTX 不由這支腳本管理
#
# 這台機器只當 publisher，伺服器是外部既有的服務，用 MEDIAMTX_RTSP_HOST 指過去。
# 因此本機不需要 mediamtx 的 image 或 container，也不需要 config/mediamtx.yml
# —— 那份設定屬於跑伺服器的那台機器。
#
# 早期版本會自己 docker run 一個 mediamtx 起來（image 不在就報錯、被 rm 掉會重建、
# 版本不符會砍掉重建），整段已經移除。理由不是麻煩，是拓樸：串流的匯集點是全隊
# 共用的一個服務，每台機器各自起一個本機 broker 只會讓觀看端不知道該連哪一台。
# 要回頭看那段 container 管理的邏輯，翻 git history。
#
# -----------------------------------------------------------------------------
# 用法（在 Jetson 的 host 上跑，不是在 container 裡）
#
#   bash scripts/publish_camera_crop.sh            # 啟動（背景），什麼都不用帶
#   bash scripts/publish_camera_crop.sh stop       # 停止推流
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
#   MEDIAMTX_RTSP_HOST="${MEDIAMTX_RTSP_HOST:-192.168.8.160}"
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
#   MEDIAMTX_RTSP_HOST=<遠端伺服器>  MEDIAMTX_RTSP_PORT=8554   ← 推流目標
#   MEDIAMTX_STREAM_PATH=camera   (等同 STREAM_PATH，設定檔裡用前綴那個名字)
#   MEDIAMTX_PUBLISH_USER=  MEDIAMTX_PUBLISH_PASS=   ← 伺服器要認證才需要
#   RTSP_URL=rtsp://<host>:<port>/<path>  ← 直接蓋掉整條 URL，最高優先
#   MEDIAMTX_WEBRTC_PORT=8889  MEDIAMTX_API_PORT=9997  ← 只影響印出的網址與查詢
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

# 腳本自身的絕對路徑。用絕對路徑是必要的：背景啟動時會重新呼叫自己，相對路徑
# 在換過工作目錄之後就找不到了。
#
# 不再需要 repo 根目錄：那是以前要拿 config/mediamtx.yml 去 bind mount 才算的，
# MediaMTX 移到遠端之後這支腳本只讀旁邊的 publish_camera_crop.env。
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
SCRIPT_DIRECTORY="$(dirname "$SCRIPT_PATH")"

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
# 推流目標（遠端 MediaMTX）
#
# 沒有 host 的預設值可以撿：伺服器是哪一台是佈署決定的事，寫死一個 IP 只會讓
# 打錯字的那次變成「推去了某個不知道是誰的位址」。所以留白，缺了就在前置檢查
# 明確擋下來，並指到設定檔。
#
# 三個 port 都在同一台 host 上。RTSP 是真正推流用的；WEBRTC 與 API 只影響印出來
# 的觀看網址與 status 的查詢。這些值必須跟伺服器那邊 mediamtx.yml 的
# rtspAddress / webrtcAddress / apiAddress 對上 —— 那個檔案在伺服器上，不在這個
# repo 裡（repo 裡的 config/mediamtx.yml 是舊的同機佈署留下來的），所以這裡的
# 值改不改，本機看不出對錯，要去伺服器上核對。
#
# 推得上去的前提是伺服器的 authInternalUsers 允許這台機器 publish。本 repo 附的
# config/mediamtx.yml 把 publish 限制在 127.0.0.1/::1（那是給「伺服器與推流端同
# 機」的舊佈署寫的），遠端推流不在允許範圍內：rtspclientsink 會在 RECORD 階段死
# 掉並回報 "Not authorized to access resource" —— 而那時編碼器早就跑起來了，所以
# 看起來像編碼失敗而不是認證失敗。伺服器端要嘛把這台的 IP 加進 ips，要嘛給一組
# 帳密（見下方 MEDIAMTX_PUBLISH_USER / MEDIAMTX_PUBLISH_PASS）。
#
# STREAM_PATH 收 MEDIAMTX_STREAM_PATH 當第二層來源，是為了讓設定檔裡的推流目標
# 三個鍵同前綴、擺在一起看得出是一組（host / port / path）。腳本內部與命令列
# 仍然用 STREAM_PATH —— PID 檔與日誌檔的命名都掛在它身上。
#
# 多台機器推同一個伺服器時，path 要帶機器名當前綴（例如 dogA/camera），否則第二
# 台推上去會跟第一台搶同一條 path，伺服器只留一個 publisher。
# -----------------------------------------------------------------------------
MEDIAMTX_RTSP_HOST="${MEDIAMTX_RTSP_HOST:-}"
MEDIAMTX_RTSP_PORT="${MEDIAMTX_RTSP_PORT:-8554}"
MEDIAMTX_WEBRTC_PORT="${MEDIAMTX_WEBRTC_PORT:-8889}"
MEDIAMTX_API_PORT="${MEDIAMTX_API_PORT:-9997}"

STREAM_PATH="${STREAM_PATH:-${MEDIAMTX_STREAM_PATH:-camera}}"

# 推流帳密是選用的：伺服器改用 ips 白名單放行這台機器時就不必帶。
#
# 只有真的有帳號才組進 URL。空的時候不要留成 rtsp://:@host —— 那是一次帶著空帳號
# 的認證嘗試，mediamtx 會當成認證失敗而不是「匿名」，跟完全不帶不一樣。
# 密碼裡有 @ : / 之類的字元要先自己做 percent-encoding，URL 的語法不會幫你跳脫。
MEDIAMTX_PUBLISH_USER="${MEDIAMTX_PUBLISH_USER:-}"
MEDIAMTX_PUBLISH_PASS="${MEDIAMTX_PUBLISH_PASS:-}"

RTSP_CREDENTIALS=""
if [[ -n "$MEDIAMTX_PUBLISH_USER" ]]; then
  RTSP_CREDENTIALS="${MEDIAMTX_PUBLISH_USER}:${MEDIAMTX_PUBLISH_PASS}@"
fi

RTSP_URL="${RTSP_URL:-rtsp://${RTSP_CREDENTIALS}${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT}/${STREAM_PATH}}"

# 給日誌與 status 用的版本，密碼遮掉。
#
# 推流的日誌是留在 /tmp 的檔案、status 是印在終端機上（常常被貼到聊天室裡），
# 兩個都不該留下密碼。真正交給 gst-launch 的永遠是未遮蔽的 RTSP_URL。
redact_rtsp_url() {
  sed -E 's#(rtsp://[^:/@]+):[^@/]*@#\1:***@#' <<< "$1"
}
RTSP_URL_DISPLAY="$(redact_rtsp_url "$RTSP_URL")"

# 背景執行用的 PID 檔與日誌檔。用 STREAM_PATH 命名，讓不同路徑可以並存。
#
# 檔名裡的斜線要換掉。推到共用伺服器時 path 通常帶機器名前綴（dogB/camera），
# 直接嵌進檔名會變成 /tmp/publish_camera_dogB/camera.log —— 那個目錄不存在，
# 於是背景啟動時連日誌都開不起來，而 start 只會回報「啟動後隨即結束」，看不出
# 是檔名的問題。
RUNTIME_NAME="${STREAM_PATH//\//_}"
RUNTIME_PID_FILE="${RUNTIME_PID_FILE:-/tmp/publish_camera_${RUNTIME_NAME}.pid}"
RUNTIME_LOG_FILE="${RUNTIME_LOG_FILE:-/tmp/publish_camera_${RUNTIME_NAME}.log}"

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
# 遠端 MediaMTX
# =============================================================================

# 確認遠端 MediaMTX 的 RTSP port 接受連線
#
# 這一步不能省，而且理由跟本機佈署時不一樣：以前是等剛 docker run 起來的
# container 完成 bind，現在伺服器在別台機器上，它可能沒在跑、網路不通，或位址就
# 是打錯了。少了這個檢查，rtspclientsink 會 connection refused 然後立刻死掉，而
# start 是背景執行的，使用者只會看到「啟動後隨即結束」，看不出是網路問題。
#
# 探測的 host/port 一定要跟 RTSP_URL 同一組變數算出來，不能寫死 8554：改了 port
# 卻探測舊 port，會變成「等到逾時就放棄」或「探到別的服務就衝出去推流」。
#
# 重試 8 次（約 2 秒）而不是以前的 10 秒。遠端服務要嘛在跑要嘛不在，不像本機剛
# 啟動的 container 需要等它 bind；留幾次重試只是為了容忍 wifi 偶發的封包遺失。
#
# 只驗證 TCP 連得上，不驗證認證。認證要到 RECORD 階段才會被拒，那時已經在推流
# process 裡面了 —— 所以認證失敗的症狀請看日誌 (logs)，不會在這裡攔到。
check_mediamtx_reachable() {
  if [[ -z "$MEDIAMTX_RTSP_HOST" ]]; then
    log_error "沒有設定 MEDIAMTX_RTSP_HOST，不知道要推去哪一台 MediaMTX"
    log_error "  設定檔:  $CONFIG_ENV_FILE"
    log_error "  或臨時帶:  MEDIAMTX_RTSP_HOST=<伺服器位址> $(basename "$SCRIPT_PATH")"
    exit 1
  fi

  local max_attempts=8      # 8 x 0.25s = 最多等 2 秒
  local attempt=0

  while [[ "$attempt" -lt "$max_attempts" ]]; do
    if timeout 1 bash -c "cat < /dev/null > /dev/tcp/${MEDIAMTX_RTSP_HOST}/${MEDIAMTX_RTSP_PORT}" 2>/dev/null; then
      log_info "遠端 MediaMTX 可連線（${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT}）"
      return 0
    fi
    attempt=$(( attempt + 1 ))
    sleep 0.25
  done

  log_error "連不到遠端 MediaMTX: ${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT}"
  log_error "  確認伺服器上的 mediamtx 還在跑，而且 rtspAddress 是這個 port"
  log_error "  確認網路可達:  ping ${MEDIAMTX_RTSP_HOST}"
  log_error "  位址設定在:  $CONFIG_ENV_FILE (MEDIAMTX_RTSP_HOST / MEDIAMTX_RTSP_PORT)"
  exit 1
}

# 列出可以看串流的網址
#
# 只印伺服器那台的位址。以前這裡是列本機每一張網卡的 IP —— 那是 MediaMTX 跑在本機
# 時才成立的做法；伺服器搬到遠端之後，印本機 IP 會給出一組連不上的網址，比不印
# 更糟。
print_viewing_urls() {
  log_info "   RTSP   rtsp://${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT}/${STREAM_PATH}"
  log_info "   WHEP   http://${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_WEBRTC_PORT}/${STREAM_PATH}"
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

  log_info "推流 ${DEVICE} 擷取 ${SRC_WIDTH}x${SRC_HEIGHT}@${FRAMERATE} → 輸出 ${OUT_WIDTH}x${OUT_HEIGHT} → ${RTSP_URL_DISPLAY}"
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

  check_mediamtx_reachable

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

  # 先確認伺服器連得上再開相機：推流端找不到伺服器會直接死掉，而那時相機已經被
  # 開過一次，restart 之後緊接著再開會撞到 Device or resource busy。
  check_mediamtx_reachable

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

# 停止推流
#
# 只停本機的推流 process。伺服器是別人的、可能還有其他機器在推，這裡不會、也不
# 應該去動它 —— 以前這支腳本會順手 docker stop 本機那個 container，那在遠端佈署
# 下等於「停我的攝影機順便把全隊的匯流伺服器關掉」。
cmd_stop() {
  stop_pipeline
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
  local live_pid
  live_pid="$(read_live_pipeline_pid)"

  echo "MediaMTX（遠端，不由這支腳本管理）"
  echo "  伺服器    : ${MEDIAMTX_RTSP_HOST:-<未設定>}"
  echo "  RTSP port : $MEDIAMTX_RTSP_PORT"
  if [[ -z "$MEDIAMTX_RTSP_HOST" ]]; then
    echo "  可連線    : -"
  elif timeout 1 bash -c "cat < /dev/null > /dev/tcp/${MEDIAMTX_RTSP_HOST}/${MEDIAMTX_RTSP_PORT}" 2>/dev/null; then
    echo "  可連線    : 是"
  else
    echo "  可連線    : 否"
  fi
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
  echo "  目標      : $RTSP_URL_DISPLAY"
  echo "  日誌      : $RUNTIME_LOG_FILE"
  echo "  設定來源  : $CONFIG_ENV_FILE"
  echo ""

  # 問伺服器的控制 API 這條 path 到底有沒有真的在收流。
  # 這是唯一能確認「串流真的活著」的方法 —— 本機的 process 活著不代表資料進得去。
  #
  # 查不到不代表串流有問題：mediamtx 的 apiAddress 通常只綁在伺服器自己的
  # loopback（本 repo 的 config/mediamtx.yml 就是這樣），從這台機器問過去本來就
  # 連不上。所以失敗只印一行提示，不當成錯誤，也不影響 exit code。
  if [[ -n "$MEDIAMTX_RTSP_HOST" ]]; then
    echo "MediaMTX 回報的 path 狀態"
    if command -v curl >/dev/null 2>&1; then
      curl -s --max-time 3 "http://${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_API_PORT}/v3/paths/list" 2>/dev/null \
        | sed 's/,/,\n/g' | grep -E '"name"|"ready"|"tracks"|"bytesReceived"' | sed 's/^/  /' \
        || echo "  (查詢失敗；控制 API 多半只開在伺服器本機，屬正常)"
    else
      echo "  (沒有 curl，跳過)"
    fi
    echo ""

    echo "觀看網址"
    echo "  RTSP   rtsp://${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_RTSP_PORT}/${STREAM_PATH}"
    echo "  WHEP   http://${MEDIAMTX_RTSP_HOST}:${MEDIAMTX_WEBRTC_PORT}/${STREAM_PATH}"
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
  start       背景啟動推流（MediaMTX 是遠端既有的服務，不由這裡啟動）
  stop        停止推流（不會動到遠端的 MediaMTX）
  restart     重啟
  status      顯示狀態、串流是否真的在收流、觀看網址
  logs        追推流日誌 (tail -f)
  foreground  前景執行，除錯用
  help        顯示這段說明

設定檔: $CONFIG_ENV_FILE
  裁切與尺寸    CROP_* / SRC_* / OUT_*
  推流目標      MEDIAMTX_RTSP_HOST / MEDIAMTX_RTSP_PORT / MEDIAMTX_STREAM_PATH
                （或直接帶完整的 RTSP_URL 蓋掉整條）
  推流認證      MEDIAMTX_PUBLISH_USER / MEDIAMTX_PUBLISH_PASS（伺服器要求才需要）
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
