# 低延遲相機串流提案:自建 WebRTC Worker

> 本檔為 `webrtc-worker-proposal.md` 的中文版。正本是英文那份(本 repo 的文件慣例
>       是英文);兩份內容若有出入,以英文版為準。
> 目標:新增 `src/syncai_webrtc/`(Go),由它端到端擁有整條相機路徑,
>       並在 `src/syncai_frontend` 加上相機面板
> 關聯:本文自成一篇,與本目錄中其他五份 agent / MCP 提案沒有共用介面
> 狀態:**提案,尚未實作**。§9 列出刻意不涵蓋的部分。

本文回答兩個問題:**「把 Go 編成 `.so`、載入 Python 以達成影格零複製交接」**是不是
機器人影像路徑該用的機制;如果不是,真正缺的能力是什麼、自己蓋要付多少代價。

結論先講:**`.so` 不需要,因為這個問題裡沒有任何環節需要碰到影格。**真正缺的能力是
**機器人本機上的 WebRTC 端點** — 今天機器人上沒有任何東西以操作台能播放的形式提供影像,
操作台也沒有顯示影像的管道。一個 Go 服務,以子行程的方式擁有 capture → crop → encode
這條 pipeline,並把它產生的 **RTP 封包**轉送給瀏覽器 — 全程不解碼、不重新編碼、不把影格
映射進 CPU 記憶體 — 就能補上這個能力,而且不需要 FFI、不需要 cgo、不需要任何共享記憶體
機制。零複製這個問題根本不會出現。

---

## 0. 心智模型:`.so` 是一種呼叫慣例,不是一種架構

「把 Go 以 `.so` 匯入 Python」回答的是*兩個 runtime 如何在同一個行程裡互相呼叫*。這個問題
只有在你先確立了「它們必須在同一個行程裡」、而且「跨越邊界的資料大到讓跨越成本重要」之後,
才值得回答。

這裡兩個前提都不成立。在編碼器與網路之間流動的,是一條 4 Mbps、**已經壓縮好的** H.264
elementary stream。搬動它在任何語言裡都不構成效能問題;**在不解碼的前提下生產並消費它**
才是全部的訣竅,而那是 pipeline 拓樸的決定,不是呼叫慣例的決定。

推論才是有用的部分:一旦你不再試圖把影格跨語言邊界遞送,語言選擇就退化成一個普通問題 —
哪個 runtime 有比較好的 WebRTC 函式庫、以及面對 N 個網路 peer 時比較好的並行模型。這個問題
有顯而易見的答案,而且不需要 `.so` 才能執行。

---

## 1. 現況:實測,而非假設

在 Jetson host(L4T R36.4.4 / JetPack 6.2, aarch64)以及執行中的 `robot01` 容器內量測。

### 1.1 硬體支援的 pipeline

相機是 TechNexion VCS-AR0234-C,位於穩定的 udev symlink `/dev/syncai/camera0` 之後。
下面這條鏈全程走 NVMM — **沒有任何影格進到 CPU 記憶體** — 而且是**在 `robot01` 容器內**
實際跑完的,不只是在 host 上:

```
v4l2src device=/dev/syncai/camera0 io-mode=2   (MJPEG 1920x1200@60)
  ! nvjpegdec                                  -> video/x-raw(memory:NVMM)
  ! nvvidconv <crop rect>                      -> NV12 1280x720, 仍在 NVMM
  ! nvv4l2h264enc bitrate=4000000 profile=0
      control-rate=1 maxperf-enable=true
      iframeinterval=30 idrinterval=30
  ! h264parse config-interval=-1
  ! rtph264pay pt=96 config-interval=-1 mtu=1200
  ! <sink>
```

裁切矩形是**校正值,不是設定值**。魚眼鏡頭的成像圓無法覆蓋矩形感光元件,所以四個角落都被
黑弧遮住,而且每一台實機遮到的位置都不一樣(138 號機是左 175 px / 右 93 px)。把黑邊裁掉
之後剩下的矩形並不是 16:9,因此在縮放到 1280x720 之前,必須先在其中擬合出最大的 16:9 區域 —
那大約是七十行算術,存在的目的是讓輸出的長寬比誠實。當 worker 接手這條 pipeline 之後,量到的
數字與這段算術都得有地方安放;§5.2 說明各自放在哪裡、以及為什麼兩者要分開放。

### 1.2 決定設計方向的事實

| 事實 | 值 | 後果 |
|---|---|---|
| GStreamer,host **與**容器 | **1.20.3** | 決定了有哪些 element 可用 |
| `rtph264pay` / `rtph264depay` / `udpsink` / `udpsrc` / `h264parse` | 全部**存在** | RTP 轉送**不需要**新增任何 GStreamer 套件 |
| `whipclientsink` / `whipsink` / `whepsrc` / `webrtcsink` | 兩邊都**缺** | 見 §4.1 |
| `webrtcbin` | 存在 | GStreamer 原生方案是可行的,見 §4.2 |
| GStreamer Python 綁定(`gi.repository.Gst`) | **未安裝** | 排除了便宜的 Python/`webrtcbin` worker |
| §1.1 完整鏈路,**在容器內**執行 | **已驗證**,exit 0 | 整條相機路徑可以住在 `robot01` 裡;worker 不必是 host 端的東西 |
| `nvbufsurftransform: Could not get EGL display connection` | 容器內會印,host **不會**;編碼結果仍正確 | 對這條 pipeline 無害,對其他 element 卻是關鍵 — §6.2 |
| 容器內的 `/dev/syncai/camera0` | 存在,解析到 `/dev/video0`;容器使用者在 `video`(44)群組內 | `v4l2src device=/dev/syncai/camera0` 可以用穩定名稱開啟它 |
| 容器內的 `gst-launch-1.0` | `/usr/bin/gst-launch-1.0` | pipeline 可以做成受監管的子行程 — 不需要 cgo,不需要綁定 |
| `nvv4l2h264enc profile=0` | **Baseline** | 對 WebRTC 相容性最好的 H.264 profile;編碼器不必改 |
| Go 工具鏈 | image 與 host 皆**沒有** | 新增一個建置步驟是實際成本,§5.3 |
| workspace 內任何 FFI | **沒有** — 沒有 ctypes、cffi、pybind11、dlopen、mmap、`/dev/shm` | `.so` 會是第一個,沒有前例可循 |
| 操作台能播放的影像路徑 | **沒有** | 沒有東西需要保留,也沒有東西需要遷移 |
| 前端 | **沒有相機元件**,沒有任何東西會講 WebRTC/WHEP | 無論如何操作台都顯示不了影像 |

其中兩點值得明講,因為它們最容易被想當然爾地略過:**這台機器人今天沒有可用的影像路徑**,
而且**操作台從來就沒辦法顯示影像**。這是一條全新路徑,不是遷移 — 也正因如此,下面的設計
才有餘裕把擷取與傳遞放在同一個地方。

---

## 2. 真正缺的是什麼

不是吞吐量,也不是在 runtime 之間搬位元組的更快方法。缺口是一個**協定端點**:瀏覽器不會播
RTSP,而機器人從來沒講過任何瀏覽器聽得懂的東西。把翻譯工作放到機外 — 推流給網路上別處的
broker、再讓操作台從那裡拉 — 代價是操作者看到畫面前多一趟往返,而這趟往返要從 < 200 ms 的
glass-to-glass 預算裡扣,還沒算上瀏覽器的抖動緩衝。

所以要做的事是:**在機器人本機終結 WebRTC**,並給操作台一個 `<video>` element 去接。

---

## 3. 反論:在 Tegra 上,零複製的前提是反過來的

這是原始構想中最該被正面回答的部分,因為它不只是多餘 — 它是**反過來的**。

在 §1.1 的 pipeline 裡**零個影格碰到 CPU 記憶體**。擷取是 MJPEG,解碼由 `nvjpegdec` 直接
進 NVMM,裁切 / 縮放 / 色彩轉換 / 編碼全部留在 NVMM。Tegra 的硬體區塊彼此交接緩衝區,CPU
從頭到尾不需要映射。

「在 Go 裡取出影格指標再交給 Python」則要求把緩衝區從 NVMM 拉回主記憶體。那是一次
device-to-host 下載加上一個同步點,**每一格都要付** — 為了消除一個本來就不存在的複製,引入
兩個本來不存在的成本。

另外三個問題,每一個單獨都足以否決:

- **Go 的 GC 管理記憶體無法以持久指標交給 Python。** cgo 的指標傳遞規則禁止在呼叫結束後
  於 C 記憶體中保留 Go 指標。你會改用 `C.malloc` 或 `mmap` — 到那一步你已經在 Go 的配置器
  之外,Go 相對於單純的共享記憶體並沒有多貢獻什麼。
- **在一個同時跑 rclpy 的 CPython 行程裡塞進 Go runtime,是訊號處理問題。** Go 在載入時會
  為一大堆訊號安裝 handler(SIGSEGV/SIGBUS/SIGPROF/SIGURG…)並串接既有的。`syncai_backend`
  以預設 signal 選項呼叫 `rclpy.init()`,所以 SIGINT 屬於 rcl;uvicorn 則刻意不安裝任何
  handler,因為它跑在非主執行緒。在一堆對 EINTR 不一致安全的 DDS、ALSA、V4L2 函式庫之上,
  再疊一個 Go 的搶佔訊號(Go 1.14 起的 SIGURG),是一類診斷起來完全不便宜的 bug。
- **波及範圍不對。** `syncai_backend` 是單一行程,裡面有 FastAPI、主執行緒上的 rclpy、
  Temporal worker、TTS gateway,以及所有 WebSocket。載入的 `.so` 出錯會把整包帶走 —
  包含模式切換與任務派送 — 只為了一個影像功能。

在 Tegra 上,推論用的*正確*零複製交接方式是 DMABUF fd / `NvBufSurface` 進 CUDA。那是
GStreamer/DeepStream/CUDA 的故事,不是 Go 的故事,更不是 `.so` 的故事。

---

## 4. 兩個被否決的替代方案

### 4.1 加裝 WHIP element(`gst-plugins-rs`)

最直覺的「不新增服務」解法,是直接從 GStreamer 推 WHIP,這樣整條路徑就是一條 pipeline。
在這裡行不通:`whipclientsink` / `whipsink` / `whepsrc` / `webrtcsink` 都住在
**`gst-plugins-rs`**,那是一個 Ubuntu 22.04 沒有打包的 Rust 專案 —
在容器裡 `apt-cache policy gstreamer1.0-plugins-rs` 與
`apt-cache search gstreamer | grep -iE 'rs|rust|webrtc'` 都回**空**。要裝它意味著在新的
Dockerfile stage 裡放 Rust 工具鏈加 `cargo-c`,再做一次 aarch64 原始碼建置,而 `CLAUDE.md`
明講了那個很慢的 `deps-builder` stage 的快取是要保護的。

### 4.2 `webrtcbin`,或部署 MediaMTX

`webrtcbin` **確實存在**,所以 GStreamer 原生的 worker 是做得出來的。兩件事讓它不划算:
Python 綁定沒裝(所以它並不是看起來那個便宜的同語言選項),以及**每多一個觀看端就需要自己
一顆 `webrtcbin` 加一條動態掛上去的 `tee` 分支** — 在 GStreamer 1.20 上做即時 pipeline 手術。
而那正是 WebRTC 函式庫免費幫你做掉的工作。

在機器人上**部署 MediaMTX** 是另一個候選,而且是個站得住腳的答案:它本身就是 Go + pion,
可以吃 RTSP、可以送 WHEP,而且這個 repo 曾經有一份可用的 `config/mediamtx.yml` — 在
`8376bf4` 被刪掉,用 `git show 8376bf4^:config/mediamtx.yml` 就能救回。它會是要寫的程式碼
最少的路。但要注意它*沒有*消除什麼:擷取 pipeline 仍然需要一個擁有者、一套生命週期,以及
每台機器各自的裁切值,所以 MediaMTX 只買到傳遞那一半,另一半原封不動留在原地。

**本文不提案它,而剩下的理由不是技術性的。** 自己蓋這個 worker 是產品所有權的選擇:出貨的
產品裡不放第三方 broker、信令介面由我們掌握且日後可以擺到機器人自己的驗證之後、以及在機器人
上有第一個真正的 Go 服務可以讓 EdgeCore 的構想長進去。這個取捨應該是被刻意做出的,所以把它
寫下來。如果哪天這些理由不再重要,MediaMTX 加上一個受監管的 pipeline 就是更便宜的路,而上面
那行救援指令就是大半的工作量。

---

## 5. 設計:由一個服務擁有擷取、編碼與傳遞

### 5.1 拓樸

```
src/syncai_webrtc   (Go, pion/webrtc v4, 單一靜態執行檔, CGO_ENABLED=0)
  │
  ├─ 監管一個子行程:gst-launch-1.0
  │     v4l2src device=/dev/syncai/camera0 io-mode=2   (MJPEG 1920x1200@60)
  │       ! nvjpegdec ! nvvidconv <來自 [sensor.camera] 的 crop rect>
  │       ! nvv4l2h264enc bitrate=4000000 profile=0 control-rate=1
  │           maxperf-enable=true iframeinterval=30 idrinterval=30
  │       ! h264parse config-interval=-1
  │       ! rtph264pay pt=96 config-interval=-1 mtu=1200
  │       ! udpsink host=127.0.0.1 port=5004 sync=false
  │                      │  RTP 走 loopback UDP
  │                      ▼
  ├─ net.ListenUDP(:5004) ─▶ TrackLocalStaticRTP ─▶ PeerConnection ─▶ SRTP
  │
  └─ net/http:  POST   /whep        (SDP offer → answer, 201 + Location)
                DELETE /whep/{id}
                GET    /health      (子行程狀態、最後 RTP 距今時間、計數器、觀看人數)
                       │
                       ▼
src/syncai_frontend  儀表板相機面板(WHEP client,約 70 行,無相依)
```

worker 的整條熱路徑就是:把一個 UDP datagram 讀進約 1500 位元組的緩衝區,寫進 track。
不解碼、不重新編碼、沒有 NVMM、沒有 CPU 影格緩衝、沒有 cgo。
`TrackLocalStaticRTP.Write([]byte)` 接受原始 RTP 封包,並針對每個訂閱者改寫 SSRC 與
payload type — 正是這裡想要的轉送語意。

**為什麼用子行程加 loopback socket,而不是內嵌 pipeline。** 另一個做法是用 `go-gst` 搭配
`appsink`,好處是全部在同一個位址空間 — 代價則是引入 **cgo**、GStreamer 開發標頭檔,以及
一個不再能是靜態執行檔的建置。`gst-launch-1.0` 子行程的成本是一次 `fork` 加上一趟以微秒計的
loopback,而換到兩件比那更值錢的事:pipeline 維持成一段字串,工程師可以整段貼進終端機,
在完全不動用 worker 的情況下重現故障;而 worker 保有純 Go 的建置。代價是真實存在的,並且在
§7.3 點名:`gst-launch` 子行程在執行中無法被重新設定。

範圍是**單一觀看端**。session map 反正要建,所以日後扇出只是從寫 1 條 track 變成寫 N 條;
現在不做 simulcast,也不做 SFU 那一套。

### 5.2 每台機器的校正值放哪裡

量到的黑弧寬度是每台機器各自的,所以它該跟機器人其他的每機身分放在一起:在
`config/instances/robotNN.ini` 新增一個 **`[sensor.camera]`** 區段,完全比照 `[sensor.lidar]`
的前例(`device`、`crop_left`、`crop_right`、`crop_top`、`crop_bottom`、`out_width`、
`out_height`、`bitrate`、`framerate`)。

這推翻了目前「相機不寫在 instance INI 裡」的規則,而推翻正是重點:那條規則存在,只是因為
相機到目前為止都是從 host 端驅動的,在那個把 INI 掛成容器內 `config/system.ini` 的
bind mount 之外。一個跑在容器裡、以 workspace 根目錄為 cwd 的 worker,讀它的方式跟每個
launch file 一模一樣;這麼一來機器人的裁切值就跟著機器人的身分走,而不是跟著某台機器的
dotfile 走。

16:9 擬合仍然留在**程式碼**裡,不進設定檔。操作者應該填他量到的東西 — 左右兩側黑弧的寬度 —
而永遠不是一個擬合後的矩形;推導出來的數字是程式的工作,而一個躺在設定檔裡的擬合矩形,是
沒有人有辦法拿影像去核對的數字。

### 5.3 它跑在哪、由誰啟動、怎麼建置

位置:`src/syncai_webrtc/`,比照 `syncai_frontend` 的前例,是一個非 ament 目錄。colcon 會
略過沒有 manifest 的目錄,所以**不需要 `COLCON_IGNORE`** — `syncai_frontend` 也沒有帶一個。

它**不是 ROS node**:沒有 namespace、沒有 TF、沒有 parameter。它的每機身分來自 INI;連接埠
(8889)寫死,而這是安全的,因為容器是 `network_mode: host`,一台機器人就是一台主機。

啟動方式是在**兩份 session spec 裡各加一個 `camera` window**
(`config/sessions/start_nav.yaml` 與 `start_mapping.yaml`)。建圖模式最需要它 — 手動遙控跑
一趟建圖的操作者,正是最需要看到機器人往哪裡走的人。它不依賴 ROS graph 裡的任何東西,所以
不需要 `sleep` 偏移;它唯一的順序限制不是競態而是互鎖,§6 第 6 點有寫:
`bringup.launch.py` 的 `use_camera` 必須維持 `false`,因為 V4L2 裝置只容許一個串流開啟者,
而現在那個開啟者是這個 worker。

建置:把 Go 工具鏈放進 Dockerfile 的 **`dev`** stage,然後手動把執行檔建到
`src/syncai_webrtc/bin/`(gitignore)— 跟前端的 `npm install` 是同一套安排,理由也相同。
workspace 是 bind mount 的,`scripts/build.sh` 只建 ROS 套件,而在一台線上機器人上迭代不能
變成每次都要重建 image。工具鏈要離 `deps-builder` 遠一點,那一層的快取是貴的。釋出版建置
則改成在拋棄式 stage 裡編譯、再把靜態執行檔複製進去 — 那正是 `CGO_ENABLED=0` 第二次付清的
地方。

---

## 6. 六個會咬人的地方,加一個可能會

寫在這裡是因為每一項都有便宜的修法,以及昂貴的診斷。

1. **要把 sender 的 RTCP 排掉。** 一個在 `rtpSender.Read(buf)` 上迴圈並丟棄的 goroutine 是
   必要的。沒有它,pion 內部的 RTCP 緩衝區會塞滿,sender 會卡住。這是 pion 最常見的 bug。
2. **CORS 需要三樣東西,而第三樣是大家會漏的。** 操作台從 `:3001` 提供,worker 監聽
   `:8889` — 不同來源。你需要 `Access-Control-Allow-Origin`;需要一個 `OPTIONS` 預檢
   handler(因為 `Content-Type: application/sdp` 不在 CORS 安全清單內,瀏覽器一定會預檢);
   以及 **`Access-Control-Expose-Headers: Location`**,少了它 JavaScript 根本讀不到 session
   URL。漏掉第三樣的症狀是「影像播得好好的,但關不掉」。
3. **payloader 上的 `config-interval=-1` 是必要而非可選。** 它讓 SPS/PPS 隨每個 IDR 內嵌重送。
   沒有它,中途加入的觀看端永遠拿不到可解碼的串流 — 而故障長得像一個黑畫面的 `<video>`
   配上一個健康的 `connected` 狀態。
4. **過濾 ICE 介面。** 用 `SettingEngine.SetInterfaceFilter` 濾掉 `docker0`、`br-*`、`veth*`。
   這台主機上有好幾個 docker 網路;不過濾的話 ICE 會蒐集到一堆不可達的候選位址,連線建立
   會慢到爬。
5. **`rtph264pay` 用 `mtu=1200`**,而不是 GStreamer 預設的 1400 — 留空間給 SRTP 標頭。
6. **好好擁有子行程,而且是雙向的。** 它退出時要帶退避地重啟(USB 重新列舉就會造成,而
   compose 裡的 `device_cgroup_rules` 存在的目的正是讓新的 minor 仍然可達);把它的
   stdout/stderr 導進 pane,好讓 multilog 像其他 window 一樣收錄;關機時要殺掉它,因為一個
   漏掉的 `gst-launch` 會抓住 V4L2 裝置,讓*下一次*啟動在 `S_FMT` 失敗並回報
   「Device or resource busy」— 而那時 pane 早就捲過去了。反方向是同一個互鎖:
   `bringup.launch.py` 的 `use_camera` 維持 `false`。

至於那個可能會咬人的:**`profile-level-id`**。`42e01f` 是 Baseline / level 3.1,對應編碼器上的
`profile=0`,但 1280x720@**60** 形式上需要 level 3.2(`42e020`);3.1 的上限是 720p30。瀏覽器
通常把 level 當作參考值,照樣解碼,所以先用 `42e01f` 起步;如果哪個瀏覽器拒絕或卡頓,就把它
調高,或把 `framerate` 設成 30 — 對遙控畫面而言完全合理,而且順便把位元率壓力砍半。

### 6.1 `/health` 是唯一誠實的存活訊號

`udpsink` 是射後不理 — 沒有連線、沒有反壓。它會無止盡地往虛空發送,而且
**GStreamer 這一側沒有任何東西能告訴你有沒有人在收**。子行程的存活也不等於串流的存活:
一個相機已經停止供圖的 `gst-launch`,仍然是一個在跑的行程。因此 worker 的「最後收到 RTP 的
時間戳」是「串流還在嗎」唯一誠實的答案,操作台的離線狀態應該讀它,而不是從行程表去推論。

### 6.2 容器裡缺少的 EGL display

在 `robot01` 內,這條 pipeline 會印出 `nvbufsurftransform: Could not get EGL display
connection`,然後照樣正確編碼 — 端到端驗證過,包含 `nvjpegdec`、會裁切縮放的 `nvvidconv`、
Baseline 的 `nvv4l2h264enc`、`h264parse` 與 `rtph264pay`。host 上不會印這行。把它當成一條
邊界、而不是一則該被消音的警告:需要*真正* EGL display 的 element — `nveglglessink`,以及
會退回 EGL 轉換路徑的那些 `nvvidconv` 用法 — 在容器裡不會動,而 `gst-inspect-1.0` 的完整
列舉也因為同樣的理由在容器裡中止。日後要加進這條 pipeline 的任何東西,都必須在容器內重新
驗證,不能只在 host 上驗。

---

## 7. 先講清楚的限制

1. **沒有壅塞控制。** 編碼器是固定 CBR,worker 盲目轉送。在區域網路內沒問題;在劣化的 wifi
   上它會凍住而不是自適應。(用 MediaMTX 也會有同樣的限制。)修法 — 讀 REMB/TWCC 並去驅動
   編碼器的 `bitrate` — 需要下面第 3 點。
2. **不支援隨選關鍵影格。** 瀏覽器的 PLI 傳不到編碼器,所以剛加入的觀看端得等下一個排定的
   IDR:以提案的 `idrinterval` 計,最差 0.5 秒。可以接受;嫌煩就把它調低。
3. **無法在執行期控制 pipeline。** worker 擁有子行程的*生命週期*,但透過 `gst-launch-1.0`
   驅動它,而後者啟動之後沒有任何辦法設定屬性或送出 `GstForceKeyUnit` 事件。所以上面兩個
   修法在子行程改成內嵌 pipeline(`go-gst`,也就是 §5.1 花錢買開的那個 cgo)或加上一個小型
   控制墊片之前都構不到。重啟子行程是粗暴版,代價是每個觀看端都要重連。這是子行程設計刻意
   付出的價格,也是一旦自適應位元率變成需求時第一個要重新檢視的地方。
4. **worker 的 HTTP 在區域網路上沒有驗證。** 任何連得到 :8889 的人都能看。這變成*我們的*
   決定而不是某個廠商的預設值,而且在這套系統離開可信網路之前應該重新檢視。
5. **單一觀看端**,刻意如此。扇出是往 session map 上加東西,不是重新設計。
6. **這東西在跑的時候,相機對 ROS 不可用**,依據的正是那條讓設計得以簡單的「單一開啟者」
   規則。任何想在 ROS 裡拿到影格的需求 — 錄影、偵測器 — 都不是改設定,而是 §9 的那場討論。

---

## 8. 落地順序

每一步都把下一步隔離開來;不要跳步,因為上面那些故障模式只有照這個順序才便宜診斷。

1. worker 骨架 + `GET /health`,沒有相機也沒有 WebRTC。確認 `live: false`。
2. 子行程監管:讀 `[sensor.camera]`、組出 pipeline 字串、把它接到 `udpsink` 起起來,確認
   worker 的封包計數器在上升、並且殺掉子行程之後它會重啟。用單張影格核對裁切是否符合量到的
   數字 — 這是唯一一個「矩形錯了會一眼看出來」的步驟。這一步的失敗全部是 GStreamer 的,
   WebRTC 還沒進場。
3. WHEP 的 `POST` / `DELETE`,用 `curl` 加手工 offer 驅動。這能把信令的 bug 跟瀏覽器的 bug
   分開。
4. 在區網筆電上開一個最小的靜態 HTML 頁面。在這裡確認 `RTCPeerConnection` 在純 `http`
   來源下可用 — 純接收的 peer connection 在 Chromium/Firefox 並不受 secure context 限制
   (不像 `getUserMedia`),但這套部署從來沒實際驗過,而它會擋住第 5 步。
5. 操作台相機面板(`NEXT_PUBLIC_WHEP_BASE`、`lib/video/whep.ts`、
   `components/dashboard/camera-panel.tsx`),跟其他推送串流一樣**放在** TanStack Query
   **之外**。
6. 用毫秒時鐘量 glass-to-glass 並**把數字記下來**。然後才調校,順序是 `playoutDelayHint` →
   `idrinterval` → 編碼器速率控制。不要預先調:pipeline 已經在跑 `control-rate=1` 與
   `maxperf-enable=true`,所以剩下的預算多半在傳輸與抖動緩衝上。

---

## 9. 本文不涵蓋的部分

最初的討論為 Go `.so` 提出了四種用途。本文**只涵蓋第一種**(雙軌影像)。另外三種**未經調查**,
這裡的任何內容都不應被讀成對它們的評估:

- **多相機融合 / 視覺避障。** 只需註記:udev 規則已經開出了 `camera0` 與 `camera1`,每個
  V4L2 裝置只容許一個串流開啟者,而在本提案下那個開啟者是 worker — 這也是
  `bringup.launch.py` 的 `use_camera` 維持 `false` 的原因。
- **BLE / EdgeCore 橋接。** 三者中最站得住腳的一個,而這裡提案的 worker 是合理的種子 —
  以**行程**的形式,經 localhost HTTP/WS 存取。要註記的是,「省下 IPC 成本」是原始說法裡最弱
  的一環:這些是每秒幾次的控制平面訊息,localhost 一趟往返以微秒計,而 AI 決策迴圈是以數百
  毫秒計的。
- **Go 版 Temporal worker。** 動工之前值得先點出一個結構性事實:`syncai_backend` 裡的
  activity *就是* Python — TTS gateway 是行程內的 kokoro-onnx,MOVE 是 rclpy 的 action
  client。Go worker 會變成每個 activity 主體都得回呼 Python。Temporal 的多語言故事是
  **多個 worker 跑在不同 task queue 上**,不是一個 worker 在行程內呼叫另一個語言。另外,
  想要的那些持久性特質來自 Temporal *伺服器*,不是 SDK 的語言。

**第一項裡的 AI 分支同樣不在範圍內**,刻意如此。但為了留下記錄:接入點是 worker 子行程
pipeline 中緊接在 `nvvidconv` 之後的一個 `tee`,此時緩衝區仍是
`video/x-raw(memory:NVMM)`,第二條分支縮到推論所需的解析度與幀率。在那項工作之前有三件事
必須先解決,沒有一件跟 Go 有關:容器裡**完全沒有 GPU 推論 runtime**(純 `ubuntu:22.04`,
沒有 CUDA / cuDNN / TensorRT / `nvcc` — 那些 `nv*` element 之所以存在,純粹是
nvidia-container-runtime 注入的);它帶著一個硬性限制,`numpy` 必須維持在 **≤ 1.26**
(`scipy>=1.8,<1.11` 與 open3d 的 pin 存在就是為了壓住它),且 `onnxruntime` 釘在 `1.18.1`,
因為 ≥ 1.19 在關掉部分核心的 Orin 上會破壞 heap;以及,那第二條分支的消費者不是
`gst-launch`,所以那裡正是 §5.1 的子行程選擇必須被重新辯論的地方。
