# Low-Latency Camera Streaming Proposal: A Self-Built WebRTC Worker

> Target: the camera path — `scripts/publish_camera_crop.sh` (host-side GStreamer)
>       + a new `src/syncai_webrtc/` (Go) + a camera panel in `src/syncai_frontend`
> Related: this note stands on its own; it shares no surface with the five agent /
>       MCP proposals in this directory
> Status: **proposal, not implemented**. §9 lists what is deliberately left out.

This note answers two questions: whether **"Go compiled to a `.so`, loaded into
Python for zero-copy frame hand-off"** is the right mechanism for the robot's
video path; and, if it is not, what the missing capability actually is and what
building it costs.

The conclusion up front: **the `.so` is not needed, because nothing in this
problem needs to touch a frame.** The capability that is genuinely missing is a
**WebRTC endpoint on the robot** — today the robot can only push RTSP to a
server that is currently unreachable, and the operator console cannot display
video at all. A WebRTC worker that relays **RTP packets** — never decoding,
never re-encoding, never mapping a frame into CPU memory — delivers it with no
FFI, no cgo, and no shared-memory scheme. The zero-copy problem does not arise.

---

## 0. Mental model: a `.so` is a calling convention, not an architecture

"Import Go into Python as a `.so`" answers the question *how do two runtimes call
each other in one process*. That is only worth answering once you have
established that they must be in one process, and that what crosses between them
is large enough for the crossing cost to matter.

Neither holds here. What crosses between the encoder and the network is an
already-compressed H.264 elementary stream at 4 Mbps. Moving it is not a
performance problem in any language; **producing and consuming it without
decoding it** is the whole trick, and that is a pipeline-topology decision, not
a calling-convention decision.

The corollary is the useful part: once you stop trying to hand frames across a
language boundary, the language question collapses into an ordinary one — which
runtime has the better WebRTC library and the better concurrency story for N
network peers. That question has an easy answer, and it does not require a `.so`
to act on it.

---

## 1. Current state, verified rather than assumed

Measured on the Jetson host (L4T R36.4.4 / JetPack 6.2, aarch64) and inside the
running `robot01` container.

### 1.1 The pipeline today

`scripts/publish_camera_crop.sh` runs on the **host**, not in the container, and
is NVMM end-to-end — no frame ever reaches CPU memory:

```
v4l2src(MJPEG 1920x1200@60, io-mode=2)
  ! nvjpegdec                                  -> video/x-raw(memory:NVMM)
  ! nvvidconv <crop rect>                      -> NV12 1280x720, still NVMM
  ! nvv4l2h264enc bitrate=4000000 profile=0
      control-rate=1 maxperf-enable=true
      iframeinterval=30 idrinterval=30
  ! h264parse config-interval=-1
  ! rtspclientsink location=rtsp://192.168.8.160:8554/dogA/camera protocols=tcp
```

Crop values are **per-machine** (fisheye black corner arcs) and live in
`scripts/publish_camera_crop.env`. `build_crop_arguments()` is ~70 lines of
carefully commented rectangle fitting; it is calibration, and no proposal should
disturb it.

### 1.2 The facts that decide the design

| Fact | Value | Consequence |
|---|---|---|
| GStreamer, host **and** container | **1.20.3** | fixes what elements exist |
| `rtph264pay` / `rtph264depay` / `udpsink` / `udpsrc` / `h264parse` | all **present** | an RTP relay needs **no** new GStreamer packages |
| `whipclientsink` / `whipsink` / `whepsrc` / `webrtcsink` | **MISSING** on both | see §4.1 |
| `webrtcbin` | present | a GStreamer-native option exists, see §4.2 |
| GStreamer Python bindings (`gi.repository.Gst`) | **not installed** | rules out a cheap Python/`webrtcbin` worker |
| `nvv4l2h264enc profile=0` | **Baseline** | the most WebRTC-compatible H.264 profile; the encoder needs no change |
| `nvvidconv` / `nvv4l2h264enc` / `nvjpegdec` **in the container** | **present** | nvidia-container-runtime injection works; the pipeline *could* move in |
| Go toolchain | **absent** from image and host | a new build stage is a real cost, §6 |
| FFI anywhere in the workspace | **none** — no ctypes, cffi, pybind11, dlopen, mmap, `/dev/shm` | a `.so` would be the first, with no precedent to follow |
| Remote MediaMTX `192.168.8.160:8554` | **`No route to host`** | there is no working video path at all right now |
| Frontend | **no camera component**, nothing speaks WebRTC/WHEP | the console cannot show video regardless |

Two of these are worth stating plainly because they are easy to assume away:
**there is no working video path on this robot today**, and **the console has
never been able to display one**.

---

## 2. What is actually missing

Not throughput, and not a faster way to move bytes between runtimes. The gap is
a **protocol endpoint**: the robot speaks RTSP, and RTSP is not a thing a browser
can play. Today the translation happens on a remote MediaMTX, which costs a
round trip off the robot before the operator sees anything — and that round trip
has to come out of a glass-to-glass budget of < 200 ms, before the browser's
jitter buffer has even started.

So the work is: terminate WebRTC **on the robot**, and give the console a
`<video>` element to attach it to.

---

## 3. Counter-argument: on a Tegra the zero-copy premise inverts

This is the part of the original idea that deserves the most direct answer,
because it is not merely unnecessary — it is **backwards**.

Today **zero frames touch CPU memory.** Capture is MJPEG, decode is `nvjpegdec`
straight into NVMM, and crop / scale / colour conversion / encode all stay in
NVMM. The Tegra hardware blocks hand buffers to each other without the CPU ever
mapping them.

"Extract a frame pointer in Go and hand it to Python" requires pulling a buffer
out of NVMM into host memory. That is a device-to-host download plus a
synchronisation point, **per frame** — two costs that do not exist today,
introduced in the name of eliminating a copy that also does not exist today.

Three further problems, each independently sufficient:

- **Go's GC-managed memory cannot be handed to Python as a durable pointer.**
  cgo's pointer-passing rules forbid retaining a Go pointer in C memory past the
  call. You would allocate with `C.malloc` or `mmap` — at which point you are
  outside Go's allocator and Go has contributed nothing that plain shared memory
  would not.
- **The Go runtime in a CPython process that also hosts rclpy is a signal-handling
  problem.** Go installs handlers for a large set (SIGSEGV/SIGBUS/SIGPROF/SIGURG…)
  at load time and chains to what is already installed. `syncai_backend` calls
  `rclpy.init()` with default signal options, so rcl owns SIGINT; uvicorn
  deliberately installs nothing because it runs off the main thread. Adding Go's
  preemption signal (SIGURG since Go 1.14) on top of DDS, ALSA and V4L2 libraries
  that are not uniformly EINTR-safe is a class of bug with no cheap diagnosis.
- **The blast radius is wrong.** `syncai_backend` is one process holding FastAPI,
  rclpy on the main thread, a Temporal worker, the TTS gateway, and every
  WebSocket. A fault in a loaded `.so` takes all of it down — including mode
  switching and task dispatch — for the sake of a video feature.

On a Tegra the *correct* zero-copy hand-off for inference is a DMABUF fd /
`NvBufSurface` into CUDA. That is a GStreamer/DeepStream/CUDA story. It is not a
Go story, and it is not a `.so` story.

---

## 4. The two rejected alternatives

### 4.1 Adding the WHIP elements (`gst-plugins-rs`)

The obvious "no new service" answer is to publish WHIP straight from GStreamer.
It does not work here: `whipclientsink` / `whipsink` / `whepsrc` / `webrtcsink`
live in **`gst-plugins-rs`**, a Rust project that Ubuntu 22.04 does not package —
`apt-cache policy gstreamer1.0-plugins-rs` and `apt-cache search gstreamer |
grep -iE 'rs|rust|webrtc'` both return **empty** in the container. Installing
them means a Rust toolchain plus `cargo-c` and an aarch64 source build in a new
Dockerfile stage, and `CLAUDE.md` is explicit that the slow `deps-builder`
stage's cache is to be protected.

### 4.2 `webrtcbin`, or deploying MediaMTX

`webrtcbin` **is** present, so a GStreamer-native worker is possible. Two things
argue against it: the Python bindings are not installed (so it is not the cheap
in-language option it looks like), and **every additional viewer needs its own
`webrtcbin` plus a dynamically added `tee` branch** — live pipeline surgery in
GStreamer 1.20. That is precisely the work a WebRTC library does for free.

Deploying **MediaMTX** was the other candidate and is a legitimate answer: it is
itself Go + pion, it ingests RTSP (so the pipeline would not change by a single
character), it serves WHEP, and this repo already had a working
`config/mediamtx.yml` — deleted in `8376bf4`, recoverable with
`git show 8376bf4^:config/mediamtx.yml`. It would have been the smallest possible
change.

**It is not proposed here, and the reason is not technical.** Building the worker
is a product-ownership choice: no third-party broker in the shipped product, a
signaling surface we control and can later put behind the robot's own auth, and
a first real Go service on the robot for the EdgeCore idea to grow into. That
trade should be made deliberately, which is why it is written down. If those
reasons stop mattering, MediaMTX is the cheaper path and the recovery command
above is all it takes.

---

## 5. The design: relay RTP, never a frame

```
host: scripts/publish_camera_crop.sh            (encoder settings UNCHANGED)
  … ! nvv4l2h264enc(Baseline, CBR 4 Mbps, IDR every 0.5 s) ! h264parse
    ! rtph264pay pt=96 config-interval=-1 mtu=1200
    ! udpsink host=127.0.0.1 port=5004 sync=false
                       │  RTP over loopback UDP
                       ▼
src/syncai_webrtc   (Go, pion/webrtc v4, one static binary)
  net.ListenUDP(:5004) ─▶ TrackLocalStaticRTP ─▶ PeerConnection ─▶ SRTP
  net/http:  POST /whep   (SDP offer → answer, 201 + Location)
             DELETE /whep/{id}
             GET  /health (last-RTP age, counters, viewer count)
                       │
                       ▼
src/syncai_frontend  dashboard camera panel (WHEP client, ~70 lines, no deps)
```

The worker's entire hot path is: read a UDP datagram into a ~1500-byte buffer,
write it to a track. No decode, no re-encode, no NVMM, no CPU frame buffer, no
cgo. `TrackLocalStaticRTP.Write([]byte)` takes a raw RTP packet and rewrites
SSRC / payload type per subscriber, which is exactly the relay semantics wanted.

Placement: `src/syncai_webrtc/`, a non-ament directory following the
`syncai_frontend` precedent. colcon skips directories with no manifest, so **no
`COLCON_IGNORE` is needed** — `syncai_frontend` does not carry one either.

Scope is **one viewer**. The session map is built anyway, so fanout later is
writing to N tracks rather than 1; no simulcast and no SFU machinery is built
now.

---

## 6. Five things that will bite, and one that might

Written here because each has a cheap fix and an expensive diagnosis.

1. **Drain RTCP from the sender.** A goroutine looping on `rtpSender.Read(buf)`
   and discarding is mandatory. Without it pion's internal RTCP buffer fills and
   the sender stalls. This is the single most common pion bug.
2. **CORS needs three things, and the third is the one people miss.** The console
   is served from `:3001`, the worker listens on `:8889` — different origins. You
   need `Access-Control-Allow-Origin`; an `OPTIONS` preflight handler (because
   `Content-Type: application/sdp` is not CORS-safelisted, so the browser always
   preflights); and **`Access-Control-Expose-Headers: Location`**, without which
   JavaScript cannot read the session URL at all. The symptom of missing the
   third is "video plays fine but teardown never works".
3. **`config-interval=-1` on the payloader is required, not optional.** It
   repeats SPS/PPS inline with every IDR. Without it a viewer joining mid-stream
   never receives a decodable stream — and the failure looks like a black
   `<video>` with a healthy `connected` state.
4. **Filter ICE interfaces.** `SettingEngine.SetInterfaceFilter` to drop
   `docker0`, `br-*`, `veth*`. This host runs several docker networks; unfiltered,
   ICE gathers a pile of unreachable candidates and setup crawls.
5. **`mtu=1200` on `rtph264pay`**, not GStreamer's 1400 default — leave room for
   SRTP overhead.

And the one that might: **`profile-level-id`**. `42e01f` is Baseline / level 3.1,
matching `profile=0` on the encoder, but 1280x720@**60** formally needs level 3.2
(`42e020`); 3.1 tops out at 720p30. Browsers generally treat the level as
advisory and decode it anyway, so start at `42e01f`; if a browser refuses or
stutters, raise it or set `FRAMERATE=30`, which is a perfectly reasonable teleop
feed and halves the bitrate pressure besides.

### 6.1 `/health` becomes the only honest liveness signal

`udpsink` is fire-and-forget — no connection, no back-pressure. It will transmit
into a void indefinitely, and **nothing on the GStreamer side can tell you
whether anyone is receiving**. The worker's last-RTP-received timestamp is
therefore the only truthful answer to "is the stream up", and both the script's
`status` subcommand and the console's offline state should read it.

This is strictly better than what exists today, where `status` can only report
whether a TCP port accepted a connection.

---

## 7. Limitations, stated up front

1. **No congestion control.** The encoder is fixed CBR and the worker relays
   blindly. Fine on a LAN; on degrading wifi it freezes rather than adapts.
   (MediaMTX would have had the same limitation.) The fix — read REMB/TWCC and
   drive the encoder's `bitrate` — needs item 3 below.
2. **No keyframe on demand.** A browser's PLI cannot reach the encoder, so a
   joining viewer waits for the next scheduled IDR: worst case 0.5 s at the
   current `IDR_INTERVAL`. Acceptable; lower it if it annoys.
3. **No control channel back to the pipeline.** Both fixes above want one. The
   clean long-term move is for the worker to spawn and own the GStreamer process,
   which would let it send `GstForceKeyUnit` and set `bitrate` live —
   deliberately not proposed now, because it means absorbing the script's
   per-machine fisheye calibration into the worker.
4. **The worker's HTTP is unauthenticated on the LAN.** Anyone who can reach
   :8889 can watch. This is parity with the previous arrangement (the old
   `config/mediamtx.yml` left `read` open to all IPs), but it becomes *our*
   decision rather than a vendor default, and should be revisited before the
   stack leaves a trusted network.
5. **Single viewer**, by decision. Fanout is an addition to the session map, not
   a redesign.

---

## 8. Landing order

Each step isolates the next; do not skip ahead, because the failure modes above
are only cheap to diagnose in this order.

1. Worker skeleton + `GET /health`, no WebRTC yet. Confirm `live: false`.
2. Retarget the script's sink to `rtph264pay ! udpsink`. Confirm the packet
   counter rises. A failure here is entirely GStreamer's, with WebRTC not yet
   involved.
3. WHEP `POST` / `DELETE`, driven by `curl` with a hand-made offer. This
   separates signaling bugs from browser bugs.
4. A minimal static HTML page on a LAN laptop. Confirm here that
   `RTCPeerConnection` works over a plain-`http` origin — receive-only peer
   connections are not gated on a secure context in Chromium/Firefox (unlike
   `getUserMedia`), but this deployment has never exercised it and it would block
   step 5.
5. The console camera panel (`NEXT_PUBLIC_WHEP_BASE`, `lib/video/whep.ts`,
   `components/dashboard/camera-panel.tsx`), kept **outside** TanStack Query like
   the other push streams.
6. Measure glass-to-glass against a millisecond clock and **record the number**.
   Only then tune, in the order `playoutDelayHint` → `IDR_INTERVAL` → encoder
   rate control. Do not pre-tune: the pipeline already runs `control-rate=1` and
   `maxperf-enable=true`, so the remaining budget is most likely transport and
   jitter buffer.

---

## 9. What this note does not cover

The originating discussion proposed four uses for a Go `.so`. This note covers
**only the first** (dual-track video). The other three were **not investigated**
and nothing here should be read as an assessment of them:

- **Multi-camera fusion / visual obstacle avoidance.** Note only that the udev
  rules already expose `camera0` and `camera1` and that exactly one streaming
  opener per V4L2 device is permitted — the existing camera is off in
  `bringup.launch.py` (`use_camera` defaults false) precisely because the
  host-side publisher owns it.
- **A BLE / EdgeCore bridge.** The most plausible of the three, and the worker
  proposed here is a reasonable seed for it — as a **process**, reached over
  localhost HTTP/WS. Note that the "saves the cost of IPC" argument is the
  weakest part of the original framing: these are control-plane messages at a few
  Hz, where a localhost round trip is microseconds against an AI decision loop
  measured in hundreds of milliseconds.
- **A Go Temporal worker.** Worth flagging one structural fact before anyone
  starts: the activities in `syncai_backend` *are* Python — the TTS gateway is
  kokoro-onnx in-process, and MOVE is an rclpy action client. A Go worker would
  have to call back into Python for every activity body. Temporal's polyglot
  story is **multiple workers on different task queues**, not one worker calling
  another language in-process. Separately, the durability properties being sought
  come from the Temporal *server*, not the SDK language.

**The AI branch of item 1 is also out of scope**, by decision. For the record,
the attachment point is a `tee` immediately after `nvvidconv` while buffers are
still `video/x-raw(memory:NVMM)`, with a second branch scaled down to inference
resolution and rate. Two things must be settled before that work, neither
related to Go: the container has **no GPU inference runtime at all** (plain
`ubuntu:22.04`, no CUDA / cuDNN / TensorRT / `nvcc` — the `nv*` elements exist
only because nvidia-container-runtime injects them), and it carries a hard
constraint that `numpy` stay **≤ 1.26** (the `scipy>=1.8,<1.11` and open3d pins
exist to hold it there) with `onnxruntime` pinned at `1.18.1`, because ≥ 1.19
heap-corrupts on an Orin with offlined cores.
