# `lib/` — out-of-tree binary artefacts

Native libraries the backend loads at runtime that this workspace's build does
not produce and git does not carry. Same arrangement as `models/` (the kokoro
TTS weights) and the frontend's `node_modules/`: the container reaches them
through the workspace bind mount, and a fresh clone starts empty.

## `libsyncai_worker.so`

The Go/pion WebRTC session core behind `POST /api/v1/webrtc/whep`. The backend
dlopens it on the first WHEP request (`syncai_backend/gateways/webrtc/webrtc.py`,
overridable with `SYNCAI_WEBRTC_LIB`); until then it costs nothing, and a
missing file is reported as a 502 naming this path rather than a crash.

Built out of tree, from the `SyncAI-WebRTC-Worker` repo:

```bash
cd ../SyncAI-WebRTC-Worker
docker compose up build-lib          # Go 1.25 + GStreamer dev headers, writes ./dist/
cp dist/libsyncai_worker.so /path/to/SyncAI-Robot-Workspace/lib/
```

Two things to check when it does not load:

- **Architecture.** `file lib/libsyncai_worker.so` must say `ARM aarch64`; the
  container runs on the Jetson, not on a build laptop.
- **GStreamer runtime.** The library links `libgstreamer-1.0` through cgo, so
  the container needs the GStreamer runtime (it does — see the `Dockerfile`),
  and the Tegra elements (`nvjpegdec`, `nvvidconv`, `nvv4l2h264enc`) arrive
  from the nvidia container runtime.
- **GStreamer *version*.** cgo resolves those symbols at load time, against
  whichever `libgstreamer-1.0` this container has — 1.20, from Ubuntu 22.04.
  A library built on a newer distro loads as
  `undefined symbol: gst_debug_message_get_id` and nothing earlier says so,
  which is why the worker's builder image is based on the deployment distro
  rather than on a Go image. If the robot container's base image ever moves,
  the worker's `Dockerfile` moves with it.

The `.so` cannot be unloaded — Go c-shared libraries do not support `dlclose`.
Replacing this file therefore means restarting the backend process, not just
re-running the request.

**Replace it with `mv`, never with `cp`.** Once the backend has served a WHEP
or WHIP request the library is mapped into that process, and `cp` writes
through the same inode: the running process's file-backed pages change
underneath it and it wedges — alive, holding its port, answering nothing. A
rename swaps the directory entry and leaves the old inode alive for whoever
still has it open, so the running backend keeps working until it is restarted
on purpose:

```bash
cp ../SyncAI-WebRTC-Worker/dist/libsyncai_worker.so lib/.libsyncai_worker.so.new
mv lib/.libsyncai_worker.so.new lib/libsyncai_worker.so
```
