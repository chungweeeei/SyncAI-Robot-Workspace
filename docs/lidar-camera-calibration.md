# Camera and lidar-camera calibration

Two calibrations put the camera image and the lidar cloud into one geometry:

| | What | Lives in | Read by |
|---|---|---|---|
| **Intrinsics** | K + distortion of `camera0` on the raw 1280×720 stream | `config/calibration/<robot_id>/camera0.yaml` | `syncai_camera_feeder`, which publishes it once as latched `camera_info` (its launch sets `camera_info_file` to that path) |
| **Extrinsic** | pose of `camera_optical_frame` in `lidar_top` | `camera_optical_joint` in `src/syncai_bringup/description/G23.urdf` | TF, through robot_state_publisher |

Do the intrinsics first and **redo the extrinsic after every intrinsics change**: the two are
coupled. The principal point is the weakly determined part of the intrinsics (robot01's two
equally good fits put `cx` 15 px apart), and 15 px of `cx` reads as 1.2° of pan to the extrinsic.

Tools: `scripts/calibration/`, run with `python3` from the workspace root. `collect_board.py` and
`snapshot.py` need rclpy (the robot container); the rest need only numpy, OpenCV and PyYAML and
run on the Jetson host as well. Bags are preprocessed by `direct_visual_lidar_calibration`
(`src/third-party/`, "dvlc" below). The Claude Code skill `lidar-camera-calibration`
(`.claude/skills/`) walks this procedure.

Work in `record/calib_<robot_id>_<yyyymmdd>/` — `record/` is gitignored.

Everything below was worked out on robot01 on 2026-10-19/20; "Why it is done this way" has the
evidence for each choice and the [results](#robot01-2026-10-20) are at the end.

## Before you start

- **The feeder must be on the raw passthrough** (`stabilize: false` in its installed params). A
  calibration of the stabilized output is wrong for everything else: the output is zoomed 1.3× and
  warped per frame. Check the last startup's pipeline — `v4l2src … ! tee …` with no decode / warp
  stage:

  ```bash
  grep -a "pipeline:" log/stack/<robot_id>/camera_feeder/current | tail -1        # AUTO
  grep -a "pipeline:" log/stack/<robot_id>/calib/camera_feeder/current | tail -1  # CALIBRATION
  ```

- **The board.** robot01's is an A4 print of 8×11 squares — **7×10 inner corners**, which is what
  the tools' `--pattern` counts — with 21 mm squares. Measure a printed square: its size only
  scales the board distances, not K or the distortion, but `--square` should still be right. Keep
  it flat; a sheet held by one edge bends (see lesson 6).
- **Mode.** The intrinsics need only the camera, so AUTO (the nav session), MANUAL (the mapping
  session) or CALIBRATION, all of which start the feeder. The extrinsic needs CALIBRATION: it is the only mode whose lidar
  publishes `PointCloud2`, which dvlc reads.

## Part 1 — intrinsics

### 1. Collect board views

```bash
docker compose exec robot01 bash               # cwd is the workspace root
source install/setup.bash
python3 scripts/calibration/collect_board.py record/calib_robot01_20261020/intrinsics/board01
```

and, in a second terminal, the status it rewrites every second:

```bash
docker compose exec robot01 watch -n1 cat record/calib_robot01_20261020/intrinsics/board01/status.txt
```

It keeps a view when the board has been still (corners moving under 25 px/s) and the view differs
from every view kept so far in position, size, skew or tilt. Each kept view is its JPEG as it
arrived plus the sub-pixel corners. Ctrl-C stops it; running it again on the same directory adds
to what is there.

How to move the board — the first two points are what failed twice before they were followed:

- **Kneel in front of the robot and stay at 0.3–0.8 m.** The camera sits low, and a board held by
  someone standing is a metre or more away; its squares then come out under 15 px and the fit drops
  the view (status: "too far away"). Far views are worse than useless (lesson 2).
- **Tilt 30–45° both ways about both axes:** tip the top edge towards and away from the camera,
  and turn the board like a door, left edge towards and away. A square-on board cannot tell a
  longer focal length from a nearer board; tilted views are what pin the focal length down.
- **Cover the whole image, corners included.** Keep all 70 corners in frame (no detection
  otherwise) but push the board into every corner and along every edge: that is where the
  distortion is measured. Image right is the board holder's left.
- **Hold each pose for about a second, then move somewhere new.** A count that stops rising while
  the board is visible means the poses repeat.

The status file's check lines are guidance for the person collecting, not a gate:

| Line | Target | robot01, 2026-10-20 |
|---|---|---|
| usable views (squares ≥ 15 px) | 60+ | 63 of 100 kept |
| every cell of the 4×4 grid covered | all 16 | all 16 (top-left thin: 24 corners) |
| tilted ≥ 0.15 each way, both axes | 3+ each | tilt_x +4 / −6, tilt_y +2 / −3 |
| close views (size ≥ 0.3) | 3+ | 2 |

`tilt_x` / `tilt_y` are log ratios of the board's opposite edges (`calib_common.board_tilt`):
`tilt_x` moves when the board is tipped about an axis along its rows of 7 corners, `tilt_y` about
its columns. Which physical motion that is depends on how the board is held; a few views tell.

### 2. Fit

```bash
python3 scripts/calibration/fit_intrinsics.py record/calib_robot01_20261020/intrinsics/board01 \
    --out record/calib_robot01_20261020/intrinsics/camera0.yaml --split-half
```

It drops views with squares under `--min-square` (15 px), does **no** outlier rejection, fits the
**equidistant** model (`--model plumb_bob` exists for comparison; it takes minutes on the Orin
where equidistant takes seconds), and with `--split-half` refits the even and the odd views alone.
It prints the coverage, the RMS for square-on and tilted views separately, K / D, the field of view
and how far off axis the model stays monotonic.

Accept when:

- RMS ≲ 0.6 px (robot01: 0.55);
- both split halves say `agrees` (fx within 1.5 %), or `local minimum` — the half's own fit stopped
  in a worse optimum than the full one, which is the optimiser, not the data. `DISAGREES` means the
  views do not determine fx: collect more close, tilted views;
- fx ≈ fy, and cx / cy within ~30 px of the image centre;
- no empty coverage cell (the fit warns otherwise).

### 3. Compare with the installed file (optional)

```bash
python3 scripts/calibration/compare_intrinsics.py record/calib_robot01_20261020/intrinsics/board01 \
    record/calib_robot01_20261020/intrinsics/camera0.yaml config/calibration/robot01/camera0.yaml
```

Each YAML is scored on the same views with K and D fixed (only the board poses fitted), split into
views near the centre and near the edge, and the last lines say how many pixels apart the files
put the same ray. Append `@1.3` to a YAML calibrated on the stabilized stream to map it back to
the raw one.

### 4. Install

```bash
cp config/calibration/robot01/camera0.yaml config/calibration/robot01/camera0_<its-date>.yaml
cp record/calib_robot01_20261020/intrinsics/camera0.yaml config/calibration/robot01/camera0.yaml
docker compose exec robot01 pkill -f camera_feeder_node     # from the host
```

`config/calibration/` is not tracked, so the copy is the only backup. The feeder reads the file
once at start; its launch respawns it two seconds after the kill (the WebRTC view drops for those
two seconds). Run `pkill` directly as above or from an interactive shell — inside
`bash -c '…'` the pattern also matches that shell. Then check:

```bash
grep -a "camera_info from" log/stack/robot01/camera_feeder/current | tail -1
ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /robot01/camera_info
```

Then redo the extrinsic.

## Part 2 — extrinsic (camera → lidar)

The extrinsic is fitted by NID — how well lidar intensity predicts image brightness — over a
static scene. **Only the rotation is fitted; the translation stays the URDF's tape measure**
(lessons 7–8: these scenes do not determine it, and the tape is good to 1–2 cm).

### 1. Record

1. In AUTO, walk the robot to the spot, lie it down (`set_motion_key` key 2), then switch:
   `ros2 service call /robot01/switch_mode syncai_common/srv/SwitchMode "{mode: 3}"`. The
   CALIBRATION session has no `driver_manager`, so nothing can move the robot while it records; to
   move it, switch back to AUTO.
2. Look at the scene (`python3 scripts/calibration/snapshot.py record/…/peek.jpg`): structure at
   several depths, **including some 1–2.5 m away** (boxes, chairs, desk legs) — near objects are
   the only check on the translation — and nobody about to walk through. People in view keep still.
3. Record 20 s into one `bags/` directory per session, one bag per position, **at least two
   headings**:

   ```bash
   cd record/calib_robot01_20261020
   timeout -s INT 22 ros2 bag record -o bags/pos01 \
       /robot01/livox/lidar /robot01/image_raw/compressed /robot01/camera_info
   ros2 bag info bags/pos01      # camera_info: Count 1
   ```

   `camera_info` is latched: the bag gets the one the feeder published at its start, i.e. the
   intrinsics installed then. A bag recorded before an intrinsics change carries the old ones.

### 2. Preprocess

```bash
ros2 run direct_visual_lidar_calibration preprocess bags preprocessed -a
python3 scripts/calibration/extrinsic_init.py preprocessed
```

`preprocess` accumulates each bag's cloud into `preprocessed/<bag>.ply` (static integration — no
`-d` — which suits the MID360's non-repetitive scan), saves the image as `<bag>.png`, and takes the
camera model from the bag's `camera_info`. It drops points under 1 m and needs no display. It
writes `calib.json` afresh, so run `extrinsic_init.py` after every preprocess: it copies the URDF's
`camera_optical_joint` into `results.init_T_lidar_camera`, the starting pose of the fit.

### 3. Fit the rotation

```bash
python3 scripts/calibration/nid_fit.py preprocessed pos01 --write T_pos01 --profile
python3 scripts/calibration/nid_fit.py preprocessed pos02 --write T_pos02
python3 scripts/calibration/nid_fit.py preprocessed pos01,pos02 --write T_joint
```

A few minutes each; run them one after another (each `--write` rewrites `calib.json`). Each prints
the rotation relative to the starting pose as tilt up / pan right / roll, and the URDF origin.
`--profile` prints the cost ±2° around the result on each axis: there must be a clear minimum on
all three (pan and tilt are sharp; roll is the shallowest). The per-bag rotations should agree
within ~0.3°.

### 4. Validate

```bash
python3 scripts/calibration/edge_shift.py preprocessed pos02 init_T_lidar_camera T_pos01 T_joint
python3 scripts/calibration/edge_shift.py preprocessed pos01 init_T_lidar_camera T_pos02 T_joint
python3 scripts/calibration/compare_extrinsics.py preprocessed/calib.json init_T_lidar_camera T_pos01 T_pos02 T_joint
python3 scripts/calibration/overlay.py preprocessed pos01 T_joint side.png --mode side
python3 scripts/calibration/overlay.py preprocessed pos01 T_joint near.png --mode depth --crop 560,360,1040,680
```

- **Cross-check.** `edge_shift.py` measures, per image region, the shift that best aligns lidar
  intensity edges with image edges — a cost the fit did not minimise. One bag's rotation applied
  to the *other* bag must shrink that bag's residual. Regions marked `?` (low correlation, or the
  shift at the ±40 px search limit: ceilings, bare walls) do not count.
- **Near objects.** In the depth overlay, the points of an object 1–2 m away must sit on it. If
  they float above or beside it, the translation is wrong — the tape measure, or a translation
  someone took from a 6-DoF fit.
- **Side-by-side.** In the 64 px tile overlay, edges should run straight across the tile seams.

### 5. Write the URDF

Only `rpy` of `camera_optical_joint` changes; `xyz` stays the tape measure. Update the comment
block above the joint in the same edit (method, date, bags, residuals). `G23.urdf` is one file for
the whole fleet, so the joint holds one robot's calibration: a second robot with a camera needs a
per-robot override, which does not exist yet.

### dvlc's own `calibrate` (optional, not used for the result)

dvlc solves all six degrees of freedom by NID. Its viewer needs an OpenGL context even with
`--background`, and the robot has no display: install Xvfb in the container (not in the image, so
a container recreate loses it) and run it under `xvfb-run`:

```bash
sudo apt-get install -y xvfb
xvfb-run -a -s "-screen 0 1920x1080x24" \
    ros2 run direct_visual_lidar_calibration calibrate preprocessed --background --auto_quit
```

It writes `results.T_lidar_camera`. Its rotation is a usable cross-check (`compare_extrinsics.py`);
its translation is not (lesson 7).

## Why it is done this way

Lessons from robot01, 2026-10-19/20, in the order they were learned.

1. **The first `camera0.yaml` was calibrated on the stabilized stream.** It had fx 954; the EIS
   output is the raw image zoomed 1.3× (954 / 1.3 = 734, near the true ~710). On the raw board
   views it scores 1.93 px RMS against 0.55 px for the new file. With it, every extrinsic run of
   2026-10-19 pulled the camera 20–37 cm back along the optical axis to shrink the projected cloud
   — a calibration whose translation runs away along the optical axis has wrong intrinsics.
2. **Far boards bias the focal length.** `cornerSubPix` searches an 11×11 window; when the squares
   are smaller than that it spans neighbouring corners. Fitting all 75 views of the first session
   gave fx 744; only the views with squares ≥ 15 px gave 715 — and the 12 tilted views alone, 711.
3. **Outlier rejection threw away the views that mattered.** The largest residuals were the tilted
   views, the only ones that constrain fx; rejecting views over 3× the median residual removed 17
   of them and left fx 818.
4. **OpenCV's plumb_bob fit has a local minimum near fx 818.** Split into halves, one half fitted
   alone converged there with 1.05 px; the full solution explained that same half at 0.62 px. Hence
   `--split-half`, and its `local minimum` verdict.
5. **plumb_bob folds back on this lens.** Its radial polynomial peaks 57° off axis: a ray at 60°
   projected onto the image's right edge and one at 65° to x = 567, the middle of the image, while
   3.9 % of the image (its corners) had no ray at all. Inside the image both models fit the board
   equally (0.55 vs 0.56 px), but a lidar sees all around, so plumb_bob would paint points from
   beside the camera into the picture. Kannala-Brandt (`equidistant`) stays monotonic past 89°.
   The two fits put the principal point 15 px apart — it is the weakly determined part, and the
   reason the extrinsic must follow any intrinsics change.
6. **Tilted views keep a higher residual** (median 0.75 px against 0.19 px square-on) that neither
   a bent board, a print-scale error nor a richer distortion model (rational, k1–k6) explained per
   view. The board, an A4 sheet, was held by hand; a rigid board is the next thing to try if the
   intrinsics ever need to be better than this.
7. **dvlc's 6-DoF translation is not trustworthy on these scenes.** On pos01 it moved the camera
   27 cm down and tilted it 3.2° up: for a scene 2–6 m away the two are nearly the same image. The
   robot lying 1.2 m in front of the camera showed it — under that pose its points floated about
   100 px above it, under the tape measure they sat on it. The tape says the camera is 6.5 cm below
   the lidar.
8. **Neither do these scenes determine the translation for the NID fit here.** Freeing it
   (`--free w,c`) moved the camera (−10, +4.4, +0.8) cm on pos01 and (−4.2, +2.6, −1.9) cm on
   pos02 — away from the tape measure, and in directions that disagree. The tape is good to
   1–2 cm, about 3–7 px at 2 m; a rotation error does not shrink with distance (1° ≈ 12 px
   everywhere).

## robot01, 2026-10-20

### Intrinsics

`config/calibration/robot01/camera0.yaml`, installed 2026-10-20; the previous file is kept beside
it as `camera0_old_eis_zoom_20261016.yaml`, and the plumb_bob fit of the same views as
`camera0_plumb_bob.yaml`.

| | old (2026-10-16) | **new (2026-10-20)** |
|---|---|---|
| calibrated on | stabilized output (zoom 1.3) | **raw passthrough** |
| model | plumb_bob | **equidistant** |
| fx / fy | 953.9 / 947.3 | **710.4 / 707.3** |
| cx / cy | 690.5 / 377.6 | **663.2 / 371.4** |
| D | −0.411, 0.292, −0.001, −0.002, −0.071 | **−0.0701, 0.0200, −0.0336, 0.0148** |
| FOV h / v | 77° / 44° | **110° / 60°** |
| RMS on the 63 views, K / D fixed | 1.93 px | **0.55 px** |
| split-half fx | — | 714.1 / 707.4 (+0.5 % / −0.4 %) |

### Extrinsic

Bags `pos01` (office, robot facing the desks) and `pos02` (another heading, projector room, cases
at 1.5–2.5 m), 21 s each, robot lying down. Rotation relative to the tape-measured URDF joint
(`xyz="0.084 0.0 -0.046" rpy="-1.3208 0 -1.5708"`), translation kept:

| Fitted on | tilt up | pan right | roll | `rpy` | `edge_shift` RMS dx / dy |
|---|---|---|---|---|---|
| — (tape measure) | 0 | 0 | 0 | `-1.3208 0 -1.5708` | pos01 9.2 / 4.8 px, pos02 11.4 / 9.2 px |
| pos01 | −0.32° | +0.89° | −0.11° | `-1.3264 0.0019 -1.5864` | on pos02: 4.1 / 7.6 px |
| pos02 | +0.10° | +0.73° | −0.20° | `-1.3191 -0.0003 -1.5840` | on pos01: 4.7 / 5.9 px |
| **pos01 + pos02** | **−0.18°** | **+0.81°** | **−0.56°** | **`-1.3240 -0.0060 -1.5868`** | pos01 5.2 / 3.0 px, pos02 4.7 / 8.2 px |

- **The correction is a pan of about 0.8° to the right**, the same on both bags (0.73–0.89°), and
  each bag's rotation removes most of the other bag's horizontal residual (9–11 px → ~4 px).
- Tilt and roll are within ±0.3° of the tape measure and coupled with each other: at the joint
  tilt, pos02's cost is flat for roll between −0.56° and −0.9°, which is why the joint roll lies
  outside both single-bag values (it is the lower point of the combined cost, 3.1e-4 above the two
  bags' own optima against 6.2e-4 for their mean). 0.3° of roll is ~3 px at the image edge.
- pos02 keeps ~8 px of vertical residual whose sign differs between the top (+8) and the bottom
  left (−12) of the image — not a tilt, which would shift every region alike. Unexplained; the
  near-field translation is the first suspect.
- The robot lying 1.2 m in front of the camera on pos01 has its points on it under the joint pose
  (`overlay.py … --mode depth --crop 560,360,1040,680`).

The last row is `camera_optical_joint`'s `rpy` in `G23.urdf` since 2026-10-20.
