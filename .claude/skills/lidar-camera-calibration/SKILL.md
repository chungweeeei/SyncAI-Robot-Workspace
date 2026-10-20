---
name: lidar-camera-calibration
description: Calibrate a SyncAI robot's camera with scripts/calibration — the intrinsics (config/calibration/<robot_id>/camera0.yaml, published by syncai_camera_feeder as camera_info) and the camera-to-lidar extrinsic (camera_optical_joint in G23.urdf). Use when asked to (re)calibrate the camera, redo camera intrinsics or lidar-camera extrinsics, check camera_info / camera0.yaml, or fix lidar points that do not line up with the camera image.
---

# Lidar-camera calibration

The procedure, its acceptance numbers and the reasons behind every step are in
`docs/lidar-camera-calibration.md` — read it before starting. This skill is how to drive that
procedure with a person who moves the board and the robot while you run the tools.

## Ground rules

- Run ROS-side commands inside the robot container, from the workspace root (the container's cwd):
  `docker compose exec -T robot01 bash -c 'source /opt/ros/humble/setup.bash; source install/setup.bash; <cmd>'`.
  `collect_board.py`, `snapshot.py`, `ros2 bag record` and dvlc's `preprocess` need the container;
  the fitting / checking tools need only numpy + OpenCV + PyYAML and also run with the host's `python3`.
- Use a work directory `record/calib_<robot_id>_<yyyymmdd>/` (gitignored). Leave existing work
  directories alone unless the user says they are stale.
- `plumb_bob` fits and `nid_fit.py` take minutes: run them with `run_in_background` and say so.
  Two `nid_fit.py --write` runs on one `calib.json` race: run those one after the other.
- Ask before: replacing `config/calibration/<robot_id>/camera0.yaml` (the directory is untracked —
  copy the old file aside first, it is the only backup), restarting the feeder (drops the WebRTC view
  ~2 s), installing anything in the container, and editing `G23.urdf` (tracked, one file for the
  whole fleet). Never commit, never `colcon build`.
- The person can only see what you tell them. When progress stalls, look: `snapshot.py` a frame
  and `Read` it, rather than guessing what they are doing.

## Part 1 — intrinsics

1. **Raw stream check.** The feeder's startup log `pipeline:` line must be the MJPEG passthrough
   (`v4l2src … ! tee`, no decode / warp), i.e. `stabilize: false`. A calibration of the stabilized
   output is wrong for everything (robot01's first file: fx 954 instead of ~710).
2. **Collect.** Start `scripts/calibration/collect_board.py <work>/intrinsics/board01` in the
   background in the container, then tell the board holder how to move (the doc's list; the two
   that matter most: kneel so the board is 0.3–0.8 m from the low camera, and tilt it 30–45 deg both
   ways about both axes). On "progress?", read `<work>/intrinsics/board01/status.txt` and turn every
   `--` line into one concrete instruction:
   - empty grid cells → where in the image to put the board (image right = the holder's left);
   - tilt counts short → which way to tip (`tilt_x`: nod the rows' edges towards / away;
     `tilt_y`: turn like a door) — check a recent view's corners to tell which physical motion
     moves which number if unsure;
   - "too far away" views piling up → closer.
   Count only usable views (squares ≥ 15 px). Kept views that stop increasing while the board is
   visible means the holder repeats poses: ask for new positions, not longer holds.
3. **Fit.** `fit_intrinsics.py <dir> --out <work>/intrinsics/camera0.yaml --split-half`
   (equidistant). Accept on the doc's criteria; both split halves must say `agrees` (a
   `local minimum` verdict is fine, `DISAGREES` means collect more close, tilted views).
4. **Compare** with the installed file: `compare_intrinsics.py <dir> <new> config/calibration/<robot_id>/camera0.yaml`.
   Report fx / fy / cx / cy, the RMS of both on the same views, and the FOV.
5. **Install** (after the user agrees): back up, copy, `docker compose exec -T robot01 pkill -f camera_feeder_node`
   (exec pkill directly — inside `bash -c '…'` the pattern matches the shell itself), then confirm
   the feeder log's `camera_info from … fx …` line and a transient-local echo of `camera_info`.
   Tell the user the extrinsic must be redone: the two are coupled through the principal point.

## Part 2 — extrinsic

1. The user lies the robot down in AUTO and switches to CALIBRATION (`switch_mode {mode: 3}`);
   nothing in that session can move the robot. `snapshot.py` a frame and judge the scene: structure
   at several depths including some 1–2.5 m away, nobody about to walk through. Ask whoever is in
   view to keep still for the 20 s.
2. **Record** each position into the same `bags/` directory (`timeout -s INT 22 ros2 bag record -o
   bags/posNN /<robot_id>/livox/lidar /<robot_id>/image_raw/compressed /<robot_id>/camera_info`)
   and check `ros2 bag info` shows one `camera_info`. At least two headings; the user repositions
   the robot (back to AUTO and again) between them.
3. **Preprocess** `ros2 run direct_visual_lidar_calibration preprocess bags preprocessed -a`, then
   `extrinsic_init.py preprocessed` — after every preprocess, which rewrites `calib.json`.
4. **Fit rotation only** (translation stays the URDF's tape measure): `nid_fit.py preprocessed posNN
   --write T_posNN --profile` per bag, then all bags jointly `--write T_joint`.
5. **Validate**: each bag's rotation must cut the *other* bag's `edge_shift.py` residual; the
   `--profile` must show a minimum on every axis; render `overlay.py --mode depth` crops of near
   objects and `Read` them — their points must sit on them. `compare_extrinsics.py` gives the table.
6. **Report** per-bag and joint rotation (tilt up / pan right / roll vs. the URDF), the edge
   residuals before / after, and the URDF line. Only `rpy` changes; `xyz` stays. Ask before writing
   it, and update in the same change the comment block above `camera_optical_joint` (method,
   date, bags, residuals), the joint paragraph of `src/syncai_bringup/README.md` and the results
   section of the doc.

## Do not

- Do not take a translation from dvlc's `calibrate` or from `nid_fit.py --free w,c` on scenes like
  these: robot01's runs moved the camera 4–27 cm in directions that disagreed between bags. The
  doc's near-object check is how to tell.
- Do not re-enable outlier rejection or fit views with squares under 15 px; do not pick plumb_bob
  for this lens (it folds back 57 deg off axis).
- Do not install the result into another robot's directory or treat robot01's numbers as fleet
  values.
