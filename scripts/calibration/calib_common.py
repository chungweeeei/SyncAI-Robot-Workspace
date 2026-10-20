"""Shared helpers for the camera and lidar-camera calibration tools in scripts/calibration/.

The procedure they serve is docs/lidar-camera-calibration.md. Everything here needs only numpy,
OpenCV and PyYAML -- no ROS, no scipy -- so every tool but collect_board.py also runs on the Jetson
host, outside the robot container.

Two camera models, by their camera_info names: "plumb_bob" (OpenCV's k1 k2 p1 p2 k3) and
"equidistant" (OpenCV's fisheye module, Kannala-Brandt k1..k4). The extrinsic tools read
direct_visual_lidar_calibration's preprocessed directories: <bag>.png, <bag>.ply and a calib.json
whose "results" hold poses as [x, y, z, qx, qy, qz, qw] -- the camera in the lidar frame, i.e.
exactly the URDF joint lidar_top -> camera_optical_frame.
"""
import configparser
import functools
import glob
import json
import os

import cv2
import numpy as np
import yaml

# A board view counts as tilted when the log ratio of its opposite edges reaches this on either
# axis: about 30 deg of tilt at half a metre. Tilted views are the ones that pin the focal length.
TILT = 0.15


def read_robot_id(ini="config/system.ini"):
    """[system] robot_id, read the way the launch files read it (cwd = the workspace root)."""
    cp = configparser.ConfigParser()
    if not cp.read(ini) or not cp.has_option("system", "robot_id"):
        raise SystemExit(f"no [system] robot_id in {ini}: run from the workspace root, or pass --robot-id")
    return cp.get("system", "robot_id")


# ---------------------------------------------------------------- camera models


def normalize_model(name):
    if name in ("equidistant", "fisheye"):
        return "equidistant"
    if name == "plumb_bob":
        return name
    raise SystemExit(f"camera model {name!r} is not one of plumb_bob, equidistant")


@functools.lru_cache(maxsize=32)
def _ray_limit(model, dist):
    th = np.linspace(0.0, np.radians(89.0), 8901)
    if model == "equidistant":
        k1, k2, k3, k4 = dist[:4]
        rd = th * (1 + k1 * th**2 + k2 * th**4 + k3 * th**6 + k4 * th**8)
    else:
        k1, k2 = dist[:2]
        k3 = dist[4] if len(dist) > 4 else 0.0
        r = np.tan(th)
        rd = r * (1 + k1 * r**2 + k2 * r**4 + k3 * r**6)
    fold = np.flatnonzero(np.diff(rd) <= 0)
    i = int(fold[0]) if len(fold) else len(th) - 1
    return float(th[i]), float(rd[i])


def ray_limit(model, D):
    """(largest angle off the optical axis, in rad, that the model still maps monotonically,
    normalized distorted radius it maps to).

    A wide lens fitted with plumb_bob folds back: on robot01's (2026-10-20) the polynomial peaks
    57 deg off axis, so a lidar point 65 deg off axis projected into the middle of the image and
    the image corners had no ray at all. Kannala-Brandt fits of the same lens stay monotonic to
    89 deg. Projection below drops every ray past the limit instead of drawing a ghost.
    """
    return _ray_limit(model, tuple(float(x) for x in np.ravel(D)))


class Camera:
    def __init__(self, K, D, model, size=None):
        self.K = np.asarray(K, np.float64).reshape(3, 3)
        self.D = np.asarray(D, np.float64).ravel()
        self.model = normalize_model(model)
        self.size = size  # (width, height), when known

    @property
    def max_angle(self):
        return ray_limit(self.model, self.D)[0]

    def project(self, pc, min_z=0.3):
        """Camera-frame points -> pixels, for rays the model maps monotonically.

        Returns (uv, mask): uv has one row per True in mask.
        """
        pc = np.asarray(pc, np.float64)
        ang = np.arctan2(np.hypot(pc[:, 0], pc[:, 1]), pc[:, 2])
        m = (pc[:, 2] > min_z) & (ang < 0.98 * self.max_angle)
        if not m.any():
            return np.zeros((0, 2)), m
        p = pc[m]
        if self.model == "equidistant":
            uv, _ = cv2.fisheye.projectPoints(p.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), self.K, self.D)
        else:
            uv, _ = cv2.projectPoints(p, np.zeros(3), np.zeros(3), self.K, self.D)
        return uv.reshape(-1, 2), m

    def unproject(self, px):
        """Pixels -> normalized coordinates (x/z, y/z); NaN where no ray of the model lands."""
        px = np.asarray(px, np.float64).reshape(-1, 2)
        if self.model == "equidistant":
            n = cv2.fisheye.undistortPoints(px.reshape(-1, 1, 2), self.K, self.D).reshape(-1, 2)
        else:
            crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 200, 1e-12)
            n = cv2.undistortPointsIter(px.reshape(-1, 1, 2), self.K, self.D, None, None, crit).reshape(-1, 2)
        rd = np.hypot((px[:, 0] - self.K[0, 2]) / self.K[0, 0], (px[:, 1] - self.K[1, 2]) / self.K[1, 1])
        n[rd > ray_limit(self.model, self.D)[1]] = np.nan
        return n

    def unreachable_fraction(self):
        """Share of the image's pixels that no ray of the model reaches (plumb_bob past its fold)."""
        w, h = self.size
        gx, gy = np.meshgrid(np.arange(0, w, 4), np.arange(0, h, 4))
        rd = np.hypot((gx - self.K[0, 2]) / self.K[0, 0], (gy - self.K[1, 2]) / self.K[1, 1])
        return float(np.mean(rd > ray_limit(self.model, self.D)[1]))

    def fov_deg(self):
        """(horizontal, vertical, diagonal) field of view; NaN where an edge has no ray."""
        w, h = self.size
        n = self.unproject([[0, h / 2], [w, h / 2], [w / 2, 0], [w / 2, h], [0, 0], [w, h]])
        a = np.degrees(np.arctan(np.hypot(n[:, 0], n[:, 1])))
        return a[0] + a[1], a[2] + a[3], a[4] + a[5]

    def view_errors(self, obj, rvecs, tvecs, views):
        """Per-view RMS reprojection error (px) for given board poses."""
        out = []
        for c, r, t in zip(views, rvecs, tvecs):
            if self.model == "equidistant":
                p, _ = cv2.fisheye.projectPoints(obj.reshape(-1, 1, 3), r, t, self.K, self.D)
            else:
                p, _ = cv2.projectPoints(obj, r, t, self.K, self.D)
            out.append(np.sqrt(((p.reshape(-1, 2) - c) ** 2).sum(1).mean()))
        return np.array(out)

    def pose_only_errors(self, obj, views):
        """Per-view RMS (px) with K and D held fixed and only each board pose fitted."""
        rvecs, tvecs = [], []
        for c in views:
            if self.model == "equidistant":
                n = cv2.fisheye.undistortPoints(c.reshape(-1, 1, 2), self.K, self.D).reshape(-1, 2)
                _, r, t = cv2.solvePnP(obj, n, np.eye(3), None)
            else:
                _, r, t = cv2.solvePnP(obj, c, self.K, self.D)
            rvecs.append(r)
            tvecs.append(t)
        return self.view_errors(obj, rvecs, tvecs, views)


def fit_camera(model, obj, views, size):
    """Joint fit of K, D and every board pose. Returns (Camera, rms, per-view errors)."""
    model = normalize_model(model)
    if model == "equidistant":
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 200, 1e-9)
        rms, K, D, rv, tv = cv2.fisheye.calibrate(
            [obj.reshape(-1, 1, 3)] * len(views), [v.reshape(-1, 1, 2) for v in views], size, None, None,
            flags=flags, criteria=crit)
        D = D.ravel()[:4]
    else:
        rms, K, D, rv, tv = cv2.calibrateCamera(
            [obj.astype(np.float32)] * len(views), [v.astype(np.float32) for v in views], size, None, None)
        D = D.ravel()[:5]
    cam = Camera(K, D, model, size)
    return cam, rms, cam.view_errors(obj, rv, tv, views)


def load_camera_yaml(spec):
    """A camera_calibration-style YAML (what syncai_camera_feeder reads as camera_info_file).

    "path@zoom" divides K by zoom about the image centre: a calibration made on the feeder's
    stabilized output (stabilize: true, zoom 1.3) mapped back to the raw stream.
    Returns (Camera, label).
    """
    path, _, zoom = spec.partition("@")
    with open(path) as f:
        y = yaml.safe_load(f)
    K = np.array(y["camera_matrix"]["data"], np.float64).reshape(3, 3)
    w, h = y["image_width"], y["image_height"]
    if zoom:
        z = float(zoom)
        K[0, 0] /= z
        K[1, 1] /= z
        K[0, 2] = (K[0, 2] - w / 2) / z + w / 2
        K[1, 2] = (K[1, 2] - h / 2) / z + h / 2
    cam = Camera(K, y["distortion_coefficients"]["data"], y["distortion_model"], (w, h))
    return cam, os.path.basename(path) + (f" /{zoom}" if zoom else "")


def write_camera_yaml(path, cam, name="camera0", comments=()):
    """camera_calibration's ost.yaml layout, with optional leading # lines (YAML comments)."""
    def fmt(a):
        return ", ".join(f"{x:.8f}" for x in np.ravel(a))

    w, h = cam.size
    P = np.hstack([cam.K, np.zeros((3, 1))])
    lines = [f"# {c}" for c in comments] + [
        f"image_width: {w}",
        f"image_height: {h}",
        f"camera_name: {name}",
        "camera_matrix:", "  rows: 3", "  cols: 3", f"  data: [{fmt(cam.K)}]",
        f"distortion_model: {cam.model}",
        "distortion_coefficients:", "  rows: 1", f"  cols: {len(cam.D)}", f"  data: [{fmt(cam.D)}]",
        "rectification_matrix:", "  rows: 3", "  cols: 3", "  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]",
        "projection_matrix:", "  rows: 3", "  cols: 4", f"  data: [{fmt(P)}]",
    ]
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


# ---------------------------------------------------------------- checkerboard views


def parse_pattern(s):
    """"7x10" -> (7, 10): inner corners per row, rows. robot01's board ("8x11" squares) is 7x10."""
    cols, rows = (int(x) for x in s.lower().split("x"))
    return cols, rows


def board_points(pattern, square):
    cols, rows = pattern
    obj = np.zeros((cols * rows, 3), np.float64)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return obj


def board_quad(c, pattern):
    cols = pattern[0]
    return np.array([c[0], c[cols - 1], c[-1], c[-cols]])


def square_px(c, pattern):
    """Median spacing of neighbouring corners in a view, in pixels."""
    cols, rows = pattern
    return float(np.median(np.linalg.norm(np.diff(c.reshape(rows, cols, 2), axis=1), axis=2)))


def board_tilt(c, pattern):
    """(tilt_x, tilt_y): log ratios of opposite board edges.

    tilt_x compares the first and the last row of corners, so it moves when the board is tipped
    about an axis along its rows; tilt_y compares the first and the last column. 0 is a board
    square-on to the camera.
    """
    q = board_quad(c, pattern)
    e = [np.linalg.norm(q[(i + 1) % 4] - q[i]) for i in range(4)]
    return float(np.log(e[0] / e[2])), float(np.log(e[1] / e[3]))


def board_size(c, pattern, shape):
    """sqrt of the board's share of the image area (shape = (h, w))."""
    return float(np.sqrt(cv2.contourArea(board_quad(c, pattern).astype(np.float32)) / (shape[0] * shape[1])))


def coverage(views, shape):
    """Corner counts on a 4x4 grid of the image, rows top to bottom."""
    h, w = shape
    cov = np.zeros((4, 4), int)
    for c in views:
        for x, y in c:
            cov[min(3, int(4 * y / h)), min(3, int(4 * x / w))] += 1
    return cov


def load_views(collect_dir, pattern):
    """The vNNN_*.npy corners collect_board.py saved. Returns (names, views, (h, w))."""
    npys = sorted(glob.glob(os.path.join(collect_dir, "v*.npy")))
    if not npys:
        raise SystemExit(f"no collect_board.py views (v*.npy) in {collect_dir}")
    img = cv2.imread(npys[0][:-4] + ".jpg", cv2.IMREAD_GRAYSCALE)
    views = [np.load(f).astype(np.float64).reshape(-1, 2) for f in npys]
    n = pattern[0] * pattern[1]
    if any(len(c) != n for c in views):
        raise SystemExit(f"views do not have {n} corners: pass the --pattern collect_board.py ran with")
    return [os.path.basename(f)[:-4] for f in npys], views, img.shape


# ---------------------------------------------------------------- poses


def quat_to_R(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def R_to_quat(R):
    """Rotation matrix -> (qx, qy, qz, qw), qw >= 0."""
    m = R
    tr = np.trace(m)
    if tr > 0:
        s = 2 * np.sqrt(tr + 1)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2 * np.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2 * np.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2 * np.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    return q if q[3] >= 0 else -q


def rpy_to_R(roll, pitch, yaw):
    """URDF rpy: fixed axes x, y, z, i.e. R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    return float(np.arctan2(R[2, 1], R[2, 2])), float(-np.arcsin(np.clip(R[2, 0], -1, 1))), \
        float(np.arctan2(R[1, 0], R[0, 0]))


def expso3(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(w) / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * Kx + (1 - np.cos(th)) * Kx @ Kx


def logso3(R):
    th = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if th < 1e-12:
        return np.zeros(3)
    return th / (2 * np.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def camera_delta_deg(R_ref, R):
    """R relative to R_ref as small rotations about R_ref's camera axes, in degrees:
    (tilt up, pan right, roll), roll positive turning the camera's x axis towards its y axis.

    Optical frame: x right, y down, z forward, so a +x rotation lifts z (tilt up) and a +y rotation
    swings z towards x (pan right).
    """
    return np.degrees(logso3(R_ref.T @ R))


def pose_from_result(v):
    return np.array(v[:3], np.float64), quat_to_R(*v[3:7])


def result_from_pose(t, R):
    return [float(x) for x in t] + [float(x) for x in R_to_quat(R)]


def urdf_origin(t, R):
    r, p, y = R_to_rpy(R)
    return f'xyz="{t[0]:.4f} {t[1]:.4f} {t[2]:.4f}" rpy="{r:.4f} {p:.4f} {y:.4f}"'


# ---------------------------------------------------------------- direct_visual_lidar_calibration data


def load_calib(d):
    with open(os.path.join(d, "calib.json")) as f:
        return json.load(f)


def save_calib(d, cfg):
    path = os.path.join(d, "calib.json")
    with open(path + ".tmp", "w") as f:
        json.dump(cfg, f, indent=2)
    os.replace(path + ".tmp", path)


def camera_from_calib(cfg, size=None):
    c = cfg["camera"]
    fx, fy, cx, cy = c["intrinsics"][:4]
    return Camera([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], c["distortion_coeffs"], c["camera_model"], size)


def load_bag(d, bag):
    """A preprocessed bag: the accumulated cloud (N x [x, y, z, intensity], lidar frame) and the
    grey image."""
    with open(os.path.join(d, f"{bag}.ply"), "rb") as f:
        props = 0
        while True:
            line = f.readline()
            if not line:
                raise SystemExit(f"{d}/{bag}.ply: no end_header")
            line = line.strip()
            if line.startswith(b"property float"):
                props += 1
            if line == b"end_header":
                break
        if props != 4:
            raise SystemExit(f"{d}/{bag}.ply has {props} float properties, expected x y z intensity")
        pts = np.frombuffer(f.read(), dtype=np.float32).reshape(-1, 4).astype(np.float64)
    img = cv2.imread(os.path.join(d, f"{bag}.png"), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f"cannot read {d}/{bag}.png")
    return pts, img
