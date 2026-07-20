"""Point-cloud helpers: voxel downsampling, rigid transform, wire packing.

Shared by the live body_cloud subscriber and the static map-cloud endpoint so
both produce the same on-the-wire layout: little-endian float32 xyz triplets.
The frontend three.js viewer reads these straight into a BufferGeometry
position attribute, so keeping the packing in one place keeps the two producers
in lockstep.
"""

import numpy as np

# PCD TYPE+SIZE -> numpy dtype char, covering the fields FAST-LIO writes.
_PCD_TYPE_MAP = {
    ("F", 4): "f4",
    ("F", 8): "f8",
    ("U", 1): "u1",
    ("U", 2): "u2",
    ("U", 4): "u4",
    ("I", 1): "i1",
    ("I", 2): "i2",
    ("I", 4): "i4",
}


def read_pcd_xyz(path: str) -> np.ndarray:
    """Read the x/y/z columns of a PCD file into an (N, 3) float64 array.

    Supports ``DATA ascii`` and ``DATA binary`` (the format FAST-LIO / PGO
    write); ``binary_compressed`` is not supported. Intensity and any other
    fields are ignored — the viewer colours by height.
    """
    fields, sizes, types, counts = [], [], [], []
    points_count = 0
    data_format = None

    with open(path, "rb") as fh:
        while True:
            raw = fh.readline()
            if not raw:
                raise ValueError(f"Unexpected EOF reading PCD header: {path}")
            line = raw.decode("ascii", errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            key, _, rest = line.partition(" ")
            key = key.upper()
            if key == "FIELDS":
                fields = rest.split()
            elif key == "SIZE":
                sizes = [int(v) for v in rest.split()]
            elif key == "TYPE":
                types = rest.split()
            elif key == "COUNT":
                counts = [int(v) for v in rest.split()]
            elif key == "POINTS":
                points_count = int(rest)
            elif key == "DATA":
                data_format = rest.strip().lower()
                break

        if data_format not in ("ascii", "binary"):
            raise ValueError(f"Unsupported PCD DATA format: {data_format}")
        if not counts:
            counts = [1] * len(fields)

        dtype = np.dtype(
            [
                (name, _PCD_TYPE_MAP[(t, s)], (c,) if c > 1 else ())
                for name, s, t, c in zip(fields, sizes, types, counts)
            ]
        )

        if data_format == "binary":
            buf = fh.read(dtype.itemsize * points_count)
            arr = np.frombuffer(buf, dtype=dtype, count=points_count)
        else:
            arr = np.loadtxt(fh, dtype=dtype, max_rows=points_count)

    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float64)
    # Drop NaN/inf rows LIO occasionally leaves in the saved map.
    return xyz[np.isfinite(xyz).all(axis=1)]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Keep one representative point per ``voxel_size`` cube.

    ``points`` is an (N, 3) float array; returns an (M, 3) array with M <= N.
    Reduces both bandwidth and the client's GPU load without distorting the
    cloud's shape (unlike a plain stride, which thins dense and sparse regions
    equally).
    """
    if voxel_size <= 0 or points.shape[0] == 0:
        return points

    # Quantise each point to its voxel index, then keep the first point that
    # lands in each unique voxel.
    keys = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(unique_idx)]


def cap_points(points: np.ndarray, max_points: int) -> np.ndarray:
    """Uniformly stride the cloud down to at most ``max_points`` points."""
    n = points.shape[0]
    if max_points <= 0 or n <= max_points:
        return points
    step = int(np.ceil(n / max_points))
    return points[::step]


def quat_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Build a 3x3 rotation matrix from a (x, y, z, w) quaternion."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def transform_points(
    points: np.ndarray, translation: np.ndarray, quat_xyzw: np.ndarray
) -> np.ndarray:
    """Apply a rigid transform (rotation then translation) to an (N, 3) cloud."""
    if points.shape[0] == 0:
        return points
    rot = quat_to_rotation_matrix(*quat_xyzw)
    return points[:, :3] @ rot.T + translation


def pack_xyz_f32(points: np.ndarray) -> bytes:
    """Pack an (N, 3) cloud as contiguous little-endian float32 xyz triplets."""
    xyz = np.ascontiguousarray(points[:, :3], dtype="<f4")
    return xyz.tobytes()
