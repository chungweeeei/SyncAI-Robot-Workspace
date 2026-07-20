"""Unit tests for the point-cloud helpers (transform, downsample, pack, PCD)."""

import struct

import pytest

pytest.importorskip("numpy")

import numpy as np  # noqa: E402

from syncai_backend.helpers.pointcloud import (  # noqa: E402
    cap_points,
    pack_xyz_f32,
    quat_to_rotation_matrix,
    read_pcd_xyz,
    transform_points,
    voxel_downsample,
)


def test_identity_quat_is_identity_matrix():
    assert np.allclose(quat_to_rotation_matrix(0, 0, 0, 1), np.eye(3))


def test_quat_rotates_90deg_about_z():
    # +90 deg about z maps +x -> +y.
    rot = quat_to_rotation_matrix(0, 0, 0.7071068, 0.7071068)
    assert np.allclose(rot @ np.array([1, 0, 0]), [0, 1, 0], atol=1e-6)


def test_transform_applies_rotation_then_translation():
    pts = np.array([[1.0, 0.0, 0.0]])
    out = transform_points(
        pts,
        translation=np.array([10.0, 0.0, 0.0]),
        quat_xyzw=np.array([0, 0, 0.7071068, 0.7071068]),
    )
    assert np.allclose(out[0], [10.0, 1.0, 0.0], atol=1e-6)


def test_voxel_downsample_collapses_nearby_points():
    pts = np.array([[1.0, 0, 0], [1.0, 0, 0], [1.01, 0, 0], [5.0, 5.0, 5.0]])
    out = voxel_downsample(pts, 0.5)
    assert out.shape[0] == 2


def test_cap_points_bounds_count():
    pts = np.arange(30).reshape(10, 3).astype(float)
    assert cap_points(pts, 4).shape[0] <= 4


def test_pack_xyz_f32_layout():
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    data = pack_xyz_f32(pts)
    assert len(data) == 2 * 3 * 4
    assert np.frombuffer(data, dtype="<f4").tolist() == [1, 2, 3, 4, 5, 6]


def test_read_pcd_binary_roundtrip(tmp_path):
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    intensity = np.array([10.0, 20.0], dtype=np.float32)

    header = (
        "# .PCD v0.7\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH 2\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS 2\n"
        "DATA binary\n"
    )
    body = b"".join(
        struct.pack("<ffff", *pts[i], intensity[i]) for i in range(2)
    )
    path = tmp_path / "map.pcd"
    path.write_bytes(header.encode("ascii") + body)

    xyz = read_pcd_xyz(str(path))
    assert xyz.shape == (2, 3)
    assert np.allclose(xyz, pts)
