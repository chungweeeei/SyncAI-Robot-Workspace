"""Tests for the binary PGM helpers.

Real files on disk via tmp_path — nothing is mocked. The thumbnail assertions go
back through cv2.imdecode rather than trusting the encoder, the same way
test_occupancy_grid.py does.
"""

import pytest

pytest.importorskip("cv2")
pytest.importorskip("numpy")

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from syncai_backend.helpers.pgm import (  # noqa: E402
    read_pgm_size,
    render_png,
    render_thumbnail,
)


# --- read_pgm_size ----------------------------------------------------------


def test_read_size_returns_extent(tmp_path, make_pgm):
    path = make_pgm(tmp_path / "gridmap.pgm", 7, 3)

    assert read_pgm_size(str(path)) == (7, 3)


def test_read_size_skips_a_header_comment(tmp_path, make_pgm):
    """GIMP writes one, and the old keepout mask was authored there."""
    path = make_pgm(
        tmp_path / "gridmap.pgm", 7, 3,
        comment="Created by GIMP version 3.2.0 PNM plug-in",
    )

    assert read_pgm_size(str(path)) == (7, 3)


def test_read_size_rejects_ascii_pgm(tmp_path, make_pgm):
    path = make_pgm(tmp_path / "gridmap.pgm", 4, 4, magic=b"P2")

    with pytest.raises(ValueError):
        read_pgm_size(str(path))


def test_read_size_rejects_other_maxval(tmp_path, make_pgm):
    path = make_pgm(tmp_path / "gridmap.pgm", 4, 4, maxval=254)

    with pytest.raises(ValueError):
        read_pgm_size(str(path))


def test_read_size_rejects_short_body(tmp_path, make_pgm):
    """A map caught mid-write must not be reported as readable."""
    path = make_pgm(tmp_path / "gridmap.pgm", 100, 100, body_bytes=b"\x00" * 10)

    with pytest.raises(ValueError):
        read_pgm_size(str(path))


def test_read_size_rejects_empty_file(tmp_path):
    path = tmp_path / "gridmap.pgm"
    path.write_bytes(b"")

    with pytest.raises(ValueError):
        read_pgm_size(str(path))


def test_read_size_rejects_non_numeric_extent(tmp_path):
    path = tmp_path / "gridmap.pgm"
    path.write_bytes(b"P5\nwide tall\n255\n" + b"\xfe" * 16)

    with pytest.raises(ValueError):
        read_pgm_size(str(path))


# --- render_png -------------------------------------------------------------


def test_png_keeps_native_size(tmp_path, make_pgm):
    data = make_pgm(tmp_path / "gridmap.pgm", 400, 200).read_bytes()

    assert _decode(render_png(data)).shape == (200, 400)


def test_png_is_byte_for_byte_lossless(tmp_path, make_pgm):
    """The editor reads cell *kinds* off these values.

    205 is unknown and 254 is free, one apart from the 204/255 a lossy or
    colour-managed path would hand back — and a map with every cell shifted by
    one still looks like a map, so this is the check that would catch it.
    """
    body = bytes([0, 205, 254, 255, 100, 89]) * 4
    data = make_pgm(tmp_path / "gridmap.pgm", 6, 4, body_bytes=body).read_bytes()

    decoded = _decode(render_png(data))

    assert decoded.tobytes() == body


def test_png_rejects_undecodable_data():
    with pytest.raises(ValueError):
        render_png(b"not an image at all")


# --- render_thumbnail -------------------------------------------------------


def _decode(png):
    return cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_GRAYSCALE)


def test_thumbnail_keeps_small_maps_unscaled(tmp_path, make_pgm):
    data = make_pgm(tmp_path / "gridmap.pgm", 8, 5).read_bytes()

    decoded = _decode(render_thumbnail(data, max_edge=512))

    assert decoded.shape == (5, 8)


def test_thumbnail_downscales_to_max_edge_keeping_aspect(tmp_path, make_pgm):
    data = make_pgm(tmp_path / "gridmap.pgm", 400, 200).read_bytes()

    decoded = _decode(render_thumbnail(data, max_edge=100))

    assert decoded.shape == (50, 100)


def test_thumbnail_preserves_grey_values(tmp_path, make_pgm):
    """The .pgm convention must survive: 205 stays 205, not remapped."""
    data = make_pgm(tmp_path / "gridmap.pgm", 4, 2, fill=205).read_bytes()

    decoded = _decode(render_thumbnail(data))

    assert np.array_equal(decoded, np.full((2, 4), 205, dtype=np.uint8))


def test_thumbnail_rejects_undecodable_data():
    with pytest.raises(ValueError):
        render_thumbnail(b"not an image at all")
