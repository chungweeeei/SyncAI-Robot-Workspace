"""Unit tests for the OccupancyGrid -> PNG conversion helper."""

import base64

import pytest

pytest.importorskip("cv2")
pytest.importorskip("numpy")
pytest.importorskip("nav_msgs")

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from syncai_backend.helpers.occupancy_grid import (  # noqa: E402
    occupancy_grid_to_png,
    occupancy_grid_to_png_base64,
)

# height=3, width=2, row-major from the bottom-left origin:
#   row0 (bottom): free(0),    occupied(100)
#   row1:          unknown(-1), 25% (25)
#   row2 (top):    50% (50),   occupied(100)
_GRID_DATA = [0, 100, -1, 25, 50, 100]

# Greyscale mapping: free->255, occupied->0, unknown->205,
# 25% -> round(75*2.55)=191, 50% -> round(50*2.55)=128; then flipped so the
# top map row is the first image row.
_EXPECTED = np.array([[128, 0], [205, 191], [255, 0]], dtype=np.uint8)


def test_to_png_maps_values_and_flips(make_occupancy_grid):
    grid = make_occupancy_grid(2, 3, _GRID_DATA)

    img = occupancy_grid_to_png(grid)

    assert img.dtype == np.uint8
    assert img.shape == (3, 2)
    assert np.array_equal(img, _EXPECTED)


def test_base64_is_data_uri_and_roundtrips(make_occupancy_grid):
    grid = make_occupancy_grid(2, 3, _GRID_DATA)

    uri = occupancy_grid_to_png_base64(grid)

    assert uri.startswith("data:image/png;base64,")

    raw = base64.b64decode(uri.split(",", 1)[1])
    decoded = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert np.array_equal(decoded, _EXPECTED)
