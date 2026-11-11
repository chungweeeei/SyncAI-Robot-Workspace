"""Tests for calib_common's pure helpers: python3 -m pytest scripts/calibration/test_calib_common.py

They need numpy and OpenCV only, like the tools themselves, so they run on the host too. The
real-data fixtures are robot01's two equidistant fits of 2026-11-11: the first, made with almost
no board corners in the image's outer ring, turns over 73.4 deg off axis; the second, after views
of the board pushed into the corners, stays monotonic to the end of the sampled range.
"""
import time

import numpy as np
import pytest

import calib_common as cc

SIZE = (1280, 720)

ROBOT01_THIN_CORNERS = cc.Camera(
    [[707.37408438, 0, 660.33235110], [0, 702.74134584, 359.25907125], [0, 0, 1]],
    [-0.06550824, -0.02350346, 0.04580159, -0.02723721], "equidistant", SIZE)

ROBOT01_WITH_CORNERS = cc.Camera(
    [[707.61326227, 0, 659.54458053], [0, 703.12382526, 360.48345274], [0, 0, 1]],
    [-0.07202781, 0.01680573, -0.02712167, 0.01385373], "equidistant", SIZE)


def pinhole_k(f=700.0):
    return [[f, 0, SIZE[0] / 2], [0, f, SIZE[1] / 2], [0, 0, 1]]


# ---------------------------------------------------------------- fold_report: analytic cases


def test_undistorted_equidistant_never_folds():
    rep = cc.Camera(pinhole_k(), [0, 0, 0, 0], "equidistant", SIZE).fold_report()
    assert rep.fold_deg == pytest.approx(cc.RAY_MAX_DEG)
    assert not rep.ghosts_in_image
    assert np.isnan(rep.ghost_px)
    assert rep.unreachable == 0


def test_equidistant_cubic_folds_where_its_derivative_vanishes():
    # theta_d = theta - 0.2 theta^3 turns over at theta = sqrt(1 / 0.6) = 73.97 deg, peaking at a
    # normalized radius of 0.861: short of the image corner's 1.049, so the corners get no ray and
    # the rays past the fold (down to 0.804 at the end of the range) land inside the image.
    rep = cc.Camera(pinhole_k(), [-0.2, 0, 0, 0], "equidistant", SIZE).fold_report()
    assert rep.fold_deg == pytest.approx(np.degrees(np.sqrt(1 / 0.6)), abs=0.02)
    theta_end = np.radians(cc.RAY_MAX_DEG)
    assert rep.ghost_px == pytest.approx(700.0 * (theta_end - 0.2 * theta_end**3), rel=1e-3)
    assert rep.ghosts_in_image
    assert 0 < rep.unreachable < 0.2


def test_plumb_bob_folds_on_the_tangent_radius():
    # r_d = r - 0.1 r^3 with r = tan(theta) turns over at r = sqrt(1 / 0.3), i.e. 61.3 deg.
    rep = cc.Camera(pinhole_k(), [-0.1, 0, 0, 0, 0], "plumb_bob", SIZE).fold_report()
    assert rep.fold_deg == pytest.approx(np.degrees(np.arctan(np.sqrt(1 / 0.3))), abs=0.02)
    assert rep.ghosts_in_image


def test_fold_report_needs_the_image_size():
    with pytest.raises(ValueError):
        cc.Camera(pinhole_k(), [0, 0, 0, 0], "equidistant").fold_report()


# ---------------------------------------------------------------- fold_report: robot01's fits


def test_robot01_fit_with_thin_corners_folds_into_the_image():
    rep = ROBOT01_THIN_CORNERS.fold_report()
    assert rep.fold_deg == pytest.approx(73.4, abs=0.1)
    assert rep.ghost_px == pytest.approx(466.0, abs=5.0)
    assert rep.ghosts_in_image
    # The peak lies a pixel beyond the farthest corner, which is why the old pixels-without-a-ray
    # test stayed silent on this fit.
    assert rep.unreachable == 0


def test_robot01_fit_with_corner_views_is_monotonic():
    rep = ROBOT01_WITH_CORNERS.fold_report()
    assert rep.fold_deg == pytest.approx(cc.RAY_MAX_DEG)
    assert not rep.ghosts_in_image
    assert rep.unreachable == 0


# ---------------------------------------------------------------- fold_report: stress


def test_fold_report_invariants_over_random_lenses():
    rng = np.random.default_rng(20261111)
    start = time.perf_counter()
    folded = 0
    for _ in range(1000):
        model = "equidistant" if rng.random() < 0.7 else "plumb_bob"
        dist = rng.normal(0, 0.05, 4 if model == "equidistant" else 5)
        cam = cc.Camera(pinhole_k(rng.uniform(400, 1000)), dist, model, SIZE)
        rep = cam.fold_report()
        assert 0 < rep.fold_deg <= cc.RAY_MAX_DEG
        assert 0 <= rep.unreachable <= 1
        if rep.fold_deg >= cc.RAY_MAX_DEG:
            assert np.isnan(rep.ghost_px) and not rep.ghosts_in_image
        else:
            folded += 1
            peak_px = cc.ray_limit(model, dist)[1] * cam.K[0, 0]
            assert rep.ghost_px <= peak_px + 1e-6
            assert rep.ghosts_in_image or rep.unreachable == 0
    assert 0 < folded < 1000, "the random lenses must exercise both branches"
    assert time.perf_counter() - start < 30.0


# ---------------------------------------------------------------- outer_ring_counts

SHAPE = (SIZE[1], SIZE[0])  # (h, w), as load_views returns it


def at_ring_ratio(ratio, quadrant):
    """The pixel `ratio` of the way from the image centre to one image corner."""
    w, h = SIZE
    corner = {"tl": (0, 0), "tr": (w, 0), "bl": (0, h), "br": (w, h)}[quadrant]
    return [w / 2 + ratio * (corner[0] - w / 2), h / 2 + ratio * (corner[1] - h / 2)]


def test_outer_ring_counts_each_quadrant_on_its_own():
    view = np.array([at_ring_ratio(0.95, "tl"), at_ring_ratio(0.95, "tr"), at_ring_ratio(0.95, "tr"),
                     at_ring_ratio(0.95, "bl"), at_ring_ratio(0.95, "br"), at_ring_ratio(0.95, "br"),
                     at_ring_ratio(0.95, "br")])
    assert cc.outer_ring_counts([view], SHAPE) == (1, 2, 1, 3)


def test_outer_ring_counts_starts_at_the_ring_ratio():
    on_ring = np.array([at_ring_ratio(cc.OUTER_RING, q) for q in ("tl", "tr", "bl", "br")])
    inside = np.array([at_ring_ratio(cc.OUTER_RING - 0.01, q) for q in ("tl", "tr", "bl", "br")])
    assert cc.outer_ring_counts([on_ring], SHAPE) == (1, 1, 1, 1)
    assert cc.outer_ring_counts([inside], SHAPE) == (0, 0, 0, 0)


def test_outer_ring_counts_ignores_the_middle_and_empty_input():
    middle = np.array([[640.0, 360.0], [300.0, 200.0], [1000.0, 500.0]])
    assert cc.outer_ring_counts([middle], SHAPE) == (0, 0, 0, 0)
    assert cc.outer_ring_counts([], SHAPE) == (0, 0, 0, 0)


def test_outer_ring_counts_matches_a_brute_force_count_at_scale():
    rng = np.random.default_rng(11)
    views = [rng.uniform([0, 0], SIZE, size=(70, 2)) for _ in range(2000)]
    start = time.perf_counter()
    counts = cc.outer_ring_counts(views, SHAPE)
    elapsed = time.perf_counter() - start
    w, h = SIZE
    expected = [0, 0, 0, 0]
    for view in views:
        for x, y in view:
            if np.hypot(x - w / 2, y - h / 2) >= cc.OUTER_RING * np.hypot(w / 2, h / 2):
                expected[(2 if y >= h / 2 else 0) + (1 if x >= w / 2 else 0)] += 1
    assert counts == tuple(expected)
    assert elapsed < 5.0
