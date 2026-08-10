"""Tests for TelemetryRepo: the single-slot caches behind the telemetry WS.

The seq-gating convention (return None unless newer than ``after_seq``) is what
lets the 20 Hz WebSocket loop poll without resending unchanged samples, and the
path TTL is the only thing that ever clears a finished route from the
operator's map — the ROS side never publishes an empty plan (see the module
comment on ``_PATH_TTL_S``). Time is controlled by patching ``time.monotonic``
at the repo's import site, so the 6 s TTL is exercised without sleeping.
"""

from unittest.mock import patch

import pytest

from syncai_backend.repositories.telemetry.telemetry import (
    _PATH_TTL_S,
    TelemetryRepo,
)


MONOTONIC = "syncai_backend.repositories.telemetry.telemetry.time.monotonic"


@pytest.fixture
def repo(logger) -> TelemetryRepo:
    return TelemetryRepo(logger)


class TestSeqGating:
    def test_empty_repo_answers_none(self, repo):
        assert repo.get_pose() is None
        assert repo.get_joints() is None
        assert repo.get_path() is None

    def test_pose_is_sent_once_per_sample(self, repo):
        repo.update_pose(x=1.0, y=2.0, z=0.0, yaw_deg=90.0, stamp=10.0)

        sample = repo.get_pose(after_seq=0)
        assert (sample.x, sample.yaw_deg) == (1.0, 90.0)
        # The WS loop hands back the seq it sent; the same sample must not
        # be offered twice.
        assert repo.get_pose(after_seq=sample.seq) is None

        repo.update_pose(x=1.1, y=2.0, z=0.0, yaw_deg=91.0, stamp=10.05)
        assert repo.get_pose(after_seq=sample.seq).x == 1.1

    def test_streams_advance_independently(self, repo):
        # A joints frame must not force a resend of an unchanged pose.
        repo.update_pose(x=1.0, y=2.0, z=0.0, yaw_deg=0.0, stamp=10.0)
        pose_seq = repo.get_pose().seq

        repo.update_joints(joints={"FL_HipX_joint": 0.5}, stamp=10.0)

        assert repo.get_pose(after_seq=pose_seq) is None
        assert repo.get_joints().joints == {"FL_HipX_joint": 0.5}


class TestPathTtl:
    def test_a_fresh_path_replays_until_consumed(self, repo):
        with patch(MONOTONIC, return_value=100.0):
            repo.update_path(points=((0.0, 0.0), (1.0, 1.0)), stamp=10.0)
            sample = repo.get_path(after_seq=0)
            assert sample.points == ((0.0, 0.0), (1.0, 1.0))
            assert repo.get_path(after_seq=sample.seq) is None

    def test_a_stale_path_expires_into_one_empty_sample(self, repo):
        with patch(MONOTONIC, return_value=100.0):
            repo.update_path(points=((0.0, 0.0),), stamp=10.0)
            seq = repo.get_path(after_seq=0).seq

        # Past the TTL the silence is turned into an explicit "no route":
        # exactly one empty sample with a bumped seq, so every connected
        # client erases the band it is drawing — then quiet.
        with patch(MONOTONIC, return_value=100.0 + _PATH_TTL_S + 1.0):
            expired = repo.get_path(after_seq=seq)
            assert expired.points == ()
            assert expired.seq == seq + 1
            assert repo.get_path(after_seq=expired.seq) is None

    def test_expiry_is_idempotent(self, repo):
        # Once empty, the branch must not keep bumping seq on every poll —
        # that would re-send "no route" to every client at 20 Hz forever.
        with patch(MONOTONIC, return_value=100.0):
            repo.update_path(points=((0.0, 0.0),), stamp=10.0)

        with patch(MONOTONIC, return_value=200.0):
            expired = repo.get_path(after_seq=0)
            assert expired.points == ()
            assert repo.get_path(after_seq=expired.seq) is None
            assert repo.get_path(after_seq=expired.seq) is None

    def test_a_replan_revives_the_route(self, repo):
        with patch(MONOTONIC, return_value=100.0):
            repo.update_path(points=((0.0, 0.0),), stamp=10.0)
            first = repo.get_path(after_seq=0)

        with patch(MONOTONIC, return_value=200.0):
            expired = repo.get_path(after_seq=first.seq)
            assert expired.points == ()

            # The planner replans: the fresh route supersedes the expiry.
            repo.update_path(points=((5.0, 5.0),), stamp=110.0)
            revived = repo.get_path(after_seq=expired.seq)
            assert revived.points == ((5.0, 5.0),)
