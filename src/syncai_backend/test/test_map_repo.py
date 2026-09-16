"""Unit tests for MapRepo: vertex CRUD against the SQLite-backed fixture.

The repo used to also cache the live map topic's OccupancyGrid; that went with
the endpoints that read it, and with it this module's need for nav_msgs.
"""

import uuid

_MISSING_ID = uuid.UUID("00000000-0000-0000-0000-000000000000")


def _create(repo, name="v", type="GENERAL", map="warehouse", x=1.0, y=2.0,
            theta=0.0):
    """Create a single vertex through the batch API and return it."""
    return repo.create_vertices(map=map, vertices=[{
        "name": name, "type": type, "x": x, "y": y, "theta": theta,
    }])[0]


def test_create_returns_persisted_vertex(map_repo):
    vertex = _create(map_repo, name="dock", x=3.0, y=-1.5, theta=90.0)

    assert vertex.id is not None
    assert vertex.name == "dock"
    assert vertex.type == "GENERAL"
    assert vertex.map == "warehouse"
    assert (vertex.x, vertex.y, vertex.theta) == (3.0, -1.5, 90.0)
    assert vertex.created_at is not None
    assert vertex.updated_at is not None


def test_create_vertices_batch_persists_all_in_order(map_repo):
    created = map_repo.create_vertices(map="warehouse", vertices=[
        {"name": "a", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
        {"name": "b", "type": "ARTIFACT", "x": 1.0, "y": 1.0, "theta": 0.0},
    ])

    assert [v.name for v in created] == ["a", "b"]
    assert all(isinstance(v.id, uuid.UUID) for v in created)
    assert {v.id for v in map_repo.list_vertices()} == {c.id for c in created}


def test_get_returns_vertex_and_none_when_missing(map_repo):
    created = _create(map_repo)

    fetched = map_repo.get_vertex(created.id)
    assert fetched is not None
    assert fetched.id == created.id

    assert map_repo.get_vertex(_MISSING_ID) is None


def test_list_vertices_orders_by_creation_time(map_repo):
    first = _create(map_repo, name="a")
    second = _create(map_repo, name="b")

    vertices = map_repo.list_vertices()
    assert [v.id for v in vertices] == [first.id, second.id]


def test_list_vertices_filters_by_map_and_type(map_repo):
    _create(map_repo, name="g1", type="GENERAL", map="warehouse")
    _create(map_repo, name="a1", type="ARTIFACT", map="warehouse")
    _create(map_repo, name="g2", type="GENERAL", map="office")

    assert len(map_repo.list_vertices(map="warehouse")) == 2
    assert len(map_repo.list_vertices(map="office")) == 1
    assert len(map_repo.list_vertices(type="GENERAL")) == 2
    assert len(map_repo.list_vertices(map="warehouse", type="ARTIFACT")) == 1
    assert map_repo.list_vertices(map="does-not-exist") == []


def test_update_changes_fields(map_repo):
    created = _create(map_repo, name="old", x=1.0)

    updated = map_repo.update_vertex(created.id, name="new", x=5.0)

    assert updated is not None
    assert updated.name == "new"
    assert updated.x == 5.0
    # Untouched fields are preserved.
    assert updated.y == 2.0


def test_update_ignores_unknown_and_none_fields(map_repo):
    created = _create(map_repo, name="keep")

    updated = map_repo.update_vertex(
        created.id, name=None, bogus="value", theta=45.0
    )

    assert updated is not None
    assert updated.name == "keep"  # None ignored
    assert updated.theta == 45.0
    assert not hasattr(updated, "bogus")


def test_update_missing_returns_none(map_repo):
    assert map_repo.update_vertex(_MISSING_ID, name="x") is None


def test_delete_removes_vertex(map_repo):
    created = _create(map_repo)

    assert map_repo.delete_vertex(created.id) is True
    assert map_repo.get_vertex(created.id) is None


def test_delete_missing_returns_false(map_repo):
    assert map_repo.delete_vertex(_MISSING_ID) is False


def test_move_vertices_rekeys_only_that_map(map_repo):
    a = _create(map_repo, name="a", map="old")
    b = _create(map_repo, name="b", map="old")
    other = _create(map_repo, name="c", map="other")

    moved = map_repo.move_vertices("old", "new")

    assert moved == 2
    assert {v.id for v in map_repo.list_vertices(map="new")} == {a.id, b.id}
    assert map_repo.list_vertices(map="old") == []
    assert map_repo.get_vertex(other.id).map == "other"


def test_move_vertices_bumps_updated_at_and_keeps_ids(map_repo):
    """A Core UPDATE skips ``onupdate``; the repo sets updated_at itself."""
    created = _create(map_repo, map="old")
    # Read back before moving: SQLite hands timestamps back naive, so the
    # comparison has to be between two rows that took the same path.
    before = map_repo.get_vertex(created.id)

    map_repo.move_vertices("old", "new")

    after = map_repo.get_vertex(created.id)
    assert after.id == before.id
    assert after.created_at == before.created_at
    assert after.updated_at >= before.updated_at


def test_move_vertices_of_an_unknown_map_moves_nothing(map_repo):
    _create(map_repo, map="warehouse")

    assert map_repo.move_vertices("ghost", "new") == 0
    assert len(map_repo.list_vertices(map="warehouse")) == 1
