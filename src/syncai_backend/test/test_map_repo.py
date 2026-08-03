"""Unit tests for MapRepo: vertex CRUD against the SQLite-backed fixture.

The repo used to also cache the live map topic's OccupancyGrid; that went with
the endpoints that read it, and with it this module's need for nav_msgs.
"""

import uuid

_MISSING_ID = uuid.UUID("00000000-0000-0000-0000-000000000000")


def _create(repo, name="v", type="GENERAL", map_name="warehouse", x=1.0, y=2.0,
            theta=0.0):
    """Create a single vertex through the batch API and return it."""
    return repo.create_vertices([{
        "name": name, "type": type, "map_name": map_name, "x": x, "y": y,
        "theta": theta,
    }])[0]


def test_create_returns_persisted_vertex(map_repo):
    vertex = _create(map_repo, name="dock", x=3.0, y=-1.5, theta=90.0)

    assert vertex.id is not None
    assert vertex.name == "dock"
    assert vertex.type == "GENERAL"
    assert vertex.map_name == "warehouse"
    assert (vertex.x, vertex.y, vertex.theta) == (3.0, -1.5, 90.0)
    assert vertex.created_at is not None
    assert vertex.updated_at is not None


def test_create_vertices_batch_persists_all_in_order(map_repo):
    created = map_repo.create_vertices([
        {"name": "a", "type": "GENERAL", "map_name": "warehouse", "x": 0.0,
         "y": 0.0, "theta": 0.0},
        {"name": "b", "type": "ARTIFACT", "map_name": "warehouse", "x": 1.0,
         "y": 1.0, "theta": 0.0},
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


def test_list_vertices_filters_by_map_name_and_type(map_repo):
    _create(map_repo, name="g1", type="GENERAL", map_name="warehouse")
    _create(map_repo, name="a1", type="ARTIFACT", map_name="warehouse")
    _create(map_repo, name="g2", type="GENERAL", map_name="office")

    assert len(map_repo.list_vertices(map_name="warehouse")) == 2
    assert len(map_repo.list_vertices(map_name="office")) == 1
    assert len(map_repo.list_vertices(type="GENERAL")) == 2
    assert len(map_repo.list_vertices(map_name="warehouse", type="ARTIFACT")) == 1
    assert map_repo.list_vertices(map_name="does-not-exist") == []


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
