"""Unit tests for MapPointRepo CRUD against an in-memory SQLite database."""


def _create(repo, name="p", type="waypoint", map_name="warehouse", x=1.0, y=2.0,
            theta=0.0):
    return repo.create(name=name, type=type, map_name=map_name, x=x, y=y, theta=theta)


def test_create_returns_persisted_point(map_point_repo):
    point = _create(map_point_repo, name="dock", x=3.0, y=-1.5, theta=90.0)

    assert point.id is not None
    assert point.name == "dock"
    assert point.type == "waypoint"
    assert point.map_name == "warehouse"
    assert (point.x, point.y, point.theta) == (3.0, -1.5, 90.0)
    assert point.created_at is not None
    assert point.updated_at is not None


def test_get_returns_point_and_none_when_missing(map_point_repo):
    created = _create(map_point_repo)

    fetched = map_point_repo.get(created.id)
    assert fetched is not None
    assert fetched.id == created.id

    assert map_point_repo.get(999999) is None


def test_list_all_orders_by_id(map_point_repo):
    first = _create(map_point_repo, name="a")
    second = _create(map_point_repo, name="b")

    points = map_point_repo.list_all()
    assert [p.id for p in points] == [first.id, second.id]


def test_list_all_filters_by_map_name_and_type(map_point_repo):
    _create(map_point_repo, name="w1", type="waypoint", map_name="warehouse")
    _create(map_point_repo, name="w2", type="patrol", map_name="warehouse")
    _create(map_point_repo, name="o1", type="waypoint", map_name="office")

    assert len(map_point_repo.list_all(map_name="warehouse")) == 2
    assert len(map_point_repo.list_all(map_name="office")) == 1
    assert len(map_point_repo.list_all(type="waypoint")) == 2
    assert len(map_point_repo.list_all(map_name="warehouse", type="patrol")) == 1
    assert map_point_repo.list_all(map_name="does-not-exist") == []


def test_update_changes_fields(map_point_repo):
    created = _create(map_point_repo, name="old", x=1.0)

    updated = map_point_repo.update(created.id, name="new", x=5.0)

    assert updated is not None
    assert updated.name == "new"
    assert updated.x == 5.0
    # Untouched fields are preserved.
    assert updated.y == 2.0


def test_update_ignores_unknown_and_none_fields(map_point_repo):
    created = _create(map_point_repo, name="keep")

    updated = map_point_repo.update(
        created.id, name=None, bogus="value", theta=45.0
    )

    assert updated is not None
    assert updated.name == "keep"  # None ignored
    assert updated.theta == 45.0
    assert not hasattr(updated, "bogus")


def test_update_missing_returns_none(map_point_repo):
    assert map_point_repo.update(999999, name="x") is None


def test_delete_removes_point(map_point_repo):
    created = _create(map_point_repo)

    assert map_point_repo.delete(created.id) is True
    assert map_point_repo.get(created.id) is None


def test_delete_missing_returns_false(map_point_repo):
    assert map_point_repo.delete(999999) is False
