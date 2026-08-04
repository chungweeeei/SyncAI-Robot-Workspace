"""Tests for SavedTaskRepo against in-memory SQLite.

Storage-level only: the vertex resolution that gives a saved MOVE step its
dispatch coordinates lives in the router, and test_saved_task_router.py covers
it. What matters here is that the JSON step blob round-trips with its order and
its explicit nulls intact, and that the two places this repo deliberately
diverges from MapRepo actually behave differently.
"""

import uuid


def _steps(*ids):
    return [
        {
            "id": step_id,
            "type": "MOVE",
            "params": {"x": 1.0, "y": 2.0, "theta": 90.0},
            "vertex_id": None,
            "vertex_name": None,
        }
        for step_id in ids
    ]


def test_create_round_trips_the_step_blob(saved_task_repo):
    """Order, nesting and explicit nulls all survive the JSON column.

    The nulls matter as much as the values: a *missing* key is what signals "this
    row was written by an older backend", so an implementation that dropped Nones
    would destroy the forward-compatibility property the JSON storage exists for.
    """
    row = saved_task_repo.create_saved_task(
        name="patrol",
        description="two stops",
        map_name="dp2f",
        steps=_steps("1-move", "2-move") + [
            {
                "id": "3-standup",
                "type": "STANDUP",
                "params": None,
                "vertex_id": None,
                "vertex_name": None,
            }
        ],
    )

    assert isinstance(row.id, uuid.UUID)
    assert row.name == "patrol"
    assert row.map_name == "dp2f"

    stored = saved_task_repo.get_saved_task(task_id=row.id)
    assert [s["id"] for s in stored.steps] == ["1-move", "2-move", "3-standup"]
    assert stored.steps[0]["params"] == {"x": 1.0, "y": 2.0, "theta": 90.0}
    assert "vertex_id" in stored.steps[2] and stored.steps[2]["vertex_id"] is None
    assert stored.steps[2]["params"] is None


def test_map_independent_rows_persist_as_null(saved_task_repo):
    row = saved_task_repo.create_saved_task(
        name="stretch", description="", map_name=None, steps=_steps("1-move")
    )
    assert saved_task_repo.get_saved_task(task_id=row.id).map_name is None


def test_list_filter_includes_map_independent_rows(saved_task_repo):
    """A map filter answers that map's tasks *plus* the run-anywhere ones.

    This is the console's actual question, and it is why `map_name=None` cannot
    be overloaded as "no filter" the way MapRepo overloads its own -- here
    `map_name IS NULL` is a real, meaningful query.
    """
    saved_task_repo.create_saved_task(
        name="on-dp2f", description="", map_name="dp2f", steps=_steps("1-move")
    )
    saved_task_repo.create_saved_task(
        name="on-other", description="", map_name="other", steps=_steps("1-move")
    )
    saved_task_repo.create_saved_task(
        name="anywhere", description="", map_name=None, steps=_steps("1-move")
    )

    scoped = saved_task_repo.list_saved_tasks(map_name="dp2f")
    assert sorted(r.name for r in scoped) == ["anywhere", "on-dp2f"]

    strict = saved_task_repo.list_saved_tasks(
        map_name="dp2f", include_map_independent=False
    )
    assert [r.name for r in strict] == ["on-dp2f"]

    assert len(saved_task_repo.list_saved_tasks()) == 3


def test_list_is_ordered_by_name_then_created_at(saved_task_repo):
    """Stable, scannable order -- not most-recently-touched first.

    Names are not unique (deliberately: there is no migration path for a unique
    constraint), so created_at breaks the tie.
    """
    first = saved_task_repo.create_saved_task(
        name="same", description="first", map_name=None, steps=_steps("1-move")
    )
    second = saved_task_repo.create_saved_task(
        name="same", description="second", map_name=None, steps=_steps("1-move")
    )
    saved_task_repo.create_saved_task(
        name="aaa", description="", map_name=None, steps=_steps("1-move")
    )

    rows = saved_task_repo.list_saved_tasks()
    assert [r.name for r in rows] == ["aaa", "same", "same"]
    assert [r.id for r in rows[1:]] == [first.id, second.id]


def test_update_replaces_the_whole_step_list(saved_task_repo):
    row = saved_task_repo.create_saved_task(
        name="patrol", description="", map_name="dp2f", steps=_steps("1-move", "2-move")
    )
    updated = saved_task_repo.update_saved_task(
        task_id=row.id, steps=_steps("only-move")
    )
    assert [s["id"] for s in updated.steps] == ["only-move"]


def test_update_can_clear_map_name(saved_task_repo):
    """Regression test for the `and v is not None` filter that must NOT be
    copied from MapRepo.update_vertex.

    Clearing map_name is how a task becomes map-independent. Filtering Nones out
    of the changes dict -- which is correct for vertices, where every column is
    non-nullable -- would make that transition silently impossible here.
    """
    row = saved_task_repo.create_saved_task(
        name="patrol", description="", map_name="dp2f", steps=_steps("1-move")
    )
    updated = saved_task_repo.update_saved_task(task_id=row.id, map_name=None)
    assert updated.map_name is None
    assert saved_task_repo.get_saved_task(task_id=row.id).map_name is None


def test_update_ignores_unknown_columns(saved_task_repo):
    row = saved_task_repo.create_saved_task(
        name="patrol", description="", map_name=None, steps=_steps("1-move")
    )
    updated = saved_task_repo.update_saved_task(
        task_id=row.id, name="renamed", nonsense="x", id=uuid.uuid4()
    )
    assert updated.name == "renamed"
    assert updated.id == row.id


def test_update_moves_updated_at_but_not_created_at(saved_task_repo):
    row = saved_task_repo.create_saved_task(
        name="patrol", description="", map_name=None, steps=_steps("1-move")
    )
    created_at, updated_at = row.created_at, row.updated_at

    updated = saved_task_repo.update_saved_task(task_id=row.id, name="renamed")
    assert updated.created_at == created_at
    assert updated.updated_at >= updated_at


def test_missing_id_reads_as_none_and_deletes_as_false(saved_task_repo):
    """"Not found" is None/False here; the router is what raises NotFoundError."""
    unknown = uuid.uuid4()
    assert saved_task_repo.get_saved_task(task_id=unknown) is None
    assert saved_task_repo.update_saved_task(task_id=unknown, name="x") is None
    assert saved_task_repo.delete_saved_task(task_id=unknown) is False


def test_delete_is_idempotent_in_its_return_value(saved_task_repo):
    row = saved_task_repo.create_saved_task(
        name="patrol", description="", map_name=None, steps=_steps("1-move")
    )
    assert saved_task_repo.delete_saved_task(task_id=row.id) is True
    assert saved_task_repo.delete_saved_task(task_id=row.id) is False
