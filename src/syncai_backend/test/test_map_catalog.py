"""Tests for the on-disk map catalogue repository.

Real directories via the ``maps_dir`` fixture; nothing is mocked. The INI lookup
is redirected with monkeypatch.setenv so no test depends on the host having a
~/robot_ws.
"""

import os

import pytest

pytest.importorskip("yaml")

from syncai_backend.exceptions import BadRequestError  # noqa: E402
from syncai_backend.helpers.system_config import (  # noqa: E402
    SYSTEM_INI_ENV,
)
from syncai_backend.repositories.map.catalog import (  # noqa: E402
    MAPS_DIR_ENV,
    init_map_catalog_repo,
)


def _names(maps):
    return [stored.name for stored in maps]


# --- list_maps --------------------------------------------------------------


def test_lists_directories_only(catalog_repo):
    """The loose stray.pgm at the root is not a map under this layout."""
    assert _names(catalog_repo.list_maps()) == ["full", "rawonly"]


def test_converted_map_reports_its_geometry(catalog_repo):
    full = catalog_repo.get_map("full")

    assert full.grid is not None
    assert full.grid.width == 6
    assert full.grid.height == 4
    assert full.grid.resolution == pytest.approx(0.05)
    assert full.grid.origin == pytest.approx((-6.94, -11.09, 0.0))
    assert full.has_pointcloud is True


def test_unconverted_map_has_no_grid(catalog_repo):
    """pgo/save_maps has run but pcd_to_gridmap.py has not."""
    rawonly = catalog_repo.get_map("rawonly")

    assert rawonly.grid is None
    assert rawonly.has_pointcloud is True


def test_size_and_mtime_cover_the_directory(catalog_repo, maps_dir):
    full = catalog_repo.get_map("full")

    on_disk = sum(
        os.path.getsize(maps_dir / "full" / name)
        for name in os.listdir(maps_dir / "full")
    )
    assert full.size_bytes == on_disk
    assert full.modified_at.tzinfo is not None


def test_missing_map_returns_none(catalog_repo):
    assert catalog_repo.get_map("nosuchmap") is None


def test_missing_maps_dir_lists_empty(logger, tmp_path):
    repo = init_map_catalog_repo(logger=logger, maps_dir=str(tmp_path / "absent"))

    assert repo.list_maps() == []


def test_unreadable_gridmap_degrades_to_no_grid(catalog_repo, maps_dir):
    """One torn gridmap must not take the whole listing down."""
    (maps_dir / "full" / "gridmap.pgm").write_bytes(b"P5\n999 999\n255\nshort")

    assert catalog_repo.get_map("full").grid is None
    assert _names(catalog_repo.list_maps()) == ["full", "rawonly"]


def test_gridmap_yaml_without_pgm_has_no_grid(catalog_repo, maps_dir):
    (maps_dir / "full" / "gridmap.pgm").unlink()

    assert catalog_repo.get_map("full").grid is None


# --- resolve_dir / gridmap_path ---------------------------------------------


@pytest.mark.parametrize(
    "name",
    [
        "..",
        ".",
        "../etc",
        "sub/dir",
        "/etc/passwd",
        "with space",
        "",
        "a" * 65,
        "full/../../etc",
    ],
)
def test_resolve_rejects_unsafe_names(catalog_repo, name):
    with pytest.raises(BadRequestError):
        catalog_repo.resolve_dir(name)


def test_resolve_rejects_symlink_escaping_the_root(catalog_repo, maps_dir, tmp_path):
    """The name pattern cannot see through a link; the realpath check must."""
    outside = tmp_path / "outside"
    outside.mkdir()
    os.symlink(outside, maps_dir / "escape")

    with pytest.raises(BadRequestError):
        catalog_repo.resolve_dir("escape")


def test_gridmap_path_is_none_without_a_gridmap(catalog_repo):
    assert catalog_repo.gridmap_path("rawonly") is None
    assert catalog_repo.gridmap_path("full").endswith("full/gridmap.pgm")


# --- active_name ------------------------------------------------------------


def test_active_name_from_ini(catalog_repo, tmp_path, monkeypatch):
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: full\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))

    assert catalog_repo.active_name() == "full"


def test_active_name_falls_back_to_the_map_path(catalog_repo, tmp_path, monkeypatch):
    """Older instance files set only the derived paths."""
    ini = tmp_path / "system.ini"
    ini.write_text("[map]\nmap: map/full/gridmap.yaml\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))

    assert catalog_repo.active_name() == "full"


def test_active_name_is_none_without_an_ini(catalog_repo, tmp_path, monkeypatch):
    """A robot with no INI must still be able to list its maps."""
    monkeypatch.setenv(SYSTEM_INI_ENV, str(tmp_path / "absent.ini"))

    assert catalog_repo.active_name() is None
    assert _names(catalog_repo.list_maps()) == ["full", "rawonly"]


def test_active_name_is_none_without_a_map_section(catalog_repo, tmp_path,
                                                   monkeypatch):
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))

    assert catalog_repo.active_name() is None


# --- init factory -----------------------------------------------------------


def test_env_overrides_the_default_maps_dir(logger, maps_dir, monkeypatch):
    monkeypatch.setenv(MAPS_DIR_ENV, str(maps_dir))

    repo = init_map_catalog_repo(logger=logger)

    assert _names(repo.list_maps()) == ["full", "rawonly"]


def test_explicit_maps_dir_wins_over_env(logger, maps_dir, tmp_path, monkeypatch):
    monkeypatch.setenv(MAPS_DIR_ENV, str(tmp_path / "ignored"))

    repo = init_map_catalog_repo(logger=logger, maps_dir=str(maps_dir))

    assert _names(repo.list_maps()) == ["full", "rawonly"]
