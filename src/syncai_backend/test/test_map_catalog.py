"""Tests for the on-disk map catalogue repository.

Real directories via the ``maps_dir`` fixture; nothing is mocked. The INI lookup
is redirected with monkeypatch.setenv so no test depends on the host having a
~/robot_ws.
"""

import os

import pytest

pytest.importorskip("yaml")

from syncai_backend.exceptions import BadRequestError, NotFoundError  # noqa: E402
from syncai_backend.helpers.system_config import (  # noqa: E402
    SYSTEM_INI_ENV,
)
from syncai_backend.repositories.map.catalog import (  # noqa: E402
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
    repo = init_map_catalog_repo(logger=logger)
    repo.maps_dir = str(tmp_path / "absent")

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


def test_gridmap_yaml_path_is_none_without_a_yaml(catalog_repo):
    assert catalog_repo.gridmap_yaml_path("rawonly") is None

    path = catalog_repo.gridmap_yaml_path("full")
    assert path.endswith("full/gridmap.yaml")
    # What map_server's LoadMap needs: it resolves the yaml's relative image key
    # against dirname() of the string it was handed, unexpanded.
    assert os.path.isabs(path)
    assert "~" not in path


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


# --- write_gridmap ----------------------------------------------------------


def test_write_gridmap_replaces_the_cells(catalog_repo, maps_dir):
    path = maps_dir / "full" / "gridmap.pgm"

    catalog_repo.write_gridmap("full", b"\x00" * 24)

    assert path.read_bytes() == b"P5\n6 4\n255\n" + b"\x00" * 24
    assert catalog_repo.get_map("full").grid.width == 6


def test_write_gridmap_returns_the_file_contents(catalog_repo, maps_dir):
    """The REST layer hashes this for the ETag."""
    written = catalog_repo.write_gridmap("full", b"\x00" * 24)

    assert written == (maps_dir / "full" / "gridmap.pgm").read_bytes()


def test_write_gridmap_creates_the_raw_backup_once(catalog_repo, maps_dir):
    """gridmap_raw.pgm must keep holding the pcd_to_gridmap.py output.

    Re-taking it on every save would, on the second save, replace the pristine
    conversion output with the first save's edit — losing the only way back.
    """
    pristine = (maps_dir / "full" / "gridmap.pgm").read_bytes()
    raw = maps_dir / "full" / "gridmap_raw.pgm"

    catalog_repo.write_gridmap("full", b"\x00" * 24)
    assert raw.read_bytes() == pristine

    catalog_repo.write_gridmap("full", b"\xfe" * 24)
    assert raw.read_bytes() == pristine


def test_write_gridmap_rejects_a_wrong_length_body(catalog_repo, maps_dir):
    path = maps_dir / "full" / "gridmap.pgm"
    before = path.read_bytes()

    with pytest.raises(BadRequestError):
        catalog_repo.write_gridmap("full", b"\x00" * 23)

    assert path.read_bytes() == before


def test_write_gridmap_without_a_gridmap_raises_not_found(catalog_repo, maps_dir):
    """And must not create one: a pgm with no yaml is an invisible half-map."""
    with pytest.raises(NotFoundError):
        catalog_repo.write_gridmap("rawonly", b"\x00" * 24)

    assert not (maps_dir / "rawonly" / "gridmap.pgm").exists()


def test_write_gridmap_rejects_an_unsafe_name(catalog_repo):
    with pytest.raises(BadRequestError):
        catalog_repo.write_gridmap("../etc", b"\x00" * 24)


def test_write_gridmap_leaves_gridmap_yaml_untouched(catalog_repo, maps_dir):
    """The geometry is a property of the conversion, never of a save."""
    yaml_path = maps_dir / "full" / "gridmap.yaml"
    before = yaml_path.read_bytes()

    catalog_repo.write_gridmap("full", b"\x00" * 24)

    assert yaml_path.read_bytes() == before
