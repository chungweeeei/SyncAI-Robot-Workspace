"""Tests for helpers/system_config.py: which map is the active one.

``active_map_name`` is what the map catalogue's ``active`` flag and the saved-
task dispatch guard both hang off, and its contract is "never raises" — a robot
with no INI must still list the maps on its disk. The INI path is injected via
the ``SYNCAI_SYSTEM_INI`` env var the helper already honours.
"""

import pytest

from syncai_backend.helpers.system_config import active_map_name


@pytest.fixture
def ini(tmp_path, monkeypatch):
    """Write the given INI text and point the helper at it."""

    def _write(text: str):
        path = tmp_path / "system.ini"
        path.write_text(text)
        monkeypatch.setenv("SYNCAI_SYSTEM_INI", str(path))
        return path

    return _write


def test_the_name_key_is_canonical(logger, ini):
    ini("[map]\nname = dp2f\nmap = map/other/gridmap.yaml\n")
    assert active_map_name(logger) == "dp2f"


def test_falls_back_to_the_map_paths_directory(logger, ini):
    # Older instance files only set the derived paths; the map's name is the
    # directory the yaml sits in.
    ini("[map]\nmap = map/dp2f/gridmap.yaml\n")
    assert active_map_name(logger) == "dp2f"


def test_no_map_section_is_none(logger, ini):
    ini("[system]\nrobot_id = robot01\n")
    assert active_map_name(logger) is None


def test_missing_ini_is_none(logger, tmp_path, monkeypatch):
    monkeypatch.setenv("SYNCAI_SYSTEM_INI", str(tmp_path / "absent.ini"))
    assert active_map_name(logger) is None


def test_malformed_ini_is_none(logger, ini):
    # Never raises: a broken identity file must not take the catalogue down.
    ini("[map\nname = dp2f\n")
    assert active_map_name(logger) is None


def test_blank_values_fall_through(logger, ini):
    ini("[map]\nname =  \nmap = \n")
    assert active_map_name(logger) is None
