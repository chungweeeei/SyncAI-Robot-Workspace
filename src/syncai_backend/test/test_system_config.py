"""Tests for helpers/system_config.py: which map is the active one.

``active_map_name`` is what the map catalogue's ``active`` flag and the saved-
task dispatch guard both hang off, and its contract is "never raises" — a robot
with no INI must still list the maps on its disk. The INI path is injected via
the ``SYNCAI_SYSTEM_INI`` env var the helper already honours.

``set_active_map`` is the writer behind the maps library's Switch-map control,
and its contract is the opposite: it raises rather than half-write, because a
switch the INI does not record leaves the running stack and every REST answer
disagreeing about which map the robot is on. Most of what is pinned below is
about *not* disturbing the rest of the file — the interpolation in ``[map]``
above all, which is what keeps the next hand edit a one-line change.
"""

import configparser

import pytest

from syncai_backend.helpers.system_config import active_map_name, set_active_map


# The real shape of config/instances/robotNN.ini, interpolation and all.
_REAL_INI = (
    "[system]\n"
    "robot_id: robot01\n"
    "\n"
    "# which map the stack loads\n"
    "[map]\n"
    "name: dp2f\n"
    "pcd: map/%(name)s/map.pcd\n"
    "map: map/%(name)s/gridmap.yaml\n"
    "\n"
    "[initial_pose]\n"
    "x: 3.25\n"
    "y: -1.5\n"
    "z: 0.0\n"
    "yaw: 1.57\n"
    "\n"
    "[sensor.lidar]\n"
    "type: mid360s\n"
    "ip: 192.168.1.172\n"
)


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


# --- set_active_map ---------------------------------------------------------

def test_the_switch_is_a_one_line_change(logger, ini):
    path = ini(_REAL_INI)

    set_active_map("warehouse01", logger)

    written = path.read_text()
    assert "name: warehouse01" in written
    # The whole reason this edits lines instead of round-tripping through
    # ConfigParser.write(): expanding these would pin the derived paths at the
    # *old* map while `name` moved on, and permanently at that.
    assert "pcd: map/%(name)s/map.pcd" in written
    assert "map: map/%(name)s/gridmap.yaml" in written
    assert active_map_name(logger) == "warehouse01"


def test_it_zeroes_the_initial_pose(logger, ini):
    """A pose measured in the old map's frame is not meaningful in the new one."""
    path = ini(_REAL_INI)

    set_active_map("warehouse01", logger)

    pose = configparser.ConfigParser(interpolation=None)
    pose.read(path)
    assert dict(pose["initial_pose"]) == {
        "x": "0.0", "y": "0.0", "z": "0.0", "yaw": "0.0"
    }


def test_it_leaves_every_other_section_alone(logger, ini):
    path = ini(_REAL_INI)

    set_active_map("warehouse01", logger)

    config = configparser.ConfigParser(interpolation=None)
    config.read(path)
    assert config["system"]["robot_id"] == "robot01"
    # A wrong [sensor.lidar] type yields no point cloud and no error, so this
    # is not a section to churn on the way past.
    assert config["sensor.lidar"]["type"] == "mid360s"
    assert config["sensor.lidar"]["ip"] == "192.168.1.172"
    assert "# which map the stack loads" in path.read_text()


def test_it_keeps_the_files_own_delimiters(logger, ini):
    """`=` files stay `=` files; ConfigParser.write() would flatten both to ` = `."""
    path = ini("[map]\nname = dp2f\npcd = map/%(name)s/map.pcd\n")

    set_active_map("warehouse01", logger)

    assert "name = warehouse01" in path.read_text()
    assert "pcd = map/%(name)s/map.pcd" in path.read_text()


def test_a_missing_initial_pose_section_is_fine(logger, ini):
    # localizer_launch.py reads these with fallback=0.0 per key, so a file
    # without the section is already at the value being written.
    path = ini("[map]\nname: dp2f\n")

    set_active_map("warehouse01", logger)

    assert active_map_name(logger) == "warehouse01"
    assert "initial_pose" not in path.read_text()


def test_no_map_name_to_rewrite_raises(logger, ini):
    # Silently appending a [map] section would be a worse answer: this file is
    # the robot's identity, and a shape this helper does not recognise is not
    # one to start guessing at.
    ini("[system]\nrobot_id: robot01\n")

    with pytest.raises(ValueError, match="no \\[map\\] name"):
        set_active_map("warehouse01", logger)


def test_a_missing_ini_raises(logger, tmp_path, monkeypatch):
    """Unlike the reader, the writer must not shrug — the caller reports this."""
    monkeypatch.setenv("SYNCAI_SYSTEM_INI", str(tmp_path / "nope.ini"))

    with pytest.raises(OSError):
        set_active_map("warehouse01", logger)


def test_switching_twice_is_stable(logger, ini):
    """The second write must not compound the first — the file is bind-mounted."""
    path = ini(_REAL_INI)

    set_active_map("warehouse01", logger)
    first = path.read_text()
    set_active_map("dp2f", logger)

    assert path.read_text() == first.replace("name: warehouse01", "name: dp2f")
    assert active_map_name(logger) == "dp2f"
