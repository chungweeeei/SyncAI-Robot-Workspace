# Launch the costmap filter info server together with the filter mask server
# (a second map_server instance publishing the keepout mask OccupancyGrid).
#
# robot_id is read from the system config INI at launch time (same convention
# as map_server.launch.py) and is used as the node namespace, prefixing the
# relative topics (costmap_filter_info -> /<robot_id>/costmap_filter_info,
# keepout_filter_mask -> /<robot_id>/keepout_filter_mask). The published
# mask's frame_id stays "map" (the shared global frame).
#
# WHERE THE MASK COMES FROM. The keepout mask lives with the map it belongs to:
# `map/<name>/keepout.yaml` (+ `keepout.pgm`), an ordinary map-server YAML/image
# pair in the same directory as `gridmap.yaml`. The path is derived from the
# INI's `[map] map` value (its directory + "keepout.yaml"), so switching maps
# switches the keepout with it and the instance INI needs no new key -- the INI
# is a file the backend writes, so adding a key there would be a cross-repo
# contract change. Like `[map] map` itself the path is relative to the workspace
# root, which is every pane's cwd. A `mask_yaml:=` argument overrides the
# derivation for tests and by-hand runs.
#
# A MISSING MASK IS GENERATED, NOT SKIPPED. map_server throws in its constructor
# when the file is absent, and no map had a keepout when the filter was wired
# in (2026-09), so the first version of this launch returned an empty
# description when `keepout.yaml` was missing -- the same pattern as
# syncai_localizer for a missing `[map] pcd`. That was replaced at the end of
# the month: when the derived `keepout.yaml` does not exist, the launch reads
# the map's `gridmap.yaml` (+ image header) and writes a blank mask of the same
# geometry next to it -- same width, height, resolution and origin -- and then
# starts both nodes on it. Two reasons:
#
#   1. The mask server should be up on every map, so `filter_mask_server/
#      load_map` is always there to call. The keepout editor (the console, via
#      the backend) needs a same-geometry canvas to draw on and a live server to
#      reload; "start the pane by hand after the first mask is drawn" is not a
#      workflow.
#   2. An empty pane and a planner log that warns "Filter mask was not
#      received" every 2 s were two ways of saying "no keepout" that both looked
#      like failures. Now both are silent on a map without zones.
#
# WHY THE BLANK MASK IS GREY (UNKNOWN), NOT WHITE (FREE). KeepoutFilter::process
# skips mask cells that are NO_INFORMATION, but a FREE mask cell overwrites a
# costmap cell that is NO_INFORMATION (`data > old_data || old_data ==
# NO_INFORMATION`, keepout_filter.cpp -- same as upstream nav2). An all-white
# mask would therefore turn every unknown cell inside the map's bounding box
# into free space, and the planner would happily route through unexplored
# grey. All-grey (pixel 205, the trinary "unknown" value map_saver writes) is a
# true no-op: every cell is skipped. Anyone drawing on this canvas should keep
# that rule in mind -- black = keepout (lethal), grey = no opinion, white =
# force free, which overrides unknown and is almost never what you want.
#
# The mask is written image-first, each file via tmp + rename, so a reader that
# sees `keepout.yaml` always finds the image it names, and a crash mid-write
# leaves no half file behind. Nothing in either file holds a path: the YAML
# says `image: keepout.pgm`, so a map rename stays one os.rename() -- the
# property the rest of map/<name>/ keeps and the backend relies on. An existing
# `keepout.yaml` is never touched, whatever its geometry: it is operator data.
#
# The one case that still starts nothing is a map whose `gridmap.yaml` is
# missing or unreadable -- there is nothing to size the mask from, and the
# map_server pane next door is dying on the same file anyway.
#
# Filters run AFTER the layer stack, inflation included, so the KeepoutFilter
# inflates the mask itself with the costmap's footprint and inflation
# parameters (2026-10). A mask is the forbidden area only -- no margin (see
# syncai_costmap_2d's README).

import configparser
import os
import re
from typing import Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Absolute path so the INI resolves no matter what cwd the launch is started
# from — the old relative path only worked because every entrypoint happened
# to run from the workspace root. ~/robot_ws is the workspace inside the robot
# container, where docker-compose bind-mounts the per-robot instance INI over
# config/system.ini.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"
# Basenames of the keepout mask pair inside a map directory (next to
# gridmap.yaml). The image is resolved by map_io relative to the YAML, so
# nothing inside the map directory holds a path that would break a map rename
# -- the same property the rest of map/<name>/ keeps, and one the backend
# relies on.
KEEPOUT_YAML_BASENAME = "keepout.yaml"
KEEPOUT_IMAGE_BASENAME = "keepout.pgm"
# Trinary-mode "unknown" pixel (what map_saver writes for -1 cells). With the
# thresholds below it loads as -1, which KeepoutFilter skips -- see the header
# comment for why the blank mask is unknown rather than free.
BLANK_MASK_PIXEL = 205
# The trinary thresholds map_saver emits; kept identical so a mask drawn with
# the same 0 / 205 / 254 palette as the gridmap loads the same way.
MASK_OCCUPIED_THRESH = 0.65
MASK_FREE_THRESH = 0.196

logger = launch_logging.get_logger("costmap_filter_info.launch")


def read_robot_id(config_path: str) -> str:
    config = configparser.ConfigParser()
    if not config.read(config_path):
        logger.warning(
            f"System config '{config_path}' not found; "
            f"falling back to robot_id '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    robot_id = config.get("system", "robot_id", fallback="").strip()
    if not robot_id:
        logger.warning(
            f"No [system] robot_id in '{config_path}'; "
            f"falling back to '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    return robot_id


def read_map_yaml(config_path: str) -> str:
    """Return the INI's [map] map value (the gridmap YAML), or "" when absent."""
    config = configparser.ConfigParser()
    if not config.read(config_path):
        return ""
    return config.get("map", "map", fallback="").strip()


def read_image_size(image_path: str) -> Tuple[int, int]:
    """Return (width, height) of a map image.

    Binary and ASCII PGM (P5 / P2) headers are parsed by hand so the common
    case -- every gridmap the backend writes is a P5 -- needs nothing beyond
    the standard library. Anything else goes through Pillow when it is
    installed; the error otherwise names the file so the operator can convert
    it (map_io itself reads the image through GraphicsMagick, which has no
    Python binding here).
    """
    with open(image_path, "rb") as f:
        head = f.read(64)
    if head[:2] in (b"P5", b"P2"):
        with open(image_path, "rb") as f:
            # Header = magic, width, height, maxval, separated by whitespace,
            # with '#' comments allowed anywhere between tokens. Read enough
            # bytes to cover any sane header; the data follows the maxval.
            blob = f.read(4096)
        tokens = []
        pos = 0
        while len(tokens) < 4 and pos < len(blob):
            m = re.compile(rb"\s*(#[^\n]*\n\s*)*(\S+)").match(blob, pos)
            if not m:
                break
            tokens.append(m.group(2))
            pos = m.end()
        if len(tokens) < 4:
            raise ValueError(f"'{image_path}': truncated PGM header")
        return int(tokens[1]), int(tokens[2])

    try:
        from PIL import Image
    except ImportError as exc:  # pragma: no cover - depends on the image
        raise ValueError(
            f"'{image_path}' is not a PGM and Pillow is not installed to read its size"
        ) from exc
    with Image.open(image_path) as img:
        return img.size


def read_gridmap_geometry(map_yaml: str) -> Optional[dict]:
    """Read width / height / resolution / origin from a map-server YAML.

    Returns None (after logging why) when the YAML or its image cannot be read.
    """
    try:
        with open(map_yaml) as f:
            meta = yaml.safe_load(f) or {}
        image = str(meta["image"])
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(map_yaml), image)
        width, height = read_image_size(image)
        origin = [float(v) for v in meta["origin"]]
        if len(origin) != 3:
            raise ValueError(f"origin has {len(origin)} components, expected 3")
        return {
            "width": width,
            "height": height,
            "resolution": float(meta["resolution"]),
            "origin": origin,
        }
    except (OSError, KeyError, ValueError, TypeError, yaml.YAMLError) as exc:
        logger.error(f"Cannot read gridmap geometry from '{map_yaml}': {exc}")
        return None


def write_blank_mask(keepout_yaml: str, geometry: dict) -> None:
    """Write an all-unknown keepout mask (YAML + PGM) with the given geometry.

    Image first, each via tmp + rename: a reader that sees the YAML always
    finds the image, and a crash leaves no half-written file for map_server to
    throw on next boot.
    """
    map_dir = os.path.dirname(keepout_yaml)
    image_path = os.path.join(map_dir, KEEPOUT_IMAGE_BASENAME)
    width, height = geometry["width"], geometry["height"]

    tmp_image = image_path + ".tmp"
    with open(tmp_image, "wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode())
        f.write(bytes([BLANK_MASK_PIXEL]) * (width * height))
    os.replace(tmp_image, image_path)

    ox, oy, oyaw = geometry["origin"]
    tmp_yaml = keepout_yaml + ".tmp"
    with open(tmp_yaml, "w") as f:
        f.write(
            "# Keepout (forbidden-zone) mask for the planner's global costmap.\n"
            "# Generated blank by syncai_map_server's costmap_filter_info.launch.py\n"
            "# with the geometry of gridmap.yaml at the time. Pixel semantics when\n"
            "# editing: black = keepout (lethal), grey = no opinion, white = force\n"
            "# free (overrides unknown cells of the map -- almost never wanted).\n"
            "# Draw the forbidden area only, no margin: the costmap's KeepoutFilter\n"
            "# inflates it with the robot footprint. Reload without a restart via\n"
            "# /<robot_id>/filter_mask_server/load_map.\n"
            f"image: {KEEPOUT_IMAGE_BASENAME}\n"
            "mode: trinary\n"
            f"resolution: {geometry['resolution']}\n"
            f"origin: [{ox}, {oy}, {oyaw}]\n"
            "negate: 0\n"
            f"occupied_thresh: {MASK_OCCUPIED_THRESH}\n"
            f"free_thresh: {MASK_FREE_THRESH}\n"
        )
    os.replace(tmp_yaml, keepout_yaml)


def ensure_keepout_mask(keepout_yaml: str, map_yaml: str) -> bool:
    """Make sure `keepout_yaml` exists, generating a blank one from `map_yaml`.

    Returns False when there is nothing to serve: no mask and no readable
    gridmap to size one from.
    """
    if os.path.isfile(keepout_yaml):
        return True

    geometry = read_gridmap_geometry(map_yaml)
    if geometry is None:
        logger.error(
            f"Keepout mask '{keepout_yaml}' does not exist and no blank one can be "
            f"generated without a readable '{map_yaml}'; nothing to launch."
        )
        return False

    try:
        write_blank_mask(keepout_yaml, geometry)
    except OSError as exc:
        logger.error(f"Cannot write blank keepout mask '{keepout_yaml}': {exc}")
        return False

    logger.info(
        f"Generated blank keepout mask '{keepout_yaml}' "
        f"({geometry['width']}x{geometry['height']} @ {geometry['resolution']} m, "
        f"origin {geometry['origin']}) -- all unknown, so it changes nothing "
        "until zones are drawn into it"
    )
    return True


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we
    # need the resolved robot_id here to namespace the nodes.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    params_file = LaunchConfiguration("params_file")

    # Mask path precedence: explicit mask_yaml:= argument > derived from the
    # INI's [map] map > the params file. An explicit path names a file its
    # author maintains, so it must exist; the derived path is generated blank
    # when missing (see the header). The params-file fallback is left to
    # map_server itself (it throws on a bad path), because that path only
    # applies when the node is being run outside the robot convention and its
    # author knows why.
    keepout_yaml = LaunchConfiguration("mask_yaml").perform(context).strip()
    if keepout_yaml:
        if not os.path.isfile(keepout_yaml):
            logger.error(
                f"Keepout mask '{keepout_yaml}' (mask_yaml:=) does not exist; "
                "nothing to launch. Only the path derived from the INI's [map] map "
                "is generated when missing."
            )
            return []
    else:
        map_yaml = read_map_yaml(config_path)
        if map_yaml:
            keepout_yaml = os.path.join(os.path.dirname(map_yaml), KEEPOUT_YAML_BASENAME)
            if not ensure_keepout_mask(keepout_yaml, map_yaml):
                return []
        else:
            logger.info(
                f"No [map] map in '{config_path}'; "
                "using the yaml_filename from the params file for the keepout mask"
            )

    mask_overrides = []
    if keepout_yaml:
        logger.info(f"Keepout mask yaml: {keepout_yaml}")
        # Later entries in the parameters list take precedence over the params
        # file, exactly as map_server.launch.py layers [map] map on top of it.
        mask_overrides = [{"yaml_filename": keepout_yaml}]

    costmap_filter_info_server_node = Node(
        package="syncai_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        namespace=robot_id,
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
    )

    # Second map_server instance publishing the filter mask OccupancyGrid.
    filter_mask_server_node = Node(
        package="syncai_map_server",
        executable="map_server",
        name="filter_mask_server",
        namespace=robot_id,
        output="screen",
        emulate_tty=True,
        parameters=[params_file, *mask_overrides],
    )

    return [costmap_filter_info_server_node, filter_mask_server_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_map_server")
    default_params_file = os.path.join(
        pkg_share, "params", "costmap_filter_info_params.yaml"
    )

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the ROS2 parameters file for both nodes",
    )

    declare_mask_yaml = DeclareLaunchArgument(
        "mask_yaml",
        default_value="",
        description=(
            "Keepout mask YAML to serve. Empty (the default) derives "
            "<dirname of [map] map>/keepout.yaml from the system INI and writes "
            "a blank (all-unknown) mask of the gridmap's geometry there when it "
            "does not exist yet. An explicit path must already exist."
        ),
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            declare_mask_yaml,
            OpaqueFunction(function=launch_setup),
        ]
    )
