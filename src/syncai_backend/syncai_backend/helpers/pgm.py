"""Binary PGM (P5) helpers for the stored gridmaps under ``map/<name>/``.

The readers and the writer are deliberately asymmetric: ``_read_header``
tolerates the ``#`` comment line GIMP writes, ``write_pgm`` never emits one. A
gridmap that has been through this stack therefore comes out with a normalised
header regardless of what authored it, which is what makes a saved map
byte-indistinguishable from a freshly converted one.

Two readers live here on purpose:

* ``read_pgm_size`` parses **only the header**. The catalogue needs every map's
  cell extent, and ``gridmap.yaml`` does not carry it (it has image / mode /
  resolution / origin / negate / the two thresholds and nothing else), so the
  header is the only source. Decoding 2.4 M pixels per map just to learn two
  integers would make listing the catalogue cost tens of megabytes of I/O.
* ``render_thumbnail`` hands the file to OpenCV, which decodes P5 properly
  (comments included) and is already a dependency of this package — see
  ``helpers/occupancy_grid.py``. Reimplementing a decoder to save one import
  would be worse code for no gain.

The sibling ``gridmap_preview.png`` that ``tools/pcd_to_gridmap.py --preview``
writes is deliberately ignored: it only exists when somebody remembered the flag
(one of the maps on the robot has none), and it is full resolution, so it is
neither reliable nor small.
"""

import os
import tempfile
from typing import List, Tuple

import cv2
import numpy as np

# The nav2 map_server only ever writes 8-bit maps, and its loader assumes it.
_MAXVAL = 255

# magic, width, height, maxval
_HEADER_TOKENS = 4

# Longest edge of a catalogue thumbnail, in pixels. 512 keeps the widest real
# map (1613 cells) legible in a card tile at ~30 KB of PNG.
DEFAULT_MAX_EDGE = 512


def _read_header(path: str) -> Tuple[List[bytes], int]:
    """Tokenize a PGM header; return its tokens and the body's byte offset.

    PGM headers are whitespace-delimited rather than line-delimited, and a
    ``#`` comment may appear anywhere in them — GIMP puts a "Created by GIMP"
    line right after the magic, and the keepout mask that used to live in
    ``map/`` was authored there. Testing only against ``pcd_to_gridmap.py``
    output would never surface this, because that tool emits no comments.

    Reads a byte at a time so a truncated or non-PGM file cannot make us pull a
    multi-megabyte body into memory looking for a token. Headers are ~20 bytes.
    """
    tokens: List[bytes] = []
    current = bytearray()
    offset = 0

    with open(path, "rb") as handle:
        while len(tokens) < _HEADER_TOKENS:
            byte = handle.read(1)
            offset += 1
            if not byte:
                raise ValueError(f"Unexpected EOF reading PGM header: {path}")

            if byte == b"#":
                # A comment ends the current token and runs to end of line.
                if current:
                    tokens.append(bytes(current))
                    current = bytearray()
                while byte and byte not in b"\r\n":
                    byte = handle.read(1)
                    offset += 1
                continue

            if byte.isspace():
                if current:
                    tokens.append(bytes(current))
                    current = bytearray()
                continue

            current += byte

    # The loop exits having consumed the single whitespace byte that terminated
    # the last token, which is exactly where the pixel body starts.
    return tokens, offset


def read_pgm_size(path: str) -> Tuple[int, int]:
    """Return ``(width, height)`` in cells from a binary PGM header.

    Raises ``ValueError`` for anything this stack cannot have produced: an ASCII
    P2 map, a maxval other than 255, a non-numeric dimension, or a file whose
    body is shorter than ``width * height`` — which is what a map caught
    mid-write looks like, and the reason this check is here rather than left to
    whoever reads the pixels.
    """
    (magic, raw_width, raw_height, raw_maxval), body_offset = _read_header(path)

    if magic != b"P5":
        raise ValueError(f"Not a binary PGM (magic {magic!r}, expected P5): {path}")

    try:
        width = int(raw_width)
        height = int(raw_height)
        maxval = int(raw_maxval)
    except ValueError as exc:
        raise ValueError(f"Malformed PGM header: {path}") from exc

    if width <= 0 or height <= 0:
        raise ValueError(f"PGM has a non-positive extent {width}x{height}: {path}")
    if maxval != _MAXVAL:
        raise ValueError(f"Unsupported PGM maxval {maxval} (expected 255): {path}")

    # One byte per cell, so the expected body length is known exactly.
    body_bytes = os.path.getsize(path) - body_offset
    if body_bytes < width * height:
        raise ValueError(
            f"PGM body is short: {body_bytes} bytes for {width}x{height}: {path}"
        )

    return width, height


def write_pgm(path: str, width: int, height: int, body: bytes) -> bytes:
    r"""Replace ``path`` with a binary PGM of ``body``, atomically.

    The header is byte-identical to what ``tools/pcd_to_gridmap.py`` emits —
    ``P5\n{w} {h}\n255\n``, no comment line — so an edited map is
    indistinguishable from a converted one and ``read_pgm_size`` agrees about
    where the body starts.

    **Atomic because three readers can arrive mid-write, and all three are
    live in this stack:**

    1. ``read_pgm_size`` rejects a body shorter than ``width * height``, and it
       runs on *every* ``GET /api/v1/maps`` through ``_read_grid``. A plain
       ``open(path, "wb")`` truncates and then streams; a listing landing in
       that window sees a short file, the catalogue degrades that map to
       ``grid: None``, and its card loses both its geometry and its thumbnail
       link. With ``os.replace`` a reader sees the old inode or the new one.
    2. ``syncai_map_server`` reads the .pgm through GraphicsMagick
       (``map_io.cpp``'s ``loadMapFromFile``) inside the ``LoadMap`` call the
       save endpoint makes milliseconds later — a guaranteed race without this.
       A torn P5 either throws (``RESULT_INVALID_MAP_DATA``) or, far worse,
       decodes with a shorter height and hands nav2 a grid whose ``origin`` and
       ``resolution`` no longer match its extent: a silently mis-registered map
       the robot then plans on.
    3. ``_content_tag`` in the map router would otherwise cache a PNG under the
       hash of bytes that never existed on disk as a whole file.

    Returns the exact file contents (header *and* body). The router's ETag is a
    hash of the whole file, so a tag taken over ``body`` alone would never match
    the one ``GET .../image`` computes, and every conditional request after a
    save would miss. Returning the bytes keeps the header format in this one
    place instead of duplicating it into the REST layer.
    """
    if len(body) != width * height:
        raise ValueError(
            f"PGM body is {len(body)} bytes for {width}x{height} "
            f"({width * height} cells expected): {path}"
        )

    blob = f"P5\n{width} {height}\n255\n".encode("ascii") + body

    directory = os.path.dirname(path) or "."
    # Same directory as the target, because os.replace is only atomic within a
    # filesystem — a temp file under /tmp would fall back to a copy on a robot
    # where map/ is a bind mount.
    handle = tempfile.NamedTemporaryFile(
        dir=directory, prefix=".gridmap-", suffix=".tmp", delete=False
    )
    try:
        with handle:
            handle.write(blob)
            handle.flush()
            # fsync before the rename, not after: the rename is what publishes
            # the file, and a power loss between the two would otherwise leave a
            # correctly named map full of zeroes. This is a battery robot.
            os.fsync(handle.fileno())
        os.replace(handle.name, path)
    except BaseException:
        try:
            os.unlink(handle.name)
        except OSError:
            pass
        raise

    # And fsync the directory so the rename itself survives the same power loss.
    # Best effort: some filesystems refuse an O_DIRECTORY open, and a map that
    # is durable but whose rename is not is still strictly better than nothing.
    try:
        dir_fd = os.open(directory, os.O_DIRECTORY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except OSError:
        pass

    return blob


def _decode_pgm(data: bytes) -> np.ndarray:
    """Decode PGM bytes to a greyscale array.

    Takes bytes rather than a path because every caller has already read the
    file to hash it for the ETag, and re-reading 2.3 MB to hand OpenCV a
    filename would double the I/O for nothing.
    """
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("Failed to decode PGM data")
    return image


def _encode_png(image: np.ndarray) -> bytes:
    ok, buffer = cv2.imencode(".png", image)
    if not ok:
        raise ValueError("Failed to encode PNG")

    return np.asarray(buffer).tobytes()


def render_png(data: bytes) -> bytes:
    """Re-encode a gridmap as a PNG at its native size, no resampling.

    For consumers that need every cell: the 3D view's ground texture, which is
    stretched over the grid's world extent and would show resampling artefacts
    at wall edges if it were scaled first.

    PNG rather than the raw .pgm because browsers cannot decode P5, and the
    format is lossless either way — a gridmap's three-value histogram
    compresses hard, so this is typically a fraction of the .pgm's size rather
    than a bloat.
    """
    return _encode_png(_decode_pgm(data))


def render_thumbnail(data: bytes, max_edge: int = DEFAULT_MAX_EDGE) -> bytes:
    """Downscale a gridmap to a PNG for the catalogue card.

    INTER_AREA because this is always a downscale and it is the only
    interpolation that averages the cells it drops — nearest sampling at ~0.3x
    would delete most of a 2-cell-thick wall and the thumbnail would read as an
    empty room. Greyscale is preserved rather than recoloured: the frontend
    tiles are built around the .pgm convention (white free, black obstacle, 205
    unknown) in both light and dark themes.
    """
    image = _decode_pgm(data)

    height, width = image.shape[:2]
    longest = max(width, height)
    if longest > max_edge:
        scale = max_edge / longest
        image = cv2.resize(
            image,
            (max(int(round(width * scale)), 1), max(int(round(height * scale)), 1)),
            interpolation=cv2.INTER_AREA,
        )

    return _encode_png(image)
