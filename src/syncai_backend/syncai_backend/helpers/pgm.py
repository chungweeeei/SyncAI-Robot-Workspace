"""Binary PGM (P5) helpers for the stored gridmaps under ``map/<name>/``.

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


def render_thumbnail(data: bytes, max_edge: int = DEFAULT_MAX_EDGE) -> bytes:
    """Downscale a gridmap to a PNG for the catalogue card.

    Takes bytes rather than a path because the caller has already read the file
    to hash it for the ETag, and re-reading 2.3 MB to hand OpenCV a filename
    would double the I/O for nothing.

    INTER_AREA because this is always a downscale and it is the only
    interpolation that averages the cells it drops — nearest sampling at ~0.3x
    would delete most of a 2-cell-thick wall and the thumbnail would read as an
    empty room. Greyscale is preserved rather than recoloured: the frontend
    tiles are built around the .pgm convention (white free, black obstacle, 205
    unknown) in both light and dark themes.
    """
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("Failed to decode PGM data")

    height, width = image.shape[:2]
    longest = max(width, height)
    if longest > max_edge:
        scale = max_edge / longest
        image = cv2.resize(
            image,
            (max(int(round(width * scale)), 1), max(int(round(height * scale)), 1)),
            interpolation=cv2.INTER_AREA,
        )

    ok, buffer = cv2.imencode(".png", image)
    if not ok:
        raise ValueError("Failed to encode thumbnail PNG")

    return np.asarray(buffer).tobytes()
