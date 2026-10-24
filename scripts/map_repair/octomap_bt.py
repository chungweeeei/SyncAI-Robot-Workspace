"""Shared helpers for the octomap repair tools in scripts/map_repair/.

The procedure they serve is docs/octomap-repair.md. numpy only -- no ROS, no OctoMap library --
so they run in the robot container, the sim container or on the host alike.

What is here:

* reading a saved map's `octomap.bt` into occupied full-depth voxel keys (`occupied_voxels`),
  and rewriting one with chosen voxels set occupied or free (`rewrite`);
* the record `apply_octomap_edits` reads (`octomap_edits.json`: one ground key per column) and
  the stacked record these tools keep beside it (`octomap_edits_stacked.json`: fills on columns
  that already hold another floor's key, and voxels to clear), both with dated backups;
* the mapping track (`poses.txt`), whose ground is the reference for every floor the mapping
  robot walked.

Keys are OctoMap's absolute voxel keys: index 32768 is the voxel whose lower edge is 0 m, so a
key k spans [(k - 32768) * res, (k - 32767) * res). "Top" below always means a voxel's upper
edge, which is where the planner (and the sim's voxel boxes) put the ground.

The .bt format (OcTreeBaseImpl::writeBinaryNode), as the sim's syncai_sim/btree.py reads it: a
text header ending in `data`, then the root node and, depth-first, its inner children; each inner
node is 16 bits, 2 per child, child i in bits (2i, 2i+1) of byte 0 for i < 4 and of byte 1 for
i >= 4 -- 00 no child, 01 free leaf, 10 occupied leaf, 11 inner node. A child's key prefix is its
parent's times two plus the child index's bit on each axis (x = 1, y = 2, z = 4). The header's
`size` is the node count, inner nodes and leaves both; OcTree::readBinary refuses a file whose
count is off, so `rewrite` recounts it.
"""

from __future__ import annotations

import datetime
import json
import os
import shutil
import sys

import numpy as np

TREE_MAX_VAL = 32768  # the key of the voxel at 0 m on each axis
DEPTH = 16
LIDAR_HEIGHT = 0.481  # poses.txt is the IMU frame; its ground is z - this (clean_map's lidar_height)
RECORD = "octomap_edits.json"
STACKED = "octomap_edits_stacked.json"
# apply_octomap_edits' --clearance default (0.6 m, the planner's body_clearance), in 0.1 m voxels:
# the cells over a fill it marks observed-free when they are unknown.
CLEARANCE_CELLS = 6

UNKNOWN, FREE, OCC, INNER = 0, 1, 2, 3
sys.setrecursionlimit(200)  # the tree is 16 deep; the walk recurses once per level


def key(m: float, res: float) -> int:
    """The voxel key holding coordinate m (metres)."""
    return int(np.floor(m / res + 1e-9)) + TREE_MAX_VAL


def top_m(k: int, res: float) -> float:
    """A voxel's upper edge, metres."""
    return (k + 1 - TREE_MAX_VAL) * res


def centre_m(k: int, res: float) -> float:
    return (k - TREE_MAX_VAL + 0.5) * res


# ---------------------------------------------------------------- reading


def _header(blob: bytes) -> tuple[float, int, list[bytes], int]:
    if not blob.startswith(b"# Octomap OcTree binary file"):
        raise SystemExit("not an OctoMap .bt file (bad first line)")
    pos, lines, res, size = 0, [], None, None
    while True:
        end = blob.index(b"\n", pos)
        line = blob[pos:end]
        pos = end + 1
        lines.append(line)
        word, _, value = line.strip().partition(b" ")
        if word == b"data":
            break
        if word == b"res":
            res = float(value)
        elif word == b"size":
            size = int(value)
    if res is None or size is None:
        raise SystemExit(".bt header lacks res or size")
    return res, size, lines, pos


def occupied_voxels(path: str) -> tuple[float, np.ndarray]:
    """(resolution, (N, 3) int64 keys of every occupied voxel), coarse leaves expanded.

    Pure Python: dp's 8 M inner nodes take about a minute.
    """
    blob = open(path, "rb").read()
    res, _, _, pos = _header(blob)
    data = memoryview(blob)[pos:]
    stack = [(0, 0, 0, 0)]  # key prefix per axis, depth
    occ: list[tuple[int, int, int, int]] = []
    i = 0
    while stack:
        px, py, pz, d = stack.pop()
        b0, b1 = data[i], data[i + 1]
        i += 2
        inner = []
        for c in range(8):
            s = ((b0 if c < 4 else b1) >> (2 * (c % 4))) & 3
            if s == UNKNOWN:
                continue
            cp = (px * 2 + (c & 1), py * 2 + ((c >> 1) & 1), pz * 2 + ((c >> 2) & 1), d + 1)
            if s == INNER:
                inner.append(cp)
            elif s == OCC:
                occ.append(cp)
        stack.extend(reversed(inner))
    parts = []
    for d in sorted({o[3] for o in occ}):
        p = np.array([o[:3] for o in occ if o[3] == d], dtype=np.int64)
        edge = 1 << (DEPTH - d)
        lo = p * edge
        if edge == 1:
            parts.append(lo)
        else:
            r = np.arange(edge)
            g = np.stack(np.meshgrid(r, r, r, indexing="ij"), axis=-1).reshape(-1, 3)
            parts.append((lo[:, None, :] + g[None]).reshape(-1, 3))
    return res, (np.concatenate(parts) if parts else np.zeros((0, 3), np.int64))


class Columns:
    """Occupied voxel keys grouped by column, for top-of-column questions."""

    def __init__(self, voxels: np.ndarray, res: float):
        self.res = res
        self.z: dict[tuple[int, int], np.ndarray] = {}
        order = np.lexsort((voxels[:, 2], voxels[:, 1], voxels[:, 0]))
        v = voxels[order]
        if v.shape[0]:
            cut = np.nonzero(np.any(np.diff(v[:, :2], axis=0) != 0, axis=1))[0] + 1
            for chunk in np.split(v, cut):
                self.z[(int(chunk[0, 0]), int(chunk[0, 1]))] = chunk[:, 2]

    @classmethod
    def from_file(cls, path: str, box: tuple[float, float, float, float] | None = None):
        res, v = occupied_voxels(path)
        if box is not None:
            x0, x1, y0, y1 = box
            v = v[(v[:, 0] >= key(x0, res)) & (v[:, 0] <= key(x1, res))
                  & (v[:, 1] >= key(y0, res)) & (v[:, 1] <= key(y1, res))]
        return cls(v, res)

    def top(self, ix: int, iy: int, zlo: float, zhi: float) -> int | None:
        """Key of the highest occupied voxel of the column with lo <= its centre < hi."""
        z = self.z.get((ix, iy))
        if z is None:
            return None
        z = z[(z >= key(zlo, self.res)) & (z < key(zhi, self.res))]
        return int(z[-1]) if z.size else None

    def between(self, ix: int, iy: int, klo: int, khi: int) -> list[int]:
        z = self.z.get((ix, iy))
        return [] if z is None else [int(k) for k in z if klo <= k < khi]


# ---------------------------------------------------------------- rewriting


def rewrite(src: str, dst: str, occupied=(), free=(), free_if_unknown=()) -> dict:
    """Write `src` to `dst` with the given full-depth keys set; returns the changes as
    {"unknown->occupied": n, ...}. `occupied` is apply_octomap_edits' fill, `free_if_unknown`
    its clearance (a known cell stays as it is), `free` overrides whatever is there. A coarse
    leaf holding an edited key is expanded down to depth 16, its other children keeping its
    state. With no edits the output is the input byte for byte (check with --roundtrip)."""
    blob = open(src, "rb").read()
    _, old_size, lines, pos = _header(blob)
    to_occ = {tuple(int(c) for c in k) for k in occupied}
    to_free = {tuple(int(c) for c in k) for k in free} - to_occ
    to_free_unknown = {tuple(int(c) for c in k) for k in free_if_unknown} - to_occ - to_free
    want = [set() for _ in range(DEPTH + 1)]  # key prefixes holding an edit, per depth
    for kx, ky, kz in to_occ | to_free | to_free_unknown:
        for d in range(DEPTH + 1):
            s = DEPTH - d
            want[d].add((kx >> s, ky >> s, kz >> s))
    data = memoryview(blob)[pos:]
    out = bytearray()
    st = {"i": 0, "nodes": 0}
    changes: dict[str, int] = {}
    names = {UNKNOWN: "unknown", FREE: "free", OCC: "occupied"}

    def edit(cp, s):
        if cp in to_occ:
            new = OCC
        elif cp in to_free or (cp in to_free_unknown and s == UNKNOWN):
            new = FREE
        else:
            return s
        if new != s:
            k = f"{names[s]}->{names[new]}"
            changes[k] = changes.get(k, 0) + 1
        return new

    def child(p, c):
        return (p[0] * 2 + (c & 1), p[1] * 2 + ((c >> 1) & 1), p[2] * 2 + ((c >> 2) & 1))

    def emit(states):
        b = [0, 0]
        for c, s in enumerate(states):
            b[c // 4] |= s << (2 * (c % 4))
        out.extend(b)
        st["nodes"] += 1 + sum(1 for s in states if s in (FREE, OCC))

    def synth(state, depth, p):
        """An inner node at `depth` standing for a leaf of `state`, with its edits."""
        states, subs = [], []
        for c in range(8):
            cp = child(p, c)
            if depth + 1 == DEPTH:
                states.append(edit(cp, state))
            elif cp in want[depth + 1]:
                states.append(INNER)
                subs.append(cp)
            else:
                states.append(state)
        emit(states)
        for cp in subs:
            synth(state, depth + 1, cp)

    def walk(depth, p):
        i = st["i"]
        b0, b1 = data[i], data[i + 1]
        st["i"] = i + 2
        states, subs = [], []
        for c in range(8):
            s = ((b0 if c < 4 else b1) >> (2 * (c % 4))) & 3
            cp = child(p, c)
            if depth + 1 == DEPTH:
                states.append(edit(cp, s))
            elif s == INNER:
                states.append(INNER)
                subs.append(("walk", cp, s))
            elif cp in want[depth + 1]:
                states.append(INNER)
                subs.append(("synth", cp, s))
            else:
                states.append(s)
        emit(states)
        for how, cp, s in subs:
            if how == "walk":
                walk(depth + 1, cp)
            else:
                synth(s, depth + 1, cp)

    walk(0, (0, 0, 0))
    if st["i"] != len(data):
        raise SystemExit(f"{src}: {len(data) - st['i']} trailing bytes after the tree")
    head = b"\n".join(b"size %d" % st["nodes"] if ln.startswith(b"size ") else ln for ln in lines)
    tmp = dst + ".tmp"
    with open(tmp, "wb") as f:
        f.write(head + b"\n" + bytes(out))
    os.replace(tmp, dst)  # like clean_map and apply_octomap_edits: no reader sees a torn tree
    changes["nodes"] = st["nodes"] - old_size
    return changes


# ---------------------------------------------------------------- records, track, backups


def stamp() -> str:
    return datetime.datetime.now().strftime("%Y%m%d-%H%M%S")


def backup(path: str) -> str | None:
    """Copy `path` to `path.bak-<stamp>` (metadata kept); None when it does not exist."""
    if not os.path.isfile(path):
        return None
    dst = f"{path}.bak-{stamp()}"
    shutil.copy2(path, dst)
    return dst


def load_record(map_dir: str) -> dict:
    path = os.path.join(map_dir, RECORD)
    if os.path.isfile(path):
        return json.load(open(path))
    return {"format": "syncai-octomap-edits", "version": 1, "res": 0.1, "source": "octomap.bt",
            "fill_key": [], "fill_xyz": []}


def save_record(map_dir: str, doc: dict) -> None:
    res = float(doc.get("res", 0.1))
    doc["fill_key"] = [[int(c) for c in k] for k in doc["fill_key"]]
    doc["fill_xyz"] = [[round(centre_m(c, res), 3) for c in k] for k in doc["fill_key"]]
    doc["created_at"] = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    json.dump(doc, open(os.path.join(map_dir, RECORD), "w"))


def load_stacked(map_dir: str) -> dict:
    path = os.path.join(map_dir, STACKED)
    doc = json.load(open(path)) if os.path.isfile(path) else {}
    doc.setdefault("format", "syncai-octomap-edits-stacked")
    doc.setdefault("res", 0.1)
    doc.setdefault("note", "written into octomap.raw.bt by scripts/map_repair/apply_stacked.py: "
                           "fill_key occupied (+ clearance free where unknown), free_key cleared; "
                           "see docs/octomap-repair.md")
    doc.setdefault("fill_key", [])
    doc.setdefault("free_key", [])
    return doc


def save_stacked(map_dir: str, doc: dict) -> None:
    for name in ("fill_key", "free_key"):
        seen, keep = set(), []
        for k in doc[name]:
            t = tuple(int(c) for c in k)
            if t not in seen:
                seen.add(t)
                keep.append(list(t))
        doc[name] = keep
    json.dump(doc, open(os.path.join(map_dir, STACKED), "w"), indent=1)


def read_track(map_dir: str) -> np.ndarray:
    """(K, 3) keyframe IMU positions from poses.txt (`<n>.pcd x y z qw qx qy qz`)."""
    path = os.path.join(map_dir, "poses.txt")
    rows = [line.split()[1:4] for line in open(path) if len(line.split()) >= 4]
    return np.array(rows, dtype=np.float64)


def raw_tree(map_dir: str) -> str:
    """The tree apply_octomap_edits starts from: octomap.raw.bt, or octomap.bt before any apply."""
    raw = os.path.join(map_dir, "octomap.raw.bt")
    return raw if os.path.isfile(raw) else os.path.join(map_dir, "octomap.bt")
