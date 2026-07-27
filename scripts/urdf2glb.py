#!/usr/bin/env python3
"""Bake a URDF + its STL meshes into a single GLB for the frontend 3D viewer.

The operator UI renders the robot with raw three.js (no react-three-fiber, no
URDF parser on the client), so the browser needs one self-contained asset with
the link hierarchy already resolved. This script is that build step.

    python3 -m venv venv && ./venv/bin/pip install trimesh yourdfpy
    ./venv/bin/python scripts/urdf2glb.py \
        src/syncai_bringup/description/G23.urdf \
        src/syncai_frontend/public/models/g23.glb

Both dependencies are pure Python on top of numpy, which matters because the
robot is an aarch64 Jetson — nothing here needs a compiler or a prebuilt wheel.

Two properties of the output the frontend depends on, so do not "optimise"
them away:

1. **Node names match URDF link names** (``base_link``, ``FL_HIP``,
   ``FL_THIGH``, …). The canvas looks links up by name to apply joint angles,
   so a pass that merges or renames nodes breaks the animation. This is also
   why the gltfpack step below passes ``-kn``.
2. **Coordinates stay in the ROS convention** (Z-up, +x forward, metres).
   trimesh does *not* rewrite the axes to glTF's nominal +Y-up, and the canvas
   builds a Z-up world to match ROS (``camera.up.set(0, 0, 1)``), so the two
   line up with no correction node. A generic glTF viewer will show the robot
   lying on its side — that is expected, not a bug.

Compression is delegated to gltfpack (meshoptimizer), which cuts the G23 from
~3.0 MB to ~600 kB. It is invoked through ``npx`` so there is nothing to
install; pass --no-pack to skip it and emit the uncompressed GLB instead. The
compressed output declares EXT_meshopt_compression, which the canvas handles by
registering three.js's MeshoptDecoder.
"""

import argparse
import os
import subprocess
import sys
import tempfile

import yourdfpy


def _make_filename_handler(urdf_path: str):
    """Resolve URDF mesh paths relative to the URDF file itself.

    G23.urdf refers to its meshes as ``../meshes/foo.stl`` rather than the
    conventional ``package://syncai_bringup/meshes/foo.stl``, which yourdfpy's
    stock handlers do not resolve. Both spellings are accepted here so this
    script keeps working if the URDF is ever normalised to package:// URIs.
    """
    base = os.path.dirname(os.path.abspath(urdf_path))

    def handler(fname: str) -> str:
        if fname.startswith("package://"):
            # package://<pkg>/<rest> -> <rest>, resolved against the URDF dir.
            fname = fname[len("package://") :].split("/", 1)[1]
        return os.path.normpath(os.path.join(base, fname))

    return handler


def convert(urdf_path: str, out_path: str, pack: bool = True) -> None:
    robot = yourdfpy.URDF.load(
        urdf_path,
        filename_handler=_make_filename_handler(urdf_path),
        # Visual meshes only. The collision geometry is primitives (e.g. the
        # 22 mm foot spheres) that would render as stray blobs in the viewer.
        load_collision_meshes=False,
    )

    scene = robot.scene
    print(f"loaded {urdf_path}: {len(scene.graph.nodes)} nodes, "
          f"{len(scene.geometry)} geometries")

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    if not pack:
        scene.export(out_path)
        print(f"wrote {out_path} ({os.path.getsize(out_path)} bytes, uncompressed)")
        return

    # Export uncompressed first, then let gltfpack rewrite it in place. -kn
    # keeps the named link nodes (see module docstring), -km keeps materials,
    # -cc is meshopt's higher-ratio compression mode.
    with tempfile.NamedTemporaryFile(suffix=".glb", delete=False) as tmp:
        raw_path = tmp.name
    try:
        scene.export(raw_path)
        raw_size = os.path.getsize(raw_path)
        subprocess.run(
            ["npx", "-y", "gltfpack", "-i", raw_path, "-o", out_path,
             "-kn", "-km", "-cc"],
            check=True,
        )
        print(f"wrote {out_path} ({raw_size} -> {os.path.getsize(out_path)} bytes)")
    finally:
        os.unlink(raw_path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", help="path to the URDF file")
    parser.add_argument("out", help="path to the GLB file to write")
    parser.add_argument(
        "--no-pack",
        action="store_true",
        help="skip gltfpack compression (no npx required)",
    )
    args = parser.parse_args()

    convert(args.urdf, args.out, pack=not args.no_pack)
    return 0


if __name__ == "__main__":
    sys.exit(main())
