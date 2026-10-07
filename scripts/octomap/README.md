# PCD → OctoMap (offline)

How `map/dp1f_1006/map.bt` was built from a pgo save on 2026-10-06, and how
its road-level free space was exported and viewed. These are offline tools:
nothing in the stack reads an OctoMap, OctoMap is **not** in the robot image,
and none of this is part of the colcon build.

## Input: a pgo save, not just `map.pcd`

The tools read a directory written by `pgo/save_maps` with
`save_patches: true`. That is the layout `map/<name>/` already has:

```
map.pcd        merged cloud (not used here)
patches/<i>.pcd  one body-frame cloud per keyframe
poses.txt      "<i>.pcd tx ty tz qw qx qy qz" per keyframe
```

`map.pcd` alone is the wrong input. It has no sensor origins, so OctoMap can
only mark its points occupied and every free voxel stays unknown. Free space,
the part that says where the robot can drive, only exists if each scan is
ray-cast from where it was taken. `poses.txt` gives that origin for every
patch, so `pcd2octomap` inserts each patch from its keyframe pose.

## 1. Environment

OctoMap is installed into a throwaway container made from the robot image,
not into robot01 and not into the `Dockerfile`. The container recipe is the
same one used for the offline bag replays: `syncai-robot-base:latest`, uid
1000, the workspace bind-mounted at `~/robot_ws`, and a Docker volume at `/ws`
for build output. Chown the volume to 1000 first, because it is created root-owned.

```bash
docker run -d --name syncai-bagrun -u 1000:1000 \
  -v "$PWD":/home/syncrobotic/robot_ws \
  -v syncai-bagrun-1006:/ws \
  -w /home/syncrobotic/robot_ws \
  syncai-robot-base:latest sleep infinity
docker exec -u 0 syncai-bagrun chown 1000:1000 /ws
docker exec -u 0 syncai-bagrun bash -c \
  'apt-get update && apt-get install -y ros-humble-octomap liboctomap-dev'
```

PCL and Eigen are already in the image.

## 2. Build

```bash
docker exec syncai-bagrun bash -c \
  'cmake -S ~/robot_ws/scripts/octomap -B /ws/octomap-build && cmake --build /ws/octomap-build -j4'
```

This builds three executables:

| Tool | What it does |
|---|---|
| `pcd2octomap` | patches + poses → `.bt` OctoMap, ray-cast |
| `slice` | `.bt` → top-down PGM of one z band, to check the result |
| `export_ply` | `.bt` → `road_free.ply` + `occupied.ply` for viewing |

## 3. Convert

```bash
docker exec syncai-bagrun bash -c 'cd ~/robot_ws && \
  /ws/octomap-build/pcd2octomap map/dp1f_1006 map/dp1f_1006/map.bt 0.05 20 0.5'
```

The arguments are resolution, max range and min range, all in metres.

- **Resolution 0.05 m** matches the gridmap.
- **Max range 20 m** truncates long rays: free space is carved only up to
  20 m, and points beyond it are not inserted as occupied. The MID360 sees
  further, but long grazing rays mostly add noise and cost time.
- **Min range 0.5 m** drops returns off the robot's own body.

Each patch goes through `insertPointCloud` with `discretize` on, so a voxel
hit by several rays of one scan is updated once rather than once per ray.
The tree is then pruned and written as a binary `.bt`, which holds
max-likelihood occupancy only. That is what octovis and
`octomap_server` load.

What dp1f_1006 produced (853 s bag, 477 m path):

| | |
|---|---|
| Keyframes / points inserted | 1195 / 3.05 M |
| Leaves | 44.3 M (1.73 M occupied, 42.6 M free) |
| `.bt` size | 25 MB |
| Wall time | about 7 min, single-threaded |
| Peak RSS | about 4.5 GB |

Mind the memory: the Docker VM on the Mac has 8 GB. For a larger site, go to
0.1 m or a shorter max range before raising the VM's memory.

## 4. Check

```bash
docker exec syncai-bagrun bash -c 'cd ~/robot_ws && \
  /ws/octomap-build/slice map/dp1f_1006/map.bt /ws/slice.pgm -0.6 0.8'
```

The PGM is black for occupied voxels inside the band, white for free, and grey
for unknown. Walls, pillars and shelving should match `gridmap.pgm`. Radial
white fans outside the building are real: rays through windows and doorways
observed free space out there.

## 5. Road-level free space

Do not look at the free voxels themselves. Every one of the 42.6 M is air the
rays passed through, and in octovis they bury the floor. `export_ply` pulls
out the layer at ground level instead:

```bash
docker exec syncai-bagrun bash -c 'cd ~/robot_ws && \
  /ws/octomap-build/export_ply map/dp1f_1006/map.bt map/dp1f_1006/poses.txt map/dp1f_1006'
```

It writes two files:

- **`road_free.ply`** holds, for each column, the lowest free voxel within
  ±0.25 m of the local floor.
- **`occupied.ply`** holds occupied voxels up to 2 m above the local floor,
  with the ceiling cut.

The local floor is the nearest keyframe's z minus the lidar height (0.481 m),
searched within 10 m. A single global floor level does not work on dp1f_1006,
whose floor changes height by about 1 m across the site.

The obvious definition, "a free voxel with an occupied floor voxel under it",
is wrong, and the header of `export_ply.cpp` explains why. Rays to distant floor
points cross the near floor at a grazing angle and mark it free. On dp1f_1006
that definition found 391 m². The lowest-free-voxel definition found 4338 m²,
which includes the free space observed outside through windows.

## 6. View

**In a browser.** `viewer/build_viewer.py` needs only numpy. It packs both PLYs
and the trajectory into one self-contained HTML file. The page is about 17 MB
for dp1f_1006 and loads three.js from a CDN.

```bash
python3 scripts/octomap/viewer/build_viewer.py map/dp1f_1006 \
  scripts/octomap/viewer/template.html map/dp1f_1006/road_free_viewer.html dp1f_1006
```

Coordinates are packed as int16 centimetres around the cloud's centre, which
caps a site at about ±327 m. Walls are downsampled to 0.1 m. The road keeps the
full 0.05 m.

**In octovis.** Homebrew's `octomap` ships no octovis. A native build lives at
`~/.local/octovis` on the development Mac, made from OctoMap v1.10.0 against
Homebrew `qt@5`. To rebuild it, build the bundled libQGLViewer by hand with
`qmake`, then pass the resulting `QGLViewer.framework` to CMake as
`QGLViewer_LIBRARY_DIR_OTHER`. For the road, open the `.bt` and leave free
voxels hidden, or use the browser view.

**Open3D's WebRTC viewer was not used.** It exists only in the linux/x86_64
wheel. On Apple Silicon that means Rosetta emulation, Xvfb, software GL and a
server on `0.0.0.0`, and it never got as far as rendering.

## Not done

- **Not in the stack.** No package in `src/` consumes an OctoMap, and the
  robot image has no OctoMap. Running `octomap_server` on the robot would be a
  `Dockerfile` change.
- **Road free is not a traversability map.** It is observed-free space at
  floor height. It also covers free space seen outside through windows and
  under tables, and it applies no slope or step check. The gridmap's z-band
  recipe and its pose-connectivity filter remain the drivable-area answer.
