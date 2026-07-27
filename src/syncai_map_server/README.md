# syncai_map_server

Everything that turns a map file on disk into an `OccupancyGrid` on a topic, and
back again. Port of `nav2_map_server` as plain `rclcpp::Node`s — no lifecycle,
so each node is live the moment it is constructed.

Three executables, all also registered as **composable components**:

| Executable | Node name | Role |
|---|---|---|
| `map_server` | `map_server` | Load a map YAML and publish it on a latched topic |
| `map_saver` | `map_saver_server` | Service that snapshots a live map topic to disk |
| `costmap_filter_info_server` | `costmap_filter_info_server` | Publish the `CostmapFilterInfo` a costmap filter needs |

Plus `map_io_core`, the shared library holding the actual image ⇄ `OccupancyGrid`
conversion (GraphicsMagick + Eigen), which the other two link against.

```
map/<name>/gridmap.yaml + .pgm
            │
            ▼
       map_server ──/<robot_id>/map (latched)──►  syncai_costmap_2d StaticLayer
            ▲                                     syncai_amcl
            │ load_map (service)                  syncai_backend  ──► GET /api/v1/map/image
            │
       map_saver ──save_map (service)──► writes <name>.pgm + <name>.yaml

  costmap_filter_info_server ──/<robot_id>/costmap_filter_info──┐
                                                                 ├─► KeepoutFilter
  map_server (2nd instance) ──/<robot_id>/keepout_filter_mask───┘
```

## map_server

Loads the YAML named by `yaml_filename` in its constructor and publishes the
grid **once**, on a `transient_local` + `reliable` + `KeepLast(1)` publisher.
That latching is what makes late-joining subscribers work: the costmaps and the
backend all start after the map server and still receive the retained sample —
provided they match the QoS, which is the single most common failure here.

| Interface | Name | Type |
|---|---|---|
| Publisher | `<topic_name>` (default `map`) | `nav_msgs/OccupancyGrid` |
| Service | `<node_name>/map` | `nav_msgs/GetMap` |
| Service | `<node_name>/load_map` | `nav2_msgs/LoadMap` |

Services are prefixed with the **node name**, not just the namespace — so the
main instance serves `/<robot_id>/map_server/load_map`, and a second instance
named `filter_mask_server` serves `/<robot_id>/filter_mask_server/load_map`. That
is what lets two map servers coexist in one robot namespace.

`load_map` swaps the map at runtime and republishes; the response carries a
`RESULT_*` code distinguishing "does not exist" / "invalid metadata" /
"invalid data" / success. `yaml_filename` may be left empty to start with no map
and supply one later over the service.

| Parameter | Default | Notes |
|---|---|---|
| `yaml_filename` | *(no default)* | Declared as `PARAMETER_STRING` with no value — see gotchas |
| `topic_name` | `map` | Relative, so it inherits the namespace |
| `frame_id` | `map` | Stamped on the published grid; stays unprefixed |

### Where the map path comes from

`map_server.launch.py` reads **two** things from `config/system.ini`: `[system]
robot_id` for the namespace, and `[map] map` for the YAML path. When the `[map]`
key is present it is appended after the params file, so it wins:

```ini
[map]
pcd: map/dp2f_full/map.pcd
map: map/dp2f_full/gridmap.yaml
```

That is how each robot instance loads its own map without anyone editing
`map_server_params.yaml`. The path is relative to the workspace root, which
works because every process in this stack runs with that as its cwd.

## map_saver

A service-only node — it holds no map. `save_map` subscribes to the requested
topic, waits up to `save_map_timeout` for one message on a **dedicated callback
group with its own `SingleThreadedExecutor`**, writes the file, and drops the
subscription. The private executor is what lets a service callback block on
another subscription without deadlocking the node's own spin.

| Parameter | Default | Notes |
|---|---|---|
| `save_map_timeout` | `5.0` | Seconds to wait for a map message |
| `free_thresh_default` | `0.25` | Applied when the request leaves it at `0.0` |
| `occupied_thresh_default` | `0.65` | Same |
| `map_subscribe_transient_local` | `true` | Must match the publisher, or nothing arrives and the save times out |

```bash
ros2 service call /<robot_id>/map_saver_server/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: 'map', map_url: 'map/newmap', image_format: 'pgm',
    map_mode: 'trinary', free_thresh: 0.25, occupied_thresh: 0.65}"
```

`map_url` is a path **without extension** — the saver writes `<map_url>.<format>`
and `<map_url>.yaml` next to each other, with the YAML's `image:` key holding
just the basename.

## costmap_filter_info_server

The smallest node here: it publishes one latched `nav2_msgs/CostmapFilterInfo`
in its constructor and then does nothing. The message tells a costmap filter
which topic carries the mask and how to interpret its values
(`space = data * multiplier + base`).

| Parameter | Default | Notes |
|---|---|---|
| `filter_info_topic` | `costmap_filter_info` | Latched publisher |
| `type` | `0` | `0` = keepout/lanes; see `syncai_costmap_2d/filter_values.hpp` |
| `mask_topic` | `filter_mask` | Must match the mask server's `topic_name` |
| `base` / `multiplier` | `0.0` / `1.0` | **Keepout requires these defaults** — the filter logs an error otherwise |

`costmap_filter_info.launch.py` brings up this node **and a second `map_server`
instance** named `filter_mask_server` that publishes the mask grid, both from one
params file. The mask is an ordinary map YAML with the same geometry as the
navigation map, black cells marking keepout zones.

The `mask_topic` stays relative on purpose: `KeepoutFilter` resolves it against
the costmap node's *parent* namespace (the costmap lives in a sub-namespace like
`/robot01/global_costmap`), so `keepout_filter_mask` reaches
`/robot01/keepout_filter_mask`.

## map_io — the conversion layer

`map_io.cpp` is the largest file in the package and is where the pixel semantics
live. Loading: GraphicsMagick reads the image, converts to greyscale, and Eigen
normalises it to 0–1 before thresholding.

**Map modes** (the YAML's `mode:` key, and the saver's `map_mode` field):

| Mode | Loading | Saving |
|---|---|---|
| `trinary` | Below `free_thresh` → free (0); above `occupied_thresh` → occupied (100); everything else → unknown (−1) | Free → 254, occupied → 0, unknown → 205 (the classic `.pgm` convention) |
| `scale` | Same thresholds, but the in-between band is scaled linearly to 1–99; a transparent alpha channel marks unknown | Grey proportional to occupancy, unknown written as transparent — needs a format with alpha |
| `raw` | Pixel value used directly as the occupancy value | Occupancy written straight to the pixel |

`negate: 1` in the YAML inverts the brightness convention (white becomes
occupied) before thresholding.

**Saving** picks its own defaults: no `image_format` given means `png` for
`scale` mode and `pgm` otherwise; a format GraphicsMagick cannot write falls back
to `png` with a warning. Thresholds are validated (`0 ≤ free < occupied ≤ 1`) and
throw if inconsistent. For `trinary` the emitted YAML hardcodes
`occupied_thresh: 0.65` / `free_thresh: 0.196` — because the saved pixel values
are fixed (0 / 205 / 254), the thresholds must match those pixels rather than
whatever was used to threshold the incoming grid.

`~` in a `yaml_filename` is expanded via `$HOME`.

## Running

```bash
ros2 launch syncai_map_server map_server.launch.py
ros2 launch syncai_map_server map_server.launch.py \
    system_config:=config/instances/robot02.ini
ros2 launch syncai_map_server map_saver.launch.py
ros2 launch syncai_map_server costmap_filter_info.launch.py
```

`map_server` is window 1 (`localization`) in both byobu sessions and must come up
**before** the planner, whose global costmap static layer blocks on the latched
map. `costmap_filter_info` is only launched in the 2D session — the 3D planner
params have no keepout filter configured yet.

```bash
ros2 topic echo /<robot_id>/map --once --qos-durability transient_local
ros2 service call /<robot_id>/map_server/load_map nav2_msgs/srv/LoadMap \
    "{map_url: 'map/other/gridmap.yaml'}"
```

Note the `--qos-durability transient_local` — without it `ros2 topic echo`
subscribes as volatile, never receives the retained sample, and the map looks
missing when it is not.

## Gotchas

- **`yaml_filename` has no default.** It is declared as a bare
  `rclcpp::PARAMETER_STRING`, so `ros2 run syncai_map_server map_server` with no
  params file throws on startup rather than starting empty. Pass `""` explicitly
  to start map-less.
- **A bad map path throws in the constructor** and kills the process. Since the
  path usually comes from `[map] map` in the instance INI, a typo there takes the
  whole map server down — check the INI before the params file.
- **QoS must be transient-local on both ends.** Every consumer in this stack
  (`StaticLayer` via `map_subscribe_transient_local`, the backend's map
  subscriber, `map_saver`) has a matching setting; a volatile subscriber simply
  never sees the map.
- **The map is published once.** There is no periodic republish — only startup
  and `load_map`. A subscriber that connects with the wrong QoS and is then fixed
  still needs the publisher restarted or `load_map` called again.
- **Two map servers in one namespace need different node names.** Topic and
  service names are derived from the node name; `costmap_filter_info.launch.py`
  relies on this for its `filter_mask_server` instance.
- **GraphicsMagick is a hand-installed system dependency.** It has no ament
  config and is found via `pkg-config`, and recreating the robot container wipes
  it — reinstall `libgraphicsmagick++1-dev` along with the other rosdep packages.
- **`map_server_params.yaml` says "Absolute path" but the value is relative**
  (`map/dp2f_full/gridmap.yaml`). Relative works — cwd is the workspace root —
  but the comment is wrong.
- `map_saver_params.yaml` sets no `use_sim_time`, unlike the other two params
  files. Harmless for a service-only node with no timers, but inconsistent.

Upstream reference: [`nav2_map_server`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_map_server),
including the [map YAML format](https://docs.nav2.org/configuration/packages/configuring-map-server.html).
