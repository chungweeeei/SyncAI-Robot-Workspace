# syncai_sys_manager

The robot's host-level plumbing, and the process that brings the rest of the
stack up: one rclpy node, `syncai_sys_manager` (class `SystemManager` in
`main.py`), that puts **WiFi control**, **mDNS name publishing**, **resource
monitoring** and **operating-mode / stack lifecycle** behind ROS interfaces. It
is the only package in the stack that shells out to host tooling (`nmcli`,
`avahi-publish`, `byobu`) instead of talking to hardware or other ROS nodes.

```
syncai_backend ──scan_wifi / connect_wifi (services)──►  ┌────────────────────┐
                                                          │  WifiManager       │──nmcli──► host NetworkManager
syncai_robot_state ◄──wifi_status (1 Hz topic)───────────│                    │
                                                          ├────────────────────┤
                                                          │  ConfManager       │  (holds robot_id)
                                                          ├────────────────────┤
                                                          │  MdnsManager       │──avahi-publish──► <robot_id>.local
                                                          ├────────────────────┤
                                                          │  MonitorManager    │──psutil──► stdout, 1 Hz (mem + disk /)
                                                          ├────────────────────┤
syncai_backend ──switch_mode (service)──────────────────►│  NodeManager       │──byobu──► syncai-dev / syncai-mapping
syncai_robot_state ──get_mode (service)─────────────────►│                    │           (the whole robot stack)
                                                          └────────────────────┘
```

The node is a thin shell: `main.py` constructs five managers **in order** —
`WifiManager`, `ConfManager`, `MdnsManager` (needs the first two), `MonitorManager`,
`NodeManager` last — and spins a `MultiThreadedExecutor`. All the behaviour
lives in `syncai_sys_manager/managers/`. The order matters at the end:
`NodeManager` is constructed last so the rest of the node exists before the
stack it brings up starts talking to it.

| Manager | Owns |
|---|---|
| `WifiManager` | `scan_wifi` / `connect_wifi` services, the `wifi_status` publisher and its 5 s refresh, and all `nmcli` invocation |
| `ConfManager` | Just the `robot_id` parameter — the seam where more system-level config would go |
| `MdnsManager` | The `avahi-publish` child process advertising `<robot_id>.local` |
| `MonitorManager` | A 1 Hz log line of host memory and disk usage (reporting only) |
| `NodeManager` | `switch_mode` / `get_mode` services, and the byobu sessions that *are* the robot stack |
| `map_manager.py` | **Empty file** (0 bytes), a placeholder; nothing implements it |

## WifiManager

**Services** (both on their own `MutuallyExclusiveCallbackGroup`, so a long scan
does not block a connect):

| Service | Type | Implementation | Timeout |
|---|---|---|---|
| `scan_wifi` | `syncai_common/ScanWifiNetworks` | `sudo nmcli device wifi rescan` then `nmcli -f BSSID,SIGNAL,SSID device wifi list` | 10 s rescan + 30 s list |
| `connect_wifi` | `syncai_common/ConnectWifiNetwork` | `sudo nmcli device wifi connect <ssid> [password <pw>]` | 60 s |

The backend's `RobotGateway` waits 45 s and 70 s respectively — deliberate
headroom over these numbers.

Details that matter:

- **`sudo` is required for `rescan` and `connect`,** but not for plain `list`.
  polkit denies `wifi.scan` to session-less processes, which is what this node is
  inside the container.
- **Arguments are passed as a list, never a shell string,** so an SSID or
  password containing shell metacharacters goes straight through `exec` and
  cannot inject. The password arguments are omitted entirely for open networks.
- **`nmcli`'s `SIGNAL` column is 0–100, not dBm.** It is converted with
  `ceil(signal / 2 - 100)` to get an approximate dBm for `WifiNetwork.rssi`.
- Hidden SSIDs (`--`) and duplicate SSIDs are dropped from scan results.

**`wifi_status` topic** — published at 1 Hz (BEST_EFFORT, VOLATILE, KeepLast(3))
and consumed only by `syncai_robot_state`, which flattens it into
`RobotState.network_status.wifi_info` for the UI.

Two timers feed it. The 1 Hz `_wifi_status_timer` only republishes a cached
`WifiStatus` struct (behind `_wifi_status_lock`); a separate 5 s
`_wifi_refresh_timer` is what keeps that cache true, by calling
`update_wifi_status()` — which parses the `IN-USE` column of
`nmcli device wifi list --rescan no` for the connected network and reads the
IP/MAC from `netifaces` (preferring `wl*` interfaces). 5 s is deliberately
slower than the publish: `--rescan no` is a cheap D-Bus read, but still a
subprocess per tick, and nobody needs the SSID at 1 Hz. A refresh failure is
logged as a warning throttled to once per 60 s, so a WiFi interface that stays
down does not spam the log every 5 s.

`setup_wifi()` runs at construction and enables the radio via
`sudo nmcli radio wifi on` if it is off.

## ConfManager

Declares the `robot_id` parameter (default `default_robot`) and hands it to the
managers that need it: `MdnsManager` for the hostname, `NodeManager` for the log
tree. It is the only thing in this package that reads a ROS parameter.

## MdnsManager

Spawns `avahi-publish -a <robot_id>.local -R <ip>` as a child process so other
machines on the LAN can reach the robot by name.

Choosing which IP to advertise is the interesting part:

1. The WiFi interface's address, if it has one.
2. Otherwise the first address on an `en*` then `eth*` interface —
3. **skipping anything in `172.16.0.0/12`**, Docker's default bridge pool. The
   robot container runs with `network_mode: host` today, so it sees the host's
   real NICs and this guard normally never fires. It is kept because it is what
   made the old dual-homed layout work (the retired Isaac Sim fleet had `eth0` on
   the compose bridge and `eth1` on the `syncai-lan` macvlan): publishing a
   bridge address advertises a name that resolves to something unreachable from
   the LAN, and if the container ever goes back to a bridge network that is still
   the failure mode.

`avahi-publish` is a long-running daemon, so the manager waits 0.5 s after
spawning: an exit inside that window means publishing failed (name collision, no
avahi daemon) rather than succeeded.

`main.py` calls `kill_mdns()` in its `finally` block, because a surviving
`avahi-publish` child would keep a stale `<robot_id>.local` record resolving to
an old IP after the node is gone.

Reaching the host's avahi and NetworkManager from inside the container needs the
D-Bus and avahi socket mounts plus `apparmor=unconfined` — see the compose file
and `CLAUDE.md`.

## MonitorManager

A 1 Hz timer that logs one line of `psutil` figures: memory used/total (as
`total - available`, so the number agrees with the percentage psutil prints next
to it) and disk used/total for `/`. Reporting only — nothing here throttles or
shuts anything down.

Two things to know when reading it:

- **The figures are host-wide, not the container's.** psutil reads
  `/proc/meminfo`, which Docker does not namespace, so this is the whole Jetson's
  memory rather than the container's cgroup. For a robot that owns its compute
  board that is the useful number.
- **`/` is the overlayfs**, i.e. the host's docker storage partition — which is
  the one that fills up and takes the stack down with it.

It lands on stdout rather than a topic because the byobu `pipe-pane` tap is what
makes console output persistent here (`ROS_LOG_DIR` is a tmpfs). Since
`sys_manager` runs as the container's main process rather than in a session
window, its own output goes to `docker logs`, not to a multilog directory.

## NodeManager

The stack's launcher, and the reason `docker compose up -d robot01` brings a
whole robot up on its own. An operating mode **is** a byobu session:

| `RobotMode` | Session spec | byobu session | Log subtree |
|---|---|---|---|
| `AUTO` = 2 (default) | `config/sessions/start_nav.yaml` | `syncai-dev` | `log/stack/<robot_id>/<name>/` |
| `MANUAL` = 1 | `config/sessions/start_mapping.yaml` | `syncai-mapping` | `log/stack/<robot_id>/mapping/<name>/` |
| `MAINTENANCE` = 0 | — | none | — |

`MAINTENANCE` is not a mode you switch *into*; it is what `get_mode` reports when
neither session exists. **The live mode is never stored** — it is derived on
demand from which sessions byobu actually has (`byobu has-session`), so it stays
correct across a `sys_manager` restart and does not drift when someone builds or
kills a session by hand. If both sessions somehow exist, `get_mode` answers
`success: false` with an "ambiguous" message and reports the lower mode.

**Services:**

- `switch_mode` (`syncai_common/SwitchMode`, request `mode: uint8`) kills
  *every* known session, then builds the target one from its spec, then reports
  the mode it actually landed in. It **refuses to rebuild the mode that is
  already live** (`success: true`, "Already in X; nothing to do") — in `MANUAL`
  a rebuild would drop an unsaved map on the floor, because `pgo_node` keeps its
  keyframes in RAM and `save_maps` is the only thing that serialises them. The
  refusal only applies when exactly one session is up; with two, the requested
  mode being among them is not good enough and it falls through to the
  kill-and-rebuild, which is exactly the cleanup that state needs. Asking for
  `MAINTENANCE` (or any value without a spec) is refused with the list of
  switchable modes.
- `get_mode` (`syncai_common/GetMode`) returns `mode`, the `session` name, and a
  message.

The whole `switch_mode` body runs under `_mode_lock`: it is a destructive
sequence of ~40 byobu commands, and two interleaving would build one session
out of two specs. The per-service `MutuallyExclusiveCallbackGroup`s only
serialise a callback with itself, not with the other service, hence the extra
lock. Every byobu invocation is bounded by `BYOBU_TIMEOUT` (10 s); a hang that
long means the byobu server is wedged, and it is logged rather than left to
block an executor thread forever.

**Startup.** `init_node_manager()` calls `setup_session()` at construction —
*not* `launch_session()`. It adopts a running session for any known mode and
leaves it strictly alone, and only when nothing is up does it build `AUTO`.
That is what makes restarting `sys_manager` in the middle of a mapping run
harmless, and what would stop the startup path from destroying itself if
`sys_manager` were ever put back into a spec as a window. A session left over
from a crashed run therefore also blocks the rebuild — deliberately, since
killing a session someone may be attached to is worse; use `switch_mode` to
force one.

**The session layout is data.** `NodeManager` holds only the byobu plumbing;
the windows, panes, commands, startup offsets and log names live in
`config/sessions/*.yaml`, with this schema (from the class docstring):

```yaml
session: <session name>
select:  <window name to select after building>
windows:
  - name: <window name>
    cwd:  <path relative to the workspace root>   # optional
    panes:
      - cmd:   <shell command>
        sleep: <seconds to delay before cmd>       # optional
        log:   <multilog dir name>                 # optional
        enter: <bool, default true>                # optional
```

`sleep` is where the startup ordering lives, since there is no lifecycle
manager: map_server / LIO → lio_bridge → planner/controller → task_runner.
`log` names a directory under `log/stack/<robot_id>/` that a `pipe-pane |
multilog t s16777215 n10 '!gzip'` tap writes into (`current` plus gzipped
rotations, 16 MiB × 10). `enter: false` types the command without executing
it. Sessions are built **detached** — the caller is a ROS node with no TTY.

## Interfaces

All relative, so they inherit the `<robot_id>` namespace.

| Kind | Name | Type | Owner |
|---|---|---|---|
| Service | `scan_wifi` | `syncai_common/ScanWifiNetworks` | `WifiManager` |
| Service | `connect_wifi` | `syncai_common/ConnectWifiNetwork` | `WifiManager` |
| Service | `switch_mode` | `syncai_common/SwitchMode` | `NodeManager` |
| Service | `get_mode` | `syncai_common/GetMode` | `NodeManager` |
| Publisher | `wifi_status` | `syncai_common/WifiStatus` | `WifiManager` |

From the operator UI, through the backend: the WiFi services are
`GET /api/v1/network/wifi/scan` and `POST /api/v1/network/wifi/connect`, and
`switch_mode` is `POST /api/v1/robot/mode`. There is no REST twin for
`get_mode`: `syncai_robot_state` polls it and relays the mode in
`RobotState.mode`, which `GET /api/v1/robot/state` already carries.

## Parameters

| Parameter | Default | Set by |
|---|---|---|
| `robot_id` | `default_robot` | The launch file, from `[system] robot_id` in the system INI |

That is the whole parameter surface — there is no params YAML. `robot_id` is used
as the node namespace (by the launch file), as the mDNS hostname, and as the
`log/stack/<robot_id>/` log tree.

## Running

`sys_manager` is the **robot container's main process**, not a byobu window:
`docker-compose.robots.yml` runs
`exec ros2 launch syncai_sys_manager sys_manager.launch.py` as the service
`command`, so `docker compose up -d robot01` starts it, and it in turn builds the
`AUTO` session. Neither session spec contains it — adding it back as a window
would let `kill_session` kill the pane it runs in.

By hand, inside the container:

```bash
ros2 launch syncai_sys_manager sys_manager.launch.py
ros2 launch syncai_sys_manager sys_manager.launch.py \
    system_config:=config/instances/robot01.ini
```

The `system_config` default is the **absolute** `~/robot_ws/config/system.ini`
(inside the container, the bind-mount of `config/instances/robotNN.ini`), so
unlike the other launch files this one does not depend on the cwd.

```bash
ros2 service call /<robot_id>/get_mode syncai_common/srv/GetMode
ros2 service call /<robot_id>/switch_mode syncai_common/srv/SwitchMode "{mode: 1}"   # MANUAL
ros2 service call /<robot_id>/scan_wifi syncai_common/srv/ScanWifiNetworks "{}"
ros2 service call /<robot_id>/connect_wifi syncai_common/srv/ConnectWifiNetwork \
    "{ssid: 'MyNetwork', password: 'secret'}"
ros2 topic echo /<robot_id>/wifi_status
getent hosts <robot_id>.local        # is the mDNS record live?
byobu list-sessions                  # what NodeManager derives the mode from
```

If `nmcli` fails from inside the container, check the D-Bus mount before
suspecting this node — `nmcli` needs to reach the *host's* NetworkManager
daemon, there is none in the container.

## udev rules

`udev/99-syncai-devices.rules` gives the hot-pluggable peripherals stable names
under `/dev/syncai/`, which is what `docker-compose.robots.yml` passes through:

| Symlink | Device | Matched on |
|---|---|---|
| `camera0`, `camera1` | Two TechNexion VCS-AR0234-C UVC cameras (`3407:57d1`) | `ATTRS{serial}` — both units are the **same model**, so neither the product id nor the USB port path can tell them apart; the serial is the only descriptor that does and it survives a re-cable. `ATTR{index}=="0"` skips each camera's UVC metadata node. |
| `speaker`, `speaker_pcm` | Jieli CD002-AUDIO USB dongle (`e2b8:0811`) | vendor/product. Presence detection only — audio consumers open the card by its stable ALSA name `hw:CARD=CD002AUDIO`, not by number. |

The header of the file records the two earlier matching strategies that failed
(port path; per-model product id, which let one rule claim both cameras) so they
are not rediscovered. Swapping in a replacement camera is the one maintenance
event that requires editing it — read the new serial with
`udevadm info -q property -n /dev/videoN | grep ID_SERIAL_SHORT`.

`setup.py` installs the rules to `share/syncai_sys_manager/udev/` for
distribution only. **udev runs on the host**, so activating them is a manual
copy there, not in the container:

```bash
sudo cp src/syncai_sys_manager/udev/99-syncai-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

## Dependencies

From `package.xml`: `rclpy`, `syncai_common` (the service and `RobotMode`
definitions), and four Python packages —

| Package | Used by |
|---|---|
| `python3-netifaces` | `WifiManager` / `MdnsManager`, interface addresses |
| `python3-psutil` | `MonitorManager`, memory and disk figures |
| `python3-yaml` | `NodeManager`, reading `config/sessions/*.yaml` |
| `python3-structlog` | declared in the manifest, but no module in this package imports it — everything logs through rclpy |

All have rosdep keys on jammy, so a container recreated from the image picks
them up with the usual `rosdep install --from-paths src --ignore-src -r -y`
(structlog is noted in the manifest as pip-installed where the key is missing).
Beware the manifest's own warning: **no double hyphens inside a `package.xml`
comment** — they make the file unparseable, which silently demotes the package
from `ament_python` to plain Python and drops it out of `AMENT_PREFIX_PATH`.

## Tests

```bash
colcon test --packages-select syncai_sys_manager
colcon test-result --verbose
# or, from the package directory:
pytest test/
```

`test/test_wifi_manager.py` (~450 lines) covers the `nmcli` parsing paths with
`pytest-mock` + `assertpy`, mocking `subprocess.run` / `subprocess.Popen` and
`netifaces`. The node itself is a `MagicMock` — `WifiManager` only touches it via
`get_logger()`, `create_publisher`, `create_service` and `create_timer`, so no
ROS graph is needed. Alongside it are the standard ament linters. The other
managers have no unit tests; `NodeManager` in particular is only exercised by
running it against a real byobu. (`syncai_backend` has the larger test suite in
the workspace.)

## Packaging

`setup.py` uses the same `InstallNoSource` command as `syncai_backend`: after a
normal install it byte-compiles the package and deletes the `.py` sources from
the install space, so deployments ship no source. It self-disables when the
installed modules are symlinks, so `--symlink-install` developer builds keep
their sources.

`data_files` installs, besides the ament index marker and `package.xml`,
`launch/*.launch.py` to `share/syncai_sys_manager/launch/` and `udev/*.rules` to
`share/syncai_sys_manager/udev/`. The console script is
`sys_manager_node = syncai_sys_manager.main:main`.

## Gotchas

- **`wifi_status` lags reality by up to 5 s**, the period of
  `_wifi_refresh_timer`. It used to lag forever — `update_wifi_status()` ran
  exactly once at init, so the console showed the boot-time SSID until
  `sys_manager` restarted, even after a successful `connect_wifi`. The refresh
  timer fixed that; a failed refresh keeps the previous snapshot and warns at
  most once a minute.
- **The mDNS record is published once**, at startup. If the IP changes later —
  including via this node's own `connect_wifi` — the advertised address goes
  stale. `setup_mdns()` is idempotent (it kills before publishing) but nothing
  calls it again.
- **`nmcli` requires the host D-Bus socket, `apparmor=unconfined` and passwordless
  `sudo`** inside the container. Any of those missing turns every WiFi operation
  into a non-obvious `nmcli` error.
- **RSSI is derived, not measured.** `ceil(signal/2 - 100)` is an approximation of
  dBm from NetworkManager's 0–100 quality percentage, so do not treat it as a
  calibrated reading.
- **`map_manager.py` is an empty file.** Map handling lives in
  `syncai_map_server` and the backend's map router instead.
- **NodeManager gotchas.** `switch_mode` kills the byobu session the backend is
  a pane of, so a `POST /api/v1/robot/mode` normally ends in a dropped connection
  rather than a response — the backend treats "dispatched, no answer" as
  success-in-progress, and only the no-op and the refusal reliably answer.
  `kill_session` uses the `session:` name from the spec, so the two specs must
  keep distinct names (`syncai-dev` vs `syncai-mapping`) or starting one tears
  the other down. `setup_session()` adopting a leftover session means a crashed
  run's session must be killed by hand (or via `switch_mode`) before a restart
  rebuilds anything. `SESSION_SPECS` hard-codes `~/robot_ws/config/sessions/`,
  so the manager only works where the workspace is at that path — i.e. inside
  the robot container.
- **This package logs through the rclpy logger, not structlog.** rclpy loggers
  format only the message string, so a structlog-style keyword argument
  (`logger.error("...", returncode=rc)`) is silently dropped — put the value in
  the message. `mdns_manager.py` had exactly that bug once; its comment at the
  exit-code log line is the reminder.
