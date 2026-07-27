# syncai_system_manager

The robot's host-level plumbing: one rclpy node, `syncai_system_manager`, that
puts **WiFi control** and **mDNS name publishing** behind ROS interfaces. It is
the only package in the stack that shells out to host tooling (`nmcli`,
`avahi-publish`) instead of talking to hardware or other ROS nodes.

```
syncai_backend ──scan_wifi / connect_wifi (services)──►  ┌────────────────────┐
                                                          │  WifiManager       │──nmcli──► host NetworkManager
syncai_robot_state ◄──wifi_status (1 Hz topic)───────────│                    │
                                                          ├────────────────────┤
                                                          │  MdnsManager       │──avahi-publish──► <robot_id>.local
                                                          ├────────────────────┤
                                                          │  SysManager        │  (holds robot_id)
                                                          └────────────────────┘
```

The node is a thin shell: `main.py` constructs three managers and spins a
`MultiThreadedExecutor`. All the behaviour lives in
`syncai_system_manager/managers/`.

| Manager | Owns |
|---|---|
| `WifiManager` | `scan_wifi` / `connect_wifi` services, the `wifi_status` publisher, and all `nmcli` invocation |
| `MdnsManager` | The `avahi-publish` child process advertising `<robot_id>.local` |
| `SysManager` | Just the `robot_id` parameter — the seam where more system-level state would go |
| `map_manager.py` | **Empty file**, a placeholder; nothing implements it |

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

The status itself comes from `update_wifi_status()`, which parses the `IN-USE`
column of `nmcli device wifi list --rescan no` for the connected network and
reads the IP/MAC from `netifaces` (preferring `wl*` interfaces).

`setup_wifi()` runs at construction and enables the radio via
`sudo nmcli radio wifi on` if it is off.

## MdnsManager

Spawns `avahi-publish -a <robot_id>.local -R <ip>` as a child process so other
machines on the LAN can reach the robot by name.

Choosing which IP to advertise is the interesting part:

1. The WiFi interface's address, if it has one.
2. Otherwise the first address on an `en*` then `eth*` interface —
3. **skipping anything in `172.16.0.0/12`**, Docker's default bridge pool. In the
   robot container `eth0` is the compose bridge and `eth1` is the `syncai-lan`
   macvlan; publishing the bridge address would advertise a name that resolves to
   an address unreachable from the LAN.

`avahi-publish` is a long-running daemon, so the manager waits 0.5 s after
spawning: an exit inside that window means publishing failed (name collision, no
avahi daemon) rather than succeeded.

`main.py` calls `kill_mdns()` in its `finally` block, because a surviving
`avahi-publish` child would keep a stale `<robot_id>.local` record resolving to
an old IP after the node is gone.

Reaching the host's avahi and NetworkManager from inside the container needs the
D-Bus and avahi socket mounts plus `apparmor=unconfined` — see the compose file
and `CLAUDE.md`.

## Interfaces

All relative, so they inherit the `<robot_id>` namespace.

| Kind | Name | Type |
|---|---|---|
| Service | `scan_wifi` | `syncai_common/ScanWifiNetworks` |
| Service | `connect_wifi` | `syncai_common/ConnectWifiNetwork` |
| Publisher | `wifi_status` | `syncai_common/WifiStatus` |

Both services are reachable from the operator UI through the backend:
`GET /api/v1/network/wifi/scan` and `POST /api/v1/network/wifi/connect`.

## Parameters

| Parameter | Default | Set by |
|---|---|---|
| `robot_id` | `default_robot` | The launch file, from `[system] robot_id` in `config/system.ini` |

That is the whole parameter surface — there is no params YAML. `robot_id` is used
both as the node namespace (by the launch file) and as the mDNS hostname.

## Running

```bash
ros2 launch syncai_system_manager system_manager.launch.py
ros2 launch syncai_system_manager system_manager.launch.py \
    system_config:=config/instances/robot02.ini
```

Started in the byobu sessions' `managers` window, next to `driver_manager`.

```bash
ros2 service call /<robot_id>/scan_wifi syncai_common/srv/ScanWifiNetworks "{}"
ros2 service call /<robot_id>/connect_wifi syncai_common/srv/ConnectWifiNetwork \
    "{ssid: 'MyNetwork', password: 'secret'}"
ros2 topic echo /<robot_id>/wifi_status
getent hosts <robot_id>.local        # is the mDNS record live?
```

If `nmcli` fails from inside the container, check the D-Bus mount before
suspecting this node — `nmcli` needs to reach the *host's* NetworkManager
daemon, there is none in the container.

## Tests

The only package in the workspace with meaningful unit tests:

```bash
colcon test --packages-select syncai_system_manager
colcon test-result --verbose
# or, from the package directory:
pytest test/
```

`test/test_wifi_manager.py` (450 lines) covers the `nmcli` parsing paths with
`pytest-mock` + `assertpy`, mocking `subprocess.run` / `subprocess.Popen` and
`netifaces`. The node itself is a `MagicMock` — `WifiManager` only touches it via
`get_logger()`, `create_publisher`, `create_service` and `create_timer`, so no
ROS graph is needed. Alongside it are the standard ament linters.

## Packaging

`setup.py` uses the same `InstallNoSource` command as `syncai_backend`: after a
normal install it byte-compiles the package and deletes the `.py` sources from
the install space, so deployments ship no source. It self-disables when the
installed modules are symlinks, so `--symlink-install` developer builds keep
their sources.

## Gotchas

- **`wifi_status` never refreshes after startup.** `update_wifi_status()` — the
  only thing that repopulates the cached status — is called exactly once, from
  `init_wifi_manager()`. The 1 Hz timer republishes that same snapshot forever,
  and `connect_wifi` does not refresh it either. So the SSID/BSSID/RSSI the UI
  shows are whatever they were when the node started; a roam, a signal change, or
  a successful `connect_wifi` will not be reflected until the node is restarted.
  A periodic `update_wifi_status()` call (or one at the end of
  `_connect_wifi_network`) is what this needs.
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
- **`map_manager.py` is an empty file.** `CLAUDE.md` describes a "map manager"
  behind ROS services; it does not exist. Map handling lives in
  `syncai_map_server` and the backend instead.
- **One log call uses structlog-style keyword arguments** —
  `self._logger.error("avahi-publish exited prematurely", returncode=...)` in
  `mdns_manager.py`. rclpy loggers format only the message string, so the
  `returncode` value never appears in the output; the rest of this package's
  Python logs through the rclpy logger, unlike the backend which uses structlog.
