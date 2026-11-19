# Release image: build, carry to a robot, install, roll back

How the stack gets onto a robot that has no checkout of this repository — a
robot in a plant with no outside network is the case it is built for. The
pieces are the Dockerfile's `release` stage, `.github/workflows/release.yml`,
`docker-compose.release.yml` and `scripts/release/`.

**arm64 only** (the robots are Jetsons). The backend and the console are other
repositories with their own images; nothing here builds or carries them.

## What a release is

One image, `ghcr.io/<owner>/<repo>:release-<version>`. The version is the build
time, `YYYYMMDD-HHMM` in Asia/Taipei. It is about 1.7 GB (the dev image is
4.5 GB), and about 530 MB as a bundle's gzipped tar. The image holds

- `install/` — a Release build without `--symlink-install`, so no link into a
  source tree, and without `direct_visual_lidar_calibration` and
  `vizionsdk_ros2`;
- `config/sessions/*.yaml` and `config/cyclonedds.xml`, which belong to a
  version of the code rather than to a robot;
- `MANIFEST` — the commit of this repository and of every checkout under
  `src/` it was built from (`dependencies.repos` follows branches, so this is
  the only record);
- `deploy/` — the compose file, `install.sh` and the templates for the host,
  so that they are never a different version from the image.

Nothing in it can be edited on the robot. A parameter that has to change on
site goes through the console's settings pages (the controller's and the
planner's overlays in `config/params/<robot_id>/`); anything else is a new
release.

The robot's own data is one directory on its host, `/opt/syncai`, which is
also what the backend's compose mounts as `ROBOT_WS`:

| Path under `/opt/syncai` | Whose | Written by |
|---|---|---|
| `config/instances/robot.ini` | the robot's | you, once; then the backend (map switch) |
| `config/state/` | the robot's | the localizer (`last_pose.json`) |
| `config/params/` | the robot's | the backend (settings pages) |
| `config/calibration/` | the robot's | the calibration procedure |
| `map/`, `log/`, `data/postgres/`, `.env` | the robot's | the stack, postgres, you |
| `docker-compose.release.yml`, `attach.sh`, `MANIFEST` | the image's | `install.sh`, on every switch |
| `config/cyclonedds.xml` | the image's | `install.sh` — the backend reads it |
| `src/syncai_controller/params/`, `src/syncai_global_planner/params/` | the image's | `install.sh` — the factory defaults the backend's settings pages show |

`install.sh` never touches a file of the first kind once it exists, and
replaces the second kind from inside the image it is switching to.

## 1. Build a release (CI)

Releases are built by CI only.

1. Merge what should ship; every `dependencies.repos` repository's `dev` branch
   is taken as it stands at that moment.
2. GitHub → Actions → **release** → Run workflow, on the branch to release.
3. The run's summary shows the version, the image reference and the manifest.

The workflow refuses a dirty tree, compiles the workspace in the Dockerfile's
`builder` stage, and fails the image build if any package is not a Release
build (`check_build.sh`), if a binary links a library the image lacks
(`check_libs.sh`), or if a program, GStreamer element, launch file or the
`sys_manager` import the stack needs at run time is missing
(`check_runtime.sh`). It then runs CI's smoke test inside the image — every
pane of the three session specs — and pushes only if that passes.

It also runs every Monday without pushing, so that a release build broken by
an ordinary commit is found before a release is wanted.

## 2. Make a bundle (a laptop with network access)

```bash
scripts/release/save_bundle.sh 20261119-1530            # an update
scripts/release/save_bundle.sh 20261119-1530 --with-infra   # a first install
```

`dist/syncai_<version>/` then holds `robot_<version>.tar.gz`, `SHA256SUMS`,
`install.sh` and `MANIFEST` (and `infra.tar.gz`: postgres and temporal, which
an offline robot cannot pull either — needed the first time, and again only
when their tags in the compose file change). Copy the directory to a USB
stick. If the package is private on ghcr, `docker login ghcr.io` first.

A robot that reaches the registry needs no bundle: copy `install.sh` out of
any bundle and run `./install.sh --image ghcr.io/<owner>/<repo>:release-<version>`.

## 3. First install on a robot

The host needs, and `install.sh` checks but does not install:

- Docker with the compose plugin (v2), and the invoking user in the `docker`
  group. Run `install.sh` as the uid-1000 user or as root.
- The `nvidia` container runtime (on a Jetson, `nvidia-container` from
  JetPack).
- `avahi-daemon` and NetworkManager, for `<robot_id>.local` and the wifi
  services.
- **This robot's udev rules**, so that `/dev/syncai/camera0` and `camera1`
  exist — compose refuses to start the robot container without them. The rules
  match each camera by its USB serial, so the file in the image is robot01's
  and has to be edited:

  ```bash
  docker run --rm --entrypoint cat <image> \
      install/syncai_sys_manager/share/syncai_sys_manager/udev/99-syncai-devices.rules \
      > 99-syncai-devices.rules
  udevadm info -a -n /dev/video0 | grep 'ATTRS{serial}' | head -n 1   # each camera
  # put this robot's two serials in the file, then:
  sudo cp 99-syncai-devices.rules /etc/udev/rules.d/
  sudo udevadm control --reload && sudo udevadm trigger
  ```

  (The image is loaded by the first `./install.sh` below, which stops before
  starting anything — do this between the two runs. `<image>` is the
  reference that run prints.)

Then, from the copied bundle directory:

```bash
./install.sh          # loads the image, creates /opt/syncai, then STOPS
```

The first run stops before starting anything and names two files to fill in:

- `/opt/syncai/config/instances/robot.ini` — `robot_id`, the lidar's `type`
  (`mid360` / `mid360s`; a wrong one gives no point cloud and no error) and
  `ip`. Leave `[map] name:` empty: AUTO comes up without a map, and the first
  one is made from the console.
- `/opt/syncai/.env` — `POSTGRES_PASSWORD` (fixed in `data/postgres` at the
  first start), `VIDEO_GID` / `AUDIO_GID` (`getent group video audio`).

```bash
./install.sh          # again: starts postgres, temporal and the robot container
```

It ends by calling `/<robot_id>/get_mode` and printing the answer; AUTO is 2.

The backend, from its own repository, needs `ROBOT_WS=/opt/syncai` in its
`.env`, the same `ROBOT_INSTANCE` and the same postgres credentials.

## 4. Update

Copy the new bundle to the robot and run `./install.sh` in it. The robot
container is replaced; postgres and temporal are left running. **Lie the robot
down first**: the whole stack stops and starts again, the bridge to the gait
controller included.

The image that was running is remembered as `SYNCAI_IMAGE_PREVIOUS` in
`/opt/syncai/.env` and stays on the machine.

## 5. Roll back

```bash
./install.sh --rollback
```

switches back to `SYNCAI_IMAGE_PREVIOUS` — its binaries, its compose file, its
`cyclonedds.xml` and its factory params, all taken out of that image. Maps,
logs, the INI, the overlays and the database are untouched in both directions;
a rollback does not undo a map saved or a parameter set while the newer image
ran. Running it a second time goes forward again.

Old images are never removed automatically. `docker image ls` shows them;
remove one by reference once it is neither `SYNCAI_IMAGE` nor
`SYNCAI_IMAGE_PREVIOUS`.

## Moving a development robot (robot01) to the release image, and back

A robot that runs the stack from a checkout has containers named `robot01`,
`postgres` and `temporal` already; `install.sh` refuses to start over them.

```bash
# in the checkout, robot lying down
docker compose down
sudo mkdir -p /opt/syncai && sudo chown 1000:1000 /opt/syncai
mkdir -p /opt/syncai/config /opt/syncai/data
cp -a config/instances config/state config/params config/calibration /opt/syncai/config/ 2>/dev/null
cp -a map /opt/syncai/
sudo cp -a data/postgres /opt/syncai/data/
cp .env /opt/syncai/.env      # then add ROBOT_INSTANCE / VIDEO_GID / AUDIO_GID if absent
./install.sh                  # from the bundle, or --image <ref>
```

and in the backend's checkout set `ROBOT_WS=/opt/syncai` and recreate its
container. To go back: `docker compose -f /opt/syncai/docker-compose.release.yml
down`, unset `ROBOT_WS`, `docker compose up -d` in the checkout. The two
copies of `map/` and of the database diverge from the moment of the copy.

## What to check the first time a release runs on a robot

The image build and the smoke test cannot see hardware. After a first
install, and after any release that changed the Dockerfile's `runtime-base`
stage:

- the console shows the camera (the feeder's GStreamer pipeline) and the live
  scan;
- relocalize on a map, then one MOVE to completion;
- a `charge.xml` goal reaches the charger (the landmark detector, the exposure
  override);
- MANUAL: start mapping, save a map, and `map_clean.recipe.json` reaches `ok`
  (`clean_map` is a detached process that outlives the mode switch);
- the backend's controller and planner settings pages show defaults and a
  reset works (the exported `src/*/params`);
- wifi scan from the console (`nmcli` through the host's D-Bus).

## Looking at a running robot

```bash
cat /opt/syncai/MANIFEST                       # what is installed
/opt/syncai/attach.sh                          # the live byobu session
tail -f /opt/syncai/log/stack/<robot_id>/planner/current
docker compose -f /opt/syncai/docker-compose.release.yml ps
```

## Limits

- The manifest traces an image to its sources; it does not make the build
  reproducible. apt packages are not pinned, so the same commits built later
  link different system libraries. Keep the image, not just the SHAs.
- The LIO ring recorder (`docker-compose.recorder.yml`) is not part of a
  release; it needs the workspace's `scripts/record/`.
- Lidar-camera calibration: `CALIBRATION` mode records the bags on the robot
  as before; the solve (`direct_visual_lidar_calibration`) runs in a dev image
  on a workstation.
- amd64 is not built. Besides a second image, `syncai_mppi_controller`
  compiles with `-mavx2 -mfma` on x86, so the target CPU would have to have
  both.
