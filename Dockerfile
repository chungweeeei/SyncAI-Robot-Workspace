# =============================================================================
# SyncAI robot workspace — multi-stage build (development).
#
#   base              shared runtime floor (ros-base + cyclonedds + uid-1000 user)
#     ├─ deps-builder   compiler + Boost / TBB / Eigen, shared by the three below
#     │    ├─ livox     Livox-SDK2 ─┐
#     │    ├─ gtsam     GTSAM       ├→ each staged under /out/usr/local (slow, cached)
#     │    └─ sophus    Sophus     ─┘
#     ├─ rust-underlay  Rust toolchain (/opt/rust) + ros2-rust underlay for rclrs
#     │                 (/opt/ros2_rust_underlay)
#     └─ dev            the interactive dev image: colcon, byobu, Node.js, -dev
#                       headers, plus the COPYs out of every stage above.
#                       Workspace bind-mounted at ~/robot_ws and built by hand
#                       (colcon). Compose target: dev.
#
# The dev target keeps today's workflow (workspace mounted at ~/robot_ws, build
# by hand). The source builds are split into stages that depend only on `base`
# so that (a) BuildKit runs them in parallel with each other and with dev's own
# apt layers, and (b) editing a dev layer -- an apt package, the VizionSDK
# version, Node -- never invalidates them: dev only COPYs their output. GTSAM
# (~30-60 min on Tegra) and the underlay (a long colcon build) are the two that
# matter; keep anything that changes often out of their stages.
#
#   docker build --target dev -t syncai-robot .
#   # or, via compose:  docker compose build robot01
#
# NOTE: the production stages (ws-builder / nav-runtime / backend-runtime) that
# baked the colcon install space into slim runtime images were removed while
# the project is in the dev phase. docker-compose.prod.yml and scripts/release/
# still reference them and will not work until the stages are re-added. See git
# history for the removed stages when it's time to ship to the IPC.
# =============================================================================

# colcon's Rust plugins, installed in two stages (rust-underlay builds rclrs
# with them, dev builds the workspace's Rust packages with them). A global ARG
# so both pip installs read one pin; each stage re-declares it to use it.
ARG COLCON_CARGO_PIP="colcon-cargo==0.2.0 colcon-ros-cargo==0.2.0"

# ---------------------------------------------------------------------------
# base: shared by dev and both production runtimes
# ---------------------------------------------------------------------------
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    sudo \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# ROS 2 runtime floor. avahi-utils: syncai_sys_manager spawns avahi-publish
# against the HOST avahi-daemon (via the mounted D-Bus socket); no daemon runs
# in the container. tzdata: containers default to UTC — set local time so log
# timestamps (ros2 launch, byobu panes) match the host / operators.
# ompl: Dubins/Reeds-Shepp state spaces for syncai_global_planner's smac plugins —
# libsyncai_global_planner.so links libompl.so, so it is a runtime dep, not dev-only.
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-tf2-tools \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-nav2-msgs \
    ros-humble-angles \
    ros-humble-ompl \
    ros-humble-nav-2d-msgs \
    ros-humble-dwb-msgs \
    python3-pip \
    iputils-ping \
    avahi-utils \
    tzdata \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Local timezone (overridable per-container via the TZ env var in compose).
# /etc/localtime is linked too so programs that ignore TZ still agree.
ENV TZ=Asia/Taipei
RUN ln -snf "/usr/share/zoneinfo/${TZ}" /etc/localtime && \
    echo "${TZ}" > /etc/timezone

# Allow any uid (overridden via compose `user:` in dev) to sudo without
# password — syncai_sys_manager needs sudo for nmcli against the host
# NetworkManager.
RUN echo "ALL ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ubuntu:22.04 has no default uid-1000 user, so create the `syncrobotic` user
# (named after the host user; uid 1000 matches so bind-mounted files keep the
# right ownership). HOME is world-writable so a runtime-overridden uid can
# still write ~/.ros, ~/.cache, ~/.bash_history.
RUN groupadd -g 1000 syncrobotic && \
    useradd -m -u 1000 -g 1000 -s /bin/bash syncrobotic && \
    chmod -R 777 /home/syncrobotic && \
    echo 'source /opt/ros/humble/setup.bash' >> /home/syncrobotic/.bashrc

# ---------------------------------------------------------------------------
# deps-builder: the toolchain the three source builds below share. It builds
# nothing itself.
#
# Each library is its own stage FROM this one, so BuildKit builds the three in
# parallel, and a bump of one (a Livox SHA, say) rebuilds only that one rather
# than everything after it in a single chain. Each installs with
# DESTDIR=/out, so its stage holds exactly that library's files under
# /out/usr/local and dev picks them up with one COPY per library -- no
# `COPY /usr/local` that would carry whatever else ended up in a builder's
# /usr/local along with it.
# ---------------------------------------------------------------------------
FROM base AS deps-builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libtbb-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Livox-SDK2: livox_ros_driver2 links liblivox_lidar_sdk_shared.so from
# /usr/local (via find_library). Pinned to the commit vendored under
# src/third-party.
FROM deps-builder AS livox
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    git checkout f5d9375f84efe2b15bc0a052d3e18482ed13adf4 && \
    mkdir build && cd build && \
    cmake .. && make -j"$(nproc)" && make install DESTDIR=/out

# GTSAM 4.2.0: syncai_mapping's pgo_node links libgtsam (find_package(GTSAM)).
# No apt/PPA GTSAM on arm64, so build from source into /usr/local. Flags follow
# the LIO-SAM recipe: system Eigen + no march-native to avoid Eigen-alignment
# crashes when mixed with PCL; TBB on; shared libs.
FROM deps-builder AS gtsam
RUN git clone --branch 4.2.0 --depth 1 https://github.com/borglab/gtsam.git /tmp/gtsam && \
    cd /tmp/gtsam && \
    mkdir build && cd build && \
    cmake .. \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_WITH_TBB=ON \
    -DBUILD_SHARED_LIBS=ON && \
    make -j"$(nproc)" && make install DESTDIR=/out

# Sophus 1.22.10: syncai_pointlio needs find_package(Sophus). Header-only;
# SOPHUS_USE_BASIC_LOGGING=ON drops the fmt dependency (matches the
# add_compile_definitions in its CMake).
FROM deps-builder AS sophus
RUN git clone --branch 1.22.10 --depth 1 https://github.com/strasdat/Sophus.git /tmp/Sophus && \
    cd /tmp/Sophus && \
    mkdir build && cd build && \
    cmake .. \
    -DSOPHUS_USE_BASIC_LOGGING=ON \
    -DBUILD_SOPHUS_TESTS=OFF \
    -DBUILD_SOPHUS_EXAMPLES=OFF && \
    make -j"$(nproc)" && make install DESTDIR=/out

# ---------------------------------------------------------------------------
# rust-underlay: the Rust toolchain for rclrs (/opt/rust) and the ros2-rust
# underlay built with it (/opt/ros2_rust_underlay). dev COPYs both trees.
#
# A stage of its own (2026-10) rather than two stanzas near the end of dev,
# where every edit to an earlier dev layer -- GStreamer, VizionSDK, Node, an
# apt package -- re-ran rustup, `cargo install cargo-ament-build` and the
# underlay's whole colcon build. It depends only on `base`, so it now
# rebuilds when its own lines or base change, and builds in parallel with the
# C++ deps and dev's apt layers.
# ---------------------------------------------------------------------------
FROM base AS rust-underlay

# Rust toolchain for rclrs (ros2-rust), the ROS 2 Rust client library.
#
# rclrs is not an apt package and there is no ros-humble-rclrs: the crate comes
# from crates.io through a package's own Cargo.toml, so what the image has to
# provide is the toolchain that builds it inside a colcon workspace:
#   - libclang-dev     : rclrs's build script runs bindgen over the rcl headers.
#                        `clang` alone is not enough — bindgen loads libclang.so
#                        and fails with "Unable to find libclang" without -dev.
#   - rustup / cargo   : pinned via RUST_TOOLCHAIN, like every other third-party
#                        dep in this image.
#   - rustfmt          : `--profile minimal` leaves it out, and colcon-ros-cargo's
#                        `colcon test` runs `cargo fmt --check` next to `cargo
#                        test` -- without the component that test fails with
#                        "'rustfmt' is not installed for the toolchain", which
#                        reads like a formatting failure and is not one. It is
#                        a component of the pinned toolchain, so its version
#                        moves with RUST_TOOLCHAIN.
#   - cargo-ament-build: `cargo ament-build --install-base`, the drop-in for
#                        `cargo build` that lays binaries out per REP 122 so
#                        `ros2 run` / `ros2 launch` find them.
#   - colcon-cargo + colcon-ros-cargo: teach colcon to discover and build a
#                        package.xml + Cargo.toml package. That is three
#                        packages since 2026-10 -- syncai_driver_manager,
#                        syncai_robot_state and syncai_lio_bridge, which vcs
#                        imports from their own repos (see
#                        dependencies.repos);
#                        the rest of src/ has no Cargo.toml and is unaffected.
#
# Installed under /opt/rust rather than ~/.cargo because compose may override
# the uid at runtime (see the syncrobotic user in base); the tree is made
# world-writable for the same reason as /home/syncrobotic — cargo writes its
# registry cache and git checkouts into CARGO_HOME on every build that fetches
# a crate. That chmod happens once, at the end of the underlay RUN below.
#
# libclang-dev and the two colcon plugins are needed twice: here, because the
# underlay's own colcon build compiles rclrs; and again in dev, because the
# workspace's three Rust packages do the same at `colcon build` time. The pip
# versions are pinned in COLCON_CARGO_PIP (top of the file) so the two copies
# cannot drift apart between builds.
#
# Message crates are the next stanza (the ros2-rust underlay), not this one.
ARG RUST_TOOLCHAIN=1.89.0
ARG COLCON_CARGO_PIP
ENV RUSTUP_HOME=/opt/rust/rustup \
    CARGO_HOME=/opt/rust/cargo \
    PATH=/opt/rust/cargo/bin:${PATH}
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libclang-dev \
    python3-colcon-common-extensions \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/* && \
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    sh -s -- -y --no-modify-path --profile minimal --default-toolchain "${RUST_TOOLCHAIN}" \
        --component rustfmt && \
    cargo install --locked cargo-ament-build && \
    pip3 install --no-cache-dir ${COLCON_CARGO_PIP} && \
    rm -rf "${CARGO_HOME}/registry" "${CARGO_HOME}/git"

# ros2-rust underlay: rclrs + the Rust message generator, built into
# /opt/ros2_rust_underlay and sourced between /opt/ros/humble and the workspace.
#
# The apt-installed interfaces ship no Rust bindings, and rclrs needs a message
# crate for every interface package a node uses (std_msgs, sensor_msgs,
# geometry_msgs, std_srvs, nav_msgs, syncai_common, ...). Those crates are
# generated by rosidl_generator_rs, which only runs on interface packages built
# AFTER it in a colcon chain -- so the generator and the source of every
# standard message package a Rust node touches are rebuilt here, once, at image
# build time. syncai_common is the one interface package that is not: it is
# built in the workspace on top of this underlay, and picks the generator up
# through the rebuilt rosidl_default_generators.
#
# In the image rather than as a workspace .repos import, because nothing here
# edits these repos -- they are toolchain, like GTSAM / Sophus, and a
# workspace checkout would put ~30 upstream packages into every clean
# `colcon build` on the Jetson. The recipe is upstream's: ros2-rust/ros2_rust's
# own ros2_rust_humble.repos, which is also what the three Rust repos'
# dev containers build into the same /opt/ros2_rust_underlay. Three differences:
#
#   - `ros2-rust/examples` is dropped (demo nodes; the dev containers drop it too).
#   - rclrs itself (ros2-rust/ros2_rust) is added and built from source.
#     rclrs 0.7.0 on crates.io depends on rosidl_runtime_rs ^0.6, the generator
#     on main emits 0.7, and the two do not compile together. Both nodes ask
#     for `rclrs = "0.8"`, which is what ros2_rust main is; colcon-ros-cargo
#     patches the crates.io name onto this copy.
#   - The three ros2-rust repos are pinned to SHAs rather than `main` (the dev
#     containers float). The rclrs / rosidl_runtime_rs / generator triple has
#     already broken once on a version skew between them, and a floating
#     `main` would let an unrelated image rebuild break it again. Bump the
#     three together (rclrs 0.8.0 / rosidl_runtime_rs 0.7.0 at these pins,
#     2026-10-02). The ros2/* repos follow upstream's `humble` branches: they
#     only take Humble patch releases.
#
# Side effect on the C++ packages: the rebuilt common_interfaces /
# rcl_interfaces overlay the apt copies of std_msgs, geometry_msgs, nav_msgs,
# builtin_interfaces, ... for every shell that sources this underlay, so the
# whole workspace, C++ included, now finds them here rather than in
# /opt/ros/humble. Same Humble branch, so same ABI -- but it is the first place
# to look if a C++ package misbehaves after an apt upgrade moves /opt/ros/humble
# ahead of the image.
#
# tf2_msgs is deliberately not rebuilt: ros-humble-tf2-msgs already ships
# generated Rust bindings, which is what syncai_robot_state's and
# syncai_lio_bridge's hand-rolled /tf handling links against (rclrs has no tf2_ros binding). Add geometry2 only if a
# future base image stops shipping them.
#
# build/ and log/ are dropped; install/ is all a consumer reads. The cargo
# registry is dropped for the same reason as in the toolchain stanza above.
#
# `chmod -R a+w /opt/rust` runs exactly once, here at the very end. When this
# stanza was part of dev, the toolchain RUN and this one each ran it, and the
# second walk re-wrote the mode of every file under /opt/rust -- which makes
# the layer carry a full copy of the 516 MB toolchain whether or not the mode
# changed. That is what made the old underlay layer 599 MB for a 77 MB
# install/. dev now COPYs the final trees, so no intermediate layer here
# reaches the image anyway; one chmod is still the right number.
ARG ROS2_RUST_SHA=1d361a8c5f0e69d530feb62f93b92e46dff369a4
ARG ROSIDL_RUST_SHA=19d57818dab3b51e418c0893b70b3a1a8b64c495
ARG ROSIDL_RUNTIME_RS_SHA=21def427689fda0156dd4b0dff28d8d0318af03b
ENV ROS2_RUST_UNDERLAY=/opt/ros2_rust_underlay
RUN mkdir -p "${ROS2_RUST_UNDERLAY}/src" && cd "${ROS2_RUST_UNDERLAY}" && \
    git clone https://github.com/ros2-rust/ros2_rust.git src/ros2-rust/ros2_rust && \
    git -C src/ros2-rust/ros2_rust checkout "${ROS2_RUST_SHA}" && \
    vcs import src < src/ros2-rust/ros2_rust/ros2_rust_humble.repos && \
    rm -rf src/ros2-rust/examples && \
    git -C src/ros2-rust/rosidl_rust checkout "${ROSIDL_RUST_SHA}" && \
    git -C src/ros2-rust/rosidl_runtime_rs checkout "${ROSIDL_RUNTIME_RS_SHA}" && \
    . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log "${CARGO_HOME}/registry" "${CARGO_HOME}/git" && \
    chmod -R a+w /opt/rust

# ---------------------------------------------------------------------------
# dev: the interactive development image (compose service robot01,
# image syncai-robot-base, target: dev): tooling, workspace bind-mounted at
# runtime, colcon build run by hand.
# ---------------------------------------------------------------------------
FROM base AS dev

# Build toolchain, PCL/ROS build deps, and operator conveniences.
#
# There is no rviz2 (removed 2026-10): the robot has no display, and rviz2 is
# run from a workstation against config/rviz2/<robot_id>.rviz.
# syncai_costmap_2d's optional `rviz:=true` test launch is its only caller in
# the workspace -- run that one where rviz2 is installed. Dropping it saved
# only ~50 MB: the bulk of this layer is VTK + Qt5, which libpcl-dev hard-
# Depends on (libvtk9-dev / libvtk9-qt-dev, and through them python3-vtk9,
# libvtk9-java, libgdal-dev, MPI). No workspace binary links either (ldd,
# 2026-10-10), but libpcl-dev is what pcl_conversions and every
# find_package(PCL) here need, so apt cannot leave them out.
#
# ros-humble-pcl-ros and ros-humble-pointcloud-to-laserscan were removed at
# the same time. Nothing depends on pcl_ros (the PCL users -- pointlio,
# mapping, localizer, small_gicp, livox_ros_driver2's ROS 2 branch -- all go
# through pcl_conversions), and pointcloud_to_laserscan was the 2D/AMCL
# path's, in no launch or session spec since bringup_2d.launch.py went.
#
# ros-humble-compressed-image-transport is not optional for the camera: the
# camera node publishes *only* sensor_msgs/CompressedImage on
# `<robot_id>/image_raw/compressed`, and bare `image_transport` declares the
# raw transport alone. Without this plugin an image_transport subscriber in
# here has no way to subscribe at all and simply gets nothing -- no error, no
# warning.
#
# python3-opencv and python3-dotenv were here for syncai_backend (cv2 encoded
# the map images its /image route serves; dotenv read the workspace .env) and
# left with it in 2026-09 — nothing in this workspace imports either now.
RUN apt-get update && apt-get install -y \
    ros-humble-compressed-image-transport \
    ros-humble-pcl-conversions \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    byobu \
    daemontools \
    net-tools \
    network-manager \
    bluez \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# System deps for workspace packages that have no ament/CMake config:
#   - libgraphicsmagick++1-dev: syncai_map_server (located via pkg-config)
#   - libzmq3-dev / libncurses-dev: behaviortree_cpp
#   - nlohmann-json3-dev: header-only JSON library (rosdep key
#     nlohmann-json-dev). Its consumer is syncai_mapping (imported by
#     dependencies.repos), whose clean_map reads and writes
#     map_clean.recipe.json with it. It used to be syncai_robot_state, which
#     left in 2026-10 and flattens WifiStatus with serde_json now; the line
#     was nearly dropped as unused in between -- do not.
#   - libapr1-dev / libaprutil1-dev: livox_ros_driver2
#   - libboost-all-dev / libtbb-dev / libeigen3-dev: GTSAM/Sophus headers
#     (the libs themselves come prebuilt from the gtsam / sophus stages)
#   - ros-humble-octomap: syncai_mapping's clean_map -- the post-save map
#     cleaning pgo_node spawns after save_maps (2026-10), which uses OctoMap's
#     ray traversal and voxel keys; rosdep key `octomap`. It had a stanza of
#     its own at the end of this stage while the underlay was built in dev,
#     so that adding it did not re-run that build; the underlay is a stage of
#     its own now, so it lives here with the other build deps.
#   - ros-humble-map-msgs: syncai_costmap_2d <depend>s on it (the costmap
#     update topics). It used to arrive only transitively, through
#     rviz2's default plugins, so dropping rviz2 in 2026-10 failed that
#     package's configure; `rosdep check` names it if it goes missing again.
#   - ros-humble-laser-geometry: the same story, for syncai_costmap_2d's
#     obstacle layer (LaserScan -> PointCloud2). It came in through
#     ros-humble-pointcloud-to-laserscan, removed in the same change.
RUN apt-get update && apt-get install -y \
    libgraphicsmagick++1-dev \
    libzmq3-dev \
    libncurses-dev \
    nlohmann-json3-dev \
    libapr1-dev \
    libaprutil1-dev \
    libboost-all-dev \
    libtbb-dev \
    libeigen3-dev \
    ros-humble-octomap \
    ros-humble-map-msgs \
    ros-humble-laser-geometry \
    && rm -rf /var/lib/apt/lists/*

# xtensor / xsimd: syncai_mppi_controller (the MPPI port) evaluates its batch
# of sampled trajectories as xtensor expressions, vectorised through xsimd
# (NEON on the Jetson). Both are header-only, so this is a build dependency
# and lives in dev, not base. Jammy ships xtensor 0.23 / xsimd 7.6, the pair
# Humble's nav2_mppi_controller is built against (rosdep keys xtensor /
# xsimd); a newer xtensor from source would need the upstream port's
# post-Humble API changes too. Its own layer, so the one above keeps its cache.
RUN apt-get update && apt-get install -y \
    libxtensor-dev \
    libxsimd-dev \
    && rm -rf /var/lib/apt/lists/*

# GStreamer for the camera stream. The base image carries only
# gstreamer1.0-plugins-base, which is why a pipeline built here fails with
# `no element "v4l2src"`:
#   - plugins-good  : v4l2src (V4L2 capture)
#   - plugins-bad   : h264parse (videoparsersbad)
#   - gstreamer1.0-rtsp : rtspclientsink, to publish into mediamtx
#   - gstreamer1.0-tools: gst-inspect-1.0 / gst-launch-1.0, without which there
#                         is no way to tell a missing element from a missing
#                         command when debugging a pipeline in here
#   - gstreamer1.0-alsa : alsasink, for the WebRTC worker's WHIP branch (the
#                         operator's microphone out of the USB speaker). NOT in
#                         plugins-base despite living in that source package,
#                         and its absence fails the pipeline string at parse
#                         time rather than at playback. pulsesink exists in
#                         this image and is a trap: there is no PulseAudio
#                         daemon here, and the speaker is reached as
#                         plughw:CARD=CD002AUDIO, the same by-name device the
#                         TTS gateway resolves to.
#
# NOT included, and not installable from apt: the Tegra elements (nvjpegdec,
# nvvidconv, nvv4l2h264enc) live in nvidia-l4t-gstreamer and there is no L4T apt
# repo in this image. They are instead injected by the nvidia container runtime,
# which already lists them in /etc/nvidia-container-runtime/host-files-for-
# container.d/drivers.csv on the host — but only when the container requests it
# via NVIDIA_VISIBLE_DEVICES / NVIDIA_DRIVER_CAPABILITIES. Hardware encoding in
# here additionally needs /dev/video0 passed through and membership of the host's
# video group; see docker-compose.robots.yml.
RUN apt-get update && apt-get install -y --no-install-recommends \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-rtsp \
    gstreamer1.0-tools \
    gstreamer1.0-alsa \
    && rm -rf /var/lib/apt/lists/*

# aplay, to check the robot's speaker from a shell in here (`aplay -l`, then
# play something at it). It used to be a dependency: the in-tree backend's TTS
# gateway shelled out to aplay for its /speak route rather than pull in a
# Python audio stack. That backend moved to SyncAI-Robot-Backend in 2026-09 and
# its container owns the speaker now, so nothing in this image plays audio on
# its own — the binary stays because the passthrough it needs (/dev/snd plus
# the host audio group, from docker-compose.robots.yml) is still wired up and a
# silent speaker is quicker to diagnose from the container that owns the robot
# than from one that does not.
RUN apt-get update && apt-get install -y --no-install-recommends \
    alsa-utils \
    && rm -rf /var/lib/apt/lists/*

# VizionSDK: the closed-source TechNexion camera SDK that
# src/third-party/vizionsdk-ros2 links against. It has no rosdep key and is in
# no distro repo, so `rosdep install` does not cover it and the wrapper's
# CMakeLists dies at configure time with `No package 'vizionsdk' found`.
#
# The .deb ships a CMake config package in /usr/lib/cmake/vizionsdk exporting
# vizionsdk::VizionSDK, so the wrapper's first branch
# (`find_package(vizionsdk CONFIG)`) resolves it and none of the
# CMAKE_PREFIX_PATH / PKG_CONFIG_PATH / LD_LIBRARY_PATH exports its README
# suggests are needed — those are for the custom-prefix (/opt) install.
#
# Installed with `apt-get install ./x.deb` rather than `dpkg -i` so the
# Depends (libusb-1.0-0, libudev1, v4l-utils, gstreamer) resolve in the same
# step; the release ships one .deb per arch and the filename is not derivable
# from uname, hence the case on dpkg's arch, as in the ROS apt line in base.
#
# Two postinst side effects worth knowing about:
#   - it writes /etc/udev/rules.d/88-cyusb.rules for Cypress FX3 USB cameras.
#     Inert here (no udevd in the container) and unrelated to the CSI camera on
#     /dev/video0, so it is left in place rather than fought.
#   - it adds download.technexion.com as an apt source. That is deleted right
#     after: every other third-party dep in this image is version-pinned, and a
#     live vendor repo would make the nodesource `apt-get update` below fail
#     whenever that host is unreachable.
ARG VIZIONSDK_VERSION=26.8.1
RUN case "$(dpkg --print-architecture)" in \
    arm64) VIZIONSDK_DEB="vizionsdk-linuxarm64-${VIZIONSDK_VERSION}.deb" ;; \
    amd64) VIZIONSDK_DEB="vizionsdk-linux64-${VIZIONSDK_VERSION}.deb" ;; \
    *) echo "VizionSDK: no release for $(dpkg --print-architecture)" >&2; exit 1 ;; \
    esac && \
    curl -fsSL -o "/tmp/${VIZIONSDK_DEB}" \
    "https://github.com/TechNexion-Vision/vizionsdk/releases/download/v${VIZIONSDK_VERSION}/${VIZIONSDK_DEB}" && \
    apt-get update && apt-get install -y --no-install-recommends "/tmp/${VIZIONSDK_DEB}" && \
    rm -f "/tmp/${VIZIONSDK_DEB}" /etc/apt/sources.list.d/vizionsdk.list && \
    ldconfig && \
    rm -rf /var/lib/apt/lists/*

# Prebuilt Livox-SDK2 / GTSAM / Sophus, one COPY per builder stage (each
# stage holds only its own library under /out/usr/local).
COPY --from=livox /out/usr/local /usr/local
COPY --from=gtsam /out/usr/local /usr/local
COPY --from=sophus /out/usr/local /usr/local
RUN ldconfig

# NOTE: there is no pip install of a web stack here any more. This image used to
# carry syncai_backend's dependencies (fastapi / uvicorn / sqlalchemy /
# temporalio / open3d / kokoro-onnx, from that package's requirements.txt),
# COPYed out of src/ at build time. The backend moved to SyncAI-Robot-Backend in
# 2026-09 and pins them in its own image, so this one is back to a pure ROS
# image and the COPY — which would now fail on a fresh clone, there being no
# src/syncai_backend to copy from — is gone with it. Anything Python that ships
# in this workspace (syncai_sys_manager) needs only rclpy and the standard
# library. Do not re-add a package here for a process that runs in another
# container.

# Node.js 22, for `scripts/urdf2glb.py`'s `npx gltfpack` step (the operator
# console's robot mesh is still baked from this repo's URDF). Nothing in this
# workspace serves a web app any more — syncai_frontend left in 2026-09 — so
# only the node/npm runtime needs to live in the image, not a project.
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

# Rust toolchain + ros2-rust underlay, from the rust-underlay stage (see there
# for what is in each tree and why). What has to be installed here as well is
# what the workspace's own Rust packages use at `colcon build` time:
# libclang-dev for rclrs's bindgen, and the colcon-cargo / colcon-ros-cargo
# plugins (pinned in COLCON_CARGO_PIP, same as the stage) for colcon to
# discover and build a package.xml + Cargo.toml package at all. The COPYs keep
# the stage's world-writable modes, which cargo needs under a runtime uid.
ARG COLCON_CARGO_PIP
ENV RUSTUP_HOME=/opt/rust/rustup \
    CARGO_HOME=/opt/rust/cargo \
    PATH=/opt/rust/cargo/bin:${PATH} \
    ROS2_RUST_UNDERLAY=/opt/ros2_rust_underlay
RUN apt-get update && apt-get install -y --no-install-recommends \
    libclang-dev \
    && rm -rf /var/lib/apt/lists/* && \
    pip3 install --no-cache-dir ${COLCON_CARGO_PIP}
COPY --from=rust-underlay /opt/rust /opt/rust
COPY --from=rust-underlay /opt/ros2_rust_underlay /opt/ros2_rust_underlay
RUN echo "source ${ROS2_RUST_UNDERLAY}/install/setup.bash" >> /home/syncrobotic/.bashrc

# rosdep: `init` needs root (it writes /etc/ros/rosdep/sources.list.d), but the
# cache `update` builds is per user, in ~/.ros -- so it runs once, as
# syncrobotic, below. There used to be a root `rosdep update` here as well; its
# cache landed in /root/.ros, which no shell in this image reads, so it was a
# network round-trip on every rebuild of this tail for nothing.
RUN rosdep init || true

USER syncrobotic
WORKDIR /home/syncrobotic

# Populate the rosdep cache for the syncrobotic user, so `rosdep check` /
# `rosdep install` work at runtime. Made world-readable and -writable for the
# same reason as /home/syncrobotic in base: rosdep rewrites its cache index on
# every check, and under a runtime uid other than 1000 (docker-compose.build.yaml
# on a Mac runs as 501) a 1000-owned cache fails with "Permission denied:
# .../sources.cache/index" before checking anything. a+rwX, not a+w: the
# cache files are written 0600, and a+w alone leaves them -rw--w--w-, which
# another uid can write but not read -- the same error.
RUN rosdep update --rosdistro humble && chmod -R a+rwX /home/syncrobotic/.ros

# Auto-source the mounted workspace overlay in every shell.
RUN echo '[ -f ~/robot_ws/install/setup.bash ] && source ~/robot_ws/install/setup.bash' >> ~/.bashrc

CMD ["bash"]
