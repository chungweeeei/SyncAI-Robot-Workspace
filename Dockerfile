# =============================================================================
# SyncAI robot workspace — multi-stage build, two images out of one file.
#
#   base              shared floor (ros-base + cyclonedds + uid-1000 user)
#     ├─ deps-builder   compiler + Boost / TBB / Eigen, shared by the five below
#     │    ├─ livox       Livox-SDK2  ─┐
#     │    ├─ gtsam       GTSAM        │
#     │    ├─ sophus      Sophus       ├→ each staged under /out/usr/local (slow, cached)
#     │    ├─ ceres       Ceres        │
#     │    └─ iridescence Iridescence ─┘
#     ├─ rust-underlay  Rust toolchain (/opt/rust) + ros2-rust underlay for rclrs
#     │                 (/opt/ros2_rust_underlay)
#     └─ runtime-base   what the built workspace needs to RUN: the linked
#          │            libraries, GTSAM, the underlay, byobu / multilog / nmcli,
#          │            the GStreamer plugins. Adds no build tool and no -dev
#          │            package to the ones ros-base itself depends on (gcc and
#          │            cmake are in base through it, and stay).
#          ├─ dev          the interactive dev image: runtime-base + colcon,
#          │    │          Node.js, -dev headers, the Rust toolchain and the
#          │    │          other source-built libraries. Workspace bind-mounted
#          │    │          at ~/robot_ws and built by hand. Compose target: dev.
#          │    └─ builder   dev + src/, a Release `colcon build` without
#          │                 --symlink-install. Never shipped.
#          └─ release      the image that goes to a robot (2026-11):
#                          runtime-base + builder's install/ + the session specs.
#                          No source, no workspace mount.
#
# The dev target keeps today's workflow (workspace mounted at ~/robot_ws, build
# by hand). The source builds are split into stages that depend only on `base`
# so that (a) BuildKit runs them in parallel with each other and with dev's own
# apt layers, and (b) editing a dev layer -- an apt package, the VizionSDK
# version, Node -- never invalidates them: dev only COPYs their output. GTSAM
# (~30-60 min on Tegra) and the underlay (a long colcon build) are the two that
# matter; keep anything that changes often out of their stages.
#
#   docker build --target dev -t syncai-robot-base .
#   # or, via compose:  docker compose build robot01
#
# The release target is built by CI only (.github/workflows/release.yml), from
# a checkout with both .repos files imported and scripts/release/prepare.sh
# run; docs/release-deployment.md has the way from there to a robot.
#
# There was a note here until 2026-11 that the production stages (ws-builder /
# nav-runtime / backend-runtime) could be recovered from git history. They
# could not: no revision ever held them, nor a docker-compose.prod.yml. The
# builder / release stages at the end of this file were written from scratch.
# =============================================================================

# colcon's Rust plugins, installed in two stages (rust-underlay builds rclrs
# with them, dev builds the workspace's Rust packages with them). A global ARG
# so both pip installs read one pin; each stage re-declares it to use it.
ARG COLCON_CARGO_PIP="colcon-cargo==0.2.0 colcon-ros-cargo==0.2.0"

# ---------------------------------------------------------------------------
# base: shared by every stage below, the source builds included
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
# Removed 2026-10-28 with 2D navigation: ros-humble-ompl (Smac's smoother, the
# only OMPL user), ros-humble-nav-2d-msgs and ros-humble-dwb-msgs (nothing in
# the workspace read either); no package needs them.
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-tf2-tools \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-nav2-msgs \
    ros-humble-angles \
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
#
# GTSAM_BUILD_UNSTABLE=ON (2026-10) for direct_visual_lidar_calibration
# (src/third-party/): its own FindGTSAM.cmake reports GTSAM found only when
# libgtsam_unstable is there too, and fails configure otherwise -- it links the
# library whether or not it uses it. libgtsam_unstable is a separate .so, so
# libgtsam itself and syncai_mapping's pgo_node are unchanged. Flipping it
# invalidated this stage's cache once: the next image build rebuilt GTSAM.
FROM deps-builder AS gtsam
RUN git clone --branch 4.2.0 --depth 1 https://github.com/borglab/gtsam.git /tmp/gtsam && \
    cd /tmp/gtsam && \
    mkdir build && cd build && \
    cmake .. \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_UNSTABLE=ON \
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

# Ceres 2.2.0: direct_visual_lidar_calibration (src/third-party/) parameterises
# its SE(3) pose with ceres::Manifold, which Ceres added in 2.1. Jammy's
# libceres-dev is 2.0.0 and has only the LocalParameterization it replaces, so
# it is built here. 2.2.0 rather than master because master requires Abseil,
# which jammy does not ship at the version it wants.
#
# Flags: SUITESPARSE / LAPACK / CUDA off -- the calibration's problems are a
# single 7-parameter pose, solved densely, so none of them would be used, and
# leaving them out keeps libceres' runtime dependencies to glog (and gflags,
# which glog pulls in). dev therefore needs libgoogle-glog-dev and nothing
# else for it: CeresConfig.cmake re-finds glog for every consumer.
FROM deps-builder AS ceres
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/* && \
    git clone --branch 2.2.0 --depth 1 https://github.com/ceres-solver/ceres-solver.git /tmp/ceres && \
    cd /tmp/ceres && \
    mkdir build && cd build && \
    cmake .. \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARKS=OFF \
    -DSUITESPARSE=OFF \
    -DLAPACK=OFF \
    -DUSE_CUDA=OFF && \
    make -j"$(nproc)" && make install DESTDIR=/out

# Iridescence v1.0.3: koide3's OpenGL visualizer, which
# direct_visual_lidar_calibration (src/third-party/) links unconditionally --
# its initial_guess_manual / calibrate / viewer tools are GUIs on it. Not in
# any apt repo. Cloned --recursive because imgui / implot / ImGuizmo /
# portable-file-dialogs are submodules it compiles in.
#
# glm and glfw3 come from apt so its CMake does not FetchContent them from
# GitHub at configure time (it silently downloads whichever is missing). Only
# glfw3, PNG, JPEG and OpenGL are needed again by a consumer
# (iridescence-config.cmake re-finds OpenGL / PNG / JPEG), which is why dev
# installs those and not glm.
#
# The robot has no display, so the GUI tools need the X11 passthrough
# docker-compose.robots.yml already carries, or a workstation.
FROM deps-builder AS iridescence
RUN apt-get update && apt-get install -y \
    libglm-dev \
    libglfw3-dev \
    libpng-dev \
    libjpeg-dev \
    libgl-dev \
    libopengl-dev \
    && rm -rf /var/lib/apt/lists/* && \
    git clone --branch v1.0.3 --depth 1 --recursive --shallow-submodules \
        https://github.com/koide3/iridescence.git /tmp/iridescence && \
    cd /tmp/iridescence && \
    mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_WITH_MARCH_NATIVE=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF && \
    make -j"$(nproc)" && make install DESTDIR=/out

# ---------------------------------------------------------------------------
# rust-underlay: the Rust toolchain for rclrs (/opt/rust) and the ros2-rust
# underlay built with it (/opt/ros2_rust_underlay). runtime-base COPYs the
# underlay, which the built nodes link; dev COPYs the toolchain on top.
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
#
# rclrs's and rosidl_runtime_rs's build.rs are patched before the build, both
# the same way: each one emits a `rustc-link-search` for every
# AMENT_PREFIX_PATH entry but declares only `rerun-if-env-changed=ROS_DISTRO`,
# so cargo keeps the search paths of whichever AMENT_PREFIX_PATH the build
# directory was first built with. ros-env, which generates the message modules,
# does rerun on AMENT_PREFIX_PATH. A workspace built once without
# install/setup.bash sourced (scripts/build.sh in the build service) and then
# with it (robot01's ~/.bashrc sources it) therefore pulls livox_ros_driver2's
# message crate in through ros-env, links its C typesupport, and fails at the
# link step: "cannot find -llivox_ros_driver2__rosidl_generator_c", although
# the library sits in install/livox_ros_driver2/lib (checked 2026-10-12, the
# Rust packages had never had livox in their link line). The patch adds the
# missing `rerun-if-env-changed=AMENT_PREFIX_PATH`. Upstream main still lacks
# it as of that date. Drop this when a pin bump brings it in. The grep makes a
# bump that moves the line fail here instead of building unpatched.
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
    for f in src/ros2-rust/ros2_rust/rclrs/build.rs \
             src/ros2-rust/rosidl_runtime_rs/rosidl_runtime_rs/build.rs; do \
        sed -i 's|^\( *\)println!("cargo:rerun-if-env-changed={ROS_DISTRO}");|&\n\1println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");|' "$f" && \
        grep -q 'rerun-if-env-changed=AMENT_PREFIX_PATH' "$f" || \
            { echo "rerun-if-env-changed patch did not apply to $f" >&2; exit 1; }; \
    done && \
    . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log "${CARGO_HOME}/registry" "${CARGO_HOME}/git" && \
    chmod -R a+w /opt/rust

# ---------------------------------------------------------------------------
# runtime-base: everything the built workspace needs to RUN, and nothing it
# needs only to build. Both images are made from it (2026-11):
#
#   dev     = runtime-base + compilers, -dev headers, Rust toolchain, colcon
#   release = runtime-base + the install/ the builder stage compiled
#
# One stage under both, rather than a release stage with an apt list of its
# own, for two reasons. The release image then runs on the very layers the
# builder linked against -- two `apt-get install`s on different days can
# resolve one package name to two versions, and packages.ros.org syncs move
# every ros-humble-* at once. And there is one list to add a runtime dependency
# to: put it here and both images have it.
#
# Which stage a new package goes in: if a node needs it while running, here;
# if only `colcon build` does (a -dev package, a header-only library, a
# generator), dev. A linked library put in dev by mistake still builds and
# still runs on robot01, and the release stage's ldd check then fails the
# release build with its name. Nothing catches the same mistake for something
# loaded at run time (a GStreamer element, a tool a node shells out to) until
# that node starts in the release image -- so when in doubt, here.
# ---------------------------------------------------------------------------
FROM base AS runtime-base

# The shared libraries the workspace's binaries link, beyond what base already
# brings. Not a guess: this is what `ldd` over every ELF file in install/
# resolved to on 2026-11-19 (74 files, direct_visual_lidar_calibration and
# vizionsdk_ros2 left out as the release leaves them out), reduced to the
# packages that neither base nor another line here already implies:
#   - libboost-context / libzmq5       : behaviortree_cpp_v3, and through it
#                                        syncai_behavior_tree / _task_runner
#   - libboost-serialization / -timer  : GTSAM's own, for syncai_mapping
#   - libpcl-filters / libpcl-io       : syncai_pointlio / _localizer / _mapping.
#       libpcl-io is what brings libvtk9.1 (97 MB) -- the one runtime library,
#       not the VTK + Qt5 -dev tree that libpcl-dev drags into dev.
#   - libopencv-contrib4.5d            : syncai_landmark_detection (aruco)
#   - libgstreamer-plugins-base1.0-0   : syncai_camera_feeder (gstapp)
#   - ros-humble-octomap               : syncai_global_planner's Octo3D and
#                                        syncai_mapping's clean_map
#   - ros-humble-map-msgs              : syncai_costmap's update topics
#   - ros-humble-pcl-msgs              : linked by the rclrs nodes
#   - python3-psutil / python3-netifaces: imported by syncai_sys_manager. base
#       has both already, but only as dependencies of ros2doctor and ros2cli;
#       named so that they do not leave with a change to either.
# With these the image's apt packages come to about 1.1 GB, against 3.7 GB in
# dev. To redo the list after a new dependency: scripts/release/check_libs.sh
# prints what does not resolve, and the release stage runs it.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-context1.74.0 \
    libboost-serialization1.74.0 \
    libboost-timer1.74.0 \
    libzmq5 \
    libpcl-filters1.12 \
    libpcl-io1.12 \
    libopencv-contrib4.5d \
    libgstreamer-plugins-base1.0-0 \
    ros-humble-octomap \
    ros-humble-map-msgs \
    ros-humble-pcl-msgs \
    python3-psutil \
    python3-netifaces \
    && rm -rf /var/lib/apt/lists/*

# What the stack shells out to, and what an operator needs in a shell in here:
#   - byobu (tmux)      : NodeManager builds each operating mode as a byobu
#                         session; without it switch_mode starts nothing
#   - daemontools       : multilog, which every pane's output is piped into
#                         (log/stack/<robot_id>/...)
#   - network-manager   : nmcli, for syncai_sys_manager's wifi services against
#                         the host's NetworkManager over the mounted D-Bus socket
#   - net-tools, bluez  : ifconfig / netstat and the bluetooth tools, to look
#                         at the host's interfaces from in here
#   - ros-humble-teleop-twist-keyboard: the fallback teleop when no backend is
#                         up (remapped onto /<robot_id>/cmd_vel by hand)
# In dev's first apt layer until 2026-11; a robot running the release image
# needs every one of them just as much.
RUN apt-get update && apt-get install -y \
    ros-humble-teleop-twist-keyboard \
    byobu \
    daemontools \
    net-tools \
    network-manager \
    bluez \
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
# Elements are loaded by name when a pipeline is parsed, so nothing here shows
# in ldd: the release stage's check cannot see one go missing, and the CI smoke
# test does not start the camera feeder. A change to this list is checked on a
# robot, with the camera.
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

# GTSAM, the one source-built library a release binary links (libgtsam and
# libmetis-gtsam, by syncai_mapping's pgo_node). The stage's whole tree, the
# 3.5 MB of headers included, so that dev needs no second COPY of it: split
# into lib/ here and include/ there, the two would have to be kept in step for
# nothing. Livox-SDK2, Sophus, Ceres and Iridescence are dev's -- the workspace
# builds its own Livox-SDK2 into install/, Sophus is header-only, and the other
# two are linked by direct_visual_lidar_calibration alone.
COPY --from=gtsam /out/usr/local /usr/local
RUN ldconfig

# The ros2-rust underlay (see the rust-underlay stage): the rebuilt standard
# interfaces every binary here links -- the C++ ones too, since this overlays
# /opt/ros/humble for the whole workspace -- and the rclrs nodes' message
# libraries. The Rust toolchain (/opt/rust) is dev's. Sourced after
# /opt/ros/humble and before the workspace, in that order, in every shell: the
# session panes take their ROS environment from ~/.bashrc and nowhere else.
ENV ROS2_RUST_UNDERLAY=/opt/ros2_rust_underlay
COPY --from=rust-underlay /opt/ros2_rust_underlay /opt/ros2_rust_underlay
RUN echo "source ${ROS2_RUST_UNDERLAY}/install/setup.bash" >> /home/syncrobotic/.bashrc && \
    echo '[ -f ~/robot_ws/install/setup.bash ] && source ~/robot_ws/install/setup.bash' \
        >> /home/syncrobotic/.bashrc

# ---------------------------------------------------------------------------
# dev: the interactive development image (compose service robot01,
# image syncai-robot-base, target: dev): tooling, workspace bind-mounted at
# runtime, colcon build run by hand. runtime-base plus what a build needs.
# ---------------------------------------------------------------------------
FROM runtime-base AS dev

# Build toolchain and PCL/ROS build deps. (The operator conveniences that used
# to share this layer -- byobu, multilog, nmcli -- are runtime-base's.)
#
# There is no rviz2 (removed 2026-10): the robot has no display, and rviz2, if
# used at all, runs on a workstation (config/rviz2/ went 2026-10-28). (The costmap
# package's `rviz:=true` test launch, its last caller in the workspace, went
# with 2D navigation on 2026-10-28.) Dropping it saved
# only ~50 MB: the bulk of this layer is VTK + Qt5, which libpcl-dev hard-
# Depends on (libvtk9-dev / libvtk9-qt-dev, and through them python3-vtk9,
# libvtk9-java, libgdal-dev, MPI). No workspace binary links either (ldd,
# 2026-10-10), but libpcl-dev is what pcl_conversions and every
# find_package(PCL) here need, so apt cannot leave them out.
#
# ros-humble-pcl-ros and ros-humble-pointcloud-to-laserscan were removed at
# the same time. None of our packages depends on pcl_ros (the PCL users --
# pointlio, mapping, localizer, small_gicp, livox_ros_driver2's ROS 2 branch --
# all go through pcl_conversions), and pointcloud_to_laserscan was the 2D/AMCL
# path's, in no launch or session spec since bringup_2d.launch.py went.
# pcl_ros came back later in the month, in the calibration layer below, for
# direct_visual_lidar_calibration alone.
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
#
# compressed-image-transport stays here, not in runtime-base: no package the
# release carries uses image_transport (the landmark detector subscribes to
# the CompressedImage topic itself), so it is a tool for a shell in dev.
RUN apt-get update && apt-get install -y \
    ros-humble-compressed-image-transport \
    ros-humble-pcl-conversions \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# System deps for workspace packages that have no ament/CMake config:
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
# ros-humble-octomap and ros-humble-map-msgs were in this list until 2026-11
# and are in runtime-base now: both ship the libraries along with the headers,
# and a release binary links them. Two things about them worth keeping:
# octomap's consumer besides Octo3D is syncai_mapping's clean_map (the
# post-save map cleaning pgo_node spawns after save_maps, rosdep key
# `octomap`); and map-msgs used to arrive only transitively, through rviz2's
# default plugins, so dropping rviz2 in 2026-10 failed syncai_costmap's
# configure -- `rosdep check` names it if it goes missing again.
# Removed 2026-10-28 with 2D navigation: libgraphicsmagick++1-dev
# (syncai_map_server's map_io, pkg-config) and ros-humble-laser-geometry (the
# costmap's obstacle layer, LaserScan -> PointCloud2); no package needs them.
RUN apt-get update && apt-get install -y \
    libzmq3-dev \
    libncurses-dev \
    nlohmann-json3-dev \
    libapr1-dev \
    libaprutil1-dev \
    libboost-all-dev \
    libtbb-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# xtensor / xsimd: syncai_mppi_controller (the MPPI port) evaluates its batch
# of sampled trajectories as xtensor expressions, vectorised through xsimd
# (NEON on the Jetson). Both are header-only, so this is a build dependency
# and lives in dev, not runtime-base. Jammy ships xtensor 0.23 / xsimd 7.6, the
# pair Humble's nav2_mppi_controller is built against (rosdep keys xtensor /
# xsimd); a newer xtensor from source would need the upstream port's
# post-Humble API changes too. Its own layer, so the one above keeps its cache.
RUN apt-get update && apt-get install -y \
    libxtensor-dev \
    libxsimd-dev \
    && rm -rf /var/lib/apt/lists/*

# GStreamer headers, .pc files and the unversioned .so links, for
# syncai_camera_feeder: its `gstreamer` / `gstreamer-app` crates find the
# libraries through pkg-config in their build scripts and link with
# -lgstreamer-1.0 / -lgstapp-1.0 / -lglib-2.0, none of which runtime-base's
# GStreamer packages ship (they carry only the versioned .so.0). The package's
# manifest and its own devcontainer both list these two; this image did not,
# and the build only ever passed in a container where they had been installed
# by hand -- the 2026-10-12 image rebuild dropped them and the link failed
# with `cannot find -lgstapp-1.0`. It got that far rather than failing at
# pkg-config because cargo reused the build-script output cached in
# build/syncai_camera_feeder from the run that had them.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
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

# direct_visual_lidar_calibration (src/third-party/, 2026-10): the apt half of
# what it needs, beside the ceres / iridescence stages and GTSAM's unstable
# library. Every <depend> in its package.xml goes through
# ament_auto_find_build_dependencies(), so each one has to be present or its
# configure fails:
#   - ros-humble-cv-bridge: its preprocess decodes bag images (and brings
#     OpenCV, which its CMake also finds directly)
#   - ros-humble-pcl-ros  : listed as a <depend>; nothing else here uses it
#   - libfmt-dev          : rosdep key `fmt`; it links `fmt` by name
# and the headers / .so links the two source-built libraries' CMake configs
# re-find for a consumer:
#   - libgoogle-glog-dev  : Ceres (pulls libgflags-dev)
#   - libglfw3-dev, libpng-dev, libjpeg-dev, libgl-dev, libopengl-dev:
#     Iridescence (its config find_dependency()s OpenGL / PNG / JPEG, and the
#     library links glfw)
# rosbag2_cpp, its last <depend>, is already in ros-base.
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-pcl-ros \
    libfmt-dev \
    libgoogle-glog-dev \
    libglfw3-dev \
    libpng-dev \
    libjpeg-dev \
    libgl-dev \
    libopengl-dev \
    && rm -rf /var/lib/apt/lists/*

# syncai_landmark_detection (dependencies.repos, 2026-11): its ArUco detector
# is OpenCV's contrib aruco module, which jammy ships apart from libopencv-dev
# (that one comes with ros-humble-cv-bridge above). Without it the package's
# find_package(OpenCV ... aruco) fails at configure; rosdep check only reports
# the key (scripts/build.sh), so the build gets that far.
RUN apt-get update && apt-get install -y \
    libopencv-contrib-dev \
    && rm -rf /var/lib/apt/lists/*

# Prebuilt Livox-SDK2 / Sophus / Ceres / Iridescence, one COPY per builder
# stage (each stage holds only its own library under /out/usr/local). GTSAM,
# the fifth, is already here: runtime-base copies it, headers and all.
COPY --from=livox /out/usr/local /usr/local
COPY --from=sophus /out/usr/local /usr/local
COPY --from=ceres /out/usr/local /usr/local
COPY --from=iridescence /out/usr/local /usr/local
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

# Rust toolchain, from the rust-underlay stage (see there for what is in the
# tree and why). The underlay itself -- /opt/ros2_rust_underlay, its ENV and
# its line in ~/.bashrc -- is runtime-base's, because the built nodes link it.
# What has to be installed here as well is what the workspace's own Rust
# packages use at `colcon build` time: libclang-dev for rclrs's bindgen, and
# the colcon-cargo / colcon-ros-cargo plugins (pinned in COLCON_CARGO_PIP, same
# as the stage) for colcon to discover and build a package.xml + Cargo.toml
# package at all. The COPY keeps the stage's world-writable modes, which cargo
# needs under a runtime uid.
ARG COLCON_CARGO_PIP
ENV RUSTUP_HOME=/opt/rust/rustup \
    CARGO_HOME=/opt/rust/cargo \
    PATH=/opt/rust/cargo/bin:${PATH}
RUN apt-get update && apt-get install -y --no-install-recommends \
    libclang-dev \
    && rm -rf /var/lib/apt/lists/* && \
    pip3 install --no-cache-dir ${COLCON_CARGO_PIP}
COPY --from=rust-underlay /opt/rust /opt/rust

# ccache, for CI (2026-10): every CI run is a clean `colcon build` of ~11 min,
# almost all of it recompiling sources that did not change, and the critical
# path (syncai_common -> util -> costmap -> mppi) is most of that, so more
# cores do not help. The workflow turns it on through CMake's
# CMAKE_<LANG>_COMPILER_LAUNCHER environment variables (CMake >= 3.17 reads
# them at a fresh configure) and keeps the cache in the Actions cache. Nothing
# sets them on the robot, so robot01's builds are unchanged. Its own layer near
# the end of the stage, so adding it re-ran only the short tail below it.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ccache \
    && rm -rf /var/lib/apt/lists/*

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
# every check, and under a runtime uid other than 1000 (docker-compose.build.yml
# on a Mac runs as 501) a 1000-owned cache fails with "Permission denied:
# .../sources.cache/index" before checking anything. a+rwX, not a+w: the
# cache files are written 0600, and a+w alone leaves them -rw--w--w-, which
# another uid can write but not read -- the same error.
RUN rosdep update --rosdistro humble && chmod -R a+rwX /home/syncrobotic/.ros

# (The line that sources the mounted workspace overlay in every shell is
# written in runtime-base, after the underlay's: the release image needs the
# same three in the same order.)

CMD ["bash"]

# ---------------------------------------------------------------------------
# builder: compiles the workspace for the release image. Never shipped, never
# tagged; `--target dev` does not build it.
#
# In the Dockerfile rather than a COPY of an install/ built beforehand in the
# build container, so that what ships depends on src/ alone: the bind-mounted
# install/ of a working checkout keeps whatever earlier builds left in it (a
# development checkout's still listed dock.xml and DockToPose.action a week
# after both were deleted), and is a --symlink-install tree in any case.
#
# The context is a checkout with both .repos files imported -- .dockerignore
# keeps build/ install/ log/ and every .git out of it. scripts/build.sh is the
# same script robot01 and CI build with, with two differences:
#   - BUILD_SYMLINK=0: a real install/, no link back into src/ or build/. It
#     is also what lets syncai_sys_manager's InstallNoSource strip its .py
#     files (the guard there skips a symlinked install).
#   - direct_visual_lidar_calibration and vizionsdk_ros2 are skipped. The
#     first is a GUI tool run by hand on recorded bags -- on a workstation's
#     dev image, the robot has no display -- and the only user of Ceres,
#     Iridescence and pcl_ros; nothing starts the second. CALIBRATION mode
#     itself (bringup + the camera feeder, to record the bags) is in the
#     release.
# The path is the one robot01 mounts the workspace at, and has to be: the four
# rclrs packages' ament hooks and every setup.sh bake the absolute install
# prefix in, and the release stage copies install/ to the same place.
#
# No ccache: a clean build is ~11 min on the arm64 runner and a release is
# built rarely. BuildKit cache mounts would not survive between CI runs anyway.
# ---------------------------------------------------------------------------
FROM dev AS builder

# mkdir as syncrobotic before WORKDIR: WORKDIR creates a missing directory
# owned by root whatever USER is, and colcon then cannot make build/ in it.
RUN mkdir -p /home/syncrobotic/robot_ws
WORKDIR /home/syncrobotic/robot_ws

COPY --chown=syncrobotic:syncrobotic colcon.meta ./
COPY --chown=syncrobotic:syncrobotic scripts/build.sh scripts/build.sh
COPY --chown=syncrobotic:syncrobotic scripts/release/check_build.sh scripts/release/check_build.sh
COPY --chown=syncrobotic:syncrobotic src/ src/

# check_build.sh fails the image on any package that did not come out as a
# release build -- build.sh's default, but a default: an extra --cmake-args
# here, or a changed BUILD_TYPE, would otherwise ship an unoptimised planner
# with nothing to show for it but a slow robot.
RUN BUILD_SYMLINK=0 BUILD_ROSDEP=off scripts/build.sh \
        --packages-skip direct_visual_lidar_calibration vizionsdk_ros2 && \
    scripts/release/check_build.sh

# ---------------------------------------------------------------------------
# release: the image a robot runs without a checkout (2026-11). runtime-base,
# the builder's install/, and the two pieces of config/ that belong to a
# version of the code rather than to a robot:
#   - config/sessions/*.yaml : the windows, commands and `sleep` offsets of
#     each operating mode. Startup order is code here (no lifecycle manager).
#   - config/cyclonedds.xml  : the DDS configuration CYCLONEDDS_URI points at.
# Everything a robot owns stays on its host and is bind-mounted over the empty
# places made below (docker-compose.release.yml): the instance INI over
# config/system.ini, config/state, config/params, config/calibration, map/ and
# log/. They are created here, owned by syncrobotic, so the image also runs
# where one is not mounted -- CI's smoke test writes its fixture maps into the
# image's own map/.
#
# Built by .github/workflows/release.yml only, which also runs the smoke test
# in it before it is pushed. scripts/release/prepare.sh writes the
# .release-manifest this stage copies (the SHA of this repository and of each
# dependencies.repos checkout); without that file the build stops here, and
# should -- an image nobody can trace to its sources is not a release.
# ---------------------------------------------------------------------------
FROM runtime-base AS release

USER syncrobotic
RUN mkdir -p /home/syncrobotic/robot_ws/config/state \
             /home/syncrobotic/robot_ws/config/params \
             /home/syncrobotic/robot_ws/config/calibration \
             /home/syncrobotic/robot_ws/map \
             /home/syncrobotic/robot_ws/log && \
    : > /home/syncrobotic/robot_ws/config/system.ini
WORKDIR /home/syncrobotic/robot_ws

COPY --from=builder --chown=syncrobotic:syncrobotic \
    /home/syncrobotic/robot_ws/install install/
COPY --chown=syncrobotic:syncrobotic config/sessions/ config/sessions/
COPY --chown=syncrobotic:syncrobotic config/cyclonedds.xml config/cyclonedds.xml

# deploy/: what the HOST needs to run this image, carried inside it so that
# it is never a different version from it. scripts/release/save_bundle.sh
# takes install.sh out to put beside the image's tar, and install.sh takes the
# rest out on the robot, every time it switches image -- on a rollback the old
# compose file comes back with the old binaries. Nothing in the container
# reads this directory.
COPY --chown=syncrobotic:syncrobotic \
    docker-compose.release.yml \
    scripts/release/install.sh \
    scripts/release/.env.example \
    scripts/release/robot.ini.example \
    scripts/attach.sh \
    deploy/

# The image checks itself, and a failure here is a failed build rather than a
# robot that does not start:
#   - check_libs.sh: every shared library every binary in install/ links must
#     resolve in THIS image. The builder had dev's packages around it; this
#     stage has only runtime-base's. A miss names the file and the library,
#     and the fix is a line in runtime-base.
#   - check_runtime.sh: what is reached by name while running, which ldd
#     cannot see -- byobu, multilog and nmcli on PATH, the camera feeder's
#     GStreamer elements, syncai_sys_manager importing from its bytecode-only
#     install, every launch the session specs name -- and that install/ holds
#     no link back into a source tree.
# Bind-mounted from the context for the one RUN, so neither script is in the
# image.
RUN --mount=type=bind,source=scripts/release,target=/tmp/release-checks \
    bash /tmp/release-checks/check_libs.sh install && \
    bash /tmp/release-checks/check_runtime.sh install

# sys_manager is the container's main process, as in docker-compose.robots.yml:
# it builds the AUTO session itself, so `docker compose up -d` is the whole
# bring-up. The three sources are spelt out rather than left to ~/.bashrc
# because this is not an interactive shell.
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /opt/ros2_rust_underlay/install/setup.bash && source install/setup.bash && exec ros2 launch syncai_sys_manager sys_manager.launch.py"]

# Last, so that two releases of the same sources share every layer above: the
# version is the build time, and changes on every build.
ARG SYNCAI_VERSION=unversioned
ARG SYNCAI_REVISION=unknown
COPY --chown=syncrobotic:syncrobotic .release-manifest MANIFEST
LABEL org.opencontainers.image.title="SyncAI robot stack" \
      org.opencontainers.image.version="${SYNCAI_VERSION}" \
      org.opencontainers.image.revision="${SYNCAI_REVISION}"
