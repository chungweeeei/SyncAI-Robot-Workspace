# =============================================================================
# SyncAI robot workspace — multi-stage build (development).
#
#   base            shared runtime floor (ros-base + cyclonedds + uid-1000 user)
#     ├─ deps-builder  GTSAM / Sophus / Livox-SDK2 → /usr/local  (slow, cached)
#     └─ dev           the interactive dev image: rviz2, colcon, byobu, Node.js,
#                      -dev headers. Workspace bind-mounted at ~/robot_ws and
#                      built by hand (colcon). Compose target: dev.
#
# The dev target keeps today's workflow (workspace mounted at ~/robot_ws, build
# by hand). deps-builder is the expensive stage (GTSAM ~30-60 min on Tegra) —
# keep it free of anything that changes often so its cache survives.
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
# timestamps (ros2 launch, backend, byobu panes) match the host / operators.
# ompl: Dubins/Reeds-Shepp state spaces for syncai_planner's smac plugins —
# libsyncai_planner.so links libompl.so, so it is a runtime dep, not dev-only.
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
# deps-builder: source-built third-party libs → /usr/local
# /usr/local is empty in base, so downstream stages pick up everything with a
# single COPY --from=deps-builder /usr/local /usr/local.
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
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    git checkout f5d9375f84efe2b15bc0a052d3e18482ed13adf4 && \
    mkdir build && cd build && \
    cmake .. && make -j"$(nproc)" && make install && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2

# GTSAM 4.2.0: pgo + hba (FASTLIO2_ROS2) link libgtsam (find_package(GTSAM)).
# No apt/PPA GTSAM on arm64, so build from source into /usr/local. Flags follow
# the LIO-SAM recipe: system Eigen + no march-native to avoid Eigen-alignment
# crashes when mixed with PCL; TBB on; shared libs.
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
    make -j"$(nproc)" && make install && \
    ldconfig && \
    rm -rf /tmp/gtsam

# Sophus 1.22.10: fastlio2 + hba need find_package(Sophus). Header-only;
# SOPHUS_USE_BASIC_LOGGING=ON drops the fmt dependency (matches the
# add_compile_definitions in their CMake).
RUN git clone --branch 1.22.10 --depth 1 https://github.com/strasdat/Sophus.git /tmp/Sophus && \
    cd /tmp/Sophus && \
    mkdir build && cd build && \
    cmake .. \
    -DSOPHUS_USE_BASIC_LOGGING=ON \
    -DBUILD_SOPHUS_TESTS=OFF \
    -DBUILD_SOPHUS_EXAMPLES=OFF && \
    make -j"$(nproc)" && make install && \
    ldconfig && \
    rm -rf /tmp/Sophus

# ---------------------------------------------------------------------------
# dev: the interactive development image (compose service robot01,
# image syncai-test-robot, target: dev). Functionally identical to the old
# single-stage image: full GUI/tooling, workspace bind-mounted at runtime,
# colcon build run by hand.
# ---------------------------------------------------------------------------
FROM base AS dev

# GUI, build toolchain, PCL/ROS build deps, and operator conveniences.
#
# ros-humble-compressed-image-transport is not optional for the camera: the
# camera node publishes *only* sensor_msgs/CompressedImage on
# `<robot_id>/image_raw/compressed`, and bare `image_transport` declares the
# raw transport alone. Without this plugin rviz2's Image display has no way to
# subscribe at all and simply stays blank -- no error, no warning.
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-compressed-image-transport \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-teleop-twist-keyboard \
    python3-opencv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-dotenv \
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
#   - nlohmann-json3-dev: header-only JSON library
#   - libapr1-dev / libaprutil1-dev: livox_ros_driver2
#   - libboost-all-dev / libtbb-dev / libeigen3-dev: GTSAM/Sophus headers
#     (the libs themselves come prebuilt from deps-builder below)
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
    && rm -rf /var/lib/apt/lists/*

# Prebuilt Livox-SDK2 / GTSAM / Sophus from the cached builder stage.
COPY --from=deps-builder /usr/local /usr/local
RUN ldconfig

# Python web stack for syncai_backend. requirements.txt is the single source
# of truth for the backend's python deps.
COPY src/syncai_backend/requirements.txt /tmp/syncai_backend_requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/syncai_backend_requirements.txt && \
    rm /tmp/syncai_backend_requirements.txt

# Node.js 22 for syncai_frontend (Next.js 16). `npm install` / `npm run dev`
# run at runtime against the mounted workspace; only the node/npm runtime
# needs to live in the image.
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update --rosdistro humble

USER syncrobotic
WORKDIR /home/syncrobotic

# Populate rosdep cache for the syncrobotic user (the root-level update above
# does not carry over to ~/.ros), so `rosdep install` works at runtime.
RUN rosdep update --rosdistro humble

# Auto-source the mounted workspace overlay in every shell.
RUN echo '[ -f ~/robot_ws/install/setup.bash ] && source ~/robot_ws/install/setup.bash' >> ~/.bashrc

CMD ["bash"]
