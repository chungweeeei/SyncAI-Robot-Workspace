FROM ros:humble

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive

# Make apt resilient behind OrbStack's transparent proxy (serialize downloads + retry)
RUN echo 'Acquire::Retries "8";' > /etc/apt/apt.conf.d/80-retries \
    && echo 'Acquire::http::Pipeline-Depth "0";' >> /etc/apt/apt.conf.d/80-retries \
    && echo 'Acquire::http::No-Cache "true";' >> /etc/apt/apt.conf.d/80-retries \
    && echo 'Acquire::Queue-Mode "access";' >> /etc/apt/apt.conf.d/80-retries

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    vim \
    sudo \
    curl \
    ca-certificates \
    iputils-ping \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# System dependencies for syncai_map_server (GraphicsMagick++ is used by map_io)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgraphicsmagick++1-dev \
    && rm -rf /var/lib/apt/lists/*

# nlohmann-json3-dev: syncai_robot_state flattens WifiStatus into the JSON
# string in RobotState.network_status (header-only, dev package is enough).
# libapr1-dev / libaprutil1-dev: livox_ros_driver2 pkg-checks apr-1 — same
# pair the robot image (/workspace/Dockerfile) installs for it.
RUN apt-get update && apt-get install -y --no-install-recommends \
    nlohmann-json3-dev \
    libapr1-dev \
    libaprutil1-dev \
    && rm -rf /var/lib/apt/lists/*

# CycloneDDS RMW (selected via RMW_IMPLEMENTATION + CYCLONEDDS_URI in devcontainer.json)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-nav2-msgs \
    ros-humble-map-msgs \
    ros-humble-laser-geometry \
    ros-humble-pcl-conversions \
    ros-humble-angles \
    ros-humble-ompl \
    && rm -rf /var/lib/apt/lists/*

# Build-from-source deps for FASTLIO2_ROS2 (hba / pgo / localizer). GTSAM and
# Sophus have no usable apt package on jammy, so the submodule expects them built
# from source (see FASTLIO2_ROS2/README.md and CLAUDE.md's "Build" note). Without
# them `find_package(GTSAM|Sophus REQUIRED)` aborts configure and colcon fails
# the whole workspace — this is exactly the build failure this block fixes. PCL /
# Eigen / Boost already arrive via ros-humble-pcl-conversions -> libpcl-dev; the
# apt line here just makes the two source builds self-contained rather than
# relying on that transitive chain.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libeigen3-dev \
    libboost-all-dev \
    libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

# Sophus 1.22.10, pinned by FASTLIO2_ROS2/README.md. SOPHUS_USE_BASIC_LOGGING=ON
# drops the fmt dependency the newer Sophus otherwise pulls in (the README calls
# this out explicitly).
RUN git clone https://github.com/strasdat/Sophus.git /tmp/Sophus \
    && cd /tmp/Sophus \
    && git checkout 1.22.10 \
    && cmake -B build -DSOPHUS_USE_BASIC_LOGGING=ON -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF \
    && cmake --build build -j"$(nproc)" \
    && cmake --install build \
    && rm -rf /tmp/Sophus

# GTSAM 4.1.1 (pgo pose-graph optimization + hba). Flags matter:
#   USE_SYSTEM_EIGEN=ON         -> link the same Eigen 3.4 as the rest of the stack;
#                                  GTSAM's bundled Eigen otherwise causes ABI/alignment mismatches
#   BUILD_WITH_MARCH_NATIVE=OFF -> -march=native miscompiles Eigen alignment and is non-portable
#   WITH_TBB=OFF                -> jammy ships oneTBB 2021, whose API GTSAM 4.1.1 predates
RUN git clone https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cd /tmp/gtsam \
    && git checkout 4.1.1 \
    && cmake -B build \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)" \
    && cmake --install build \
    && ldconfig \
    && rm -rf /tmp/gtsam

# Livox-SDK2: livox_ros_driver2 links liblivox_lidar_sdk_shared.so out of
# /usr/local via find_library. colcon does build the submodule copy (plain
# CMake package -> install/livox_sdk2), but the driver's package.xml declares
# no dependency on it, so that prefix never reaches the driver's CMake
# environment — the SDK has to be preinstalled system-wide, same as in the
# robot image (/workspace/Dockerfile). Pinned to the commit the submodule
# points at (Mid-360s support; newer than the robot image's f5d9375 — keep
# both in sync when bumping the submodule).
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cd /tmp/Livox-SDK2 \
    && git checkout 68ae1e1dc77f61f03c95d7c2809831e198d0aedd \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)" \
    && cmake --install build \
    && ldconfig \
    && rm -rf /tmp/Livox-SDK2

RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/* \
    && npm install -g @anthropic-ai/claude-code

# uv (fast Python package manager). Installed system-wide as root so the binary
# lands in /usr/local/bin (already on PATH); a user-level `pip3 install --user`
# would drop it in ~/.local/bin, which is NOT on PATH -> `uv: command not found`.
RUN pip3 install --no-cache-dir uv

# NOTE: this image used to pip-install syncai_backend's web stack (fastapi /
# uvicorn / sqlalchemy / temporalio / opencv-python-headless / httpx), kept in
# sync with that package's requirements.txt, so its pytest suite ran in THIS
# container and not only in the robot one. The backend moved to
# SyncAI-Robot-Backend in 2026-09 and takes its own dev environment with it, so
# the stack is gone from here too. What is left in this workspace is C++ plus
# syncai_sys_manager, which needs rclpy, netifaces, psutil and yaml — all
# rosdep keys, resolved by `rosdep install` against src/, not pinned here.

RUN if id -u ubuntu >/dev/null 2>&1 && [ "$USERNAME" != "ubuntu" ]; then \
    userdel -r ubuntu 2>/dev/null || true; \
    fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Append shell setup to .bashrc. A quoted heredoc (<<'BASHRC') writes the body
# verbatim, so $HOME/$PATH stay unexpanded in the file and are resolved by bash
# at runtime. (The previous echo-with-escapes approach was fragile and could end
# up writing the literal string into PATH, wiping /usr/bin and breaking every
# command — even `ls`.)
#
# ROS_DOMAIN_ID must match the robot containers (docker-compose.robots.yml sets
# a literal 1 in x-robot-env) — otherwise nodes started in here are on a
# different domain and silently see nothing from the robot stack. It was 0,
# which stopped matching once the robot containers moved to 1.
RUN cat >> /home/$USERNAME/.bashrc <<'BASHRC'
export ROS_DOMAIN_ID=1
source /opt/ros/humble/setup.bash
export PATH="$HOME/.local/bin:$PATH"
BASHRC

RUN mkdir -p /home/$USERNAME/.claude /workspace \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME/.claude /workspace

USER $USERNAME
WORKDIR /workspace

CMD ["/bin/bash"]
