FROM ubuntu:22.04

# Install pre-requisites
RUN apt-get update && apt-get install -y \
    curl \
    git \
    gnupg \
    lsb-release \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS 2 Humble + Navigation2 + dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-tf2-tools \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rviz2 \
    ros-humble-nav2-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-angles \
    ros-humble-teleop-twist-keyboard \
    python3-opencv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-dotenv \
    python3-pip \
    byobu \
    net-tools \
    iputils-ping \
    network-manager \
    bluez \
    && rm -rf /var/lib/apt/lists/*

# System deps for workspace packages that have no ament/CMake config:
#   - libgraphicsmagick++1-dev: syncai_map_server (located via pkg-config)
#   - libzmq3-dev / libncurses-dev: behaviortree_cpp
#   - nlohmann-json3-dev: header-only JSON library (found via find_package(nlohmann_json))
#   - libapr1-dev / libaprutil1-dev: livox_ros_driver2 (found via APR_INCLUDE_DIRS)
#   - avahi-utils: syncai_system_manager spawns avahi-publish; talks to the
#     HOST avahi-daemon via the mounted /run/dbus/system_bus_socket, so no
#     avahi-daemon runs inside the container
RUN apt-get update && apt-get install -y \
    libgraphicsmagick++1-dev \
    libzmq3-dev \
    libncurses-dev \
    nlohmann-json3-dev \
    libapr1-dev \
    libaprutil1-dev \
    avahi-utils \
    && rm -rf /var/lib/apt/lists/*

# Livox-SDK2: livox_ros_driver2 links liblivox_lidar_sdk_shared.so from
# /usr/local (via find_library). The SDK source lives in the mounted workspace
# at runtime, so it can't be built from there at image-build time — clone and
# install it here instead. Pinned to the commit vendored under src/third-party.
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
# crashes when mixed with PCL; TBB on; shared libs. libboost-all-dev / libtbb-dev
# are GTSAM's own deps; libeigen3-dev backs -DGTSAM_USE_SYSTEM_EIGEN=ON.
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    libtbb-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*
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

# Sophus 1.22.10: fastlio2 + hba (FASTLIO2_ROS2) need find_package(Sophus).
# Header-only Lie-group lib; install into /usr/local. SOPHUS_USE_BASIC_LOGGING=ON
# drops the fmt dependency (matches the add_compile_definitions in their CMake);
# tests/examples off to keep the build fast. Uses libeigen3-dev installed above.
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

# Python web stack for syncai_backend (no reliable apt key on jammy; installed
# via pip). Keep in sync with src/syncai_backend/requirements.txt.
RUN pip3 install --no-cache-dir \
    fastapi \
    "uvicorn[standard]" \
    structlog \
    dotenv \
    sqlalchemy \
    sqlalchemy-utils \
    psycopg2 \
    temporalio \
    requests

# Initialize rosdep
RUN rosdep init || true && rosdep update --rosdistro humble

# Allow any uid (overridden via compose `user:`) to sudo without password.
RUN echo "ALL ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ubuntu:22.04 has no default uid-1000 user, so create the `ubuntu` user and
# its home dir. Then make HOME world-writable so a runtime-overridden uid
# (via compose `user:`) can still write ~/.ros, ~/.cache, ~/.bash_history.
RUN groupadd -g 1000 ubuntu && \
    useradd -m -u 1000 -g 1000 -s /bin/bash ubuntu && \
    chmod -R 777 /home/ubuntu

USER ubuntu
WORKDIR /home/ubuntu

# Populate rosdep cache for the ubuntu user (the root-level update above does not
# carry over to ~ubuntu/.ros), so `rosdep install` works at runtime.
RUN rosdep update --rosdistro humble

# Auto-source ROS 2 and workspace in every shell
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo '[ -f ~/robot_ws/install/setup.bash ] && source ~/robot_ws/install/setup.bash' >> ~/.bashrc

CMD ["bash"]
