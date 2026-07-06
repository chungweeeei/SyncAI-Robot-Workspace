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
    ros-humble-pointcloud-to-laserscan \
    ros-humble-angles \
    ros-humble-teleop-twist-keyboard \
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
RUN apt-get update && apt-get install -y \
    libgraphicsmagick++1-dev \
    libzmq3-dev \
    libncurses-dev \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

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
    temporalio

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
