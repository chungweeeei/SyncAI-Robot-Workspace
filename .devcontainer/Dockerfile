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

# ROS2 package dependencies for ros2_laser_scan_merger.
# pcl-ros pulls a huge boost/VTK dependency tree (~500MB); the OrbStack proxy
# tends to drop a few files near the end. Retry in-layer so already-fetched
# .debs in the apt cache are reused and only the stragglers are re-fetched.
RUN apt-get update \
    && n=0; until [ $n -ge 6 ]; do \
        apt-get install -y --no-install-recommends --fix-missing \
            ros-humble-pcl-conversions \
            ros-humble-pcl-ros \
            ros-humble-pointcloud-to-laserscan \
        && break; \
        n=$((n+1)); echo "apt install attempt $n failed, retrying..."; sleep 5; \
    done; [ $n -lt 6 ] \
    && rm -rf /var/lib/apt/lists/*

# System dependencies for syncai_map_server (GraphicsMagick++ is used by map_io)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgraphicsmagick++1-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/* \
    && npm install -g @anthropic-ai/claude-code

RUN if id -u ubuntu >/dev/null 2>&1 && [ "$USERNAME" != "ubuntu" ]; then \
        userdel -r ubuntu 2>/dev/null || true; \
    fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN <<EOF
echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc
echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
EOF

RUN mkdir -p /home/$USERNAME/.claude /workspace \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME/.claude /workspace

USER $USERNAME
WORKDIR /workspace

CMD ["/bin/bash"]
