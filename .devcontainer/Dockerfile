FROM ros:humble

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive

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

# ROS2 package dependencies for ros2_laser_scan_merger
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

# System dependencies for syncai_map_server (GraphicsMagick++ is used by map_io)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgraphicsmagick++1-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/* \
    && npm install -g @anthropic-ai/claude-code

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN <<EOF
echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc
echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
EOF

RUN mkdir -p /home/$USERNAME/.claude \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME/.claude /workspace

USER $USERNAME
WORKDIR /workspace

CMD ["/bin/bash"]
