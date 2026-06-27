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

# CycloneDDS RMW (selected via RMW_IMPLEMENTATION + CYCLONEDDS_URI in devcontainer.json)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-nav2-msgs \
    ros-humble-map-msgs \
    ros-humble-laser-geometry \
    ros-humble-pcl-conversions \
    ros-humble-angles \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/* \
    && npm install -g @anthropic-ai/claude-code

# Python web stack for syncai_backend (no reliable apt key on jammy; installed
# via pip). Keep in sync with src/syncai_backend/requirements.txt.
RUN pip3 install --no-cache-dir fastapi "uvicorn[standard]"

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
