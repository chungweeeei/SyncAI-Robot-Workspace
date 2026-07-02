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

# rclrs (ros2_rust) toolchain: Rust + bindgen deps + colcon cargo plugins.
# libclang-dev is required by bindgen; python3-vcstool imports the aux repos.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libclang-dev \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Install Rust into system-wide dirs so both root (image build) and the ros
# user share one toolchain. cargo-ament-build lets colcon build cargo packages.
ENV RUSTUP_HOME=/opt/rust/rustup \
    CARGO_HOME=/opt/rust/cargo \
    PATH=/opt/rust/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path --profile minimal \
    && cargo install cargo-ament-build

RUN pip3 install --no-cache-dir \
    git+https://github.com/colcon/colcon-cargo.git \
    git+https://github.com/colcon/colcon-ros-cargo.git

# Build ros2_rust as a standalone overlay on top of /opt/ros/humble so rclrs is
# available image-wide without vendoring it into the user's /workspace repo.
RUN mkdir -p /opt/ros2_rust_ws/src \
    && git clone https://github.com/ros2-rust/ros2_rust.git /opt/ros2_rust_ws/src/ros2_rust \
    && cd /opt/ros2_rust_ws \
    && vcs import src < src/ros2_rust/ros2_rust_humble.repos \
    && . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /opt/ros2_rust_ws/build /opt/ros2_rust_ws/log \
    # Make the shared Rust toolchain + crate cache readable/writable for the
    # non-root user so they can build cargo packages in their workspace. Must
    # run AFTER the overlay build, which populates the registry as root.
    && chmod -R a+rwX /opt/rust

RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/* \
    && npm install -g @anthropic-ai/claude-code

# Python web stack for syncai_backend (no reliable apt key on jammy; installed
# via pip). Keep in sync with src/syncai_backend/requirements.txt.
RUN pip3 install --no-cache-dir \
    fastapi \
    "uvicorn[standard]" \
    sqlalchemy \
    structlog \
    dotenv

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
echo "source /opt/ros2_rust_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
EOF

RUN mkdir -p /home/$USERNAME/.claude /workspace \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME/.claude /workspace

USER $USERNAME
WORKDIR /workspace

CMD ["/bin/bash"]
