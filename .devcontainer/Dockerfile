FROM ros:humble

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    vim \
    sudo \
    iputils-ping \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN <<EOF
echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc
echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
EOF

USER $USERNAME
WORKDIR /workspace

CMD ["/bin/bash"]
