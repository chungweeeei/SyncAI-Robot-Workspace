FROM ubuntu:24.04

# Install pre-requisites
RUN apt-get update && apt-get install -y \
    curl \
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

# Install ROS 2 Jazzy + Navigation2 + dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-tf2-tools \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rviz2 \
    python3-colcon-common-extensions \
    python3-rosdep \
    byobu \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update --rosdistro jazzy

# Allow any uid (overridden via compose `user:`) to sudo without password.
RUN echo "ALL ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Make HOME world-writable so a runtime-overridden uid can still write
# things like ~/.ros, ~/.cache, ~/.bash_history.
RUN chmod -R 777 /home/ubuntu

USER ubuntu
WORKDIR /home/ubuntu

# Auto-source ROS 2 and workspace in every shell
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    echo '[ -f ~/robot_ws/install/setup.bash ] && source ~/robot_ws/install/setup.bash' >> ~/.bashrc

CMD ["bash"]
