# Start from Jazzy base image
FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV GZ_VERSION=harmonic

# Install basic utilities
RUN apt-get update && apt-get install -y \
    curl \
    evtest\
    git \
    gnupg2 \
    lsb-release \
    nano \
    sudo \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo package repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Jazzy packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-python-orocos-kdl-vendor \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Install development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
RUN mkdir -p /workspace/src

# Initialize rosdep
RUN rosdep init || true

# Copy source code
COPY src /workspace/src

# Copy and apply patches
COPY .devcontainer/patches /tmp/patches
RUN cd /workspace/src/kobuki && \
    patch -p1 < /tmp/patches/kobuki-camera-optional.patch && \
    rm -rf /tmp/patches

# Kobuki setup - import third-party dependencies
RUN cd /workspace/src && vcs import < kobuki/thirdparty.repos
RUN apt-get update \
    && rosdep install --from-paths /workspace/src --ignore-src -r -y \
    && apt-get install -y libusb-1.0-0-dev libftdi1-dev libuvc-dev

# Build the workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd /workspace \
    && colcon build --packages-up-to g1_task --symlink-install"

# Set up ROS environment for root
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc \
    && echo "export GZ_VERSION=harmonic" >> /root/.bashrc \
    && echo "alias build='cd /workspace && colcon build --packages-up-to g1_task --symlink-install && . install/setup.bash'" >> /root/.bash_aliases \
    && echo "alias ik='cd /workspace && ros2 launch g1_task ik.launch.py use_rviz:=true log_level:=info'" >> /root/.bash_aliases \
    && echo "alias teleop='cd /workspace && ros2 launch g1_task teleop.launch.py use_rviz:=true log_level:=info'" >> /root/.bash_aliases

WORKDIR /workspace

CMD ["/bin/bash"]
