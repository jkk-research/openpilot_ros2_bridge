# Use ROS 2 Jazzy (Ubuntu 24.04) as the base image
FROM osrf/ros:jazzy-desktop

# Prevent interactive prompts during apt-get installations
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO_NAME=jazzy

# 1. Install prerequisites, system utilities, and Cyclone DDS
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    curl \
    vim \
    nano \
    tmux \
    libqt5charts5-dev \
    file \
    git-lfs \
    python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set Cyclone DDS as the default ROS middleware for the build environment
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV OPENPILOT_ROOT=/workspace/openpilot
ENV ROS2_WS=/workspace/ros2_ws

# Set up the workspace
WORKDIR /workspace

# 2. Clone Openpilot
# Using depth=1 to save space
RUN git clone --recurse-submodules https://github.com/commaai/openpilot.git

# op.sh automatically detects Ubuntu Noble (24.04), installs all system 
# dependencies via apt, and sets up the uv virtual environment in .venv
RUN cd openpilot && tools/op.sh setup

# Change the default shell to bash
SHELL ["/bin/bash", "-c"]

# 3. Activate the environment and compile Openpilot
# -u tells scons to search up the directory tree for the root SConstruct file
# -j$(nproc) parallelizes the build using all available CPU cores
RUN source openpilot/.venv/bin/activate && cd openpilot && scons -u

# Copy the ROS 2 bridge source tree into a colcon workspace
RUN mkdir -p /workspace/ros2_ws/src
COPY . /workspace/ros2_ws/src/openpilot_ros2_bridge
RUN chmod +x /workspace/ros2_ws/src/openpilot_ros2_bridge/docker/*.sh

# 4. Set up environment variables and sourcing for bash
# Ensures ROS 2, Openpilot's venv, and Cyclone DDS are active when attaching to the container
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/openpilot/.venv/bin/activate" >> /root/.bashrc && \
    echo "source /workspace/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

# 5. Build the ROS 2 bridge package
RUN source /opt/ros/jazzy/setup.bash && \
    source /workspace/openpilot/.venv/bin/activate && \
    cd /workspace/ros2_ws && \
    colcon build --symlink-install --packages-select openpilot_ros2_bridge

# Set default command
CMD ["bash"]