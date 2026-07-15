# Use ROS 2 Jazzy (Ubuntu 24.04) as the base image
FROM osrf/ros:jazzy-desktop

# Prevent interactive prompts during apt-get installations
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

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
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set Cyclone DDS as the default ROS middleware for the build environment
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

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

# Clone the ROS 2 bridge
RUN git clone https://github.com/jkk-research/openpilot_ros2_bridge.git

# 4. Set up environment variables and sourcing for bash
# Ensures ROS 2, Openpilot's venv, and Cyclone DDS are active when attaching to the container
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/openpilot/.venv/bin/activate" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

# Set default command
CMD ["bash"]