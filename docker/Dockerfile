# Use ROS 2 Jazzy (Ubuntu 24.04) as the base image
FROM osrf/ros:jazzy-desktop

# Prevent interactive prompts during apt-get installations
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO_NAME=jazzy
ENV SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt
ENV GIT_SSL_CAINFO=/etc/ssl/certs/ca-certificates.crt

# 1. Install prerequisites and system utilities
RUN apt-get update && apt-get install -y \
    ca-certificates \
    sudo \
    git \
    curl \
    python3-pip \
    python3-venv \
    vim \
    nano \
    tmux \
    libqt5charts5-dev \
    file \
    git-lfs \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN git config --system http.sslCAInfo /etc/ssl/certs/ca-certificates.crt

# Use the ROS 2 desktop image default middleware
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS2_WS=/workspace/ros2_ws

# Set up the workspace
WORKDIR /workspace

# Change the default shell to bash
SHELL ["/bin/bash", "-c"]

# 2. Copy the ROS 2 bridge source tree into a colcon workspace
RUN mkdir -p /workspace/ros2_ws/src
COPY . /workspace/ros2_ws/src/openpilot_ros2_bridge
RUN chmod +x /workspace/ros2_ws/src/openpilot_ros2_bridge/docker/*.sh /workspace/ros2_ws/src/openpilot_ros2_bridge/scripts/*.sh

# 3. Create the bridge Python virtual environment
RUN python3 -m venv --system-site-packages /workspace/ros2_ws/src/openpilot_ros2_bridge/.venv && \
    source /workspace/ros2_ws/src/openpilot_ros2_bridge/.venv/bin/activate && \
    python3 -m pip install --upgrade pip && \
    python3 -m pip install -r /workspace/ros2_ws/src/openpilot_ros2_bridge/requirements.txt

# 4. Set up environment variables and sourcing for bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/ros2_ws/src/openpilot_ros2_bridge/.venv/bin/activate" >> /root/.bashrc && \
    echo "source /workspace/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc

# 5. Build the ROS 2 bridge package
RUN source /opt/ros/jazzy/setup.bash && \
    source /workspace/ros2_ws/src/openpilot_ros2_bridge/.venv/bin/activate && \
    cd /workspace/ros2_ws && \
    colcon build --symlink-install --packages-select openpilot_ros2_bridge

# Set default command
CMD ["bash"]