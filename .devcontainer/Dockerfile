# Development environment for the planning stack: ROS 2 Humble on Ubuntu 22.04 + CARLA client.
# The CARLA *server* is not in here; it runs next to this container on the AWS GPU instance
# (scripts/start_carla.sh). The container uses host networking, so it reaches CARLA at localhost:2000.
FROM osrf/ros:humble-desktop

# Must match the CARLA server version exactly, or the client refuses to connect.
ARG CARLA_VERSION=0.9.16

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-vcstool python3-colcon-common-extensions git git-lfs tmux \
      ros-humble-ackermann-msgs ros-humble-vision-msgs \
      ros-humble-robot-localization ros-humble-slam-toolbox \
      ros-humble-pointcloud-to-laserscan ros-humble-foxglove-bridge \
      ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pre-commit "carla==${CARLA_VERSION}"

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /etc/bash.bashrc
