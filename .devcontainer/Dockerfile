# Same environment for every team member: ROS 2 Humble on Ubuntu 22.04.
FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-vcstool python3-colcon-common-extensions git git-lfs \
      ros-humble-ackermann-msgs ros-humble-vision-msgs \
      ros-humble-robot-localization ros-humble-slam-toolbox \
      ros-humble-teleop-twist-keyboard ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pre-commit

# f1tenth_gym: the Python simulator behind f1tenth_gym_ros.
# If this step fails on a newer gym release, pin a commit with `git checkout <hash>`.
RUN git clone https://github.com/f1tenth/f1tenth_gym /opt/f1tenth_gym \
    && pip3 install --no-cache-dir -e /opt/f1tenth_gym

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /etc/bash.bashrc
