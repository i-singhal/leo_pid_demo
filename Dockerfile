FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-jazzy-leo-simulator \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-tf2-ros \
    xterm \
    git \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/rover_ws/src

RUN git clone https://github.com/LeoRover/leo_common-ros2.git && \
    git clone https://github.com/LeoRover/leo_simulator-ros2.git

COPY . /root/rover_ws/src/leo_pid_demo/

WORKDIR /root/rover_ws
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    colcon build

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /root/rover_ws/install/setup.bash' >> /root/.bashrc && \
    echo 'export ROS_LOCALHOST_ONLY=1' >> /root/.bashrc && \
    echo "alias killros='bash /root/rover_ws/install/leo_pid_demo/share/leo_pid_demo/scripts/kill_all.sh'" >> /root/.bashrc

CMD ["/bin/bash"]   