FROM ros:jazzy


RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    build-essential \
    nano \
    mesa-utils \
    ros-jazzy-moveit \
    ros-jazzy-moveit-common \
    ros-jazzy-moveit-resources-panda-moveit-config \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*


RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws

CMD ["bash"]