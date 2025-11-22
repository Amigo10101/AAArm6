FROM ros:jazzy

# Install dev tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    build-essential \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Auto-source ROS 2
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws

CMD ["bash"]
RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-common \
    ros-jazzy-moveit-resources-panda-moveit-config \
    && rm -rf /var/lib/apt/lists/*

    

