# Use ROS 2 Jazzy base image (Ubuntu 24.04)
FROM ros:jazzy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install common tools
RUN apt-get update && apt-get install -y \
    git python3-colcon-common-extensions python3-vcstool \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-turtlebot3* \
    ros-jazzy-rmw-microxrcedds \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Setup micro-ROS workspace
# -------------------------
RUN mkdir -p /micro_ros_ws/src
WORKDIR /micro_ros_ws/src
RUN git clone -b jazzy https://github.com/micro-ros/micro_ros_setup.git

WORKDIR /micro_ros_ws
RUN source /opt/ros/jazzy/setup.bash && colcon build

# -------------------------
# Setup TurtleBot3 workspace
# -------------------------
ENV TURTLEBOT3_MODEL=waffle_pi

RUN mkdir -p /turtlebot3_ws/src
WORKDIR /turtlebot3_ws/src

# clone turtlebot3 simulation repos
RUN vcs import --input https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/master/turtlebot3.repos

WORKDIR /turtlebot3_ws
RUN source /opt/ros/jazzy/setup.bash \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y \
 && colcon build

# Source both workspaces at container startup
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /micro_ros_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /turtlebot3_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /root
CMD ["bash"]
    