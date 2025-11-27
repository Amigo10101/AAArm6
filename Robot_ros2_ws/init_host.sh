#!/bin/bash
set -e

# 1. Source global ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

colcon build 

# 2. Source the local workspace
source install/setup.bash

# 3. Launch Robot Display (Background)
echo "Launching aar6 monitor..."
ros2 launch aar6 monitor.launch.py &

# Wait for monitor to start
sleep 3

# 4. Start trajectory to MCU bridge
echo "Starting trajectory to MCU bridge..."
ros2 run aar6 trajectory_to_mcu_bridge &
sleep 2

ros2 launch moveit demo.launch.py 
