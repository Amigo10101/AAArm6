#!/bin/bash
# Kickstart script to trigger servo node initialization
# This sends a small delta_joint_cmds to force the planning scene monitor to update

source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

echo "Waiting for servo node to start..."
sleep 5

echo "Sending kickstart joint command to wake up servo..."
ros2 topic pub --once /servo_node/delta_joint_cmds control_msgs/msg/JointJog "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, joint_names: ['L1', 'L2', 'L3', 'L4', 'L5', 'L6'], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration: 0.1}"

echo "Kickstart complete!"
