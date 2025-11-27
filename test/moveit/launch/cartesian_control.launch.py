import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("aar6", package_name="moveit")
        .robot_description(file_path="config/aar6.urdf.xacro")
        .robot_description_semantic(file_path="config/aar6.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package='moveit',
            executable='cartesian_control_aar6',
            name='cartesian_control_aar6',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ]
        )
    ])
