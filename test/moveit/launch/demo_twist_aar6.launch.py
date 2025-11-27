import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder

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

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit")
        .yaml("config/aar6_servo_config.yaml")
        .to_dict()
    }

    return LaunchDescription([
        Node(
            package='moveit',
            executable='demo_twist_aar6',
            name='demo_twist_aar6',
            output='screen',
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ]
        )
    ])
