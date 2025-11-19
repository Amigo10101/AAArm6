from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("aar6", package_name="moveit").to_moveit_configs()

    # Generate the demo launch
    demo_launch = generate_demo_launch(moveit_config)

    # Override joint_state_broadcaster
    # Stop loading controllers that publish fake joint states
    demo_launch = LaunchDescription([
        # Include everything except the joint_state_broadcaster
        *demo_launch.entities,

  
    ])

    return demo_launch
