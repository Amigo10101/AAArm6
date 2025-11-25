from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'my_robot.urdf')

    return LaunchDescription([
        # Publishes robot transforms from URDF and joint states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False,
                         'robot_description': open(urdf_path).read()}]
        ),

        # (Optional) allows manual slider control
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'view_config.rviz')]
        ),
    ])
