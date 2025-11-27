import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('aar6')
    urdf_path = os.path.join(pkg_path, 'urdf', 'PAROL6.urdf')
    rviz_config = os.path.join(pkg_path, 'rviz', 'rviz_monitor.rviz')

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),
        Node(
    package='aar6',
    executable='joint_state_pub',
    name='joint_state_pub',
    output='screen'
),  
        # Trajectory to MCU Bridge - forwards goals to MCU
        Node(
            package='aar6',
            executable='trajectory_to_mcu_bridge',
            name='trajectory_to_mcu_bridge',
            output='screen'
        ),

        # Minimal RViz monitor
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_monitor',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
