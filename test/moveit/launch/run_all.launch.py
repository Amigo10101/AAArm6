import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_pkg = get_package_share_directory('moveit')
    aar6_pkg = get_package_share_directory('aar6')

    # MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder("aar6", package_name="moveit")
        .robot_description(file_path="config/aar6.urdf.xacro")
        .robot_description_semantic(file_path="config/aar6.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # 1. Joint State Fixer Node (aar6 package)
    # Subscribes to /joint_states (bad time), publishes to /joint_states_fixed (good time)
    joint_state_fixer = Node(
        package='aar6',
        executable='joint_state_pub',
        name='joint_state_fixer',
        output='screen'
    )

    # 2. Move Group (MoveIt)
    # Remap /joint_states to /joint_states_fixed so it sees valid data
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_pkg, 'launch', 'move_group.launch.py')),
    )
    # Note: We need to pass the remapping to the nodes inside move_group.launch.py
    # But since we can't easily inject remappings into an included launch file without it exposing args,
    # we might rely on the fact that move_group looks for /joint_states.
    # Actually, the best way to remap a topic for an included launch description is using the 'launch_arguments' 
    # if it supports it, or wrapping it in a GroupAction with SetRemap.
    # However, for simplicity, let's try passing it via the Node definition if we were defining it, 
    # but here we are including.
    
    # Alternative: We can use a global remapping for this launch scope?
    # Or just use the 'remappings' argument in IncludeLaunchDescription? No, that's not supported directly.
    # We use 'launch_ros.actions.SetRemap' or similar?
    
    # Let's try to use GroupAction to apply remapping to the included launch.
    from launch.actions import GroupAction
    from launch_ros.actions import SetRemap

    move_group_with_remap = GroupAction(
        actions=[
            SetRemap(src='/joint_states', dst='/joint_states_fixed'),
            move_group_launch
        ]
    )

    # 3. Cartesian Control Node
    cartesian_control_node = Node(
        package='moveit',
        executable='cartesian_control_aar6',
        name='cartesian_control_aar6',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        remappings=[
            ('/joint_states', '/joint_states_fixed')
        ]
    )
    
    # 4. Robot State Publisher (needed if move_group doesn't launch it, but move_group usually relies on rsp)
    # Usually rsp.launch.py is included or run separately. 
    # Let's check move_group.launch.py... it usually doesn't include RSP.
    # We need RSP to publish TF based on joint states.
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
        remappings=[
            ('/joint_states', '/joint_states_fixed')
        ]
    )

    # 5. Trajectory to MCU Bridge
    mcu_bridge_node = Node(
        package='aar6',
        executable='trajectory_to_mcu_bridge',
        name='trajectory_to_mcu_bridge',
        output='screen'
    )

    return LaunchDescription([
        joint_state_fixer,
        rsp_node,
        move_group_with_remap,
        mcu_bridge_node
    ])
