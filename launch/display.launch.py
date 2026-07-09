import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # find urdf path
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    rviz_config = os.path.join(os.path.dirname(__file__), '..', 'config', 'rviz.rviz')
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=rviz_args
        )
    ])
