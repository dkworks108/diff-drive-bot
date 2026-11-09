#!/usr/bin/env python3
"""
Unified Autonomous Frontier-Based Exploration Launch File.

Launches:
1. Gazebo Fortress simulation with differential drive robot and GPU LiDAR.
2. ros2_control diff_drive_controller and joint_state_broadcaster.
3. ROS-Ignition parameter bridges for /clock and /scan.
4. SLAM Toolbox in online asynchronous mapping mode (owns map -> odom TF).
5. Nav2 Planning, Control, Behaviors and BT Navigator (SLAM-compatible mode, NO AMCL/map_server).
6. Frontier Exploration Node (explorer_node).
7. RViz2 configured with exploration visualization displays.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Default file paths
    default_nav2_params = os.path.join(pkg_dir, 'config', 'nav2_exploration_params.yaml')
    default_slam_params = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    default_exploration_params = os.path.join(pkg_dir, 'config', 'exploration.yaml')
    default_rviz = os.path.join(pkg_dir, 'config', 'exploration.rviz')
    gazebo_launch_path = os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')

    # Fallback to package share if running from install directory
    if not os.path.exists(default_nav2_params):
        try:
            pkg_share = get_package_share_directory('diff_drive_description')
            default_nav2_params = os.path.join(pkg_share, 'config', 'nav2_exploration_params.yaml')
            default_slam_params = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
            default_exploration_params = os.path.join(pkg_share, 'config', 'exploration.yaml')
            default_rviz = os.path.join(pkg_share, 'config', 'exploration.rviz')
            gazebo_launch_path = os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        except Exception:
            pass

    declared_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Full path to the ROS2 parameters file for Nav2 exploration'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='Full path to parameters file for SLAM Toolbox'
        ),
        DeclareLaunchArgument(
            'exploration_params_file',
            default_value=default_exploration_params,
            description='Full path to parameters file for frontier_exploration node'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode (no GUI)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 configured for exploration visualization'
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Automatically start Nav2 and exploration'
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    exploration_params_file = LaunchConfiguration('exploration_params_file')
    headless_arg = LaunchConfiguration('headless')
    rviz_arg = LaunchConfiguration('rviz')
    auto_start = LaunchConfiguration('auto_start')

    # SLAM-mode Nav2 lifecycle nodes (NO AMCL, NO map_server)
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator'
    ]

    # 1. Base Gazebo simulation (robot spawn, controllers, /scan and /clock bridges)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'headless': headless_arg,
            'rviz': 'false'  # RViz handled by exploration launch
        }.items()
    )

    # 2. SLAM Toolbox Online Async Node (sole map -> odom transform authority)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 3. Nav2 Controller Server (DWB Local Planner)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    # 4. Nav2 Planner Server (Navfn Global Planner)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 5. Nav2 Behavior Server (Spin, Backup, Wait recovery behaviors)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    # 6. Nav2 BT Navigator (NavigateToPose Action Server)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 7. Nav2 Lifecycle Manager (coordinates the 4 navigation nodes)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': auto_start,
            'node_names': lifecycle_nodes
        }]
    )

    delayed_lifecycle_manager = TimerAction(
        period=4.0,
        actions=[lifecycle_manager_node]
    )

    # 8. Frontier Exploration Node
    explorer_node = Node(
        package='frontier_exploration',
        executable='explorer_node',
        name='frontier_explorer',
        output='screen',
        parameters=[
            exploration_params_file,
            {'use_sim_time': use_sim_time, 'auto_start': auto_start}
        ]
    )

    delayed_explorer = TimerAction(
        period=6.0,
        actions=[explorer_node]
    )

    # 9. RViz2 configured for exploration visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_exploration',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_arg)
    )

    return LaunchDescription(
        declared_args + [
            gazebo_sim,
            slam_node,
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            delayed_lifecycle_manager,
            delayed_explorer,
            rviz_node
        ]
    )

