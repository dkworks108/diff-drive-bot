import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_dir, 'maps', 'test_world_map.yaml')
    default_rviz = os.path.join(pkg_dir, 'config', 'navigation.rviz')
    gazebo_launch_path = os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')

    if not os.path.exists(default_params):
        try:
            pkg_share = get_package_share_directory('diff_drive_description')
            default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
            default_map = os.path.join(pkg_share, 'maps', 'test_world_map.yaml')
            default_rviz = os.path.join(pkg_share, 'config', 'navigation.rviz')
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
            default_value=default_params,
            description='Full path to the ROS2 parameters file for Nav2'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode (no GUI)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 configured for Navigation visualization'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    headless_arg = LaunchConfiguration('headless')
    rviz_arg = LaunchConfiguration('rviz')
    autostart = LaunchConfiguration('autostart')

    lifecycle_nodes = [
        'map_server',
        'amcl',
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
            'rviz': 'false'  # RViz handled by navigation launch
        }.items()
    )

    # 2. Nav2 Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml_file, 'use_sim_time': use_sim_time}]
    )

    # 3. AMCL Localization Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 4. Controller Server (DWB Local Planner)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    # 5. Planner Server (Navfn Global Planner)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 6. Behavior Server (Spin, Backup, Wait recovery behaviors)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    # 7. BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 8. Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes
        }]
    )

    # 9. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_arg)
    )

    return LaunchDescription(
        declared_args + [
            gazebo_sim,
            map_server_node,
            amcl_node,
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            lifecycle_manager_node,
            rviz_node
        ]
    )

