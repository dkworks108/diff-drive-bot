import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_params = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    default_rviz = os.path.join(pkg_dir, 'config', 'slam.rviz')
    gazebo_launch_path = os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')

    if not os.path.exists(default_params):
        try:
            pkg_share = get_package_share_directory('diff_drive_description')
            default_params = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
            default_rviz = os.path.join(pkg_share, 'config', 'slam.rviz')
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
            description='Full path to the ROS2 parameters file for slam_toolbox'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode (no GUI)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 configured for SLAM visualization'
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    headless_arg = LaunchConfiguration('headless')
    rviz_arg = LaunchConfiguration('rviz')

    # 1. Include the base Gazebo simulation (spawns robot, controllers, /scan & /clock bridges)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'headless': headless_arg,
            'rviz': 'false'  # RViz handled by slam launch with SLAM configuration
        }.items()
    )

    # 2. SLAM Toolbox Online Async Node
    start_async_slam_toolbox_node = Node(
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # 3. RViz2 configured with SLAM displays (Fixed frame: map, OccupancyGrid, LaserScan, TF, Odom)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_arg)
    )

    return LaunchDescription(
        declared_args + [
            gazebo_sim,
            start_async_slam_toolbox_node,
            rviz_node
        ]
    )
