import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Resolve package directory (supports both local source tree and colcon install)
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    xacro_path = os.path.join(pkg_dir, 'urdf', 'robot.xacro')
    default_world = os.path.join(pkg_dir, 'worlds', 'test_world.sdf')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz.rviz')

    if not os.path.exists(xacro_path):
        try:
            pkg_share = get_package_share_directory('diff_drive_description')
            xacro_path = os.path.join(pkg_share, 'urdf', 'robot.xacro')
            default_world = os.path.join(pkg_share, 'worlds', 'test_world.sdf')
            rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')
        except Exception:
            pass

    declared_args = [
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo Fortress in headless mode (server only)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 alongside Gazebo Fortress'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Full path to Gazebo SDF world file'
        )
    ]

    headless_arg = LaunchConfiguration('headless')
    rviz_arg = LaunchConfiguration('rviz')
    world_arg = LaunchConfiguration('world')

    # Expand xacro to URDF string
    robot_desc = subprocess.check_output([
        'xacro', xacro_path,
        'gazebo_system:=ignition'
    ]).decode('utf-8')

    # 1. Robot State Publisher with use_sim_time=True
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 2. Launch Gazebo Fortress (Ignition) via ros_gz_sim
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    gz_args = PythonExpression([
        "'-s -r ' + '", world_arg, "' if '", headless_arg, "' == 'true' else '-r ' + '", world_arg, "'"
    ])

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # 3. Spawn robot entity in Gazebo Fortress
    # Robot wheels touch ground at z=0.0; spawn with base_link at z=0.05
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_diff_drive_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'diff_drive_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ]
    )

    # 4. ROS-Gazebo parameter bridge for /clock (simulation time sync)
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    )

    # 5. Optional RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz_arg)
    )

    return LaunchDescription(
        declared_args + [
            node_robot_state_publisher,
            gz_sim_launch,
            spawn_entity,
            bridge_clock,
            rviz_node
        ]
    )
