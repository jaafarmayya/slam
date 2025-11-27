import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- launch arg ---
    folder_arg = DeclareLaunchArgument(
        'folder_name',
        default_value='experiment',
        description='Name of the experiment folder'
    )
    folder_name = LaunchConfiguration('folder_name')

    dy = get_package_share_directory('dynamic_slam')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dy, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(dy, 'worlds', 'turtlebot3_world.world'),
            'model': 'burger',
            'use_sim_time': 'true',
            'gui': 'true',
            'extra_gazebo_args': '-s libgazebo_ros_api_plugin.so',
        }.items()
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dy, 'launch', 'online_sync_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dy, 'launch', 'rviz2.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    lidar_subscriber = Node(
        package='dynamic_slam',
        executable='lidar_subscriber.py',
        name='lidar_subscriber',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'dynamic_slam:=DEBUG'],
    )

    teleop_window = ExecuteProcess(
        cmd=[
            'xterm', '-hold', '-e',
            'bash', '-lc',
            'export TURTLEBOT3_MODEL=burger && ros2 run turtlebot3_teleop teleop_keyboard'
        ],
        output='screen',
    )

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
        arguments=[
            '0','0','0','0','0','0','base_footprint','base_scan'
        ],
    )

    localization_error_node = Node(
        package='dynamic_slam',
        executable='localization_error_node.py',
        name='localization_error_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': True},
            {'folder_name': folder_name},      # ← pass the launch‐arg here
        ],
        arguments=['--ros-args', '--log-level', 'dynamic_slam:=DEBUG'],
    )
    spawn_box = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(dy, 'models', 'moving_box.sdf'),
            '-entity', 'moving_box',
            '-x', '0', '-y', '1', '-z', '0'
        ],
        output='screen'
    )

    move_box_node = Node(
        package='dynamic_slam',
        executable='move_box.py',
        name='move_box',
        output='screen',
    )
    scan_static_filter = Node(
        package='dynamic_slam',
        executable='scan_based_filter.py',
        name='scan_based_filter',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'dynamic_slam:=DEBUG'],
    )
    return LaunchDescription([
        folder_arg,
        gazebo_launch,
        #lidar_subscriber,
        teleop_window,
        slam_toolbox_launch,
        rviz_launch,
        #localization_error_node,
        #lidar_static_tf,  
        spawn_box,
        move_box_node,
        #scan_static_filter
    ])