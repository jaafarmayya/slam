import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    dy = get_package_share_directory('dynamic_slam')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    world_file = os.path.join(dy, 'worlds', 'turtlebot3_world.world')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dy, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'world': world_file,
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
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dy, 'launch', 'rviz2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    lidar_subscriber = Node(
        package='dynamic_slam',
        executable='lidar_subscriber.py',
        name='lidar_subscriber',
        output='screen',
        emulate_tty=True,
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

    teleop_cmd = [
        'bash', '-lc',
        'export TURTLEBOT3_MODEL=burger && ros2 run turtlebot3_teleop teleop_keyboard'
    ]
    teleop_window = ExecuteProcess(
        cmd=['xterm', '-hold', '-e'] + teleop_cmd,
        output='screen',
    )
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
        arguments=[
            '0', '0', '0',     # x y z offset
            '0', '0', '0',     # roll pitch yaw
            'base_footprint',  # parent frame
            'base_scan'        # child frame
        ],
    )
    # scan_static_filter = Node(
    #     package='dynamic_slam',
    #     executable='scan_based_filter.py',
    #     name='scan_based_filter',
    #     output='screen',
    #     emulate_tty=True,
    #     arguments=['--ros-args', '--log-level', 'dynamic_slam:=DEBUG'],
    # )
    localization_error_node = Node(
        package='dynamic_slam',
        executable='localization_error_node.py',     
        name='localization_error_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],        
        arguments=['--ros-args', '--log-level', 'dynamic_slam:=DEBUG'],
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_box,
        lidar_subscriber,
        move_box_node,
        teleop_window,
        slam_toolbox_launch,
        rviz_launch,
        # scan_static_filter,
        localization_error_node,
        lidar_static_tf
    ])