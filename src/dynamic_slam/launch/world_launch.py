import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    world_file = os.path.join(tb3_pkg, 'worlds', 'turtlebot3_world.world')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_pkg, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'model': 'burger',
            'use_sim_time': 'true',
            'gui': 'true',
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

    # slam_image_subscriber = Node(
    #     package='dynamic_slam',
    #     executable='slam_image_subscriber.py',
    #     name='slam_image_subscriber',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    # )

    teleop_cmd = [
        'bash', '-lc',
        'export TURTLEBOT3_MODEL=burger && ' 
        'ros2 run turtlebot3_teleop teleop_keyboard'
    ]
    teleop_window = ExecuteProcess(
        cmd=['xterm', '-hold', '-e'] + teleop_cmd,
        output='screen',
    )

    return LaunchDescription([
        gazebo_launch,
        # slam_image_subscriber,
        lidar_subscriber,
        teleop_window,
    ])
