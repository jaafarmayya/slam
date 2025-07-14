import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(tb3_gazebo_pkg, 'worlds', 'turtlebot3_world.world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'robot_model': 'waffle',
            'use_sim_time': 'true',
            'gui': 'true'
        }.items()
    )

    return LaunchDescription([
        gazebo_launch
    ])
