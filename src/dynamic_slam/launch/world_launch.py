import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    folder_arg = DeclareLaunchArgument('folder_name', default_value='experiment')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    slam_arg = DeclareLaunchArgument('slam', default_value='true')
    map_arg = DeclareLaunchArgument('map', default_value='')
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([FindPackageShare('dynamic_slam'), 'config', 'nav2_params.yaml'])
    )

    folder_name = LaunchConfiguration('folder_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    dy = get_package_share_directory('dynamic_slam')

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(dy, 'launch', 'turtlebot3_world.launch.py')),
        launch_arguments={
            'world': os.path.join(dy, 'worlds', 'turtlebot3_world.world'),
            'model': 'burger',
            'use_sim_time': use_sim_time,
            'gui': 'true',
            'extra_gazebo_args': '-s libgazebo_ros_api_plugin.so'
        }.items()
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(dy, 'launch', 'online_sync_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(dy, 'launch', 'rviz2.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    lidar_subscriber = Node(
        package='dynamic_slam',
        executable='lidar_subscriber.py',
        name='lidar_subscriber',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'dynamic_slam:=DEBUG']
    )

    teleop_window = ExecuteProcess(
        cmd=['xterm', '-hold', '-e', 'bash', '-lc', 'export TURTLEBOT3_MODEL=burger && ros2 run turtlebot3_teleop teleop_keyboard'],
        output='screen'
    )

    localization_error_node = Node(
        package='dynamic_slam',
        executable='localization_error_node.py',
        name='localization_error_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}, {'folder_name': folder_name}],
        arguments=['--ros-args', '--log-level', 'dynamic_slam:=DEBUG']
    )

    spawn_box = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(dy, 'models', 'moving_box.sdf'), '-entity', 'moving_box', '-x', '0', '-y', '1', '-z', '0'],
        output='screen'
    )

    move_box_node = Node(
        package='dynamic_slam',
        executable='move_box.py',
        name='move_box',
        output='screen'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': params_file,
            'slam': slam,
            'map': map_yaml
        }.items()
    )

    return LaunchDescription([
        folder_arg,
        use_sim_time_arg,
        slam_arg,
        map_arg,
        params_arg,
        set_tb3_model,
        gazebo_launch,
        slam_toolbox_launch,
        nav2_bringup,
        rviz_launch,
        lidar_subscriber,
        teleop_window,
        localization_error_node,
        spawn_box,
        move_box_node
    ])
