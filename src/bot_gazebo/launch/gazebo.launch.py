import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable,FindExecutable,Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = True
    bot_type = os.getenv('BOT_TYPE')
    bot_type = "false"

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("bot_drivers"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("aws_robomaker_small_warehouse_world"), "worlds", "no_roof_small_warehouse" , "no_roof_small_warehouse.world"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('bot_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value="true",
            description='launch rviz or not'
        ),

        DeclareLaunchArgument(
            name='spawn_x', 
            default_value='1.8',
            description='Robot spawn position in X axis'
        ),

        DeclareLaunchArgument(
            name='spawn_y', 
            default_value='-9.5',
            description='Robot spawn position in Y axis'
        ),

        DeclareLaunchArgument(
            name='spawn_z', 
            default_value='0.1',
            description='Robot spawn position in Z axis'
        ),
            
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='1.5714',
            description='Robot spawn heading'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory(
                    'aws_robomaker_small_warehouse_world'), '/launch/small_warehouse.launch.py']
            ),
            launch_arguments={
                'world': os.path.join(get_package_share_directory('aws_robomaker_small_warehouse_world'), 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')
            }.items()
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_husky',
            arguments=['-entity',
                    'husky',
                    '-x', LaunchConfiguration('spawn_x'),
                    '-y', LaunchConfiguration('spawn_y'),
                    '-z', LaunchConfiguration('spawn_z'),
                    '-Y', LaunchConfiguration('spawn_yaw'),
                    '-topic',
                    'robot_description'
                    ],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'true',
                # 'bot_type' : bot_type
                'rviz' : LaunchConfiguration("rviz")
            }.items()
        )
    ])
