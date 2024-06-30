from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    config1 = os.path.join(
    get_package_share_directory('bot_drivers'),
    'config',
    'pc2laser.yaml')

    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[('cloud_in', '/velodyne_points'),
                        ('scan', '/scan')],
            parameters=[config1],
            output='screen'
        )
    ])