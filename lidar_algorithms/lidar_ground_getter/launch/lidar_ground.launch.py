import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    publisher_node_ground_lidar = launch_ros.actions.Node(
        package='lidar_ground_getter',
        executable='lidar_ground_node',
        name='lidar_ground_getter',
        output='screen',

    )
    
    return launch.LaunchDescription([
        publisher_node_ground_lidar
    ])