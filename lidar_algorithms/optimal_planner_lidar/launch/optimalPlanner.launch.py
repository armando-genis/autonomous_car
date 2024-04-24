import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    publisher_node_planner = launch_ros.actions.Node(
        package='optimal_planner_lidar',
        executable='lidar_planner',
        name='optimal_planner_lidar',
        output='screen',

    )
    
    return launch.LaunchDescription([
        publisher_node_planner
    ])