import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    publisher_node = launch_ros.actions.Node(
        package='convex_hull_operation',
        executable='convex_hull_op.py',
        name='publisher',
        output='screen',
    )
    
    return launch.LaunchDescription([
        publisher_node
    ])