import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    cloud_detected = launch_ros.actions.Node(
        package='pointscloud_detector',
        executable='pointsDectector_node',
        name='pointsDectector_node',
        output='screen'

    )
    
    return launch.LaunchDescription([
        cloud_detected
    ])


