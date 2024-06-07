import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    yolov8 = launch_ros.actions.Node(
        package='yolov8_niagara',
        executable='yolov8NiagaraBetter.py',
        name='yolov8_niagara_node',
        output='screen'

    )
    
    return launch.LaunchDescription([
        yolov8
    ])


