from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='frenet_frame',
            executable='frenet_frame_node',
            name='frenet_frame_node',
            output='screen'
        )
    ])


