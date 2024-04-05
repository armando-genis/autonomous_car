import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch

from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
import launch_ros



def generate_launch_description():


    stanley_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sdv_control'),
                'launch',
                'control_launch.py'
            ])
        ]),
    )

    can_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sdv_can'),
                'launch',
                'can_devices.launch.py'
            ])
        ]),
    )



    return LaunchDescription([

        stanley_controller,
        TimerAction(
            actions=[
                can_controller
            ],
            period='3.0',  
        ),
        
    ])