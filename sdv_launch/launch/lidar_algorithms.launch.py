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


    voxel_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('voxel_grid_filter'),
                'launch',
                'filter.launch.py'
            ])
        ]),
    )

    ground_getter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_ground_getter'),
                'launch',
                'lidar_ground.launch.py'
            ])
        ]),
    )


    lidar3d_cluster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar3d_clustering'),
                'launch',
                'lidar3d.launch.py'
            ])
        ]),
    )

    return LaunchDescription([

        voxel_filter,
        TimerAction(
            actions=[
                ground_getter,
                lidar3d_cluster
            ],
            period='2.0', 
        ),
        
    ])