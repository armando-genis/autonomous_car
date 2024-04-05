import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


def generate_launch_description():

    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sensors_launch'),
                'launch',
                'vectornav.launch.py'
            ])
        ]),
    )

    velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sensors_launch'),
                'launch',
                'velodyne-VLP32C-launch.py'
            ])
        ]),
    )

    lidar_imu_sync = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_imu_sync'),
                'launch',
                'imu_lidar.launch.py'
            ])
        ]),
    )

    urdf_sdv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sdv_robot_description'),
                'launch',
                'display.launch.py'
            ])
        ]),
    )

    return LaunchDescription([
        vectornav,
        velodyne,
        lidar_imu_sync,
        # urdf_sdv,
    ])