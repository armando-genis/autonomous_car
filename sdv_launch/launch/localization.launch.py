import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch

from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
import launch_ros


def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='lidar_localization_ros2').find('lidar_localization_ros2')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/localization.rviz')

    lidar_localization_ros2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_localization_ros2'),
                'launch',
                'lidar_localization.launch.py'
            ])
        ]),
    )

    path_to_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_path_to_odom'),
                'launch',
                'path_to_odom.launch.py'
            ])
        ]),
    )

    waypoints_loader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('waypoints_niagara_loader'),
                'launch',
                'waypointsLoader.launch.py'
            ])
        ]),
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        rviz_node,
        lidar_localization_ros2,
        waypoints_loader,
        path_to_odom,
        
    ])