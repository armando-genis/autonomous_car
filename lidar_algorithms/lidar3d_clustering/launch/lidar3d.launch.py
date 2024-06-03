import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable


def generate_launch_description():
    
    configuration_path = os.path.join(get_package_share_directory('lidar3d_clustering'),'config','detection_variables.yaml')

    lidar3d_Clustering_node_ = launch_ros.actions.Node(
        package='lidar3d_clustering',
        executable='lidar3d_clustering_node',
        name='lidar3d_clustering_node',
        output='screen',
        parameters=[configuration_path]
    )
    
    return launch.LaunchDescription([
        # SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value='{message}'),
        lidar3d_Clustering_node_
    ])
