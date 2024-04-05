import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    coniguration_path = os.path.join(get_package_share_directory('lidar3d_clustering'),'config','detection_variables.yaml')

    lidar3d_Clustering_node_ = launch_ros.actions.Node(
        package='lidar3d_clustering',
        executable='lidar3d_clustering_node',
        name='lidar3d_clustering_node',
        output='screen',
        parameters=[coniguration_path]
    )
    
    return launch.LaunchDescription([
        lidar3d_Clustering_node_
    ])