import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals

def generate_launch_description():

   this_dir = get_package_share_directory('sdv_localization')

   # For real life tests (config in sdv_localization)
   rviz_config = os.path.join(
      get_package_share_directory('sdv_localization'),
      'launch/rviz_cfg',

      # 'sdv_anniversary.rviz'
      # 'sdv_parking_lot_cetec2.rviz'
      'sdv_parking_lot_cetec.rviz'
   )

    # *********** LAUNCH ARGUMENTS ***********
   is_simulation = DeclareLaunchArgument(
      'is_simulation',
      default_value = 'false',
      description = 'Defines if the application will run in simulation or in real life'
   )

   odometry_source = DeclareLaunchArgument(
      'odometry_source',
      default_value = 'rl', # or rl
      description = 'Defines if the odometry source comes directly from the \
                     vectornav (vn) or from the robot localization pkg (rl)'
   )

    # *********** VECTORNAV VN-300 ***********
    # Vectornav Odometry and Path
   vectornav_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
            PathJoinSubstitution([
               FindPackageShare('sdv_launch'),
               'launch',
               'sensors',
               'vectornav.launch.py'
            ])
      ]),
      condition=UnlessCondition(LaunchConfiguration('is_simulation'))
   )
   
   vn_processing = Node(
      package='sdv_localization', 
      executable='vn_processing',
      output='screen',
      parameters=[
         os.path.join('sdv_launch', 'config', 'vectornav', 'vn_proc_params.yaml'),
                     {'odometry_source': LaunchConfiguration('odometry_source')},
         ],
      condition=UnlessCondition(LaunchConfiguration('is_simulation'))
    )

    # *********** SBG ELLIPSE-D ***********
    # SBG Odometry and Path
   # sbg_launch = IncludeLaunchDescription(
   #    PythonLaunchDescriptionSource([
   #          PathJoinSubstitution([
   #             FindPackageShare('sdv_localization'),
   #             'launch',
   #             'sensors',
   #             'sbg_device_launch.py'
   #          ])
   #    ]),
   #    condition=UnlessCondition(LaunchConfiguration('is_simulation'))
   # )
    
   # sbg_processing = Node(
   #    package='sdv_localization', 
   #    executable='sbg_processing',
   #    output='screen',
   #    parameters=[os.path.join(this_dir, 'config', 'vectornav', 'vn_proc_params.yaml')],
   #    condition=UnlessCondition(LaunchConfiguration('is_simulation'))
   # )

   rviz = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config]
   )

   # *********** TRANSFORMS ***********
    
   # ODOM is in the NED frame
   tf_map_odom_NED = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name="tf_odom_base_link",
      arguments = ['0', '0', '0', '0', '0', '-3.14159', 'map', 'odom'], # x y z yaw pitch roll
      # condition=UnlessCondition(LaunchConfiguration('is_simulation'))
      condition=IfCondition(
         PythonExpression([
            "'",
            LaunchConfiguration('is_simulation'),
            "' == 'true' or '",
            LaunchConfiguration('odometry_source'),
            "' == 'vn'"
         ])
      )
   )

   # ODOM is in the ENU frame
   tf_map_odom_ENU = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name="tf_map_to_odom",
      arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
      # condition=UnlessCondition(LaunchConfiguration('is_simulation'))
      condition=IfCondition(
         PythonExpression([
            "'",
            LaunchConfiguration('is_simulation'),
            "' == 'false' and '",
            LaunchConfiguration('odometry_source'),
            "' == 'rl'"
         ])
      )
   )

   tf_base_link_vectornav = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name="tf_base_link_to_vectornav",
      arguments = ['0', '0', '1.9', '0', '0.0', '0.0', 'base_link', 'vectornav'],
      condition=UnlessCondition(LaunchConfiguration('is_simulation'))
   )

   # tf_base_link_sbg = Node(
   #    package='tf2_ros',
   #    executable='static_transform_publisher',
   #    name="tf_vectornav_to_sbg",
   #    arguments = ['-0.545', '0', '1.9', '0', '0.0', '0.0', 'base_link', 'sbg'],
   #    condition=UnlessCondition(LaunchConfiguration('is_simulation'))
   # )

   tf_base_link_velodyne = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name="tf_base_link_to_velodyne",
      arguments = ['0.45', '0', '2.25', '-0.05', '0.0', '0.0', 'base_link', 'velodyne'],  #-0.05 y
      condition=UnlessCondition(LaunchConfiguration('is_simulation'))
   )
   
   # For AMCL and Robot Localization
   tf_map_to_scan = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name="tf_map_to_scan",
      arguments = ['0', '0', '0', '0', '0', '0', 'map', 'scan'],
      # condition=UnlessCondition(LaunchConfiguration('is_simulation'))
      condition=IfCondition(
         PythonExpression([
            "'",
            LaunchConfiguration('is_simulation'),
            "' == 'false' and '",
            LaunchConfiguration('odometry_source'),
            "' == 'rl'"
         ])
      )
   )

   ld = LaunchDescription()

   ld.add_action(is_simulation)
   ld.add_action(odometry_source)

   ld.add_action(vectornav_launch)
   ld.add_action(vn_processing)
   #ld.add_action(sbg_launch)
   #ld.add_action(sbg_processing)
   #ld.add_action(rviz)

   ld.add_action(tf_map_odom_NED)
   ld.add_action(tf_map_odom_ENU)
   ld.add_action(tf_base_link_vectornav)
   # ld.add_action(tf_base_link_sbg)
   ld.add_action(tf_base_link_velodyne)
   ld.add_action(tf_map_to_scan)
   return ld