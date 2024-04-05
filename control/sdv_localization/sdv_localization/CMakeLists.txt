cmake_minimum_required(VERSION 3.5)
project(sdv_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# TODO[DERECK] fix library warnings
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# endif()

# find dependencies
# Possibly add for rosdep
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sbg_driver REQUIRED)
#find_package(sdv_localization_msgs REQUIRED)

# find_package(ament_cmake_python REQUIRED)
# find_package(rclpy REQUIRED)

# vncxx

# include_directories(
#   #${catkin_INCLUDE_DIRS}
#   ../sensors/sbg_ros2_driver/include/sbg_driver/
#   ../sensors/sbg_ros2_driver/external/sbgECom/common
#   ../sensors/sbg_ros2_driver/external/sbgECom/src
# )

add_executable(vn_processing src/vn_processing.cc)
# add_executable(diagnostics src/diagnostics.cc)
# add_executable(vslam_repub src/visual_slam_republisher.cc)
# add_executable(isaac_awake src/isaac_vslam_awake2.cpp)
add_executable(sbg_processing src/sbg_processing.cc)
add_executable(temp_tf_odom_map src/temp_tf_odom_map_pub.cc)
ament_target_dependencies(vn_processing rclcpp sensor_msgs vectornav_msgs tf2_geometry_msgs nav_msgs)
# ament_target_dependencies(diagnostics rclcpp sensor_msgs vectornav_msgs tf2_geometry_msgs nav_msgs sdv_localization_msgs)
# ament_target_dependencies(vslam_repub rclcpp sensor_msgs vectornav_msgs tf2_geometry_msgs nav_msgs sdv_localization_msgs)
# ament_target_dependencies(isaac_awake rclcpp geometry_msgs)

# add_executable(sbg_processing src/sbg_processing.cc
#               ../sensors/sbg_ros2_driver/external/sbgECom/src/sbgEComLib.h)
# ament_target_dependencies(sbg_processing rclcpp sensor_msgs vectornav_msgs tf2_geometry_msgs nav_msgs sbg_driver std_srvs)
ament_target_dependencies(sbg_processing rclcpp sensor_msgs tf2_geometry_msgs nav_msgs)
ament_target_dependencies(temp_tf_odom_map rclcpp sensor_msgs vectornav_msgs tf2_geometry_msgs nav_msgs)

install(TARGETS
  vn_processing
  sbg_processing
  # diagnostics
  # vslam_repub
  # isaac_awake
  temp_tf_odom_map
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  # meshes
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/save2csv.py
  scripts/path_amcl.py
  scripts/waypoint_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
