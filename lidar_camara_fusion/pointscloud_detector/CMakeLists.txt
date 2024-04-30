cmake_minimum_required(VERSION 3.5)
project(pointscloud_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(tf2_ros REQUIRED) 
find_package(tf2_eigen REQUIRED) 



# External dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_geometry REQUIRED) 



include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(pointsDectector_node src/PointsDectector.cpp)

ament_target_dependencies(pointsDectector_node rclcpp std_msgs pcl_conversions sensor_msgs message_filters vision_msgs image_geometry visualization_msgs tf2_ros tf2_eigen)
target_link_libraries(pointsDectector_node Eigen3::Eigen)
target_link_libraries(pointsDectector_node ${OpenCV_LIBS})

target_link_libraries(pointsDectector_node ${PCL_LIBRARIES})

install(TARGETS
  pointsDectector_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
