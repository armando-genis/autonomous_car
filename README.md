# Autonomous_car
This codes were made in the autonomous car platform by Vanttec in campus Monterrey.
 
## Install dependencies 
Install this depndeceies in you **home path**. 
```bash
#fundamental libraries
sudo apt-get install libeigen3-dev
sudo apt install libpcl-dev
sudo apt-get install libpcap-dev
sudo apt install can-utils

#ros2 packages
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt-get install ros-humble-pcl-ros
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-perception-pcl
sudo apt install ros-humble-pcl-msgs
sudo apt install ros-humble-vision-opencv
sudo apt install ros-humble-xacro
sudo apt install ros-humble-velodyne-msgs
sudo apt install ros-humble-diagnostic-updater
sudo apt-get install libqt5serialport5-dev

#GTSAM librari for LIO-SAM
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## Observation 
If it is the fist time you build the workspace follow the next commands to do not crash your computer. 
 ```bash
colcon build --packages-select lio_sam
colcon build --packages-select ndt_omp_ros2
colcon build --packages-select lidar_localization_ros2
colcon build --packages-select vectornav_msgs
colcon build --packages-select vectornav
colcon build --packages-select velodyne_msgs
colcon build --packages-select velodyne_driver
colcon build --packages-select velodyne_laserscan
colcon build --packages-select velodyne_pointcloud
colcon build --packages-select velodyne
colcon build --packages-select lidar_imu_sync
colcon build --packages-select mapping_localization_launch
colcon build
```

## Distribution
```bash
src/
|-- control/
    |-- sdv_control
    |-- sdv_localization

|-- interfaces/
    |-- lidar_msgs
    |-- sdv_can
    |-- sdv_msgs

|-- lidar_algorithms/
    |-- convex_hull_operation
    |-- lidar3d_clustering
    |-- lidar_ground_getter
    |-- optimal_planner_lidar
    |-- voxel_grid_filter

|-- mapping_localization/
    |-- lidar_imu_sync
    |-- lidar_localization_ros2
    |-- lidar_path_to_odom
    |-- LIO-SAM
    |-- mapping_localization_launch
    |-- ndt_omp_ros2

|-- sensors/
    |-- sensors_launch
    |-- vectornav
    |-- velodyne-humble-devel

|-- waypoints/
    |-- waypoints_calculations
    |-- waypoints_niagara_creator
    |-- waypoints_niagara_loader

|-- sdv_launch
|-- sdv_robot_description
```

- `control/`: Contains the package related to vehicle control (sdv_control).


- `Interfaces/`: Holds interface modules such as lidar_msgs for LIDAR message definitions, sdv_can for CAN bus interfaces, and sdv_msgs for custom message definitions relevant to the vehicle.

- `Lidar_algorithms/`: Includes various LIDAR processing algorithms like convex_hull_operation for geometric calculations, lidar3d_clustering for object clustering, lidar_ground_getter for ground detection, optimal_planner_lidar for LIDAR-based path planning, and voxel_grid_filter for point cloud filtering.

- `mapping_localization`/: Features tools for syncing LIDAR and IMU data (lidar_imu_sync), ROS2 nodes for LIDAR-based localization (lidar_localization_ros2), utilities to convert LIDAR paths to odometry (lidar_path_to_odom), implementations of LIO-SAM for simultaneous localization and mapping (LIO-SAM), and launch configurations for Lidar-localization and lio-sam (mapping_localization_launch)

    If lidar_localization_ros2, LIO-SAM and ndt_omp_ros2 packages are not installed, enter the appropriate path and follow the commands:

    ```bash
    #LIO SAM
    git clone https://github.com/TixiaoShan/LIO-SAM.git
    cd LIO-SAM/
    git checkout ros2
    cd ..

    #ndt opm ros2
    git clone https://github.com/rsasaki0109/ndt_omp_ros2

    #for LIDAR localition
    git clone https://github.com/rsasaki0109/lidar_localization_ros2.git

    ```

- `Sensors/`: Contains launch files and configurations for sensors (sensors_launch), integration of VectorNav sensors (vectornav), and development packages for Velodyne sensors in ROS2 Humble (velodyne-humble-devel).

    If vectornav package is not installed, enter the appropriate path and follow the commands:

    ```bash
    git clone https://github.com/dawonn/vectornav.git -b ros2
    ```

- `Waypoints/`: Provides tools for calculating the waypoint that the robot should follow (waypoints_calculations), creating waypoints for the robot (waypoints_niagara_creator), and loading these waypoints to be visualized (waypoints_niagara_loader).

- `sdv_launch`: Launch files for starting up the software-defined vehicle's system.

- `sdv_robot_description`: Contains the URDF files and other descriptions necessary for modeling the robot.

# Getting Started

## Set up ROS2
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash #for ro2 humble
source install/setup.bash
```

 ## Sensors Launcher 
![alt text](https://github.com/armando-genis/autonomous_car/blob/main/imgs/sensors.png?raw=true)


 This launch includes the vectornav launch file from the sensors_launch package, which is responsible for initiating and configuring the VectorNav sensors. The velodyne launch configuration from the same package is used to start and set up the Velodyne VLP32C LIDAR sensor. Additionally, the lidar_imu_sync package's launch file is called to synchronize LIDAR and IMU data, facilitating more accurate localization and mapping. Also, the urdf_sdv is launched from the sdv_robot_description package, which display the robot's URDF in a visualization environment.
 
```bash
source install/setup.bash
ros2 launch sdv_launch sensors.launch.py
```

 ## Localization Launcher 
![alt text](https://github.com/armando-genis/autonomous_car/blob/main/imgs/mapping.png?raw=true)

 This launch configuration sets up several components for navigation and visualization. It initializes lidar_localization_ros2 from the mapping_localization_launch package to handle LIDAR-based localization. It launch the lidar_path_to_odom pkg that converts LIDAR-detected paths into odometry information. waypoints_loader from waypoints_niagara_loader is used to manage the loading of navigation waypoints. Additionally, an rviz_node from the rviz2 package is configured to launch the RVIZ visualization tool. 

```bash
source install/setup.bash
ros2 launch sdv_launch localization.launch.py
```

## Creation of waypoints
![alt text](https://github.com/armando-genis/autonomous_car/blob/main/imgs/waypoints.png?raw=true)

Waypoints are generated based on the data received from the **/odom** topic type `nav_msgs::msg::odometry`. The **interval_** variable, expressed in meters, determines the distance between each waypoint. By adjusting the value of **interval_**,the spacing of the waypoints will change. Additionally, it is important to update the **file_path_** variable to specify the location where the waypoint data should be saved. The output topics, to visualize the creation of the path, of this package are **/waypoints** and **/waypoints_info** type `visualization_msgs::msg::MarkerArray`. 

```bash
colcon build --packages-select waypoints_niagara_creator
source install/setup.bash
ros2 launch waypoints_niagara_creator waypoints.launch.py
```


 ## Control Launcher 
![alt text](https://github.com/armando-genis/autonomous_car/blob/main/imgs/control.png?raw=true)

 This launch starts by launching the stanley_controller from the sdv_control package, which manages the vehicle's steering and navigation algorithms using the Stanley method. There's also a setup for the can_controller from the sdv_can package, intended to manage CAN bus devices. The TimerAction is configured to delay the launch of the can_controller by 3 seconds.
```bash
source install/setup.bash
ros2 launch sdv_launch control.launch.py
```
