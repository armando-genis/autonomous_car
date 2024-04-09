# autonomous_car
 
## Install dependencies 
Install this depndeceies in you home path. 
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

#GTSAM librari for LIO-SAM
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## Observation 
If is the fist time you build the workspace follow the next commands to do not crash your computer. 
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

 ## Set up ROS2
```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
source install/setup.bash
```



# Creation of waypoints


 ## Sensors
```bash
source install/setup.bash
ros2 launch sdv_launch sensors.launch.py
```

 ## Localization
```bash
source install/setup.bash
ros2 launch sdv_launch localization.launch.py
```

 ## Control
```bash
source install/setup.bash
ros2 launch sdv_launch control.launch.py
```
