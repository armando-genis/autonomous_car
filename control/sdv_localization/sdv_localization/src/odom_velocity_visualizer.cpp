#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/time_group.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sdv_localization_msgs/msg/sensor_diag.hpp"
#include "sdv_localization_msgs/msg/sensor_stat_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


using namespace std::chrono_literals;

//ROS2 node class diagnostics of SDV sensor topics
class sdv_odom_velocity_marker_node : public rclcpp::Node
  {
    

public:
  sdv_odom_velocity_marker_node() : Node("sdv_odom_velocity_marker_node")
  {

    //Publishers

    pub_vn_odom_visualizer_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sdv_localization/vn_velocity_visualizer", 10);
    pub_visual_slam_odom_visualizer_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sdv_localization/visual_slam_velocity_visualizer", 10);

    // Subscribers
    
    auto sub_vn_odom_cb = std::bind(&sdv_odom_velocity_marker_node::sub_vn_odom, this, std::placeholders::_1);
    sub_vn_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "sdv_localization/odom", 
      10, 
      sub_vn_odom_cb);

    auto sub_visual_slam_odom_cb = std::bind(&sdv_odom_velocity_marker_node::sub_visual_slam_odom, this, std::placeholders::_1);
    sub_visual_slam_odom_= this->create_subscription<nav_msgs::msg::Odometry>(
      "visual_slam/tracking/odometry", 
      10, 
      sub_visual_slam_odom_cb);
    



  }



private:
  /** Callback to 
   *
   */
  void sub_vn_odom(const nav_msgs::msg::Odometry::SharedPtr msg_in)
  {
    vn_odom_marker_array.markers.clear();
    visualization_msgs::msg::Marker tempMarker;
    tempMarker.header = msg_in->header;
    tempMarker.header.frame_id = "base_link";
    tempMarker.type = tempMarker.ARROW;

    tempMarker.color.a = 1;
    tempMarker.color.r = 1;
    tempMarker.color.g = 0;
    tempMarker.color.b = 0;
    tempMarker.scale.x = 0.1;
    tempMarker.scale.y = 0.1;
    tempMarker.scale.z = 0.1 ;



      
      //Visualizacion de velocidad en X - ROJO
      tempMarker.set__id(0);
      geometry_msgs::msg::Pose TempPose;
      TempPose.orientation.set__w(1);
      TempPose.position.x = msg_in->twist.twist.linear.y;
      tempMarker.pose = TempPose;
      vn_odom_marker_array.markers.push_back(tempMarker);
      TempPose.position.x = 0;




      //Visualizacion de velocidad en Y - VERDE
      tempMarker.set__id(1);
      tempMarker.color.a = 1;
      tempMarker.color.r = 0;
      tempMarker.color.g = 1;
      tempMarker.color.b = 0;
      TempPose.orientation.set__w(0.707);
      TempPose.orientation.set__x(0);
      TempPose.orientation.set__y(0);
      TempPose.orientation.set__z(0.707);

      TempPose.position.y = msg_in->twist.twist.linear.x;
      tempMarker.pose = TempPose;
      vn_odom_marker_array.markers.push_back(tempMarker);
      TempPose.position.y = 0;


      //Visualizacion de velocidad en  - AZUL
      tempMarker.set__id(2);
      tempMarker.color.a = 1;
      tempMarker.color.r = 0;
      tempMarker.color.g = 0;
      tempMarker.color.b = 1;
      TempPose.orientation.set__w(0.707);
      TempPose.orientation.set__x(0);
      TempPose.orientation.set__y(-0.707);
      TempPose.orientation.set__z(0);
      TempPose.position.z = msg_in->twist.twist.linear.z;
      tempMarker.pose = TempPose;
      vn_odom_marker_array.markers.push_back(tempMarker);
      TempPose.position.z = 0;

      pub_vn_odom_visualizer_ ->publish(vn_odom_marker_array);    
  }

    void sub_visual_slam_odom(const nav_msgs::msg::Odometry::SharedPtr msg_in) const
  {
  
  }
  


  /// Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vn_odom_visualizer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_visual_slam_odom_visualizer_;

  /// Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vn_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_visual_slam_odom_;

  // Marker arrays

  visualization_msgs::msg::MarkerArray vn_odom_marker_array;
  visualization_msgs::msg::MarkerArray visual_slam_odom_marker_array;


  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdv_odom_velocity_marker_node>());
  rclcpp::shutdown();
  return 0;
}