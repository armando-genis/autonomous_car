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

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "vectornav_msgs/msg/common_group.hpp"



using namespace std::chrono_literals;

//ROS2 node class diagnostics of SDV sensor topics
class sdv_diagnostics : public rclcpp::Node
  {
    

public:
  sdv_diagnostics() : Node("sdv_diagnosticsNode")
  {
    auto options = rclcpp::SubscriptionOptions();
    timer_ = this->create_wall_timer(250ms,  std::bind(&sdv_diagnostics::timer_callback, this));

    //Subscriber to get header
    auto sub_vn_common_cb = std::bind(&sdv_diagnostics::sub_vn_common, this, std::placeholders::_1);
    sub_vn_common_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "vectornav/raw/common", 10, sub_vn_common_cb, options);

    //Publishers
    temp_odom_map_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }








private:
  /** Convert VN common group data to ROS2 standard message types
   *
   */

  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped odom2map_tf;

    odom2map_tf.header.stamp = tempHeader.stamp;
    odom2map_tf.header.frame_id = "map";
    odom2map_tf.child_frame_id = "odom";

    if (counter < 8)
    {
    temp_odom_map_tf ->sendTransform(odom2map_tf);
    }
    counter ++;
  }

  void sub_vn_common(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in)
  {
        tempHeader.set__stamp(msg_in->header.stamp);
  }

  






  
  //vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX
  // Member Variables
  
  //Timer variable
  rclcpp::TimerBase::SharedPtr timer_;


  /// Publishers   
    std::unique_ptr<tf2_ros::TransformBroadcaster> temp_odom_map_tf;


  ///Callbacks for publishing tf
       int counter = 0;
      std_msgs::msg::Header tempHeader;

    /// Subscribers
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr sub_vn_common_;
 
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdv_diagnostics>());
  rclcpp::shutdown();
  return 0;
}