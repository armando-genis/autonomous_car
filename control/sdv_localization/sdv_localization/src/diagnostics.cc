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


using namespace std::chrono_literals;

//ROS2 node class diagnostics of SDV sensor topics
class sdv_diagnostics : public rclcpp::Node
  {
    

public:
  sdv_diagnostics() : Node("sdv_diagnosticsNode")
  {
    auto options = rclcpp::SubscriptionOptions();


    timer_ = this->create_wall_timer(250ms,  std::bind(&sdv_diagnostics::timer_callback, this));

   

    //Publishers
    pub_diagnostics_ = this->create_publisher<sdv_localization_msgs::msg::SensorStatArray>("sdv_localization/diagnostics", 10);

    // Subscribers
    
    auto sub_vn_common_cb = std::bind(&sdv_diagnostics::sub_vn_common, this, std::placeholders::_1);
    sub_vn_common_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "vectornav/raw/common", 10, sub_vn_common_cb, options);

    auto sub_velodyne_cb = std::bind(&sdv_diagnostics::sub_velodyne, this, std::placeholders::_1);
    sub_velodyne_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", 10, sub_velodyne_cb, options);

    auto sub_multisense_cb = std::bind(&sdv_diagnostics::sub_multisense, this, std::placeholders::_1);
    sub_multisense_ = this->create_subscription<sensor_msgs::msg::Image>(
      "multisense/left/image_mono", 
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), 
      sub_multisense_cb, 
      options);

      
    //Callback for liveness changes 
    std::function<void(size_t)> vn_liveliness_event_change_cb = std::bind(&sdv_diagnostics::vn_on_liveliness_changed, this, std::placeholders::_1);    
    sub_vn_common_->set_on_new_qos_event_callback(vn_liveliness_event_change_cb, rcl_subscription_event_type_t::RCL_SUBSCRIPTION_LIVELINESS_CHANGED);
  
    //Callback for liveness changes 
    std::function<void(size_t)> vl_liveliness_event_change_cb = std::bind(&sdv_diagnostics::vl_on_liveliness_changed, this, std::placeholders::_1);    
    sub_velodyne_pcl_ ->set_on_new_qos_event_callback(vl_liveliness_event_change_cb, rcl_subscription_event_type_t::RCL_SUBSCRIPTION_LIVELINESS_CHANGED);

    //Callback for liveness changes 
    std::function<void(size_t)> ms_liveliness_event_change_cb = std::bind(&sdv_diagnostics::ms_on_liveliness_changed, this, std::placeholders::_1);    
    sub_multisense_ ->set_on_new_qos_event_callback(ms_liveliness_event_change_cb, rcl_subscription_event_type_t::RCL_SUBSCRIPTION_LIVELINESS_CHANGED);


      
     

      //sub_multisense_ ->set_on_new_qos_event_callb



    //     // Create the subscriber with the Liveliness QoS policy.
    // subscriber_ = this->create_subscription<std_msgs::msg::String>(
    //   "liveliness_topic", qos_profile,
    //   std::bind(&LivelinessSubscriber::on_message_received, this, std::placeholders::_1),
    //   rclcpp::SubscriptionOptions()
    //     .set_liveliness_changed_callback(std::bind(&LivelinessSubscriber::on_liveliness_changed, this, std::placeholders::_1)));

    





  }



private:
  /** Convert VN common group data to ROS2 standard message types
   *
   */
  void sub_vn_common(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in)
  {
    if (!hasPublishervn)
    {
      hasPublishervn = true;
      isChangingvn = true;
    }

    if (msg_in->position.x != 0) {
      isGpsAvailable = true;
    }
    else{
      isGpsAvailable = false;
    }

    if (msg_in->insstatus.mode == 2 && msg_in->insstatus.gps_compass == true)
    {
      isGpsBestQuality = true;
    }
    else {
      isGpsBestQuality = false;
    }
  }

  void sub_velodyne(const sensor_msgs::msg::PointCloud2::SharedPtr msg_in)
  {
    if (!hasPublishervl)
    {
      hasPublishervl = true;
      isChangingvl = true;
    }
  }

  void sub_multisense(const sensor_msgs::msg::Image::SharedPtr msg_in)
  {
    if (!hasPublisherms)
    {
      hasPublisherms = true;
      isChangingms = true;
    }
  }

  void timer_callback()
  {
    sdv_localization_msgs::msg::SensorStatArray msg_output;
    int novnpub = sub_vn_common_->get_publisher_count();
    int novlpub  = sub_velodyne_pcl_->get_publisher_count();
    int nomspub = sub_multisense_ ->get_publisher_count();

    msg_output.header.stamp = this->get_clock()->now();
    
    if (novnpub == 0){
      RCLCPP_INFO_ONCE(get_logger(), "No vectornav publisher");

      hasInitializedvn = false;
      hasPublishervn = false;
      isGpsAvailable = false;
      isChangingvn = false;
    }

    else if (novnpub >= 1 && !hasInitializedvn){
      hasInitializedvn = true;
      
    }

    if (novlpub == 0){
      RCLCPP_INFO_ONCE(get_logger(), "No velodyne publisher");
      hasInitializedvl = false;
      hasPublishervl = false;
      isChangingvl = false;
      
    }
    else if (novlpub >= 1 && !hasInitializedvl) {
      hasInitializedvl = true;
      
    }

    if (nomspub == 0){
      RCLCPP_INFO_ONCE(get_logger(), "No multisense publisher");
      hasInitializedms = false;
      hasPublisherms = false;
      isChangingms = false;
      
    }

    else if (nomspub >= 1 && !hasInitializedms) {
      hasInitializedms = true;
      
    }

    
    //If topic has a single publisher

      msg_output.vn_status.is_enabled = hasInitializedvn;
      msg_output.vl_status.is_enabled = hasInitializedvl;
      msg_output.ms_status.is_enabled = hasInitializedms;
    
    //If topic has been published

      msg_output.vn_status.is_publishing = hasPublishervn;
      msg_output.vl_status.is_publishing = hasPublishervl;
      msg_output.ms_status.is_publishing = hasPublisherms;
    
    //If topic is changing

    msg_output.vn_status.is_changing = isChangingvn;
    msg_output.vl_status.is_changing = isChangingvl;
    msg_output.ms_status.is_changing = isChangingms;

    //If topic has working gps
    msg_output.is_gps_ready = isGpsAvailable;

    //If topic has gps aligned and is using gps compass
    msg_output.is_gps_best_quality = isGpsBestQuality;

    pub_diagnostics_ -> publish(msg_output); 
  }

  void vn_on_liveliness_changed(size_t test)
  {
    // Print "hello" when Liveliness QoS event changes.
    isChangingvn = !isChangingvn;
  }

  void vl_on_liveliness_changed(size_t test)
  {
    // Print "hello" when Liveliness QoS event changes.
    isChangingvl = !isChangingvl;
  }

  void ms_on_liveliness_changed(size_t test)
  {
    // Print "hello" when Liveliness QoS event changes.
    isChangingms = !isChangingms;
  }
  


  
  //vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX
  // Member Variables
  
  //Timer variable
  rclcpp::TimerBase::SharedPtr timer_;


  /// Publishers
  rclcpp::Publisher<sdv_localization_msgs::msg::SensorStatArray>::SharedPtr pub_diagnostics_;

  /// Subscribers
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr sub_vn_common_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_velodyne_pcl_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_multisense_;

  ///Callbacks for liveliness change


  /// Status variables - Initialized in false 
  bool hasInitializedvn = false;
  bool hasInitializedvl = false;
  bool hasInitializedms = false;
  bool hasPublishervn = false;
  bool hasPublishervl = false;
  bool hasPublisherms = false;
  bool isChangingvn = false;
  bool isChangingvl = false;
  bool isChangingms = false;

  //Vn state variables
  bool isGpsAvailable = false;
  bool isGpsBestQuality = false;


  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdv_diagnostics>());
  rclcpp::shutdown();
  return 0;
}