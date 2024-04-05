/** ----------------------------------------------------------------------------
 * @file: vn_processing.cc
 * @date: August 31, 2023
 * @author: Rogelio Salais
 * @author: Sebas Mtz
 *
 * @brief: Processing of the Vectornav Messages is made in this node.
 *         This node publishes:
 *          - ODOM
 *          - A TF Broadcaster
 *          - POSE
 *          - INS reference
 *          - ECEF reference
 *        The information published depends on the frame used (ENU, NED) from
 *        the configuration of the vehicle.
 *        This can be improved to receive from a parameter the desired output frame,
 *        and the parent and child frame ID's, so they are not hardcoded.
 * -----------------------------------------------------------------------------
 **/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/time_group.hpp"

#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"


//ROS2 node class for odometry, NED pose, and velodyne transform - 
class vnGPSPose : public rclcpp::Node
{

public:
  vnGPSPose() : Node("vn_processing")
  {
    //Parameter
    this->declare_parameter<std::vector<double>>("orientation_covariance", orientation_covariance_);
    this->declare_parameter<std::vector<double>>("angular_velocity_covariance", angular_velocity_covariance_);
    this->declare_parameter<std::vector<double>>("linear_acceleration_covariance", linear_acceleration_covariance_);
    this->declare_parameter<std::vector<double>>("magnetic_covariance", magnetic_field_covariance_);

    this->declare_parameter<std::vector<double>>("global_lla_reference", global_ref_ins_poslla_);
    this->declare_parameter<std::vector<double>>("global_ecef_reference", global_ref_ins_posecef_);

    this->declare_parameter("odometry_source", rclcpp::PARAMETER_STRING);


    odom_src_ = this->get_parameter("odometry_source").as_string();

    // Publishers
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("sdv_localization/vectornav/odom", 10);

    pub_pose =  this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("sdv_localization/vectornav/pose", 10);
    
    pub_ref_ecef = this->create_publisher<geometry_msgs::msg::Point>("sdv_localization/vectornav/ref_ecef", 10);
    pub_ref_ins = this->create_publisher<geometry_msgs::msg::Point>("sdv_localization/vectornav/ref_ins", 10);

    pub_path =  this->create_publisher<nav_msgs::msg::Path>("sdv_localization/nav/vectornav/path", 10);
    pub_path_enu =  this->create_publisher<nav_msgs::msg::Path>("sdv_localization/nav/vectornav/path_enu", 10);

    // Subscribers
    auto sub_vn_common_cb = std::bind(&vnGPSPose::sub_vn_common, this, std::placeholders::_1);
    sub_vn_common_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "vectornav/raw/common", 10, sub_vn_common_cb);

    auto sub_vn_time_cb = std::bind(&vnGPSPose::sub_vn_time, this, std::placeholders::_1);
    sub_vn_time_ = this->create_subscription<vectornav_msgs::msg::TimeGroup>(
      "vectornav/raw/time", 10, sub_vn_time_cb);

    auto sub_vn_imu_cb = std::bind(&vnGPSPose::sub_vn_imu, this, std::placeholders::_1);
    sub_vn_imu_ = this->create_subscription<vectornav_msgs::msg::ImuGroup>(
      "vectornav/raw/imu", 10, sub_vn_imu_cb);

    auto sub_vn_gps_cb = std::bind(&vnGPSPose::sub_vn_gps, this, std::placeholders::_1);
    sub_vn_gps_ = this->create_subscription<vectornav_msgs::msg::GpsGroup>(
      "vectornav/raw/gps", 10, sub_vn_gps_cb);

    auto sub_vn_attitude_cb =
      std::bind(&vnGPSPose::sub_vn_attitude, this, std::placeholders::_1);
    sub_vn_attitude_ = this->create_subscription<vectornav_msgs::msg::AttitudeGroup>(
      "vectornav/raw/attitude", 10, sub_vn_attitude_cb);

    auto sub_vn_ins_cb = std::bind(&vnGPSPose::sub_vn_ins, this, std::placeholders::_1);
    sub_vn_ins_ = this->create_subscription<vectornav_msgs::msg::InsGroup>(
      "vectornav/raw/ins", 10, sub_vn_ins_cb);

    auto sub_vn_gps2_cb = std::bind(&vnGPSPose::sub_vn_gps2, this, std::placeholders::_1);
    sub_vn_gps2_ = this->create_subscription<vectornav_msgs::msg::GpsGroup>(
      "vectornav/raw/gps2", 10, sub_vn_gps2_cb);
  }

private:
  /** Convert VN common group data to ROS2 standard message types
   *
   */
  void sub_vn_common(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in)
  {
    std::array<double, 3> global_ref_ins_poslla_;
    std::array<double, 3> global_ref_ins_posecef_;

    fill_insreference_from_param("global_lla_reference", global_ref_ins_poslla_);
    fill_insreference_from_param("global_ecef_reference", global_ref_ins_posecef_);
    
    //RCLCPP_INFO(get_logger(), "Frame ID: '%s'", msg_in->header.frame_id.c_str());
    // Time Reference (Startup)

    //NED_POSE_Publish
    geometry_msgs::msg::PoseWithCovarianceStamped ned_pose_msg;

    //ENU_POSE_Publish
    geometry_msgs::msg::PoseWithCovarianceStamped enu_pose_msg;

    if (msg_in->insstatus.mode == msg_in->insstatus.MODE_NO_GPS) //Verify if GPS signal is not lost
    {
      hasInitialized = false;
    }

    if(hasInitialized){

        RCLCPP_INFO_EXPRESSION(get_logger(), (msg_in->insstatus.mode == msg_in->insstatus.MODE_ALIGNING), "Aligning INS Compass");
        std::array<double, 3> currECEF;
        currECEF[0] = ins_posecef_.x;
        currECEF[1] = ins_posecef_.y;
        currECEF[2] = ins_posecef_.z;

        std::array<double, 3> globalNED = calculateNED(global_ref_ins_posecef_, global_ref_ins_poslla_, currECEF);

        //We changed to TF2 standard messages for easier matrix rotation and subtraction

        /* POSE MSGS */
          // NED
        ned_pose_msg.header.frame_id = "odom";
        ned_pose_msg.header.set__stamp(msg_in->header.stamp);

        ned_pose_msg.pose.pose.position.x = globalNED[0];
        ned_pose_msg.pose.pose.position.y = globalNED[1]; //To define 
        ned_pose_msg.pose.pose.position.z = 0;

        ned_pose_msg.pose.pose.orientation = msg_in->quaternion;

          // ENU
        enu_pose_msg.header.frame_id = "odom";
        enu_pose_msg.header.set__stamp(msg_in->header.stamp);

        enu_pose_msg.pose.pose.position.x = globalNED[1]; //Change between 
        enu_pose_msg.pose.pose.position.y = globalNED[0]; 
        enu_pose_msg.pose.pose.position.z = 0;

        tf2::Quaternion quaternion(msg_in->quaternion.x, msg_in->quaternion.y, msg_in->quaternion.z, msg_in->quaternion.w);
        tf2::Matrix3x3 mat(quaternion);

        double roll, pitch, yaw;

        mat.getRPY(roll, pitch, yaw);

        tf2::Quaternion quat;
        quat.setRPY(0, 0, -yaw);
        // quat.setYPR(yaw, pitch, roll);
        geometry_msgs::msg::Quaternion q;
        q.x = quat.x(); 
        q.y = quat.y();
        q.z = quat.z();
        q.w = quat.w();

        // tf2::Quaternion quaternion1( q.x,
        //                             q.y,
        //                             q.z, 
        //                             q.w);
        // tf2::Matrix3x3 mat2(quaternion1);

        // mat2.getRPY(roll, pitch, yaw);

        // std::cout << "Vectornav" << std::endl;
        // std::cout << "yaw = " << yaw * 180 / M_PI << " pitch = " << pitch * 180 / M_PI << " roll = " << roll * 180 / M_PI<< std::endl;

        // enu_pose_msg.pose.pose.orientation = tf2::toMsg(quaternionResult);
        enu_pose_msg.pose.pose.orientation = q;

        pub_pose->publish(ned_pose_msg);

        /* PATH MSGS */
        geometry_msgs::msg::PoseStamped pose;

          // NED
        pose.pose = ned_pose_msg.pose.pose;
        pose.header = ned_pose_msg.header;

        ned_path.header = ned_pose_msg.header;
        ned_path.poses.push_back(pose);
        pub_path->publish(ned_path);

          // ENU
        pose.pose = enu_pose_msg.pose.pose;
        pose.header = enu_pose_msg.header;
        enu_path.header = enu_pose_msg.header;
        enu_path.poses.push_back(pose);
        pub_path_enu->publish(enu_path);

        /* ODOMETRY MSGS */
        nav_msgs::msg::Odometry odom_msg;

          // ENU
        odom_msg.header = ned_pose_msg.header;
        odom_msg.child_frame_id = "vectornav";
        odom_msg.pose = ned_pose_msg.pose;
        geometry_msgs::msg::Vector3 vel;
        //Switch between Yawpitchroll in NED to ENU format = pitch <-> roll
        vel.x = msg_in->angularrate.x;
        vel.y = msg_in->angularrate.z;
        vel.z = msg_in->angularrate.y;
        odom_msg.twist.twist.angular = vel;

        odom_msg.twist.twist.linear = ins_velbody_;

        //Switch from NED to ENU velocity
        // vel.x = msg_in->velocity.y;
        // vel.y = msg_in->velocity.x;
        // vel.z = -msg_in->velocity.z;
        // odom_msg.twist.twist.linear = vel;

        // Publish Odometry in ENU
        pub_odom_->publish(odom_msg);

        /* TRANSFORM BROADCASTER */
        geometry_msgs::msg::TransformStamped tf;

        tf.header.frame_id = "odom";
        tf.header.set__stamp(msg_in->header.stamp);
        tf.child_frame_id = "base_link";
        
        if(odom_src_ == "rl"){
          tf.transform.translation.x = enu_pose_msg.pose.pose.position.x;
          tf.transform.translation.y = enu_pose_msg.pose.pose.position.y;
          tf.transform.translation.z = enu_pose_msg.pose.pose.position.z;
          tf.transform.set__rotation(enu_pose_msg.pose.pose.orientation);
        } else {
          tf.transform.translation.x = ned_pose_msg.pose.pose.position.x;
          tf.transform.translation.y = ned_pose_msg.pose.pose.position.y;
          tf.transform.translation.z = ned_pose_msg.pose.pose.position.z;
          tf.transform.set__rotation(ned_pose_msg.pose.pose.orientation);
        }
        
        odom_tf_broadcaster_->sendTransform(tf);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Reference can't be calculated, location data is 0");
    } 

  }

  /** Convert VN time group data to ROS2 standard message types
   *
   */
  void sub_vn_time(const vectornav_msgs::msg::TimeGroup::SharedPtr msg_in) const {}

  /** Convert VN imu group data to ROS2 standard message types
   *
   */
  void sub_vn_imu(const vectornav_msgs::msg::ImuGroup::SharedPtr msg_in) const {}

  /** Convert VN gps group data to ROS2 standard message types
   *
   * TODO(Dereck): Consider alternate sync methods
   */
  void sub_vn_gps(const vectornav_msgs::msg::GpsGroup::SharedPtr msg_in)
  {
    gps_fix_ = msg_in->fix;
  }

  /** Convert VN attitude group data to ROS2 standard message types
   *
   */
  void sub_vn_attitude(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg_in) const {}

  /** Convert VN ins group data to ROS2 standard message types
   *
   */
  void sub_vn_ins(const vectornav_msgs::msg::InsGroup::SharedPtr msg_in)
  {
    //RCLCPP_INFO(this->get_logger(), "posecef: %f, %f, %f", msg_in->posecef.x, msg_in->posecef.y, msg_in->posecef.z);
    ins_velbody_ = msg_in->velbody;
    ins_posecef_ = msg_in->posecef;
    ins_poslla_ = msg_in->poslla;
    ins_velned_ = msg_in->velned;
    
    
    //If the system still hasnt initialized
    if(!hasInitialized){
    
        //If the system has data different than zero
        if(ins_posecef_.x!=0 && ins_posecef_.y!=0 && ins_poslla_.x!=0 & ins_poslla_.y!= 0){  
            RCLCPP_INFO_ONCE(get_logger(), "GPS is initialized, starting publishers and tf");
            hasInitialized = true;
      }
    }
  }

  /** Convert VN gps2 group data to ROS2 standard message types
   *
   */
  void sub_vn_gps2(const vectornav_msgs::msg::GpsGroup::SharedPtr msg_in) const {}

  /** Copy a covariance matrix array from a parameter into a msg array
   *
   * If a single value is provided, this will set the diagonal values
   * If three values are provided, this will set the diagonal values
   * If nine values are provided, this will fill the matrix
   *
   * \param param_name Name of the parameter to read
   * \param array Array to fill
   */
  void fill_covariance_from_param(std::string param_name, std::array<double, 9> & array) const
  {
    auto covariance = get_parameter(param_name).as_double_array();

    auto length = covariance.size();
    switch (length) {
      case 1:
        array[0] = covariance[0];
        array[3] = covariance[0];
        array[8] = covariance[0];
        break;

      case 3:
        array[0] = covariance[0];
        array[3] = covariance[1];
        array[8] = covariance[3];
        break;

      case 9:
        std::copy(covariance.begin(), covariance.end(), array.begin());
        break;

      default:
        RCLCPP_ERROR(
          get_logger(), "Parameter '%s' length is %zu; expected 1, 3, or 9", param_name.c_str(),
          length);
    }
  }


    void fill_insreference_from_param(std::string param_name, std::array<double, 3> & array) const
  {
    auto references = get_parameter(param_name).as_double_array();


    auto length = references.size();
    switch (length) {
      case 2:
        array[0] = references[0];
        array[1] = references[1];
        break;

      case 3:
        array[0] = references[0];
        array[1] = references[1];
        array[2] = references[2];
        break;

      default:
        RCLCPP_ERROR(
          get_logger(), "Parameter '%s' length is %zu; expected 1, 3, or 9", param_name.c_str(),length);
    }
    
  }

  //Calculates NED position, without orientation or Z value
  std::array<double, 3> calculateNED(std::array<double, 3> ecefref, std::array<double, 3> llaref, std::array<double, 3> ecefcurr){
    
    std::array<double, 3> NED;  //X(N), Y(E), Z(D)

     float latInRads = deg2rad(llaref[0]); 
      float lonInRads = deg2rad(llaref[1]);

        tf2::Matrix3x3 RotMatEcef2Ned;
        
        
        RotMatEcef2Ned.setValue(-sin(latInRads) * cos(lonInRads), -sin(latInRads) * sin(lonInRads), cos(latInRads),
		   -sin(lonInRads), cos(lonInRads), 0,
		   -cos(latInRads) * cos(lonInRads), -cos(latInRads) * sin(lonInRads), -sin(latInRads));
            
        tf2::Vector3 VectorRefEcef, VectorPoseEcef;

        VectorRefEcef.setX(ecefref[0]);
        VectorRefEcef.setY(ecefref[1]);
        VectorRefEcef.setZ(ecefref[2]);

        VectorPoseEcef.setX(ecefcurr[0]);
        VectorPoseEcef.setY(ecefcurr[1]);
        VectorPoseEcef.setZ(ecefcurr[2]);

        //We changed to TF2 standard messages for easier matrix rotation and subtraction

        tf2::Vector3 VectorPoseInNED;

        VectorPoseInNED = RotMatEcef2Ned * (VectorPoseEcef - VectorRefEcef);

        NED[0] = VectorPoseInNED.getX();
        NED[1] = VectorPoseInNED.getY();
        NED[2] = 0;

        return NED;

  }

  /// Convert from DEG to RAD
  inline static double deg2rad(double in) { return in * M_PI / 180.0; }

  //vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX
  // Member Variables
  //

  /// Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ref_ins;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ref_ecef;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_enu;

  /// Subscribers
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr sub_vn_common_;
  rclcpp::Subscription<vectornav_msgs::msg::TimeGroup>::SharedPtr sub_vn_time_;
  rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr sub_vn_imu_;
  rclcpp::Subscription<vectornav_msgs::msg::GpsGroup>::SharedPtr sub_vn_gps_;
  rclcpp::Subscription<vectornav_msgs::msg::AttitudeGroup>::SharedPtr sub_vn_attitude_;
  rclcpp::Subscription<vectornav_msgs::msg::InsGroup>::SharedPtr sub_vn_ins_;
  rclcpp::Subscription<vectornav_msgs::msg::GpsGroup>::SharedPtr sub_vn_gps2_;

  /// Default orientation Covariance
  const std::vector<double> orientation_covariance_ = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                       0.0000, 0.0000, 0.0000, 0.0000};

  /// Default angular_velocity Covariance
  const std::vector<double> angular_velocity_covariance_ = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                            0.0000, 0.0000, 0.0000, 0.0000};

  /// Default linear_acceleration Covariance
  const std::vector<double> linear_acceleration_covariance_ = {
    0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};

  /// Default magnetic field Covariance
  const std::vector<double> magnetic_field_covariance_ = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                          0.0000, 0.0000, 0.0000, 0.0000};


  //Parameters
  const std::vector<double> global_ref_ins_poslla_ = {25.65014586802158, -100.28985364572286}; // Coordenadas entre Biblio y CETEC
  const std::vector<double> global_ref_ins_posecef_ = {-1027768.8799482058, -5661145.344370203, 2744403.2051628013};

  std::string odom_src_ = "vn";

  /// TODO(Dereck): Find default covariance values

  //Vars to store data from the INS Common groups
  uint8_t gps_fix_ = vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX;
  geometry_msgs::msg::Vector3 ins_velbody_;
  geometry_msgs::msg::Point ins_posecef_;
  geometry_msgs::msg::Point ins_poslla_;
  geometry_msgs::msg::Vector3 ins_velned_;

  //Path message to publish
  nav_msgs::msg::Path ned_path;
  nav_msgs::msg::Path enu_path;


  bool hasInitialized = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vnGPSPose>());
  rclcpp::shutdown();
  return 0;
}