/** ----------------------------------------------------------------------------
 * @file: sbg_processing.cc
 * @date: August 31, 2023
 * @author: Rogelio Salais
 * @author: Sebas Mtz
 *
 * @brief: Processing of the SBG Messages is made in this node.
 *         This node publishes:
 *          - POSE
 *          - A TF Broadcaster
 *        The information published depends on the frame used (ENU, NED) from
 *        the configuration of the vehicle.
 *        This can be improved to receive from a parameter the desired output frame,
 *        and the parent and child frame ID's, so they are not hardcoded.
 * -----------------------------------------------------------------------------
 **/

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "sensor_msgs/msg/imu.hpp"

// #include "sensor_msgs/msg/fluid_pressure.hpp"
// #include "sensor_msgs/msg/magnetic_field.hpp"
// #include "sensor_msgs/msg/nav_sat_fix.hpp"
// #include "sensor_msgs/msg/temperature.hpp"
// #include "sensor_msgs/msg/time_reference.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include <eigen3/Eigen/Dense>

//ROS2 node class for odometry, NED pose, and velodyne transform - 
class sbgGPSPose : public rclcpp::Node
{
  
public:
  sbgGPSPose() : Node("sbg_gps_pose_node")
  {
    //Parameter
    declare_parameter<std::vector<double>>("orientation_covariance", orientation_covariance_);
    declare_parameter<std::vector<double>>("angular_velocity_covariance", angular_velocity_covariance_);
    declare_parameter<std::vector<double>>("linear_acceleration_covariance", linear_acceleration_covariance_);
    declare_parameter<std::vector<double>>("magnetic_covariance", magnetic_field_covariance_);

    declare_parameter<std::vector<double>>("global_lla_reference", global_ref_ins_poslla_);
    declare_parameter<std::vector<double>>("global_ecef_reference", global_ref_ins_posecef_);

    // Publishers
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("sdv_localization/sbg/odom", 10);

    // pub_pose =  this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("sdv_localization/sbg/pose", 10);
    
    pub_path =  this->create_publisher<nav_msgs::msg::Path>("sdv_localization/nav/sbg/path", 10);
    
    // Subscribers
    // auto imu_sub_cb_ = std::bind(&sbgGPSPose::sub_sbg_imu, this, std::placeholders::_1);
    // // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("sbg/imu/data", 10, imu_sub_cb_);
    
    auto sub_sbg_ecef_cb = std::bind(&sbgGPSPose::sub_sbg_ecef, this, std::placeholders::_1);
    sub_sbg_ecef_ = this->create_subscription<geometry_msgs::msg::PointStamped>("sbg/pos_ecef", 10, sub_sbg_ecef_cb);
    
    auto sub_sbg_odom_cb = std::bind(&sbgGPSPose::sub_sbg_odom, this, std::placeholders::_1);
    sub_sbg_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("sbg/odom", 10, sub_sbg_odom_cb);

  }

private:
  // void sub_sbg_imu(const sensor::msg::Imu::SharedPtr msg)
  // {

  // }


  void sub_sbg_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    /* ODOMETRY MSGS */
    nav_msgs::msg::Odometry odom_msg;

    // ENU
    odom_msg.header = enu_pose_msg.header;
    odom_msg.child_frame_id = msg->child_frame_id;

    odom_msg.pose.pose.position = enu_pose_msg.pose.pose.position;

    // SBG publishes orientation from 0 to 2*PI, with zero with respect to East
    tf2::Quaternion quaternion( msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z, 
                                msg->pose.pose.orientation.w);
    tf2::Matrix3x3 mat(quaternion);

    double roll, pitch, yaw;

    mat.getRPY(roll, pitch, yaw);
    yaw = -yaw;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
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

    // std::cout << "SBG" << std::endl;
    // std::cout << "yaw = " << yaw * 180 / M_PI << " pitch = " << pitch * 180 / M_PI << " roll = " << roll * 180 / M_PI<< std::endl;

    // odom_msg.pose.pose.orientation = msg->pose.pose.orientation;
    odom_msg.pose.pose.orientation = q;

    Eigen::Matrix3f R = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f T = Eigen::Matrix3f::Zero();
    Eigen::MatrixXf J = Eigen::MatrixXf::Zero(6,6);

    Eigen::VectorXf eta_dot = Eigen::MatrixXf::Zero(6,1);
    Eigen::VectorXf vel_body = Eigen::MatrixXf::Zero(6,1);

    eta_dot << msg->twist.twist.linear.x,
               msg->twist.twist.linear.y,
               msg->twist.twist.linear.z,
               msg->twist.twist.angular.x,
               msg->twist.twist.angular.y,
               msg->twist.twist.angular.z;

    R <<    std::cos(yaw)*std::cos(pitch),      -std::sin(yaw)*std::cos(roll) + std::cos(yaw)*std::sin(pitch)*std::sin(roll),     std::sin(yaw)*std::sin(roll) + std::cos(yaw)*std::cos(roll)*std::sin(pitch),
            std::sin(yaw)*std::cos(pitch),       std::cos(yaw)*std::cos(roll) + std::sin(roll)*std::sin(pitch)*std::sin(yaw),    -std::cos(yaw)*std::sin(roll) + std::sin(pitch)*std::sin(yaw)*std::cos(roll),
            -std::sin(pitch),                    std::cos(pitch)*std::sin(roll),                                                  std::cos(pitch)*std::cos(roll);

    T <<   1,     std::sin(roll)*std::tan(pitch),  std::cos(roll)*std::tan(pitch),
            0,     std::cos(roll),                  -std::sin(roll),
            0,     std::sin(roll)/std::cos(pitch),  std::cos(roll)/std::cos(pitch);

    J << R,                             Eigen::Matrix3f::Zero(3, 3),
          Eigen::Matrix3f::Zero(3, 3),  T;

    vel_body = J.inverse()*eta_dot;

    odom_msg.twist.twist.linear.x  = -vel_body(0); // for some reason this must be negated to be similar to the vectornav
    odom_msg.twist.twist.linear.y  = -vel_body(1);
    odom_msg.twist.twist.linear.z  = -vel_body(2);
    odom_msg.twist.twist.angular.x = vel_body(3);
    odom_msg.twist.twist.angular.y = -vel_body(4);
    odom_msg.twist.twist.angular.z = -vel_body(5);
    
    // Publish Odometry in ENU
    pub_odom_->publish(odom_msg);

    // geometry_msgs::msg::TransformStamped tf;

    // tf.header.frame_id = msg->header.frame_id;
    // tf.header.set__stamp(msg->header.stamp);
    // tf.child_frame_id = msg->child_frame_id;
    // tf.transform.translation.x = msg->pose.pose.position.x;
    // tf.transform.translation.y = msg->pose.pose.position.y;
    // tf.transform.translation.z = msg->pose.pose.position.z;
    // tf.transform.rotation = msg->pose.pose.orientation;
    // odom_tf_broadcaster_->sendTransform(tf);
  }

  void sub_sbg_ecef(const geometry_msgs::msg::PointStamped::SharedPtr msg_in)
  {
    std::array<double, 3> global_ref_ins_poslla_;

    std::array<double, 3> global_ref_ins_posecef_;

    fill_insreference_from_param("global_lla_reference", global_ref_ins_poslla_);
    fill_insreference_from_param("global_ecef_reference", global_ref_ins_posecef_);
    
    //RCLCPP_INFO(get_logger(), "Frame ID: '%s'", msg_in->header.frame_id.c_str());
    // Time Reference (Startup)


    // if(hasInitialized){

        std::array<double, 3> currECEF;;
        currECEF[0] = msg_in->point.x;
        currECEF[1] = msg_in->point.y;
        currECEF[2] = msg_in->point.z;


        globalNED = calculateNED(global_ref_ins_posecef_, global_ref_ins_poslla_, currECEF);

        //We changed to TF2 standard messages for easier matrix rotation and subtraction

        /* POSE MSGS */
        // NED
        ned_pose_msg.header.frame_id = "odom";
        ned_pose_msg.header.set__stamp(msg_in->header.stamp);

        ned_pose_msg.pose.pose.position.x = globalNED[0];
        ned_pose_msg.pose.pose.position.y = globalNED[1]; //To define 
        ned_pose_msg.pose.pose.position.z = 0;

        // ned_pose_msg.pose.pose.orientation = msg_in->quaternion;

        // ENU
        enu_pose_msg.header.frame_id = "odom";
        enu_pose_msg.header.set__stamp(msg_in->header.stamp);
        
        enu_pose_msg.pose.pose.position.x = globalNED[1]; //Change between 
        enu_pose_msg.pose.pose.position.y = globalNED[0]; 
        enu_pose_msg.pose.pose.position.z = 0;

        // tf2::Quaternion quaternionResult;
        // tf2::convert(msg_in->quaternion, quaternionResult);
        // quaternionResult.setZ(-quaternionResult.getZ());
        // quaternionResult.normalize();

        //Removal of pitch and roll angles, since we're only interested in working in a x,y plane 
        //tf2::Quaternion quaternionTransformNed;

        //quaternionTransformNed.setRPY(0, 0, msg_in->yawpitchroll.x);
        //ned_pose_msg.pose.pose.orientation = tf2::toMsg(quaternionTransformNed);

        // tf2::Quaternion quaternionTransformEnu;
        //quaternionTransformEnu.setRPY(0, 0, -90);
        //tf2::convert(msg_in->quaternion, quaternionResult);
        //quaternionResult = quaternionResult * quaternionTransformEnu;
        //quaternionResult.setZ(-quaternionResult.getZ());
        //quaternionResult.normalize();
        
        // enu_pose_msg.pose.pose.orientation = tf2::toMsg(quaternionResult);

        // // pub_pose->publish(ned_pose_msg);
        // // pub_pose->publish(enu_pose_msg);

        /* PATH MSGS */
        geometry_msgs::msg::PoseStamped pathToAdd;

        // NED
        pathToAdd.pose = ned_pose_msg.pose.pose;
        pathToAdd.header = ned_pose_msg.header;
        ned_path.header = ned_pose_msg.header;
        ned_path.poses.push_back(pathToAdd);
        // pub_path->publish(ned_path);

        // ENU
        pathToAdd.pose = enu_pose_msg.pose.pose;
        pathToAdd.header = enu_pose_msg.header;
        enu_path.header = enu_pose_msg.header;
        enu_path.poses.push_back(pathToAdd);
        pub_path->publish(enu_path);
        
    // }
    // else
    // {
    //     RCLCPP_INFO(get_logger(), "Reference can't be calculated, location data is 0");
    // } 

  }

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
  // rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;


  /// Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_sbg_ecef_;
  // rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_sbg_odom_;


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

  // Parameter declaration
  
  

  //Vars to store data from the INS Common groups
  // uint8_t gps_fix_ = vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX;
  geometry_msgs::msg::Vector3 ins_velbody_;
  geometry_msgs::msg::Point ins_posecef_;
  geometry_msgs::msg::Point ins_poslla_;
  geometry_msgs::msg::Vector3 ins_velned_;

  //Path message to publish
  nav_msgs::msg::Path ned_path;
  nav_msgs::msg::Path enu_path;

  //NED_POSE_Publish
  geometry_msgs::msg::PoseWithCovarianceStamped ned_pose_msg;

  //ENU_POSE_Publish
  geometry_msgs::msg::PoseWithCovarianceStamped enu_pose_msg;

  std::array<double, 3> globalNED;



  // bool hasInitialized = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sbgGPSPose>());
  rclcpp::shutdown();
  return 0;
}