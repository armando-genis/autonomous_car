#include <rclcpp/rclcpp.hpp>

// ROS
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp> 
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>


// C++
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>

using namespace std;

#include "CubicSpline1D.h"


class WaypointsCalculations : public rclcpp::Node
{
private:
    // Variables from parameters
    double lookahead_min = 0.0; // [m]
    double lookahead_max = 0.0; // [m]
    double mps_alpha = 0.0; // [m/s]
    double mps_beta = 0.0; // [m/s]
    // Variables
    double current_yaw_ = 0.0; // [rad]
    double current_x_ = 0.0; // [m]
    double current_y_ = 0.0; // [m]
    double current_velocity_ = 0; // [m/s]
    // Waypoints variables
    int closest_waypoint = 0;
    double setVelocity = 0.0;
    float average_distance = 0.0, maximum_distance = 0.0;
    bool waypoints_in_loop = false;
    int average_distance_count = 0;
    size_t target_waypoint = 0;

    vector<Eigen::VectorXd> waypoints; // [x, y, z, yaw]
    vector<float> velocities;
    // Cube lines
    vector<Eigen::VectorXd> waypointsCubeLines;

    // Variables of the Stanley Control
    double L = 0.6; // [m] Wheel base of vehicle
    double error_front_axle = 0.0; // [m]
    double K = 0.6; // Gain of the Stanley Control
    double stering_angle = 0.0; // [rad]
    double target_idx = 0.0;
    bool isCubicSplineCalculated = false;

    // Function of the Stanley Control
    void computeCrossTrackError();
    void computeSteeringAngle();
    double GetNormaliceAngle(double angle);
    void publishWaypointsCubeLinesMarker();

    // Computations functions
    void pub_callback();
    double LookAheadDistance();
    void waypointsComputation();
    double getDistance(Eigen::VectorXd point1, Eigen::VectorXd point2);
    void setVelocityToSend();
    double getDistanceFromOdom(Eigen::VectorXd wapointPoint);
    void publishTargetWaypointPose();
    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
    
    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void velocities_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void marketTargetWaypoint();
    // Subcribers, timers & publishers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocities_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_waypoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;  // Updated to Float32
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_waypoint_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoints_cube_lines_pub_;



public:
    WaypointsCalculations(/* args */);
    ~WaypointsCalculations();
};

WaypointsCalculations::WaypointsCalculations(/* args */) : Node("waypoints_calculations")
{

    this->declare_parameter("lookahead_min", double(0.0));
    this->declare_parameter("lookahead_max", double(0.0));
    this->declare_parameter("mps_alpha", double(0.0));
    this->declare_parameter("mps_beta", double(0.0));

    this->get_parameter("lookahead_min", lookahead_min);
    this->get_parameter("lookahead_max", lookahead_max);
    this->get_parameter("mps_alpha", mps_alpha);
    this->get_parameter("mps_beta", mps_beta);

    target_waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_waypoint_marker", 10);

    speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("speedtarget", 10);  // Updated to Float32

    target_waypoint_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("target_waypoint_pose", 10);

    waypoints_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "waypoints_loaded", 10,
        std::bind(&WaypointsCalculations::waypoints_callback, this, std::placeholders::_1)
    );

    velocities_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "waypointarray_speeds", 10,
        std::bind(&WaypointsCalculations::velocities_callback, this, std::placeholders::_1)
    );

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom_lidar", 10, std::bind(&WaypointsCalculations::odom_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&WaypointsCalculations::pub_callback, this));

    waypoints_cube_lines_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoints_cube_lines_marker", 10);




    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> waypoints_calculations initialized.\033[0m");


}

WaypointsCalculations::~WaypointsCalculations()
{

}

// TIMER CALLBACK
void WaypointsCalculations::pub_callback()
{
    waypointsComputation();
    // setVelocityToSend();
    // marketTargetWaypoint();
    // publishTargetWaypointPose();
    // computeCrossTrackError();
    // computeSteeringAngle();
    // publishWaypointsCubeLinesMarker();
    // auto msg = geometry_msgs::msg::Twist();

    // Set the linear and angular velocities
    // msg.linear.x = setVelocity;  
    // msg.angular.z = stering_angle;  
    // print path_velocity_ and stering_angle
    // RCLCPP_INFO(this->get_logger(), "path_velocity_: %f", path_velocity_);
    // RCLCPP_INFO(this->get_logger(), "stering_angle: %f", stering_angle);
    // print eror_front_axle
    // RCLCPP_INFO(this->get_logger(), "error_front_axle: %f", error_front_axle);
    // Publish the message
    // cmd_vel_pub_->publish(msg);
    // print the size of the cubelines vector
    // RCLCPP_INFO(this->get_logger(), "waypointsCubeLines.size(): %d", waypointsCubeLines.size());
    
    // print the size of the waypoints cube lines indec_targ
    // RCLCPP_INFO(this->get_logger(), "target_idx: %f", target_idx);
    
}

// ODOM CALLBACK    
void WaypointsCalculations::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_velocity_ = msg->twist.twist.linear.x;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
}

void WaypointsCalculations::velocities_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    velocities.clear();
    velocities.insert(velocities.end(), msg->data.begin(), msg->data.end());
}

void WaypointsCalculations::waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    waypoints.clear();

    for (const auto& marker : msg->markers) {
        Eigen::VectorXd waypoint(4);
        waypoint(0) = marker.pose.position.x;
        waypoint(1) = marker.pose.position.y;
        waypoint(2) = marker.pose.position.z;

        tf2::Quaternion q(
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w);

        // Extract yaw from the quaternion
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        waypoint(3) = yaw;

        waypoints.push_back(waypoint);
    }

    // Only perform cubic spline calculations if they haven't been done before
    if (!isCubicSplineCalculated) {
        vector<double> x, y;
        for (const auto& waypoint : waypoints) {
            x.push_back(waypoint[0]);
            y.push_back(waypoint[1]);
        }

        std::vector<double> t_values(x.size());
        std::iota(t_values.begin(), t_values.end(), 0);  
        CubicSpline1D spline_x(t_values, x);
        CubicSpline1D spline_y(t_values, y);

        for (double t = 0; t < t_values.back(); t += 0.25) {
            double x_val = spline_x.calc_der0(t);
            double y_val = spline_y.calc_der0(t);

            Eigen::VectorXd point(2);
            point << x_val, y_val;
            waypointsCubeLines.push_back(point);
        }
        isCubicSplineCalculated = true;  // Set the flag to true so calculations are not repeated
    }
}

void WaypointsCalculations::setVelocityToSend(){
    if(velocities.empty()) return;
    if(closest_waypoint == 0){
        setVelocity = velocities[1];
    }else{
        setVelocity = velocities[closest_waypoint+1];
    }
    std_msgs::msg::Float32 msg;  // Updated to Float32
    msg.data = setVelocity;
    speed_pub_->publish(msg);

}

double WaypointsCalculations::getDistance(Eigen::VectorXd point1, Eigen::VectorXd point2){
    double distance = sqrt(pow(point1(0) - point2(0), 2) + pow(point1(1) - point2(1), 2));
    return distance;
}

double WaypointsCalculations::LookAheadDistance(){
    double clamped_speed = std::clamp(current_velocity_, mps_alpha, mps_beta);
    return (lookahead_max - lookahead_min) / (mps_beta - mps_alpha) * (clamped_speed - mps_alpha) + lookahead_min;
}

double WaypointsCalculations::getDistanceFromOdom(Eigen::VectorXd wapointPoint){
    double x1 = wapointPoint(0);
    double y1 = wapointPoint(1);
    double x2 = current_x_;
    double y2 = current_y_;
    double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    return distance;
}

void WaypointsCalculations::waypointsComputation(){
    if (waypoints.empty()) return;
    constexpr size_t first_wp = 0;                        
    const size_t last_wp = waypoints.size() - 1; 
    constexpr double max_distance_threshold = 10.0;
    constexpr int search_offset_back = 5;
    constexpr int search_offset_forward = 15;

    // loop closure is true, when the first and last waypoint are closer than 4.0 meters
    double distance = getDistance(waypoints[first_wp], waypoints[last_wp]);
    waypoints_in_loop = distance < 4.0;

    // print waypoints_in_loop
    // RCLCPP_INFO(this->get_logger(), "waypoints_in_loop: %d", waypoints_in_loop);

    int search_start = std::max(static_cast<int>(closest_waypoint) - search_offset_back, static_cast<int>(first_wp));
    int search_end = std::min(static_cast<int>(closest_waypoint) + search_offset_forward, static_cast<int>(last_wp));

    double smallest_curr_distance = std::numeric_limits<double>::max();
    for (int i = search_start; i <= search_end; i++)
    {
        double curr_distance = getDistanceFromOdom(waypoints[i]);
        if (smallest_curr_distance > curr_distance)
        {
            closest_waypoint = i;
            smallest_curr_distance = curr_distance;
        }
    }

    if (average_distance_count == 0) {
        average_distance = smallest_curr_distance;
    } else {
        average_distance = (average_distance * average_distance_count + smallest_curr_distance) / (average_distance_count + 1);
    }
    average_distance_count++;

    // Updating maximum_distance
    if (maximum_distance < smallest_curr_distance && smallest_curr_distance < max_distance_threshold) {
        maximum_distance = smallest_curr_distance;
    }

    double lookahead_actual = LookAheadDistance(); 

    for (int i = closest_waypoint; i <= static_cast<int>(last_wp); i++)
    {
        double curr_distance = getDistanceFromOdom(waypoints[i]);
        if (curr_distance > lookahead_actual && curr_distance < lookahead_actual + 8.0 )
        {
            target_waypoint = i;            
            break;
        }
        // set if is loop true and the current waypoint is the last waypoint set the target waypoint to the first waypoint
        if (waypoints_in_loop && i == static_cast<int>(last_wp)) {
            target_waypoint = first_wp;
        }else{
            target_waypoint = last_wp;
            // finishedpath = true;
        }
        // target_waypoint = last_wp;
    }
}

void WaypointsCalculations::marketTargetWaypoint(){
    // Publish target waypoint marker

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";  // Adjust this to your frame
    marker.header.stamp = this->now();
    marker.ns = "target_waypoint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = waypoints[target_waypoint](0);
    marker.pose.position.y = waypoints[target_waypoint](1);
    marker.pose.position.z = waypoints[target_waypoint](2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    target_waypoint_pub_->publish(marker);
}

void WaypointsCalculations::publishTargetWaypointPose()
{
    geometry_msgs::msg::Pose msg;
    msg.position.x = waypoints[target_waypoint](0);
    msg.position.y = waypoints[target_waypoint](1);
    msg.position.z = waypoints[target_waypoint](2);

    // Assuming you have a function to convert yaw to a quaternion
    geometry_msgs::msg::Quaternion q = yawToQuaternion(waypoints[target_waypoint](3));
    msg.orientation = q;

    target_waypoint_pose_pub_->publish(msg);
}

geometry_msgs::msg::Quaternion WaypointsCalculations::yawToQuaternion(double yaw)
{

    tf2::Quaternion quaternion;
    
    quaternion.setRPY(0, 0, yaw);

    geometry_msgs::msg::Quaternion quaternion_msg;
    quaternion_msg.x = quaternion.x();
    quaternion_msg.y = quaternion.y();
    quaternion_msg.z = quaternion.z();
    quaternion_msg.w = quaternion.w();

    return quaternion_msg;
}

double WaypointsCalculations::GetNormaliceAngle(double angle){
    if (angle > M_PI) angle -= 2 * M_PI;

    if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

// Stanely Controller
void WaypointsCalculations::computeCrossTrackError() {
    // Normalize the current yaw angle
    double current_yaw_stanley = GetNormaliceAngle(current_yaw_);
    // Compute the front axle's position
    double fx = current_x_ + L * cos(current_yaw_stanley);
    double fy = current_y_ + L * sin(current_yaw_stanley);

    // Initialize the minimum distance and the target index
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    // Iterate through the waypoints to find the closest waypoint to the front axle's position
    for (size_t i = 0; i < waypointsCubeLines.size(); i++) {
        double dx = fx - waypointsCubeLines[i](0);  // Access x-coordinate of Eigen::VectorXd
        double dy = fy - waypointsCubeLines[i](1);  // Access y-coordinate of Eigen::VectorXd
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
            RCLCPP_INFO(this->get_logger(), "i: %zu, dist: %f, min_dist: %f, closest_idx: %zu", i, dist, min_dist, closest_idx);
    }
    // Store the closest index as the target index
    target_idx = closest_idx;

    // Calculate the cross track error
    double front_axle_vec[2] = {-std::cos(current_yaw_stanley + M_PI / 2), -std::sin(current_yaw_stanley + M_PI / 2)};
    error_front_axle = (fx - waypointsCubeLines[target_idx](0)) * front_axle_vec[0] + (fy - waypointsCubeLines[target_idx](1)) * front_axle_vec[1];
}


void WaypointsCalculations::computeSteeringAngle(){
    

    double yaw_goal = waypoints[target_waypoint](3);

    
    double current_yaw_car = GetNormaliceAngle(current_yaw_);

    double yaw_path = GetNormaliceAngle(yaw_goal);

    double theta_e = GetNormaliceAngle(yaw_path - current_yaw_car);
    double theta_d = atan2(K * error_front_axle, current_velocity_);
    theta_d = GetNormaliceAngle(theta_d);

    double delta = theta_e + theta_d ;
    stering_angle = GetNormaliceAngle(delta);

    // print stering_angle
    // RCLCPP_INFO(this->get_logger(), "stering_angle: %f", stering_angle);
    // print ther error_front_axle
    // RCLCPP_INFO(this->get_logger(), "error_front_axle: %f", error_front_axle);
    // print the theta_e
    // RCLCPP_INFO(this->get_logger(), "theta_e: %f", theta_e);
}

void WaypointsCalculations::publishWaypointsCubeLinesMarker() {
    // if (target_idx >= waypointsCubeLines.size()) {
    //     RCLCPP_ERROR(this->get_logger(), "target_idx out of range");
    //     return;
    // }

    Eigen::VectorXd waypoint = waypointsCubeLines[target_idx];

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";  
    marker.header.stamp = this->now();
    marker.ns = "waypoints_cube_lines";
    marker.id = 10;  // Use target_idx as the marker ID
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = waypoint(0);
    marker.pose.position.y = waypoint(1);
    
    // Assuming waypoint(2) is the z-coordinate
    marker.pose.position.z = 0.0;  // Adjust if your waypoint has a z-coordinate
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.7;
    marker.scale.y = 0.7;
    marker.scale.z = 0.7;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    waypoints_cube_lines_pub_->publish(marker);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointsCalculations>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
