#include <rclcpp/rclcpp.hpp>

// Ros2
#include <std_msgs/msg/int8_multi_array.hpp>
#include <lidar_msgs/msg/obstacle_data.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frenet_utils.hpp"

// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>

// Eigen
#include <Eigen/Dense>

using namespace std;


class FranetFrame : public rclcpp::Node
{
private:

    FrenetUtils frenet_utils;
    double laneWidth_ = 4;
	double d0 = 0; 
	double dv0 = 0;
	double da0 = 0;
	double s0 = 0;
	double sv0 = 10/3.6;

	double x = 0;
	double y = 0;


    /* data */
    vector<int8_t> obstacle_bool_array_;
    vector<std::vector<geometry_msgs::msg::Point>> hull_vector;
    vector<Eigen::VectorXd> waypoints; // [x, y, z]

    // Callbacks
    void arrayObstacleCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
    void obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg);
    void waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

    void publishLaneMarkers(const std::vector<std::vector<double>>& centerLane);
    void publishFrenetPaths(const std::vector<FrenetPath>& paths);


    // Subcriber and publisher 
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr collision_subscriber_;
    rclcpp::Subscription<lidar_msgs::msg::ObstacleData>::SharedPtr obstacle_data_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_sub_;


    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frenet_paths_publisher_;

public:
    FranetFrame(/* args */);
    ~FranetFrame();
};

FranetFrame::FranetFrame(/* args */) : Node("frenet_frame_node")
{

    // Subcriber and publisher

    collision_subscriber_ = this->create_subscription<std_msgs::msg::Int8MultiArray>("/collision_data", 10, std::bind(&FranetFrame::arrayObstacleCallback, this, std::placeholders::_1));
    obstacle_data_sub_ = this->create_subscription<lidar_msgs::msg::ObstacleData>("/obstacle_data",10,std::bind(&FranetFrame::obstacleDataCallback, this, std::placeholders::_1));
    waypoints_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>( "waypoints_loaded", 10, std::bind(&FranetFrame::waypoints_callback, this, std::placeholders::_1));
    
    lane_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lane_markers", 10);
    frenet_paths_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frenet_paths", 10);


    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> lidar_ground_getter_node initialized.\033[0m");
}

FranetFrame::~FranetFrame()
{
}

void FranetFrame::arrayObstacleCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg)
{
    // Convert the message to a vector
    obstacle_bool_array_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "\033[1;35m----> obstacle array data recived.\033[0m");

}

void FranetFrame::obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg)
{
    auto init_time = std::chrono::system_clock::now();

    hull_vector.clear();

    for (const auto& point_array : msg->cluster_points)
    {
        std::vector<geometry_msgs::msg::Point> cluster;
        for (const auto& point : point_array.points)
        {
            cluster.push_back(point);
        }
        hull_vector.push_back(cluster);
    }

    auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now() - init_time) .count();
    RCLCPP_INFO(this->get_logger(),"\033[1;31m----> Planar Segmentation callback finished in %ld ms. \033[0m", execution_time);

}

void FranetFrame::waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    waypoints.clear();

    for (const auto& marker : msg->markers) {
        Eigen::VectorXd waypoint(3);
        waypoint(0) = marker.pose.position.x;
        waypoint(1) = marker.pose.position.y;
        waypoint(2) = marker.pose.position.z;
        waypoints.push_back(waypoint);

        // double yaw = std::sqrt(std::pow(marker.pose.orientation.x, 2) + std::pow(marker.pose.orientation.y, 2) + std::pow(marker.pose.orientation.z, 2));
    }

    RCLCPP_INFO(this->get_logger(), "\033[1;36m----> waypoints array data received.\033[0m");

    RCLCPP_INFO(this->get_logger(), "\033[1;36m----> obstacle_bool_array_ data received. Size: %zu\033[0m", obstacle_bool_array_.size());

    RCLCPP_INFO(this->get_logger(), "\033[1;36m----> Hull vector array data received. Size: %zu\033[0m", hull_vector.size());


    // Ensure the obstacle_bool_array_ and hull_vector have the same size
    if (obstacle_bool_array_.size() != hull_vector.size()) {
        RCLCPP_ERROR(this->get_logger(), "Size mismatch: obstacle_bool_array_ size: %zu, hull_vector size: %zu", obstacle_bool_array_.size(), hull_vector.size());
        return;
    }

	// Center Lane
    std::vector<std::vector<double>> centerLane;
    double y = 0.0; // Constant y for a horizontal line
    double distanceTraced = 0.0;
    double prevX = 0.0;
    double prevY = y;
    for (double x = 0; x < 140; x += 0.01) {
        double curvature = 0.0; // Straight line has zero curvature
        double yaw = 0.0; // Straight line has zero yaw
        distanceTraced += std::sqrt(std::pow(x - prevX, 2) + std::pow(y - prevY, 2));
        centerLane.push_back({x, y, yaw, curvature, distanceTraced});
        prevX = x;
        prevY = y;
    }
    

    // Convert hull_vector to obstacles
    std::vector<std::vector<double>> obstacles = {{10.58, 12,}, {60, 7.93}, {25.57, 0}, {94.42,-4}};

    // Convert hull_vector to obstacles based on obstacle_bool_array_
    // std::vector<std::vector<double>> obstacles;
    // for (size_t i = 0; i < hull_vector.size(); ++i) {
    //     if (obstacle_bool_array_[i]) {
    //         for (const auto& point : hull_vector[i]) {
    //             obstacles.push_back({point.x, point.y});
    //         }
    //     }
    // }





    // Convert waypoints to Frenet frame and perform obstacle avoidance
    std::vector<FrenetPath> allPaths;


    FrenetPath optimalPath = frenet_utils.optimalTrajectory(d0, dv0, da0, s0, sv0, centerLane, obstacles, allPaths);

    // d0 = optimalPath.d[1][0];
    // dv0 = optimalPath.d[1][1];
    // da0 = optimalPath.d[1][2];
    // s0 = optimalPath.s[1][0];
    // sv0 = optimalPath.s[1][1];

	// x = optimalPath.world[1][0];
	// y = optimalPath.world[1][1];

    RCLCPP_INFO(this->get_logger(), "\033[1;36m----> optimal trajectory calculated.\033[0m");

    // Publish lane markers
    publishLaneMarkers(centerLane);
    publishFrenetPaths(allPaths);


}


void FranetFrame::publishLaneMarkers(const std::vector<std::vector<double>>& centerLane)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "base_footprint";
    marker.header.stamp = this->now();
    marker.ns = "lanes";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (size_t i = 0; i < centerLane.size(); ++i)
    {
        geometry_msgs::msg::Point p;
        p.x = centerLane[i][0];
        p.y = centerLane[i][1];
        p.z = 0.0;
        marker.points.push_back(p);
    }

    marker_array.markers.push_back(marker);
    lane_publisher_->publish(marker_array);

    RCLCPP_INFO(this->get_logger(), "\033[1;36m----> lane markers published.\033[0m");
}

void FranetFrame::publishFrenetPaths(const std::vector<FrenetPath>& paths)
{
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto& path : paths)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_footprint";
        marker.header.stamp = this->now();
        marker.ns = "frenet_paths";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.5;

        for (const auto& point : path.world)
        {
            geometry_msgs::msg::Point p;
            p.x = point[0];
            p.y = point[1];
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker_array.markers.push_back(marker);
    }

    frenet_paths_publisher_->publish(marker_array);

    RCLCPP_INFO(this->get_logger(), "\033[1;36m----> frenet paths published.\033[0m");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FranetFrame>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}