#include <rclcpp/rclcpp.hpp>

// ROS2
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

class FranetFrame : public rclcpp::Node {
private:
    FrenetUtils frenet_utils;
    double laneWidth_ = 4;
    double d0 = 0;
    double dv0 = 0;
    double da0 = 0;
    double s0 = 0;
    double sv0 = 10 / 3.6;

    double x = 0;
    double y = 0;

    /* data */
    vector<int8_t> obstacle_bool_array_;
    vector<std::vector<geometry_msgs::msg::Point>> hull_vector;
    vector<Eigen::VectorXd> waypoints; // [x, y, z]


    std::vector<std::vector<double>> obstacles;

    // Callbacks
    void arrayObstacleCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
    void obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg);
    void waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

    void publishLaneMarkers(const std::vector<std::vector<double>>& centerLane);
    void publishFrenetPaths(const std::vector<FrenetPath>& paths);
    void publishOptimalFrenetPath(const FrenetPath& optimalPath);
    void publishObstacles(const std::vector<std::vector<double>>& obstacles);

    // Subscriber and publisher
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr collision_subscriber_;
    rclcpp::Subscription<lidar_msgs::msg::ObstacleData>::SharedPtr obstacle_data_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frenet_paths_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimal_frenet_path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;

public:
    FranetFrame(/* args */);
    ~FranetFrame();
};

FranetFrame::FranetFrame(/* args */) : Node("frenet_frame_node") {
    // Subscriber and publisher
    collision_subscriber_ = this->create_subscription<std_msgs::msg::Int8MultiArray>("/collision_data", 10, std::bind(&FranetFrame::arrayObstacleCallback, this, std::placeholders::_1));
    obstacle_data_sub_ = this->create_subscription<lidar_msgs::msg::ObstacleData>("/obstacle_data", 10, std::bind(&FranetFrame::obstacleDataCallback, this, std::placeholders::_1));
    waypoints_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("waypoints_loaded", 10, std::bind(&FranetFrame::waypoints_callback, this, std::placeholders::_1));

    lane_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lane_markers", 10);
    frenet_paths_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frenet_paths", 10);
    optimal_frenet_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("optimal_frenet_path", 10);
    obstacles_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles_frenet", 10);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> frenet_frame_node initialized.\033[0m");
}

FranetFrame::~FranetFrame() {}

void FranetFrame::arrayObstacleCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
    // Convert the message to a vector
    obstacle_bool_array_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "\033[1;35m----> obstacle array data received.\033[0m");
}

void FranetFrame::obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg) {
    hull_vector.clear();
    for (const auto& point_array : msg->cluster_points) {
        std::vector<geometry_msgs::msg::Point> cluster;
        for (const auto& point : point_array.points) {
            cluster.push_back(point);
        }
        hull_vector.push_back(cluster);
    }
}

void FranetFrame::waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    waypoints.clear();
    for (const auto& marker : msg->markers) {
        Eigen::VectorXd waypoint(3);
        waypoint(0) = marker.pose.position.x;
        waypoint(1) = marker.pose.position.y;
        waypoint(2) = marker.pose.position.z;
        waypoints.push_back(waypoint);
    }

    RCLCPP_INFO(this->get_logger(), "\033[1;36m----> waypoints array data received. Size: %zu\033[0m", waypoints.size());
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
        double yaw = 0.0;       // Straight line has zero yaw
        distanceTraced += std::sqrt(std::pow(x - prevX, 2) + std::pow(y - prevY, 2));
        centerLane.push_back({x, y, yaw, curvature, distanceTraced});
        prevX = x;
        prevY = y;
    }

    // Convert hull_vector to obstacles based on obstacle_bool_array_
    std::vector<std::vector<double>> obstacles;
    for (size_t i = 0; i < hull_vector.size(); ++i) {
        if (obstacle_bool_array_[i]) {
            for (const auto& point : hull_vector[i]) {
                obstacles.push_back({point.x, point.y});
                RCLCPP_INFO(this->get_logger(), "Obstacle at (%.6f, %.6f)", point.x, point.y);
            }
        }
    }

    // std::vector<std::vector<double>> obstacles = {{10, 0,}, {60, 7.93}, {25.57, 0}, {94.42,-4}};

    RCLCPP_INFO(this->get_logger(), "\033[1;36m----> Obstacles generated. Size: %zu\033[0m", obstacles.size());

    // Convert waypoints to Frenet frame and perform obstacle avoidance
    std::vector<FrenetPath> allPaths;

    FrenetPath optimalPath = frenet_utils.optimalTrajectory(d0, dv0, da0, s0, sv0, centerLane, obstacles, allPaths);

    if (optimalPath.d.size() > 1 && optimalPath.s.size() > 1 && optimalPath.world.size() > 1) {
        RCLCPP_INFO(this->get_logger(), "\033[1;36m----> optimal trajectory calculated.\033[0m");
        // d0 = optimalPath.d[1][0];
        // dv0 = optimalPath.d[1][1];
        // da0 = optimalPath.d[1][2];
        // s0 = optimalPath.s[1][0];
        // sv0 = optimalPath.s[1][1];

        // x = optimalPath.world[1][0];
        // y = optimalPath.world[1][1];
        // Publish optimal path
        publishOptimalFrenetPath(optimalPath);

    } else {
        RCLCPP_ERROR(this->get_logger(), "optimalPath does not have enough points.");
    }

    // Publish lane markers
    publishLaneMarkers(centerLane);
    publishFrenetPaths(allPaths);
    publishObstacles(obstacles);
}

void FranetFrame::publishLaneMarkers(const std::vector<std::vector<double>>& centerLane) {
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

    for (size_t i = 0; i < centerLane.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = centerLane[i][0];
        p.y = centerLane[i][1];
        p.z = 0.0;
        marker.points.push_back(p);
    }

    marker_array.markers.push_back(marker);
    lane_publisher_->publish(marker_array);
}

void FranetFrame::publishFrenetPaths(const std::vector<FrenetPath>& paths) {
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto& path : paths) {
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

        for (const auto& point : path.world) {
            geometry_msgs::msg::Point p;
            p.x = point[0];
            p.y = point[1];
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker_array.markers.push_back(marker);
    }

    frenet_paths_publisher_->publish(marker_array);
}

void FranetFrame::publishOptimalFrenetPath(const FrenetPath& optimalPath) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "base_footprint";
    marker.header.stamp = this->now();
    marker.ns = "optimal_frenet_path";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.3;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (size_t i = 1; i < optimalPath.world.size(); ++i) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = optimalPath.world[i - 1][0];
        p1.y = optimalPath.world[i - 1][1];
        p1.z = 0.0;
        p2.x = optimalPath.world[i][0];
        p2.y = optimalPath.world[i][1];
        p2.z = 0.0;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    marker_array.markers.push_back(marker);
    optimal_frenet_path_publisher_->publish(marker_array);
}

void FranetFrame::publishObstacles(const std::vector<std::vector<double>>& obstacles) {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = this->now();
    marker.ns = "obstacles";
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (const auto& obs : obstacles) {
        geometry_msgs::msg::Point p;
        p.x = obs[0];
        p.y = obs[1];
        p.z = 0.0;
        marker.points.push_back(p);
    }

    marker_array.markers.push_back(marker);
    obstacles_publisher_->publish(marker_array);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FranetFrame>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
