#include <rclcpp/rclcpp.hpp>

// ROS2
#include <std_msgs/msg/int8_multi_array.hpp>
#include <lidar_msgs/msg/obstacle_data.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>

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
    void publishFrenetPaths(const std::vector<FrenetPath>& paths, bool hasOptimalPath);
    void publishOptimalFrenetPath(const FrenetPath& optimalPath, bool hasOptimalPath);
    void publishObstacles(const std::vector<std::vector<double>>& obstacles);

    // Subscriber and publisher
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr collision_subscriber_;
    rclcpp::Subscription<lidar_msgs::msg::ObstacleData>::SharedPtr obstacle_data_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frenet_paths_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_frenet_path_publisher_;
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

    optimal_frenet_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("optimal_frenet_path", 10); 
    
    obstacles_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles_frenet", 10);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> frenet_frame_node initialized.\033[0m");
}

FranetFrame::~FranetFrame() {}

void FranetFrame::arrayObstacleCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
    // Convert the message to a vector
    obstacle_bool_array_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "\033[1;35m----> obstacle array data received.\033[0m");
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

    // RCLCPP_INFO(this->get_logger(), "\033[1;36m----> waypoints array data received. Size: %zu\033[0m", waypoints.size());
    // RCLCPP_INFO(this->get_logger(), "\033[1;36m----> obstacle_bool_array_ data received. Size: %zu\033[0m", obstacle_bool_array_.size());
    // RCLCPP_INFO(this->get_logger(), "\033[1;36m----> Hull vector array data received. Size: %zu\033[0m", hull_vector.size());

    // Ensure the obstacle_bool_array_ and hull_vector have the same size
    if (obstacle_bool_array_.size() != hull_vector.size()) {
        // RCLCPP_ERROR(this->get_logger(), "Size mismatch: obstacle_bool_array_ size: %zu, hull_vector size: %zu", obstacle_bool_array_.size(), hull_vector.size());
        return;
    }

    // Center Lane
    // std::vector<std::vector<double>> centerLane;
    // double y = 0.0; // Constant y for a horizontal line
    // double distanceTraced = 0.0;
    // double prevX = 0.0;
    // double prevY = y;
    // for (double x = 0; x < 140; x += 0.01) {
    //     double curvature = 0.0; // Straight line has zero curvature
    //     double yaw = 0.0;       // Straight line has zero yaw
    //     distanceTraced += std::sqrt(std::pow(x - prevX, 2) + std::pow(y - prevY, 2));
    //     centerLane.push_back({x, y, yaw, curvature, distanceTraced});
    //     prevX = x;
    //     prevY = y;
    // }


    std::vector<std::vector<double>> centerLane;
    if (waypoints.size() > 1) {
        double totalDistanceTraced = 0.0; // Initialize total distance traced

        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            Eigen::VectorXd start = waypoints[i];
            Eigen::VectorXd end = waypoints[i + 1];

            double segmentLength = (end - start).norm();
            double steps = segmentLength / 0.05; // Adjust the step size if necessary
            double yaw = std::atan2(end(1) - start(1), end(0) - start(0));

            Eigen::VectorXd prevPoint = start;
            for (int step = 0; step <= steps; ++step) {
                double t = step / steps;
                Eigen::VectorXd currentPoint = start + t * (end - start);
                double distanceTraced = (currentPoint - prevPoint).norm();
                totalDistanceTraced += distanceTraced;

                std::vector<double> lanePoint = {
                    currentPoint(0), // x
                    currentPoint(1), // y
                    yaw,             // yaw
                    0.0,             // curvature (approximated as 0 for straight segments)
                    totalDistanceTraced // accumulated distance
                };
                centerLane.push_back(lanePoint);
                prevPoint = currentPoint;
            }
        }
    }

    // Convert hull_vector to obstacles based on obstacle_bool_array_
    std::vector<std::vector<double>> obstacles;
    for (size_t i = 0; i < hull_vector.size(); ++i) {
        if (obstacle_bool_array_[i]) {
            for (const auto& point : hull_vector[i]) {
                obstacles.push_back({point.x, point.y});
                // RCLCPP_INFO(this->get_logger(), "Obstacle at (%.6f, %.6f)", point.x, point.y);
            }
        }
    }

    // RCLCPP_INFO(this->get_logger(), "\033[1;36m----> Obstacles generated. Size: %zu\033[0m", obstacles.size());

    // Convert waypoints to Frenet frame and perform obstacle avoidance
    std::vector<FrenetPath> allPaths;

    FrenetPath optimalPath = frenet_utils.optimalTrajectory(d0, dv0, da0, s0, sv0, centerLane, obstacles, allPaths);
    bool hasOptimalPath = optimalPath.d.size() > 1 && optimalPath.s.size() > 1 && optimalPath.world.size() > 1;

    if (hasOptimalPath) {
        RCLCPP_INFO(this->get_logger(), "\033[1;36m----> optimal trajectory calculated.\033[0m");

    } else {
        RCLCPP_ERROR(this->get_logger(), "optimalPath does not have enough points.");
    }

    // Publish lane markers
    publishLaneMarkers(centerLane);
    publishOptimalFrenetPath(optimalPath, hasOptimalPath);
    publishFrenetPaths(allPaths, hasOptimalPath); // Pass hasOptimalPath to the function
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

void FranetFrame::publishFrenetPaths(const std::vector<FrenetPath>& paths, bool hasOptimalPath) {
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

        size_t max_points = hasOptimalPath ? path.world.size() : std::min(path.world.size(), static_cast<size_t>(path.world.size() / 2));

        for (size_t i = 0; i < max_points; ++i) {
            geometry_msgs::msg::Point p;
            p.x = path.world[i][0];
            p.y = path.world[i][1];
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker_array.markers.push_back(marker);
    }

    frenet_paths_publisher_->publish(marker_array);
}


void FranetFrame::publishOptimalFrenetPath(const FrenetPath& optimalPath, bool hasOptimalPath) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "base_footprint";
    path_msg.header.stamp = this->now();


    size_t max_points = hasOptimalPath ? optimalPath.world.size() : std::min(optimalPath.world.size(), static_cast<size_t>(optimalPath.world.size() / 2));

    for (size_t i = 0; i < max_points; ++i) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "base_footprint";
        // pose_stamped.header.stamp = this->now();
        pose_stamped.pose.position.x = optimalPath.world[i][0];
        pose_stamped.pose.position.y = optimalPath.world[i][1];
        
        // pose_stamped.pose.position.z = 0.0;

        // pose_stamped.pose.orientation.w = 1.0;
        // pose_stamped.pose.orientation.x = 0.0;
        // pose_stamped.pose.orientation.y = 0.0;
        // pose_stamped.pose.orientation.z = 0.0;

        path_msg.poses.push_back(pose_stamped);
    }

    optimal_frenet_path_publisher_->publish(path_msg);
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
