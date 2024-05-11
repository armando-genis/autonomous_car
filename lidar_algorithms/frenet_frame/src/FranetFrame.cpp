#include <rclcpp/rclcpp.hpp>

// Ros2
#include <std_msgs/msg/int8_multi_array.hpp>
#include <lidar_msgs/msg/obstacle_data.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


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
    /* data */
    vector<int8_t> obstacle_bool_array_;
    vector<std::vector<geometry_msgs::msg::Point>> hull_vector;
    vector<Eigen::VectorXd> waypoints; // [x, y, z]

    // Callbacks
    void arrayObstacleCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
    void obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg);
    void waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

    // Subcriber and publisher 
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr collision_subscriber_;
    rclcpp::Subscription<lidar_msgs::msg::ObstacleData>::SharedPtr obstacle_data_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_sub_;

public:
    FranetFrame(/* args */);
    ~FranetFrame();
};

FranetFrame::FranetFrame(/* args */) : Node("frenet_frame_node ")
{

    // Subcriber and publisher

    collision_subscriber_ = this->create_subscription<std_msgs::msg::Int8MultiArray>("/collision_data", 10, std::bind(&FranetFrame::arrayObstacleCallback, this, std::placeholders::_1));
    obstacle_data_sub_ = this->create_subscription<lidar_msgs::msg::ObstacleData>("/obstacle_data",10,std::bind(&FranetFrame::obstacleDataCallback, this, std::placeholders::_1));
    waypoints_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>( "waypoints_loaded", 10, std::bind(&FranetFrame::waypoints_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> lidar_ground_getter_node initialized.\033[0m");
}

FranetFrame::~FranetFrame()
{
}

void FranetFrame::arrayObstacleCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg)
{
    // Convert the message to a vector
    obstacle_bool_array_ = msg->data;
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
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FranetFrame>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}