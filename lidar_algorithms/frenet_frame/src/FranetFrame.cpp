#include <rclcpp/rclcpp.hpp>

// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


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
public:
    FranetFrame(/* args */);
    ~FranetFrame();
};

FranetFrame::FranetFrame(/* args */) : Node("franet_frame_node ")
{
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> lidar_ground_getter_node initialized.\033[0m");
}

FranetFrame::~FranetFrame()
{
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FranetFrame>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}