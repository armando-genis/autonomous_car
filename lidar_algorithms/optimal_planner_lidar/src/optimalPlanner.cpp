
#include <rclcpp/rclcpp.hpp>

// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <lidar_msgs/msg/obstacle_data.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>

// Eigen
#include <Eigen/Dense>

using namespace std;

class optimalPlanner : public rclcpp::Node
{
private:
    /* data */
    void obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg);

    // Subscriber & Publisher
    rclcpp::Subscription<lidar_msgs::msg::ObstacleData>::SharedPtr obstacle_data_sub_;
public:
    optimalPlanner(/* args */);
    ~optimalPlanner();
};

optimalPlanner::optimalPlanner(/* args */): Node("optimal_planner_node")
{

    obstacle_data_sub_ = this->create_subscription<lidar_msgs::msg::ObstacleData>("/obstacle_data",10,std::bind(&optimalPlanner::obstacleDataCallback, this, std::placeholders::_1))
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> optimal_planner_node initialized.\033[0m");

}

optimalPlanner::~optimalPlanner()
{
}

void optimalPlanner::obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> obstacleDataCallback.\033[0m");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<optimalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}