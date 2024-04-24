
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
#include <nav_msgs/msg/occupancy_grid.hpp>

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
#include <cmath> 

// Eigen
#include <Eigen/Dense>

using namespace std;

// nav_msgs/OccupancyGrid

class optimalPlanner : public rclcpp::Node
{
private:
    vector<std::vector<geometry_msgs::msg::Point>> hull_vector;

    /* data */
    void obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg);
    void publishOccupancyGrid();
    // Subscriber & Publisher
    rclcpp::Subscription<lidar_msgs::msg::ObstacleData>::SharedPtr obstacle_data_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;


public:
    optimalPlanner(/* args */);
    ~optimalPlanner();
};

optimalPlanner::optimalPlanner(/* args */): Node("optimal_planner_node")
{

    obstacle_data_sub_ = this->create_subscription<lidar_msgs::msg::ObstacleData>("/obstacle_data",10,std::bind(&optimalPlanner::obstacleDataCallback, this, std::placeholders::_1));
    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid", 10);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> optimal_planner_node initialized.\033[0m");


}

optimalPlanner::~optimalPlanner()
{

}

void optimalPlanner::obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg)
{
    auto init_time = std::chrono::system_clock::now();
    // RCLCPP_INFO(this->get_logger(), "\033[1;31m----> obstacleDataCallback.\033[0m");

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

    for (size_t i = 0; i < hull_vector.size(); i++)
    {
        for (size_t j = 0; j < hull_vector[i].size(); j++)
        {
            RCLCPP_INFO(this->get_logger(), "Hull Vector [%d]: %f, %f", i, hull_vector[i][j].x, hull_vector[i][j].y);
        }
    }

    publishOccupancyGrid();

    auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now() - init_time) .count();

      RCLCPP_INFO(this->get_logger(),"\033[1;31m----> Planar Segmentation callback finished in %ld ms. \033[0m", execution_time);
    
    
}


void optimalPlanner::publishOccupancyGrid()
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "base_link";  // Or whatever frame is appropriate
    grid.info.resolution = 0.1;  // in meters
    grid.info.width = 100;  // grid width
    grid.info.height = 100;  // grid height
    grid.info.origin.position.x = -5.0;  // Center the origin of the grid
    grid.info.origin.position.y = -5.0;  // Center the origin of the grid
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    // Initialize grid data
    grid.data.resize(grid.info.width * grid.info.height, 0);  // Initialize all cells as free

    auto mark_grid = [&](int x, int y, int value) {
        if (x >= 0 && x < static_cast<int>(grid.info.width) && y >= 0 && y < static_cast<int>(grid.info.height)) {
            grid.data[y * grid.info.width + x] = value;  // Mark the cell
        }
    };

    // Inflate cells around the given point
    auto inflate_point = [&](int x, int y, int radius) {
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                if (dx * dx + dy * dy <= radius * radius) {  // Circle equation
                    mark_grid(x + dx, y + dy, 100);
                }
            }
        }
    };

    // Simple line drawing between two points with inflation
    auto draw_inflated_line = [&](int x0, int y0, int x1, int y1, int radius) {
        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int n = 1 + dx + dy;
        int x_inc = (x1 > x0) ? 1 : -1;
        int y_inc = (y1 > y0) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; --n)
        {
            inflate_point(x0, y0, radius);

            if (error > 0)
            {
                x0 += x_inc;
                error -= dy;
            }
            else
            {
                y0 += y_inc;
                error += dx;
            }
        }
    };

    // Convert each point in hull_vector to grid coordinates, draw lines and inflate
    int inflation_radius = 3; // Inflated radius in cells (30 cm)
    for (const auto& cluster : hull_vector) {
        for (size_t i = 0; i < cluster.size(); ++i) {
            auto& current_point = cluster[i];
            auto& next_point = cluster[(i + 1) % cluster.size()]; // Wrap around to create a closed loop

            // Convert point coordinates to grid indices
            int x0 = static_cast<int>((current_point.x - grid.info.origin.position.x) / grid.info.resolution);
            int y0 = static_cast<int>((current_point.y - grid.info.origin.position.y) / grid.info.resolution);
            int x1 = static_cast<int>((next_point.x - grid.info.origin.position.x) / grid.info.resolution);
            int y1 = static_cast<int>((next_point.y - grid.info.origin.position.y) / grid.info.resolution);

            // Draw inflated line between points
            draw_inflated_line(x0, y0, x1, y1, inflation_radius);
        }
    }

    // Publish the occupancy grid
    occupancy_grid_pub_->publish(grid);
}



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<optimalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}