
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
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

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

#include "CubicSpline1D.h"


// nav_msgs/OccupancyGrid

class optimalPlanner : public rclcpp::Node
{
private:
    vector<std::vector<geometry_msgs::msg::Point>> hull_vector;

    vector<std::vector<geometry_msgs::msg::Point>> hull_vector_inflate;

    vector<std::vector<geometry_msgs::msg::Point>> prev_hull_vector;




    
    static Eigen::Vector3d bbox_size();

    double yaw_car = 0.0;
    double turning_radius = 3.0;
    double num_points = 13;
    double track_car = 1;

    Eigen::MatrixXd car_steering_zone; 

    /* data */
    void obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg);
    void publishOccupancyGrid();


    vector<std::pair<double, double>> calculate_trajectory(double yaw, double turning_radius, int num_points);
    void yawCarCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void line_steering_wheels_calculation();

    // Subscriber & Publisher
    rclcpp::Subscription<lidar_msgs::msg::ObstacleData>::SharedPtr obstacle_data_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lane_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_car_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lane_steering_publisher_;


    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hull_publisher_inflated_;


    

public:
    optimalPlanner(/* args */);
    ~optimalPlanner();
};

optimalPlanner::optimalPlanner(/* args */): Node("optimal_planner_node")
{

    obstacle_data_sub_ = this->create_subscription<lidar_msgs::msg::ObstacleData>("/obstacle_data",10,std::bind(&optimalPlanner::obstacleDataCallback, this, std::placeholders::_1));

    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid", 10);

    yaw_car_sub_ = this->create_subscription<std_msgs::msg::Float64>("/yaw_car", 10, std::bind(&optimalPlanner::yawCarCallback, this, std::placeholders::_1));

    lane_publisher_ = this->create_publisher<nav_msgs::msg::Path>("lane", 10);

    lane_steering_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("steering_lane", 10);

    // hull_publisher_inflated_ = this->create_publisher<visualization_msgs::msg::Marker>("hull_inflated", 10);


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

    // for (size_t i = 0; i < hull_vector.size(); i++)
    // {
    //     std::vector<geometry_msgs::msg::Point> hull_points;

    //     for (size_t j = 0; j < hull_vector[i].size(); j++)
    //     {
    //         RCLCPP_INFO(this->get_logger(), "Hull Vector [%d]: %f, %f", i, hull_vector[i][j].x, hull_vector[i][j].y);

    //         geometry_msgs::msg::Point p;
    //         p.x = hull_vector[i][j].x;
    //         p.y = hull_vector[i][j].y;
    //         p.z = 0.0;
    //         hull_points.push_back(p);
    //     }

    //     hull_points.push_back(hull_points.front()); 

    //     // Publish the inflated hull
    //     visualization_msgs::msg::Marker hull_marker;
    //     hull_marker.header.frame_id = "base_footprint";
    //     hull_marker.ns = "hull_inflated";
    //     hull_marker.id = i + 2000;
    //     hull_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    //     hull_marker.action = visualization_msgs::msg::Marker::ADD;
    //     hull_marker.scale.x = 0.15;  
    //     hull_marker.color.a = 1.0; 
    //     hull_marker.color.r = 0.0;  
    //     hull_marker.color.g = 1.0;  
    //     hull_marker.color.b = 0.0;  
    //     hull_marker.points = hull_points;

    //     hull_publisher_inflated_->publish(hull_marker);
    // }

    publishOccupancyGrid();

    

    auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now() - init_time) .count();

      RCLCPP_INFO(this->get_logger(),"\033[1;31m----> Planar Segmentation callback finished in %ld ms. \033[0m", execution_time);
    
}


void optimalPlanner::publishOccupancyGrid()
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "base_link";  // Or whatever frame is appropriate
    grid.info.resolution = 0.1;  // in meters
    grid.info.width = 115;  // grid width
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
    for (const auto& cluster : hull_vector) 
    {

        for (size_t i = 0; i < cluster.size(); ++i) 
        {
            auto& current_point = cluster[i];
            auto& next_point = cluster[(i + 1) % cluster.size()];

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


    line_steering_wheels_calculation();


}


vector<pair<double, double>> optimalPlanner::calculate_trajectory(double steering_angle, double wheelbase, int num_points)
{
    vector<pair<double, double>> path;

    if (fabs(steering_angle) < 1e-6) {
        double step_length = 0.5; 
        for (int i = 0; i < num_points; ++i) {
            path.push_back(make_pair(i * step_length, 0.0));
        }
    } else {
        double radius = wheelbase / tan(fabs(steering_angle));
        double circumference = 2 * M_PI * radius; 
        double arc_length = circumference / 4; 
        double angular_step = M_PI / 2 / num_points; 

        double theta = 0; // Starting angle
        for (int i = 0; i <= num_points; ++i) {
            
            double x = radius * sin(theta);
            double y = radius * (1 - cos(theta));

            if (steering_angle < 0) {
                y = -y; 
            }

            path.push_back(make_pair(x, y));
            theta += angular_step;
        }
    }

    return path;
}


void optimalPlanner::line_steering_wheels_calculation(){


    // Calculate the trajectory of the car in base of the yaw
    vector<pair<double, double>> trajectory = calculate_trajectory(yaw_car, turning_radius, num_points);

    std::vector<double> t_values(trajectory.size());
    std::iota(t_values.begin(), t_values.end(), 0);  // fills t_values with increasing values starting from  0

    std::vector<double> x;
    std::vector<double> y;

    for (const auto& point : trajectory)
    {
        x.push_back(point.first);
        y.push_back(point.second);
    }


    CubicSpline1D spline_x(t_values, x);
    CubicSpline1D spline_y(t_values, y);

    // Interpolate waypoints using the t_values range
    std::vector<double> x_new, y_new;
    for (double t = 0; t < t_values.size() - 1; t += 0.1) {
        x_new.push_back(spline_x.calc_der0(t));
        y_new.push_back(spline_y.calc_der0(t));
    }

    // Publish the lane
    nav_msgs::msg::Path path;
    path.header.frame_id = "base_footprint";
    path.header.stamp = this->now();

    for (size_t i = 0; i < x_new.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = x_new[i];
        pose.pose.position.y = y_new[i];
        path.poses.push_back(pose);
    }

    lane_publisher_->publish(path);
    
    // Prepare Marker for visualization
    visualization_msgs::msg::Marker lane_maker;
    lane_maker.header.frame_id = "base_footprint";
    lane_maker.header.stamp = this->now();

    lane_maker.ns = "vehicle_path";
    lane_maker.id = 0;
    lane_maker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lane_maker.action = visualization_msgs::msg::Marker::ADD;
    lane_maker.scale.x = 0.1;
    lane_maker.scale.y = 0.5;
    lane_maker.scale.z = 0.5; 
    lane_maker.color.a = 1.0;  
    lane_maker.color.r = 0.0;  
    lane_maker.color.g = 0.35;  
    lane_maker.color.b = 0.12;  

    car_steering_zone.resize(x_new.size() * 2, 2);

    size_t matrix_index = 0;

    // Publish lane with the track of the car 
    for (size_t i = 0; i < x_new.size(); ++i) 
    {
        double angle = atan2(y_new[i] - y_new[std::max(int(i) - 1, 0)], x_new[i] - x_new[std::max(int(i) - 1, 0)]);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        geometry_msgs::msg::Point left_point;

        left_point.x = x_new[i] + offset_x;
        left_point.y = y_new[i] + offset_y;

        car_steering_zone(matrix_index, 0) = left_point.x;
        car_steering_zone(matrix_index++, 1) = left_point.y;

        lane_maker.points.push_back(left_point);

    }

    //  invertion of the right side to math with the left side  
    for (size_t i = x_new.size(); i-- > 0; ) 
    {
        int previous_index = (i > 0) ? (i - 1) : 0;
        double angle = atan2(y_new[i] - y_new[previous_index], x_new[i] - x_new[previous_index]);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        geometry_msgs::msg::Point right_point;

        right_point.x = x_new[i] - offset_x;
        right_point.y = y_new[i] - offset_y;

        car_steering_zone(matrix_index, 0) = right_point.x;
        car_steering_zone(matrix_index++, 1) = right_point.y;

        lane_maker.points.push_back(right_point);
    }

    lane_maker.points.push_back(lane_maker.points.front()); // Close the loop

    lane_steering_publisher_->publish(lane_maker);

}


void optimalPlanner::yawCarCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Yaw Car: %f", msg->data);
    yaw_car = msg->data;
}

Eigen::Vector3d optimalPlanner::bbox_size()
{
    // distance between front and rear axles, distance from C to front/rear axle
return Eigen::Vector3d{1.8, 2.0, 2.0};
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<optimalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}