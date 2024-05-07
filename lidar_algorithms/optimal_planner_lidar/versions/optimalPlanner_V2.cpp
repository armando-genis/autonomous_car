
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
#include "sat_collision_checker.h"

// nav_msgs/OccupancyGrid

class optimalPlanner : public rclcpp::Node
{
private:
    vector<std::vector<geometry_msgs::msg::Point>> hull_vector;

    vector<std::vector<geometry_msgs::msg::Point>> hull_vector_inflate;

    vector<std::vector<geometry_msgs::msg::Point>> prev_hull_vector;

    fop::SATCollisionChecker collision_checker; 
    geometry_msgs::msg::Polygon vehicle_path;

    bool collition = false;


    
    static Eigen::Vector3d bbox_size();

    double yaw_car = 0.0;
    double turning_radius = 3.0;
    double num_points = 15;
    double track_car = 1;

    Eigen::MatrixXd car_steering_zone; 

    /* data */
    void obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg);
    void publishOccupancyGrid();

    void extract_segment_cubic_lines(const std::vector<double>& x, const std::vector<double>& y, std::vector<double>& segment_x, std::vector<double>& segment_y, double length);

    vector<std::pair<double, double>> calculate_trajectory(double yaw, double turning_radius, int num_points);
    void yawCarCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void line_steering_wheels_calculation();

    void publishMarker();

    // Subscriber & Publisher
    rclcpp::Subscription<lidar_msgs::msg::ObstacleData>::SharedPtr obstacle_data_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lane_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_car_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lane_steering_publisher_;


    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hull_publisher_inflated_;

    rclcpp::TimerBase::SharedPtr timer_;



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

    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);


    hull_publisher_inflated_ = this->create_publisher<visualization_msgs::msg::Marker>("hull_inflated", 10);

    timer_ = this->create_wall_timer( std::chrono::milliseconds(500), std::bind(&optimalPlanner::publishMarker, this));


    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> optimal_planner_node initialized.\033[0m");

}

optimalPlanner::~optimalPlanner()
{

}

void optimalPlanner::publishMarker()
{
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "warning_obstacle";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header.
        marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side.
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        publisher_->publish(marker);

}

void optimalPlanner::obstacleDataCallback(const lidar_msgs::msg::ObstacleData::SharedPtr msg)
{
    auto init_time = std::chrono::system_clock::now();


    hull_vector.clear();

    for (const auto& point_array : msg->cluster_points)
    {

        geometry_msgs::msg::Polygon obstacle_poly;
        std::vector<geometry_msgs::msg::Point> cluster;
        
        for (const auto& point : point_array.points)
        {
            cluster.push_back(point);

            geometry_msgs::msg::Point32 p;
            p.x = point.x;
            p.y = point.y;
            obstacle_poly.points.push_back(p);
        }
        hull_vector.push_back(cluster);

        obstacle_poly.points.push_back(obstacle_poly.points.front()); // Close the loop

        collition = collision_checker.check_collision(vehicle_path, obstacle_poly);

        // print the colliton 
    
        if (collition) {
            RCLCPP_WARN(this->get_logger(), "Collision detected with an obstacle!");
        }

    }


    publishOccupancyGrid();

    auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now() - init_time) .count();

      RCLCPP_INFO(this->get_logger(),"\033[1;31m----> Planar Segmentation callback finished in %ld ms. \033[0m", execution_time);
    
}


void optimalPlanner::publishOccupancyGrid()
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "warning_obstacle"; 
    grid.info.resolution = 0.1;  
    grid.info.width = 115; 
    grid.info.height = 100;  
    grid.info.origin.position.x = -5.0;  
    grid.info.origin.position.y = -5.0;  
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



void optimalPlanner::extract_segment_cubic_lines(const std::vector<double>& x, const std::vector<double>& y, std::vector<double>& segment_x, std::vector<double>& segment_y, double length) {
    if (x.empty() || y.empty()) return;  // Early exit if input is empty

    // Initialize with the first point
    segment_x.push_back(x.front());
    segment_y.push_back(y.front());
    
    double accumulated_length = 0.0;
    double previous_x = x.front();
    double previous_y = y.front();

    for (size_t i = 1; i < x.size() && accumulated_length < length; ++i) {
        double current_x = x[i];
        double current_y = y[i];

        // Calculate the distance from the previous point to the current point
        double dx = current_x - previous_x;
        double dy = current_y - previous_y;
        double segment_length = sqrt(dx * dx + dy * dy);

        // Update the accumulated length
        accumulated_length += segment_length;

        if (accumulated_length >= length) {
            // Calculate how much to scale back the last segment to fit exactly 'length' meters
            double excess_length = accumulated_length - length;
            double ratio = segment_length > 0 ? (segment_length - excess_length) / segment_length : 0;
            current_x = previous_x + ratio * dx;
            current_y = previous_y + ratio * dy;

            segment_x.push_back(current_x);
            segment_y.push_back(current_y);
            break;
        } else {
            // Add the current point and update the previous points
            segment_x.push_back(current_x);
            segment_y.push_back(current_y);
            previous_x = current_x;
            previous_y = current_y;
        }
    }
}



void optimalPlanner::line_steering_wheels_calculation(){

    vehicle_path.points.clear();


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
    std::vector<double> x_new_entire, y_new_entire;

    for (double t = 0; t < t_values.size() - 1; t += 0.25) {
        x_new.push_back(spline_x.calc_der0(t));
        y_new.push_back(spline_y.calc_der0(t));
    }


    // RCLCPP_INFO(this->get_logger(), "Size of x_new: %d", x_new.size());
    // RCLCPP_INFO(this->get_logger(), "Size of y_new: %d", y_new.size());

    std::vector<double> segment_x, segment_y;
    extract_segment_cubic_lines(x_new, y_new, segment_x, segment_y, 2.5);

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
    lane_maker.scale.x = 0.02;
    lane_maker.scale.y = 0.5;
    lane_maker.scale.z = 0.5; 
    lane_maker.color.a = 1.0;  
    lane_maker.color.r = 1.0;  
    lane_maker.color.g = 1.0;  
    lane_maker.color.b = 0.12;  

    car_steering_zone.resize(segment_x.size() * 2, 2);

    size_t matrix_index = 0;

    // Publish lane with the track of the car 
    for (size_t i = 0; i < segment_x.size(); ++i) 
    {
        double angle = atan2(segment_y[i] - segment_y[std::max(int(i) - 1, 0)], segment_x[i] - segment_x[std::max(int(i) - 1, 0)]);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        geometry_msgs::msg::Point left_point;

        left_point.x = segment_x[i] + offset_x;
        left_point.y = segment_y[i] + offset_y;

        car_steering_zone(matrix_index, 0) = left_point.x;
        car_steering_zone(matrix_index++, 1) = left_point.y;

        lane_maker.points.push_back(left_point);

        geometry_msgs::msg::Point32 converted_point;
        converted_point.x = left_point.x;
        converted_point.y = left_point.y;
        converted_point.z = 0.0;
        vehicle_path.points.push_back(converted_point);

    }

    //  invertion of the right side to math with the left side  
    for (size_t i = segment_x.size(); i-- > 0; ) 
    {
        int previous_index = (i > 0) ? (i - 1) : 0;
        double angle = atan2(segment_y[i] - segment_y[previous_index], segment_x[i] - segment_x[previous_index]);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        geometry_msgs::msg::Point right_point;

        right_point.x = segment_x[i] - offset_x;
        right_point.y = segment_y[i] - offset_y;

        car_steering_zone(matrix_index, 0) = right_point.x;
        car_steering_zone(matrix_index++, 1) = right_point.y;

        lane_maker.points.push_back(right_point);

        geometry_msgs::msg::Point32 converted_point;
        converted_point.x = right_point.x;
        converted_point.y = right_point.y;
        converted_point.z = 0.0;
        vehicle_path.points.push_back(converted_point);
    }

    lane_maker.points.push_back(lane_maker.points.front()); // Close the loop

    vehicle_path.points.push_back(vehicle_path.points.front());

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