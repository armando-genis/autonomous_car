// 3D LiDAR Object Detection & Tracking using Euclidean Clustering, RANSAC, & Hungarian Algorithm
// RORS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>



#include <geometry_msgs/msg/pose_stamped.hpp>


// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>

// Eigen
#include <Eigen/Dense>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


using namespace std;

#include "obstacle_detector.hpp"

struct BBox
{
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
    double r = 42.0 / 255.0;
    double g = 157.0 / 255.0;
    double b = 244.0 / 255.0;
};


struct Point {
    float x, y;
};


struct Zone {
    std::vector<Point> corners; // Four corners of the zone

    Zone(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
        corners.push_back(p1);
        corners.push_back(p2);
        corners.push_back(p3);
        corners.push_back(p4);
    }

    bool contains(const pcl::PointXYZ& point) const {
        int count = 0;
        size_t n = corners.size();

        for (size_t i = 0; i < n; i++) {
            size_t j = (i + 1) % n;
            if (isIntersecting(point, corners[i], corners[j])) {
                count++;
            }
        }

        return count % 2 == 1; // Inside if odd number of intersections
    }

private:
    bool isIntersecting(const pcl::PointXYZ& p_orig, const Point& p1, const Point& p2) const {
        Point p = {p_orig.x, p_orig.y};

        if (p.y == p1.y || p.y == p2.y) {
            p.y += 0.0001; 
        }

        if (p.y < std::min(p1.y, p2.y) || p.y > std::max(p1.y, p2.y)) {
            return false;
        }

        if (p.x > std::max(p1.x, p2.x)) {
            return true;
        }

        if (p.x < std::min(p1.x, p2.x)) {
            return false;
        }

        double slope = (p2.y - p1.y) / (p2.x - p1.x);
        double y_intercept = p1.y - slope * p1.x;
        double intersect_x = (p.y - y_intercept) / slope;

        return p.x < intersect_x;
    }
};


class ObjectDetection: public rclcpp::Node
{
private:
    // variables
    float GROUND_THRESHOLD;
    float CLUSTER_THRESH;
    int CLUSTER_MAX_SIZE;
    int CLUSTER_MIN_SIZE;
    size_t obstacle_id_;

    bool USE_PCA_BOX;
    float DISPLACEMENT_THRESH;
    float IOU_THRESH;
    bool USE_TRACKING;
    std::vector<Box> curr_boxes_; 
    std::vector<Box> prev_boxes_;

    Zone front_zone;
    Zone front_zone_warning;

    int32_t int_side_value_ = 0;
    int32_t int_warining_value = 0;
    std_msgs::msg::Int32 message_int_warining_value;

    std::shared_ptr<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>> obstacle_detector;

    void box3dcreation(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::msg::Header& header);
    void publisherboxes(std::vector<BBox>&& bboxes, const std_msgs::msg::Header& header);
    void warnning_display(const int warning_code);
    void check_zones_all_points_version2(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters);
    void publish_zone(const Zone& zone1, const Zone& zone2);
    visualization_msgs::msg::Marker create_zone_marker(const Zone& zone, int id, const std::string& color);
    void side_topic_callback(const std_msgs::msg::Int32::SharedPtr msg);



    // Point Cloud callback
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Subscriber & Publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_next;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wall_warning;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zone_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_warning_;


public:
    ObjectDetection(/* args */);
    ~ObjectDetection();
};
// lower left, upper left, upper right, lower rigth   ->   y, x
ObjectDetection::ObjectDetection(/* args */) : Node("lidar3d_clustering_node"), front_zone(Point{-1.2, 0.0}, Point{-1.2, 2.0}, Point{1.2, 2.0}, Point{1.2, 0.0}), front_zone_warning(Point{-1.2, 2.1}, Point{-1.2, 4.0}, Point{1.2, 4.0}, Point{1.2, 2.1}) 
{
    // Parameters
    this->declare_parameter("GROUND_THRESHOLD", 0.2);
    this->declare_parameter("CLUSTER_THRESH", 0.5);
    this->declare_parameter("CLUSTER_MAX_SIZE", 5000);
    this->declare_parameter("CLUSTER_MIN_SIZE", 10);
    this->declare_parameter("USE_PCA_BOX", true);
    this->declare_parameter("DISPLACEMENT_THRESH", 0.2);
    this->declare_parameter("IOU_THRESH", 0.5);
    this->declare_parameter("USE_TRACKING", true);
    

    // Get parameters
    this->get_parameter("GROUND_THRESHOLD", GROUND_THRESHOLD);
    this->get_parameter("CLUSTER_THRESH", CLUSTER_THRESH);
    this->get_parameter("CLUSTER_MAX_SIZE", CLUSTER_MAX_SIZE);
    this->get_parameter("CLUSTER_MIN_SIZE", CLUSTER_MIN_SIZE);
    this->get_parameter("USE_PCA_BOX", USE_PCA_BOX);
    this->get_parameter("DISPLACEMENT_THRESH", DISPLACEMENT_THRESH);
    this->get_parameter("IOU_THRESH", IOU_THRESH);
    this->get_parameter("USE_TRACKING", USE_TRACKING);

    
    // Create subscriber
    sub_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_ground", 10, std::bind(&ObjectDetection::pointCloudCallback, this, std::placeholders::_1)); // roi points cloud

    // Create publisher
    // ground_seg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points", 10); // ground points
    marker_pub_next = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_bounding_box", 10); // detector objects
    wall_warning = this->create_publisher<visualization_msgs::msg::Marker>("warning_visualization_tool", 10);
    // Create point processor
    obstacle_detector = std::make_shared<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>>();

    zone_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_zone", 10);
    publisher_warning_ = this->create_publisher<std_msgs::msg::Int32>("warning_status", 10);


    obstacle_id_ = 0;
    // front_zone = Zone(Point{0.0, -1.0}, Point{0.0, 1.0}, Point{2.0, 1.0}, Point{2.0, -1.0});
    // front_zone_warning = Zone(Point{2.0, -1.0}, Point{2.0, 1.0}, Point{4.0, 1.0}, Point{4.0, -1.0});

    RCLCPP_INFO(this->get_logger(), "lidar3d_Clustering_node initialized");

    // Print parameters
    // RCLCPP_INFO(this->get_logger(), "GROUND_THRESHOLD: %f", GROUND_THRESHOLD);
    // RCLCPP_INFO(this->get_logger(), "CLUSTER_THRESH: %f", CLUSTER_THRESH);
    // RCLCPP_INFO(this->get_logger(), "CLUSTER_MAX_SIZE: %d", CLUSTER_MAX_SIZE);
    // RCLCPP_INFO(this->get_logger(), "CLUSTER_MIN_SIZE: %d", CLUSTER_MIN_SIZE);
    // RCLCPP_INFO(this->get_logger(), "USE_PCA_BOX: %d", USE_PCA_BOX);
    // RCLCPP_INFO(this->get_logger(), "DISPLACEMENT_THRESH: %f", DISPLACEMENT_THRESH);
    // RCLCPP_INFO(this->get_logger(), "IOU_THRESH: %f", IOU_THRESH);
    // RCLCPP_INFO(this->get_logger(), "USE_TRACKING: %d", USE_TRACKING);

}

ObjectDetection::~ObjectDetection()
{
}

// Point Cloud callback
void ObjectDetection::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    auto init_time = std::chrono::system_clock::now();

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);

    // Check if the input cloud is empty
    if (input_cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
        return;
    }

    try {


        // RCLCPP_INFO(this->get_logger(), "Number of points in the input cloud: %zu", input_cloud->size());


        // auto segmented_clouds = obstacle_detector->segmentPlane(input_cloud, 100, GROUND_THRESHOLD);
        auto cloud_clusters = obstacle_detector->clustering(input_cloud, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);

        // Proceed with further processing only if valid data is present
        if (!cloud_clusters.empty()) {

            box3dcreation(std::move(cloud_clusters), msg->header);

            // RCLCPP_INFO(this->get_logger(), "Number of clusters: %zu", cloud_clusters.size());

            check_zones_all_points_version2(std::move(cloud_clusters));

            publish_zone(front_zone, front_zone_warning);

        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }

  auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now() - init_time)
                            .count();

  RCLCPP_INFO(this->get_logger(),
              "Planar Segmentation callback finished in %ld ms", execution_time);


}


void ObjectDetection::box3dcreation(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::msg::Header& header)
{

    std::vector<BBox> bboxes;
    
    int num_reasonable_clusters = 0;
    
    for (auto& cluster : cloud_clusters)
    {

        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D<pcl::PointXYZ>(*cluster, min_pt, max_pt);

        BBox bbox;
        bbox.x_min = min_pt[0];
        bbox.y_min = min_pt[1];
        bbox.z_min = min_pt[2];
        bbox.x_max = max_pt[0];
        bbox.y_max = max_pt[1];
        bbox.z_max = max_pt[2];

        bboxes.push_back(bbox);
        num_reasonable_clusters++;

    }

    publisherboxes(std::move(bboxes), header);

}

void ObjectDetection::publisherboxes(std::vector<BBox>&& bboxes, const std_msgs::msg::Header& header){
//==================================== Drawing Boxes  ====================================

    visualization_msgs::msg::MarkerArray marker_array;


    int id = 0;
    const std_msgs::msg::Header& inp_header = header;

    geometry_msgs::msg::Quaternion neutral_orientation;
    neutral_orientation.x = 0.0;
    neutral_orientation.y = 0.0;
    neutral_orientation.z = 0.0;
    neutral_orientation.w = 1.0; 


    // Create a marker for each bounding box


    for (const auto& bbox : bboxes)
    {
        // Create the marker for the top square
        visualization_msgs::msg::Marker top_square_marker;
        top_square_marker.header = inp_header;
        top_square_marker.ns = "bounding_boxes";
        top_square_marker.id = id++;
        top_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        top_square_marker.action = visualization_msgs::msg::Marker::ADD;
        top_square_marker.pose.orientation = neutral_orientation;
        top_square_marker.scale.x = 0.06;
        top_square_marker.color.r = bbox.r;
        top_square_marker.color.g = bbox.g;
        top_square_marker.color.b = bbox.b;
        top_square_marker.color.a = 1.0;

        // Add the points to the top square marker
        geometry_msgs::msg::Point p1, p2, p3, p4;
        p1.x = bbox.x_max; p1.y = bbox.y_max; p1.z = bbox.z_max;
        p2.x = bbox.x_min; p2.y = bbox.y_max; p2.z = bbox.z_max;
        p3.x = bbox.x_min; p3.y = bbox.y_min; p3.z = bbox.z_max;
        p4.x = bbox.x_max; p4.y = bbox.y_min; p4.z = bbox.z_max;
        top_square_marker.points.push_back(p1);
        top_square_marker.points.push_back(p2);
        top_square_marker.points.push_back(p3);
        top_square_marker.points.push_back(p4);
        top_square_marker.points.push_back(p1);

        // Add the top square marker to the array
        marker_array.markers.push_back(top_square_marker);

        // Create the marker for the bottom square
        visualization_msgs::msg::Marker bottom_square_marker;
        bottom_square_marker.header = inp_header;
        bottom_square_marker.ns = "bounding_boxes";
        bottom_square_marker.id = id++;
        bottom_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        bottom_square_marker.action = visualization_msgs::msg::Marker::ADD;
        bottom_square_marker.pose.orientation = neutral_orientation;
        bottom_square_marker.scale.x = 0.04;
        bottom_square_marker.color.r = bbox.r;
        bottom_square_marker.color.g = bbox.g;
        bottom_square_marker.color.b = bbox.b;
        bottom_square_marker.color.a = 1.0;

        // Add the points to the bottom square marker
        geometry_msgs::msg::Point p5, p6, p7, p8;
        p5.x = bbox.x_max; p5.y = bbox.y_max; p5.z = bbox.z_min;
        p6.x = bbox.x_min; p6.y = bbox.y_max; p6.z = bbox.z_min;
        p7.x = bbox.x_min; p7.y = bbox.y_min; p7.z = bbox.z_min;
        p8.x = bbox.x_max; p8.y = bbox.y_min; p8.z = bbox.z_min;

        bottom_square_marker.points.push_back(p5);
        bottom_square_marker.points.push_back(p6);
        bottom_square_marker.points.push_back(p7);
        bottom_square_marker.points.push_back(p8);
        bottom_square_marker.points.push_back(p5); // connect the last point to the first point to close the square

        // Add the bottom square marker to the marker array
        marker_array.markers.push_back(bottom_square_marker);


        // Create the marker for the lines connecting the top and bottom squares
        visualization_msgs::msg::Marker connecting_lines_marker;
        connecting_lines_marker.header = inp_header;
        connecting_lines_marker.ns = "bounding_boxes";
        connecting_lines_marker.id = id++;
        connecting_lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        connecting_lines_marker.action = visualization_msgs::msg::Marker::ADD;
        connecting_lines_marker.pose.orientation = neutral_orientation;
        connecting_lines_marker.scale.x = 0.04;
        connecting_lines_marker.color.r = bbox.r;
        connecting_lines_marker.color.g = bbox.g;
        connecting_lines_marker.color.b = bbox.b;
        connecting_lines_marker.color.a = 0.5;

        // Add the points to the connecting lines marker
        connecting_lines_marker.points.push_back(p1);
        connecting_lines_marker.points.push_back(p5);

        connecting_lines_marker.points.push_back(p2);
        connecting_lines_marker.points.push_back(p6);

        connecting_lines_marker.points.push_back(p3);
        connecting_lines_marker.points.push_back(p7);

        connecting_lines_marker.points.push_back(p4);
        connecting_lines_marker.points.push_back(p8);

        // Add the connecting lines marker to the marker array
        marker_array.markers.push_back(connecting_lines_marker);


        // Create a marker for the corners
        visualization_msgs::msg::Marker corner_marker;
        corner_marker.header = inp_header;
        corner_marker.ns = "bounding_boxes";
        corner_marker.id = id++;
        corner_marker.type = visualization_msgs::msg::Marker::SPHERE;
        corner_marker.action = visualization_msgs::msg::Marker::ADD;
        corner_marker.pose.orientation = neutral_orientation;
        corner_marker.scale.x = 0.15;
        corner_marker.scale.y = 0.15;
        corner_marker.scale.z = 0.15;
        corner_marker.color.r = bbox.r;
        corner_marker.color.g = bbox.g;
        corner_marker.color.b = bbox.b;
        corner_marker.color.a = 0.64;

        // Create a sphere for each corner and add it to the marker array

        corner_marker.pose.position = p1;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p2;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p3;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p4;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p5;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p6;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p7;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p8;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);


        rclcpp::Duration marker_lifetime = rclcpp::Duration::from_seconds(3);


        top_square_marker.lifetime = marker_lifetime;
        bottom_square_marker.lifetime = marker_lifetime;
        connecting_lines_marker.lifetime = marker_lifetime;
        corner_marker.lifetime = marker_lifetime;

        marker_pub_next->publish(marker_array);
    }



}




void ObjectDetection::warnning_display(const int warning_code)
{

    visualization_msgs::msg::Marker wall_marker;
    wall_marker.header.frame_id = "detector_wall";
    wall_marker.header.stamp = rclcpp::Clock().now();
    wall_marker.ns = "wall";
    wall_marker.id = 0;
    wall_marker.type = visualization_msgs::msg::Marker::CUBE;
    wall_marker.action = visualization_msgs::msg::Marker::ADD;
    wall_marker.pose.position.x = 0.0;
    wall_marker.pose.position.y = 0.0;
    wall_marker.pose.position.z = 0.0;
    wall_marker.pose.orientation.x = 0.0;
    wall_marker.pose.orientation.y = 0.0;
    wall_marker.pose.orientation.z = 0.0;
    wall_marker.pose.orientation.w = 1.0;
    wall_marker.scale.x = 0.25;
    wall_marker.scale.y = 0.25;
    wall_marker.scale.z = 0.25;
    wall_marker.color.a = 0.7;

    if (warning_code == 1) {
        wall_marker.color.r = 1.0;
        wall_marker.color.g = 0.0;
        wall_marker.color.b = 0.0;
    } else if (warning_code == 2) {
        wall_marker.color.r = 1.0;
        wall_marker.color.g = 1.0;
        wall_marker.color.b = 0.0;
    } else if (warning_code == 3) {
        wall_marker.color.r = 0.0;
        wall_marker.color.g = 1.0;
        wall_marker.color.b = 0.0;
    } else {
        wall_marker.color.r = 0.0;
        wall_marker.color.g = 1.0;
        wall_marker.color.b = 0.0;
    }

    wall_warning->publish(wall_marker);

}


void ObjectDetection::check_zones_all_points_version2(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters) 
{

   int highest_warning_code = 4; // To store the highest severity warning code

    for (const auto& cluster : cloud_clusters) 
    {
        if (cluster->empty()) continue;

        bool zone_checked = false; // Flag to break out of the loop once a point in a zone is found

        for (const auto& point : cluster->points) {

        
            if (front_zone.contains(point)) {
                highest_warning_code = std::min(highest_warning_code, 1); // Obstacle detected in red zone
                zone_checked = true;
                RCLCPP_INFO(this->get_logger(), "[WARNING 220] Obstacle detected in red zone.");
                break; // No need to check further points in this cluster
            } else if (front_zone_warning.contains(point)) {
                highest_warning_code = std::min(highest_warning_code, 2); // Obstacle detected in yellow zone
                zone_checked = true;
                RCLCPP_INFO(this->get_logger(), "[WARNING 330] Obstacle detected in yellow zone.");
                break; // No need to check further points in this cluster
            }


        }

        if (zone_checked) {

            continue; // Move to the next cluster
        }
    }

    if (highest_warning_code < 4) {
        warnning_display(highest_warning_code);


        message_int_warining_value.data = highest_warning_code;
        publisher_warning_->publish(message_int_warining_value);

    } else {
        RCLCPP_INFO(this->get_logger(), "[INFO 000] No obstacle detected in any zone.");
        warnning_display(4);


        message_int_warining_value.data = 4;
        publisher_warning_->publish(message_int_warining_value);
    }


}


void ObjectDetection::publish_zone(const Zone& zone1, const Zone& zone2) {
    visualization_msgs::msg::MarkerArray marker_array;

    // Create and add markers for each zone
    visualization_msgs::msg::Marker marker1 = create_zone_marker(zone1, 1000, "red");
    visualization_msgs::msg::Marker marker2 = create_zone_marker(zone2, 1001, "yellow");

    marker_array.markers.push_back(marker1);
    marker_array.markers.push_back(marker2);

    // Publish the marker array
    zone_publisher_->publish(marker_array);
}

visualization_msgs::msg::Marker ObjectDetection::create_zone_marker(const Zone& zone, int id, const std::string& color) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_footprint"; 
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "zone";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05; // Line width
    marker.color.a = 1.0; // Alpha (opacity)

    if (color == "yellow") {
        marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; // Yellow
    } else {
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // Red (default)
    }

    for (const auto& corner : zone.corners) {
        geometry_msgs::msg::Point p;
        p.x = corner.x; p.y = corner.y; p.z = 0;
        marker.points.push_back(p);
    }
    marker.points.push_back(marker.points.front()); // Close the loop

    return marker;
}


void ObjectDetection::side_topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{

    RCLCPP_INFO(this->get_logger(), "Received from /side_topic: %d", msg->data);
    int_side_value_ = msg->data;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}