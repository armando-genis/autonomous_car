// 3D LiDAR Object Detection & Tracking using Euclidean Clustering, RANSAC, & Hungarian Algorithm
// RORS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>



#include <lidar_msgs/msg/obstacle_data.hpp>

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

#include "obstacle_detector_2.hpp"
#include "HungarianTracker.hpp"
#include "bbox.hpp"



class ObjectDetection: public rclcpp::Node
{
private:
    // variables
    float GROUND_THRESHOLD;
    float CLUSTER_THRESH;
    int CLUSTER_MAX_SIZE;
    int CLUSTER_MIN_SIZE;

    bool USE_PCA_BOX;
    float DISPLACEMENT_THRESH;
    float IOU_THRESH;
    bool USE_TRACKING;

    HungarianTracker tracker_;
    std::vector<BBox> prev_boxes_; // Store previous boxes





    vector<std::vector<geometry_msgs::msg::Point>> hull_vector;
    std::shared_ptr<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>> obstacle_detector;

    // functions
    void convex_hull(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters);
    BBox get_futures_clouster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);



    // Point Cloud callback
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Subscriber & Publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hull_publisher_;
    rclcpp::Publisher<lidar_msgs::msg::ObstacleData>::SharedPtr obstacle_data_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centroid_marker_publisher_;



public:
    ObjectDetection(/* args */);
    ~ObjectDetection();
};
// lower left, upper left, upper right, lower rigth   ->   y, x
ObjectDetection::ObjectDetection(/* args */) : Node("lidar3d_clustering_node"), tracker_(3.5, 3.5)
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
    sub_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ground_removal", 10, std::bind(&ObjectDetection::pointCloudCallback, this, std::placeholders::_1)); // roi points cloud

    // Create publisher

    hull_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("convex_hull_marker_array", 10);

    obstacle_data_publisher_ = this->create_publisher<lidar_msgs::msg::ObstacleData>("obstacle_data", 10);

    centroid_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("centroid_topic", 10);

    centroid_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("centroid_marker_array", 10);


    // Create point processor
    obstacle_detector = std::make_shared<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>>();



    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> lidar3d_Clustering_node initialized.\033[0m");


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

        auto cloud_clusters = obstacle_detector->clustering(input_cloud, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);
        auto& clusters = cloud_clusters.first;
        auto& centroids = cloud_clusters.second;

        // Proceed with further processing only if valid data is present
        if (!clusters.empty()) {

            convex_hull(std::move(clusters));

            // clean the tracker ========================================


            // Prepare bounding boxes and centroids for tracking
            std::vector<Eigen::Vector3f> eigen_centroids;
            std::vector<BBox> curr_boxes;
            for (const auto& centroid : centroids) {
                eigen_centroids.push_back(Eigen::Vector3f(centroid.x, centroid.y, centroid.z));
            }
            for (const auto& cluster : clusters) {
                curr_boxes.push_back(get_futures_clouster(cluster));
            }

            RCLCPP_INFO(this->get_logger(), "Tracking %zu current boxes", curr_boxes.size());


            tracker_.update(prev_boxes_, eigen_centroids, curr_boxes);


            visualization_msgs::msg::MarkerArray centroid_markers;
            int id = 0;
            for (const auto& track : tracker_.getTracks()) {

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "velodyne";
                marker.ns = "centroids";
                marker.id = id++;
                marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = track.centroid.x();
                marker.pose.position.y = track.centroid.y();
                marker.pose.position.z = track.centroid.z() + 0.5;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.z = 0.4;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                marker.text = std::to_string(track.id);
                centroid_markers.markers.push_back(marker);

                // publish as log the id 
                RCLCPP_INFO(this->get_logger(), "Track ID: %d", track.id);


            }

            // Publish the new markers
            centroid_marker_publisher_->publish(centroid_markers);

            RCLCPP_INFO(this->get_logger(), "Updating previous boxes.");
            prev_boxes_ = curr_boxes;


            // ==========================================================================

            // print the size of the trakers
            RCLCPP_INFO(this->get_logger(), "Number of tracks: %zu", tracker_.getTracks().size());
            // print the size of the centroids_markers
            RCLCPP_INFO(this->get_logger(), "Number of centroids markers: %zu", centroid_markers.markers.size());

            // RCLCPP_INFO(this->get_logger(), "Number of clusters: %zu", clusters.size());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }

  auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now() - init_time)
                            .count();

//   RCLCPP_INFO(this->get_logger(),"Planar Segmentation callback finished in %ld ms", execution_time);
}

BBox ObjectDetection::get_futures_clouster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster) {
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D<pcl::PointXYZ>(*cluster, min_pt, max_pt);

    BBox bbox;
    bbox.x_min = min_pt[0];
    bbox.y_min = min_pt[1];
    bbox.z_min = min_pt[2];
    bbox.x_max = max_pt[0];
    bbox.y_max = max_pt[1];
    bbox.z_max = max_pt[2];

    return bbox;
}



// ------------------------------- Convex Hull -------------------------------

void ObjectDetection::convex_hull(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters)
{
    if (!cloud_clusters.empty())
    {
        hull_vector.resize(cloud_clusters.size());
        visualization_msgs::msg::MarkerArray hull_markers;

        int index = 0;  // Declare an index variable
        for (auto& cluster : cloud_clusters)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ConvexHull<pcl::PointXYZ> hull;
            hull.setInputCloud(cluster);
            hull.setDimension(2);
            hull.reconstruct(*convexHull);

            if (convexHull->empty())
            {
                RCLCPP_INFO(this->get_logger(), "Convex hull is empty.");
                continue;
            }

            if (hull.getDimension() == 2)
            {
                std::vector<geometry_msgs::msg::Point> hull_points;
                for (const auto& point : convexHull->points)
                {
                    geometry_msgs::msg::Point p;
                    p.x = point.x;
                    p.y = point.y;
                    p.z = 0.0; 
                    hull_points.push_back(p);
                }

                // Close the loop
                hull_points.push_back(hull_points.front());

                hull_vector[index] = hull_points;

                // Create a marker for the convex hull
                visualization_msgs::msg::Marker hull_marker;
                hull_marker.header.frame_id = "base_footprint";
                hull_marker.ns = "hull";
                hull_marker.id = index;
                hull_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                hull_marker.action = visualization_msgs::msg::Marker::ADD;
                hull_marker.scale.x = 0.07;
                hull_marker.color.r = 1.0;
                hull_marker.color.g = 1.0;
                hull_marker.color.b = 1.0;
                hull_marker.color.a = 1.0;
                hull_marker.points = hull_points;

                hull_markers.markers.push_back(hull_marker);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "The chosen hull dimension is not correct.");
            }
            index++;
        }

        if (!hull_markers.markers.empty()) {
            hull_publisher_->publish(hull_markers);
        }



        lidar_msgs::msg::ObstacleData obstacle_msg;

        for (const auto &cluster : hull_vector)
        {   
            lidar_msgs::msg::PointArray point_array_msg;
            point_array_msg.points = cluster;
            obstacle_msg.cluster_points.push_back(point_array_msg);
        }

        // RCLCPP_INFO(this->get_logger(), "Hull size: %ld",  hull_vector.size());

        obstacle_data_publisher_->publish(obstacle_msg);


    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



