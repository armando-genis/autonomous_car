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
#include "bbox.hpp"
#include "HungarianTracker.hpp"




class ObjectDetection: public rclcpp::Node
{
private:
    // variables
    double GROUND_THRESHOLD;
    double CLUSTER_THRESH;
    int CLUSTER_MAX_SIZE;
    int CLUSTER_MIN_SIZE;
    bool USE_PCA_BOX;
    double DISPLACEMENT_THRESH;
    double IOU_THRESH;
    bool USE_TRACKING;

    std::vector<BBox> prev_boxes_; // Store previous boxes
    std::vector<Eigen::Vector3f> prev_centroids;
    std::vector<BBox> curr_boxes;


    HungarianTracker tracker;

    
    vector<std::vector<geometry_msgs::msg::Point>> hull_vector;
    std::shared_ptr<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>> obstacle_detector;

    // functions
    void convex_hull(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters);
    BBox get_futures_clouster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);
    void id_pose_publisher();



    // Point Cloud callback
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Subscriber & Publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hull_publisher_;
    rclcpp::Publisher<lidar_msgs::msg::ObstacleData>::SharedPtr obstacle_data_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr id_publishers;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_marker_publisher_;



public:
    ObjectDetection(/* args */);
    ~ObjectDetection();
};

ObjectDetection::ObjectDetection(/* args */) : Node("lidar3d_clustering_node")
{
    // Parameters
    this->declare_parameter("GROUND_THRESHOLD", 0.0);
    this->declare_parameter("CLUSTER_THRESH", 0.0);
    this->declare_parameter("CLUSTER_MAX_SIZE", 0);
    this->declare_parameter("CLUSTER_MIN_SIZE", 0);
    this->declare_parameter("USE_PCA_BOX", false);
    this->declare_parameter("DISPLACEMENT_THRESH", 0.0);
    this->declare_parameter("IOU_THRESH", 0.0);
    this->declare_parameter("USE_TRACKING", false);

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

    id_publishers = this->create_publisher<visualization_msgs::msg::MarkerArray>("/id_publishers", 10);

    bbox_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bbox_marker_array", 10);


    // Create point processor
    obstacle_detector = std::make_shared<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>>();



    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> lidar3d_Clustering_node initialized.\033[0m");


    // print paraments in blue
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> GROUND_THRESHOLD: %f\033[0m", GROUND_THRESHOLD);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> CLUSTER_THRESH: %f\033[0m", CLUSTER_THRESH);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> CLUSTER_MAX_SIZE: %d\033[0m", CLUSTER_MAX_SIZE);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> CLUSTER_MIN_SIZE: %d\033[0m", CLUSTER_MIN_SIZE);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> DISPLACEMENT_THRESH: %f\033[0m", DISPLACEMENT_THRESH);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> IOU_THRESH: %f\033[0m", IOU_THRESH);
    

    // RCLCPP_INFO(this->get_logger(), "USE_TRACKING: %d", USE_TRACKING);

}

ObjectDetection::~ObjectDetection()
{
}

// Point Cloud callback
void ObjectDetection::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
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
        auto cloud_clusters = obstacle_detector->clustering(input_cloud, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);
        auto& clusters = cloud_clusters.first;
        auto& centroids = cloud_clusters.second;

        // Proceed with further processing only if valid data is present
        if (!clusters.empty()) {
            convex_hull(std::move(clusters));

            
            for (const auto& cluster : clusters) {
                curr_boxes.emplace_back(get_futures_clouster(cluster));
            }

            try {
                tracker.obstacleTracking(prev_boxes_, curr_boxes, DISPLACEMENT_THRESH, IOU_THRESH);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in obstacleTracking: %s", e.what());
            }   

            // Publish IDs as MarkerArray
            int id_ = 0;
            visualization_msgs::msg::MarkerArray id_markers;
            for (const auto& box : curr_boxes) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "velodyne";
                marker.header.stamp = this->now();
                marker.ns = "ids";
                marker.id = 7000 + id_++;
                marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = box.position[0];
                marker.pose.position.y = box.position[1];
                marker.pose.position.z = box.position[2] + 0.7;
                marker.pose.orientation.w = 1.0;
                marker.scale.z = 0.5;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                marker.text = std::to_string(box.id);
                id_markers.markers.push_back(marker);
            }
            id_publishers->publish(id_markers);

            id_ = 0;
            // Publish BBox transformations as MarkerArray with arrows
            visualization_msgs::msg::MarkerArray bbox_markers;
            for (const auto& box : curr_boxes) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "velodyne";
                marker.header.stamp = this->now();
                marker.ns = "bbox";
                marker.id = 8000 + id_++;
                marker.type = visualization_msgs::msg::Marker::ARROW;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = box.position[0];
                marker.pose.position.y = box.position[1];
                marker.pose.position.z = box.position[2];
                marker.pose.orientation.x = box.quaternion.x();
                marker.pose.orientation.y = box.quaternion.y();
                marker.pose.orientation.z = box.quaternion.z();
                marker.pose.orientation.w = box.quaternion.w();
                marker.scale.x = 1.0;  // Arrow length
                marker.scale.y = 0.1;  // Arrow width
                marker.scale.z = 0.1;  // Arrow height
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                bbox_markers.markers.push_back(marker);
            }
            bbox_marker_publisher_->publish(bbox_markers);

            // publish size of the current boxes
            RCLCPP_INFO(this->get_logger(), "Current boxes size: %ld", curr_boxes.size());

            // Store current boxes and centroids as previous for the next iteration
            prev_boxes_.swap(curr_boxes);
            // Log the number of clusters
            RCLCPP_INFO(this->get_logger(), "\033[1;31m ----->Number of clusters: %zu\033[0m", clusters.size());
            // print the size of the curr_boxes
            RCLCPP_INFO(this->get_logger(), "\033[1;31m ----->Number of current id: %zu\033[0m", curr_boxes.size());
            curr_boxes.clear();

            
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }

    auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now() - init_time)
                            .count();
    RCLCPP_INFO(this->get_logger(), "PointCloud callback finished in %ld ms", execution_time);
}



BBox ObjectDetection::get_futures_clouster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster)
{
    // Compute the bounding box height (to be used later for recreating the box)
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    const float box_height = max_pt.z - min_pt.z;
    const float box_z = (max_pt.z + min_pt.z) / 2;

    // Compute the cluster centroid
    Eigen::Vector4f pca_centroid;
    pcl::compute3DCentroid(*cluster, pca_centroid);

    // Squash the cluster to x-y plane with z = centroid z
    for (size_t i = 0; i < cluster->size(); ++i) {
        cluster->points[i].z = pca_centroid(2);
    }

    // Compute principal directions & Transform the original cloud to PCA coordinates
    pcl::PointCloud<pcl::PointXYZ>::Ptr pca_projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *pca_projected_cloud);

    const auto eigen_vectors = pca.getEigenVectors();

    // Get the minimum and maximum points of the transformed cloud.
    pcl::getMinMax3D(*pca_projected_cloud, min_pt, max_pt);
    const Eigen::Vector3f meanDiagonal = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf quaternion(eigen_vectors); // Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f position = eigen_vectors * meanDiagonal + pca_centroid.head<3>();
    const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y), box_height);

    return BBox(-1, position, dimension, quaternion);
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



