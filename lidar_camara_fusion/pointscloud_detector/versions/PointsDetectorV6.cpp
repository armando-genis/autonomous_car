// Based in this repo https://github.com/Alpaca-zip/ultralytics_ros/tree/noetic-devel

#include <rclcpp/rclcpp.hpp>
// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// C++
#include <iostream>
#include <vector>
#include <algorithm>

// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/opencv.hpp>

// OpenCV and ROS
#include <image_geometry/pinhole_camera_model.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>



using namespace std;
using namespace message_filters::sync_policies;

class PointsDetector : public rclcpp::Node
{
private:
    typedef ApproximateTime<sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2, vision_msgs::msg::Detection2DArray> SyncPolicy;
    // Callbacks
    // void timer_callback();
    void callback_sync(const sensor_msgs::msg::CameraInfo::SharedPtr cam, const sensor_msgs::msg::PointCloud2::SharedPtr cloud, const vision_msgs::msg::Detection2DArray::SharedPtr detections);
    pcl::PointCloud<pcl::PointXYZ> msg2TransformedCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg);

    std::tuple<vision_msgs::msg::Detection3DArray, sensor_msgs::msg::PointCloud2> projectCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision_msgs::msg::Detection2DArray::SharedPtr& detections2d_msg, const std_msgs::msg::Header& header);

    float _cluster_tolerance = 0.5;
    int _min_cluster_size = 100;
    int _max_cluster_size =  25000;


    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cam_sub;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
    image_geometry::PinholeCameraModel _cam_model;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    void project3DPointsTo2D(const std::vector<cv::Point3f>& points3D, std::vector<cv::Point2f>& points2D);
    pcl::PointCloud<pcl::PointXYZ> cloud2TransformedCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std_msgs::msg::Header& header);
    pcl::PointCloud<pcl::PointXYZ> euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    void createBoundingBox(vision_msgs::msg::Detection3DArray& detections3d_msg, const pcl::PointCloud<pcl::PointXYZ>& cloud, const std::vector<vision_msgs::msg::ObjectHypothesisWithPose>& detection_results);
    visualization_msgs::msg::MarkerArray createMarkerArray(const vision_msgs::msg::Detection3DArray& detections3d_msg);

    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr _detection3d_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _detection_cloud_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_pub;
    // rclcpp::TimerBase::SharedPtr timer_;

public:
    PointsDetector(/* args */);
    ~PointsDetector();
    void initialize();
};

PointsDetector::PointsDetector(/* args */) : Node("PointsDetector_node")
{
    cam_sub.subscribe(this, "/zed2/camera_info");
    lidar_sub.subscribe(this, "/points_raw");
    det_sub.subscribe(this, "/detections");

    sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), cam_sub, lidar_sub, det_sub);
    sync->registerCallback(&PointsDetector::callback_sync, this);
    
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&PointsDetector::timer_callback, this));

    _detection3d_pub = this->create_publisher<vision_msgs::msg::Detection3DArray>("/detections3d", 10);
    _detection_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detection_cloud", 10);
    _marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_array", 10);
    
    RCLCPP_INFO(this->get_logger(), "PointsDetector_node initialized");
}

PointsDetector::~PointsDetector()
{
}

void PointsDetector::initialize()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(), false);
}

// void PointsDetector::timer_callback()
// {

// }

void PointsDetector::callback_sync(const sensor_msgs::msg::CameraInfo::SharedPtr cam, const sensor_msgs::msg::PointCloud2::SharedPtr cloud, const vision_msgs::msg::Detection2DArray::SharedPtr detections)
{
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    // vision_msgs::msg::Detection3DArray detections3d_msg;
    // sensor_msgs::msg::PointCloud2 detection_cloud_msg;
    visualization_msgs::msg::MarkerArray marker_array_msg;

    cameraMatrix = (cv::Mat1d(3, 3) << cam->k[0], cam->k[1], cam->k[2], cam->k[3], cam->k[4], cam->k[5], cam->k[6], cam->k[7], cam->k[8]);
    distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    // print something
    RCLCPP_INFO(this->get_logger(), "callback_sync");

    _cam_model.fromCameraInfo(cam);
    transformed_cloud = msg2TransformedCloud(cloud);
    auto [detections3d_msg, detection_cloud_msg] = projectCloud(transformed_cloud, detections, cloud->header);
    marker_array_msg = createMarkerArray(detections3d_msg);

    _detection3d_pub->publish(detections3d_msg);
    _detection_cloud_pub->publish(detection_cloud_msg);
    _marker_pub->publish(marker_array_msg);
}


void PointsDetector::project3DPointsTo2D(const std::vector<cv::Point3f>& points3D, std::vector<cv::Point2f>& points2D)
{
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);  // Assuming no rotation
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);  // Assuming no translation

    cv::projectPoints(points3D, rvec, tvec, cameraMatrix, distCoeffs, points2D);
}

pcl::PointCloud<pcl::PointXYZ> PointsDetector::msg2TransformedCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    geometry_msgs::msg::TransformStamped tf;

    try
    {
        tf = tf_buffer_->lookupTransform(_cam_model.tfFrame(), cloud_msg->header.frame_id, tf2::TimePointZero);
        pcl::fromROSMsg(*cloud_msg, cloud);
        Eigen::Affine3d transform = tf2::transformToEigen(tf.transform);
        pcl::transformPointCloud(cloud, transformed_cloud, transform);
    }
    catch (tf2::TransformException& e)
    {
        RCLCPP_WARN(this->get_logger(), "%s", e.what());
    }
    return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ> PointsDetector::cloud2TransformedCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const std_msgs::msg::Header& header)
{
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    geometry_msgs::msg::TransformStamped tf;
    try
    {
        tf = tf_buffer_->lookupTransform(header.frame_id, _cam_model.tfFrame(), tf2::TimePoint(std::chrono::nanoseconds(header.stamp.nanosec)));
        Eigen::Affine3d transform = tf2::transformToEigen(tf.transform);
        pcl::transformPointCloud(cloud, transformed_cloud, transform);
    }
    catch (tf2::TransformException& e)
    {
        RCLCPP_WARN(this->get_logger(), "%s", e.what());
    }
    return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ> PointsDetector::euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    pcl::PointCloud<pcl::PointXYZ> closest_cluster;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    std::vector<pcl::PointIndices> cluster_indices;
    float min_distance = std::numeric_limits<float>::max();
    tree->setInputCloud(cloud.makeShared());
    ec.setInputCloud(cloud.makeShared());
    ec.setClusterTolerance(_cluster_tolerance);  // Assuming _cluster_tolerance is a class member
    ec.setMinClusterSize(_min_cluster_size);    // Assuming _min_cluster_size is a class member
    ec.setMaxClusterSize(_max_cluster_size);    // Assuming _max_cluster_size is a class member
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);
    for (const auto& cluster_indice : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
        Eigen::Vector4f centroid;
        for (int indice : cluster_indice.indices)
        {
            cloud_cluster.points.push_back(cloud.points[indice]);
        }
        pcl::compute3DCentroid(cloud_cluster, centroid);
        float distance = centroid.norm();
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_cluster = cloud_cluster;
        }
    }
    return closest_cluster;
}

void PointsDetector::createBoundingBox(
    vision_msgs::msg::Detection3DArray& detections3d_msg,
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<vision_msgs::msg::ObjectHypothesisWithPose>& detections_results)
{
    vision_msgs::msg::Detection3D detection3d;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::PointXYZ min_pt, max_pt;
    Eigen::Vector4f centroid;
    Eigen::Vector4f bbox_center;
    Eigen::Vector4f transformed_bbox_center;
    Eigen::Affine3f transform;
    pcl::compute3DCentroid(cloud, centroid);
    double theta = -atan2(centroid[1], sqrt(pow(centroid[0], 2) + pow(centroid[2], 2)));
    transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(cloud, transformed_cloud, transform);
    pcl::getMinMax3D(transformed_cloud, min_pt, max_pt);
    transformed_bbox_center =
        Eigen::Vector4f((min_pt.x + max_pt.x) / 2, (min_pt.y + max_pt.y) / 2, (min_pt.z + max_pt.z) / 2, 1);
    bbox_center = transform.inverse() * transformed_bbox_center;
    detection3d.bbox.center.position.x = bbox_center[0];
    detection3d.bbox.center.position.y = bbox_center[1];
    detection3d.bbox.center.position.z = bbox_center[2];
    Eigen::Quaternionf q(transform.inverse().rotation());
    detection3d.bbox.center.orientation.x = q.x();
    detection3d.bbox.center.orientation.y = q.y();
    detection3d.bbox.center.orientation.z = q.z();
    detection3d.bbox.center.orientation.w = q.w();
    detection3d.bbox.size.x = max_pt.x - min_pt.x;
    detection3d.bbox.size.y = max_pt.y - min_pt.y;
    detection3d.bbox.size.z = max_pt.z - min_pt.z;
    detection3d.results = detections_results;
    detections3d_msg.detections.push_back(detection3d);
}


visualization_msgs::msg::MarkerArray PointsDetector::createMarkerArray(const vision_msgs::msg::Detection3DArray& detections3d_msg)
{
    visualization_msgs::msg::MarkerArray marker_array;

    for(size_t i = 0; i < detections3d_msg.detections.size(); ++i)
    {
        const auto& detection = detections3d_msg.detections[i];
        const auto& bbox = detection.bbox;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.header.stamp = detections3d_msg.header.stamp;
        marker.ns = "bounding_boxes";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = bbox.center.position;
        marker.pose.orientation = bbox.center.orientation;

        marker.scale.x = bbox.size.x;
        marker.scale.y = bbox.size.y;
        marker.scale.z = bbox.size.z;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        marker_array.markers.push_back(marker);
    }

    return marker_array;
}

std::tuple<vision_msgs::msg::Detection3DArray, sensor_msgs::msg::PointCloud2> PointsDetector::projectCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision_msgs::msg::Detection2DArray::SharedPtr& detections2d_msg, const std_msgs::msg::Header& header)
{
    pcl::PointCloud<pcl::PointXYZ> detection_cloud_raw;
    pcl::PointCloud<pcl::PointXYZ> detection_cloud;
    pcl::PointCloud<pcl::PointXYZ> closest_detection_cloud;
    pcl::PointCloud<pcl::PointXYZ> combine_detection_cloud;
    vision_msgs::msg::Detection3DArray detections3d_msg;
    sensor_msgs::msg::PointCloud2 combine_detection_cloud_msg;
    detections3d_msg.header = header;
    

    
    for (const auto& detection : detections2d_msg->detections)
    {
        // RCLCPP_INFO(this->get_logger(), "Detection: %s", detection.results[0].id.c_str());

        // RCLCPP_INFO(this->get_logger(), "Point: %f", cloud.points[0].x);
        for (const auto& point : cloud.points)
        {
            
            std::vector<cv::Point3f> points3D = { cv::Point3f(point.x, point.y, point.z) };
            std::vector<cv::Point2f> points2D;

            project3DPointsTo2D(points3D, points2D);
            cv::Point2d uv = points2D[0];  

            if (point.z > 0 && uv.x > 0 && uv.x >= detection.bbox.center.x - detection.bbox.size_x / 2 &&
                uv.x <= detection.bbox.center.x + detection.bbox.size_x / 2 &&
                uv.y >= detection.bbox.center.y - detection.bbox.size_y / 2 &&
                uv.y <= detection.bbox.center.y + detection.bbox.size_y / 2)
            {
                detection_cloud_raw.points.push_back(point);
                // debbug
                // RCLCPP_INFO(this->get_logger(), "Detection: %s", detection.results[0].id.c_str());
            }

        }
        detection_cloud = cloud2TransformedCloud(detection_cloud_raw, header);
        if (!detection_cloud.points.empty())
        {
            closest_detection_cloud = euclideanClusterExtraction(detection_cloud);
            // debbug detection3d_msg size
            


            createBoundingBox(detections3d_msg, closest_detection_cloud, detection.results);
            combine_detection_cloud.insert(combine_detection_cloud.end(), closest_detection_cloud.begin(),
                                           closest_detection_cloud.end());
            detection_cloud_raw.points.clear();

            // debbug

            // RCLCPP_INFO(this->get_logger(), "Detection: %d", detections3d_msg.detections.size());
            // sizes of detections3d_msg

            RCLCPP_INFO(this->get_logger(), "Detection: %s", detection.results[0].id.c_str());
        }

    }
    pcl::toROSMsg(combine_detection_cloud, combine_detection_cloud_msg);
    combine_detection_cloud_msg.header = header;
    return std::make_tuple(detections3d_msg, combine_detection_cloud_msg); 
}



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointsDetector>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}