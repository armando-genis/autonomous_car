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




using namespace std;
using namespace message_filters::sync_policies;

class PointsDetector : public rclcpp::Node
{
private:
    typedef ApproximateTime<sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2, vision_msgs::msg::Detection2DArray> SyncPolicy;
    // Callbacks
    void timer_callback();
    void callback_sync(const sensor_msgs::msg::CameraInfo::SharedPtr cam, const sensor_msgs::msg::PointCloud2::SharedPtr cloud, const vision_msgs::msg::Detection2DArray::SharedPtr detections);
    pcl::PointCloud<pcl::PointXYZ> msg2TransformedCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg);

    std::tuple<vision_msgs::msg::Detection3DArray, sensor_msgs::msg::PointCloud2> projectCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const vision_msgs::msg::Detection2DArray::SharedPtr& detections2d_msg, const std_msgs::msg::Header& header);

    rclcpp::TimerBase::SharedPtr timer_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cam_sub;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
    image_geometry::PinholeCameraModel _cam_model;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&PointsDetector::timer_callback, this));
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

void PointsDetector::timer_callback()
{

    
}

void PointsDetector::callback_sync(const sensor_msgs::msg::CameraInfo::SharedPtr cam, const sensor_msgs::msg::PointCloud2::SharedPtr cloud, const vision_msgs::msg::Detection2DArray::SharedPtr detections)
{
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    vision_msgs::msg::Detection3DArray detections3d_msg;
    sensor_msgs::msg::PointCloud2 detection_cloud_msg;
    visualization_msgs::msg::MarkerArray marker_array_msg;

    // print something
    RCLCPP_INFO(this->get_logger(), "callback_sync");

    _cam_model.fromCameraInfo(cam);
    transformed_cloud = msg2TransformedCloud(cloud);
    auto [detections3d_msg, detection_cloud_msg] = projectCloud(transformed_cloud, detections, cloud->header);


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


std::tuple<vision_msgs::msg::Detection3DArray, sensor_msgs::msg::PointCloud2>
PointsDetector::projectCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                             const vision_msgs::msg::Detection2DArray::SharedPtr& detections2d_msg,
                             const std_msgs::msg::Header& header)
{
    RCLCPP_INFO(this->get_logger(), "Im here");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointsDetector>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}