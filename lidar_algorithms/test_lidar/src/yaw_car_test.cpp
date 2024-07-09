#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <vector>

class YawCarPublisher : public rclcpp::Node
{
public:
    YawCarPublisher() : Node("yaw_car_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_car", 10);
        waypoints_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "waypoints_loaded", 10,
            std::bind(&YawCarPublisher::waypointsCallback, this, std::placeholders::_1));
        target_index_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "target_waypoint_index", 10,
            std::bind(&YawCarPublisher::targetIndexCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&YawCarPublisher::timerCallback, this));
    }

private:
    void waypointsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        waypoints.clear();
        for (const auto &marker : msg->markers)
        {
            Eigen::VectorXd waypoint(4);
            waypoint(0) = marker.pose.position.x;
            waypoint(1) = marker.pose.position.y;
            waypoint(2) = marker.pose.position.z;

            tf2::Quaternion q(
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            waypoint(3) = yaw;

            waypoints.push_back(waypoint);
        }
    }

    void targetIndexCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        target_waypoint = msg->data;
    }

    void timerCallback()
    {
        if (waypoints.size() < 2 || target_waypoint >= waypoints.size())
        {
            RCLCPP_WARN(this->get_logger(), "Not enough waypoints to compute yaw or invalid target waypoint index.");
            return;
        }

        size_t previous_waypoint = (target_waypoint == 0) ? waypoints.size() - 1 : target_waypoint - 1;

        Eigen::VectorXd current_point = waypoints[target_waypoint];
        Eigen::VectorXd previous_point = waypoints[previous_waypoint];

        double yaw_current = current_point(3);
        double yaw_previous = previous_point(3);

        double yaw_difference = yaw_current - yaw_previous;

        auto message = std_msgs::msg::Float64();
        message.data = yaw_difference;
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing yaw difference: '%f'", message.data);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_index_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Eigen::VectorXd> waypoints;
    size_t target_waypoint = 0;  // Initialize to the first waypoint
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawCarPublisher>());
    rclcpp::shutdown();
    return 0;
}
