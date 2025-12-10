#ifndef DEXI_APRILTAG__APRILTAG_ODOMETRY_HPP_
#define DEXI_APRILTAG__APRILTAG_ODOMETRY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <cmath>

class AprilTagOdometry : public rclcpp::Node
{
public:
    AprilTagOdometry(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~AprilTagOdometry();

private:
    // QoS profile
    rclcpp::QoS qos_profile_;

    // ROS publishers
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_publisher_;

    // ROS subscribers
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscriber_;

    // TF2 for looking up tag transforms
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    std::string base_link_frame_{"base_link"};
    std::string tag_family_{"tag36h11"};
    double min_detection_quality_{0.5};
    std::array<double, 3> position_variance_{0.01, 0.01, 100.0};  // XY from vision, Z from distance sensor
    std::array<double, 3> orientation_variance_{0.01, 0.01, 0.01};
    std::array<double, 3> velocity_variance_{0.1, 0.1, 0.1};
    int target_tag_id_{-1};  // -1 = use best quality, >=0 = specific tag ID

    // State
    uint8_t reset_counter_{0};

    // Methods
    void initializePublishers();
    void initializeSubscribers();

    // Callbacks
    void apriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    // Processing methods
    double calculateDetectionQuality(const apriltag_msgs::msg::AprilTagDetection& detection);
    int selectBestTag(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    px4_msgs::msg::VehicleOdometry createOdometryMessage(
        const geometry_msgs::msg::TransformStamped& transform);

    // Utility methods
    uint64_t getTimestamp();
};

#endif // DEXI_APRILTAG__APRILTAG_ODOMETRY_HPP_
