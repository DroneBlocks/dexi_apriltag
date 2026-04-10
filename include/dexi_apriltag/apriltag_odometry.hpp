#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>

class AprilTagOdometry : public rclcpp::Node
{
public:
    AprilTagOdometry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // Callbacks
    void detectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void publishOdometry();

    // Tag lookup
    struct TagTransformResult {
        bool success;
        int tag_id;
        double tx, ty, tz;
        double yaw;  // drone heading derived from tag orientation
    };
    TagTransformResult getTagTransform();
    TagTransformResult lookupTagTf(int tag_id);

    // Parameters
    std::string tag_family_;
    int target_tag_id_;
    double publish_rate_;
    std::vector<double> position_variance_;
    std::vector<double> orientation_variance_;
    int filter_length_;
    bool dry_run_;

    // Tag map
    std::unordered_map<int, std::pair<double, double>> tag_map_;
    bool use_tag_map_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers / Subscribers
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Drone state
    double drone_heading_ = 0.0;
    double drone_x_ = 0.0;
    double drone_y_ = 0.0;
    double drone_z_ = 0.0;
    double drone_pitch_ = 0.0;
    double drone_roll_ = 0.0;
    bool heading_valid_ = false;
    bool position_valid_ = false;
    bool attitude_valid_ = false;

    // Tag tracking state
    bool tag_visible_ = false;
    int last_visible_tag_id_ = -1;
    int current_tag_id_ = -1;
    int consecutive_readings_ = 0;
    std::unordered_set<int> detected_tag_ids_;
    std::chrono::steady_clock::time_point last_detection_time_;

    // Tag switch hysteresis
    int tag_miss_count_ = 0;

    // Stale detection
    std::unordered_map<int, double> tf_stamps_;
    std::unordered_map<int, int> stale_counts_;
    uint64_t tf_timestamp_us_ = 0;

    // Origin offset and image-derived heading
    bool origin_locked_ = false;
    double image_yaw_offset_ = 0.0;  // Aligns image yaw with EKF NED frame
    double latest_image_yaw_ = 0.0;  // Yaw from tag corners in image (2D, no gimbal lock)
    bool image_yaw_valid_ = false;
    double offset_north_ = 0.0;
    double offset_east_ = 0.0;
    double offset_down_ = 0.0;

    // Moving average filter
    std::deque<double> north_buffer_;
    std::deque<double> east_buffer_;
    std::deque<double> down_buffer_;

    // Logging throttle
    std::chrono::steady_clock::time_point last_log_time_;
};
