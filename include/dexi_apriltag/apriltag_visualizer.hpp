#ifndef DEXI_APRILTAG__APRILTAG_VISUALIZER_HPP_
#define DEXI_APRILTAG__APRILTAG_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

class AprilTagVisualizer : public rclcpp::Node
{
public:
    AprilTagVisualizer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~AprilTagVisualizer();

private:
    // ROS publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_image_compressed_publisher_;

    // ROS subscribers
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr visual_offset_subscriber_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscriber_;

    // State variables
    cv::Mat latest_image_;
    bool image_received_{false};

    double offset_x_pixels_{0.0};
    double offset_y_pixels_{0.0};
    double tag_distance_{0.0};
    bool offset_received_{false};

    double image_center_x_{320.0};
    double image_center_y_{240.0};

    std::vector<cv::Point2d> tag_corners_;
    double tag_center_x_{0.0};
    double tag_center_y_{0.0};
    bool tag_detected_{false};

    // Parameters
    bool publish_raw_{true};
    bool publish_compressed_{true};

    // Callbacks
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void visualOffsetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void apriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    // Processing
    void drawVisualization();
};

#endif // DEXI_APRILTAG__APRILTAG_VISUALIZER_HPP_
