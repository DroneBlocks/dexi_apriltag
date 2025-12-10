#include "dexi_apriltag/apriltag_odometry.hpp"

AprilTagOdometry::AprilTagOdometry(const rclcpp::NodeOptions &options)
: Node("apriltag_odometry", options),
  qos_profile_(1),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
    // Declare parameters
    this->declare_parameter("base_link_frame", "base_link");
    this->declare_parameter("tag_family", "tag36h11");
    this->declare_parameter("min_detection_quality", 0.5);
    this->declare_parameter("position_variance", std::vector<double>{0.01, 0.01, 100.0});  // XY from vision, Z from distance sensor
    this->declare_parameter("orientation_variance", std::vector<double>{0.01, 0.01, 0.01});
    this->declare_parameter("velocity_variance", std::vector<double>{0.1, 0.1, 0.1});
    this->declare_parameter("target_tag_id", -1);

    // Get parameter values
    base_link_frame_ = this->get_parameter("base_link_frame").as_string();
    tag_family_ = this->get_parameter("tag_family").as_string();
    min_detection_quality_ = this->get_parameter("min_detection_quality").as_double();

    auto pos_var = this->get_parameter("position_variance").as_double_array();
    auto ori_var = this->get_parameter("orientation_variance").as_double_array();
    auto vel_var = this->get_parameter("velocity_variance").as_double_array();

    std::copy(pos_var.begin(), pos_var.end(), position_variance_.begin());
    std::copy(ori_var.begin(), ori_var.end(), orientation_variance_.begin());
    std::copy(vel_var.begin(), vel_var.end(), velocity_variance_.begin());

    target_tag_id_ = this->get_parameter("target_tag_id").as_int();

    // Configure QoS profile (match PX4 expectations)
    qos_profile_ = qos_profile_.best_effort()
                              .transient_local()
                              .keep_last(1);

    // Initialize publishers and subscribers
    initializePublishers();
    initializeSubscribers();

    RCLCPP_INFO(get_logger(), "AprilTag Odometry initialized");
    RCLCPP_INFO(get_logger(), "Base link frame: %s", base_link_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Tag family: %s", tag_family_.c_str());
    RCLCPP_INFO(get_logger(), "Min quality threshold: %.2f", min_detection_quality_);
    if (target_tag_id_ >= 0) {
        RCLCPP_INFO(get_logger(), "Target tag ID: %d", target_tag_id_);
    } else {
        RCLCPP_INFO(get_logger(), "Target tag ID: Best quality tag");
    }
}

AprilTagOdometry::~AprilTagOdometry()
{
}

void AprilTagOdometry::initializePublishers()
{
    odometry_publisher_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
        "/fmu/in/vehicle_visual_odometry", qos_profile_);
}

void AprilTagOdometry::initializeSubscribers()
{
    apriltag_subscriber_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections",
        10,
        std::bind(&AprilTagOdometry::apriltagCallback, this, std::placeholders::_1));
}

void AprilTagOdometry::apriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    if (msg->detections.empty()) {
        RCLCPP_DEBUG(get_logger(), "No detections in message");
        return;
    }

    RCLCPP_DEBUG(get_logger(), "Received %zu tag detections", msg->detections.size());

    // Select best tag based on quality and target_tag_id parameter
    int best_tag_id = selectBestTag(msg);
    if (best_tag_id < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "No high-quality AprilTag detections found (min quality: %.2f)", min_detection_quality_);
        return;
    }

    // Construct tag frame name (standard apriltag_ros convention)
    std::string tag_frame = tag_family_ + ":" + std::to_string(best_tag_id);

    RCLCPP_DEBUG(get_logger(), "Looking up TF: %s -> %s", base_link_frame_.c_str(), tag_frame.c_str());

    // Look up base_link -> tag transform from TF
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
            base_link_frame_,
            tag_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1));

        // Create and publish VehicleOdometry message
        auto odometry_msg = createOdometryMessage(transform);
        odometry_publisher_->publish(odometry_msg);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Publishing odometry for tag %d (quality: %.2f, position: [%.2f, %.2f, %.2f])",
            best_tag_id,
            calculateDetectionQuality(msg->detections[0]),
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);

    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Could not get transform from '%s' to '%s': %s",
            base_link_frame_.c_str(), tag_frame.c_str(), ex.what());
    }
}

double AprilTagOdometry::calculateDetectionQuality(const apriltag_msgs::msg::AprilTagDetection& detection)
{
    // Combine multiple quality factors
    double base_quality = 0.8;  // Default if no goodness field

    // Decision margin contributes to quality
    double decision_factor = std::min(detection.decision_margin / 50.0, 1.0);

    // Hamming distance (lower is better)
    double hamming_factor = std::max(0.0, 1.0 - detection.hamming / 5.0);

    // Combine factors
    double quality = base_quality * decision_factor * hamming_factor;
    return std::min(quality, 1.0);
}

int AprilTagOdometry::selectBestTag(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    int best_tag_id = -1;
    double best_quality = 0.0;

    for (const auto& detection : msg->detections) {
        // If target_tag_id is specified, only consider that tag
        if (target_tag_id_ >= 0 && detection.id != target_tag_id_) {
            continue;
        }

        double quality = calculateDetectionQuality(detection);

        if (quality > best_quality && quality >= min_detection_quality_) {
            best_quality = quality;
            best_tag_id = detection.id;
        }
    }

    return best_tag_id;
}

px4_msgs::msg::VehicleOdometry AprilTagOdometry::createOdometryMessage(
    const geometry_msgs::msg::TransformStamped& transform)
{
    px4_msgs::msg::VehicleOdometry msg;

    // Timestamp (PX4 system time in microseconds)
    msg.timestamp = getTimestamp();
    msg.timestamp_sample = msg.timestamp;

    // Pose frame (FRD world-fixed frame)
    msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;

    // Position (in meters)
    // The transform gives us base_link position in tag frame
    // We publish this as the vehicle's position
    msg.position[0] = static_cast<float>(transform.transform.translation.x);
    msg.position[1] = static_cast<float>(transform.transform.translation.y);
    msg.position[2] = static_cast<float>(transform.transform.translation.z);

    // Orientation (quaternion from FRD body frame to reference frame)
    // PX4 expects [w,x,y,z] order
    msg.q[0] = static_cast<float>(transform.transform.rotation.w);
    msg.q[1] = static_cast<float>(transform.transform.rotation.x);
    msg.q[2] = static_cast<float>(transform.transform.rotation.y);
    msg.q[3] = static_cast<float>(transform.transform.rotation.z);

    // Velocity frame and data (set to NaN since we don't have velocity from single pose)
    msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;
    msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
    msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
    msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
    msg.angular_velocity[0] = std::numeric_limits<float>::quiet_NaN();
    msg.angular_velocity[1] = std::numeric_limits<float>::quiet_NaN();
    msg.angular_velocity[2] = std::numeric_limits<float>::quiet_NaN();

    // Variance estimates
    msg.position_variance[0] = static_cast<float>(position_variance_[0]);
    msg.position_variance[1] = static_cast<float>(position_variance_[1]);
    msg.position_variance[2] = static_cast<float>(position_variance_[2]);

    msg.orientation_variance[0] = static_cast<float>(orientation_variance_[0]);
    msg.orientation_variance[1] = static_cast<float>(orientation_variance_[1]);
    msg.orientation_variance[2] = static_cast<float>(orientation_variance_[2]);

    msg.velocity_variance[0] = static_cast<float>(velocity_variance_[0]);
    msg.velocity_variance[1] = static_cast<float>(velocity_variance_[1]);
    msg.velocity_variance[2] = static_cast<float>(velocity_variance_[2]);

    // Quality (0-100) - high quality since we're using direct tag detection
    msg.quality = 90;

    // Reset counter
    msg.reset_counter = reset_counter_;

    return msg;
}

uint64_t AprilTagOdometry::getTimestamp()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Main function
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagOdometry>());
    rclcpp::shutdown();
    return 0;
}
