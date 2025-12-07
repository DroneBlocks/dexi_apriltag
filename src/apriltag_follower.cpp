#include "dexi_apriltag/apriltag_follower.hpp"

AprilTagFollower::AprilTagFollower(const rclcpp::NodeOptions &options)
: Node("apriltag_follower", options),
  qos_profile_(1),
  last_detection_time_(this->now()),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
    // Declare parameters
    this->declare_parameter("following_enabled", false);
    this->declare_parameter("target_altitude", 2.0);
    this->declare_parameter("proportional_gain_xy", 2.0);  // Aggressive response
    this->declare_parameter("proportional_gain_z", 0.3);
    this->declare_parameter("max_velocity_xy", 3.0);  // Max speed 3.0 m/s
    this->declare_parameter("max_velocity_z", 1.0);
    this->declare_parameter("deadband_pixels", 20.0);
    this->declare_parameter("target_tag_id", 0);
    this->declare_parameter("detection_timeout", 10.0);

    // Get parameter values
    following_enabled_ = this->get_parameter("following_enabled").as_bool();
    target_altitude_ = this->get_parameter("target_altitude").as_double();
    proportional_gain_xy_ = this->get_parameter("proportional_gain_xy").as_double();
    proportional_gain_z_ = this->get_parameter("proportional_gain_z").as_double();
    max_velocity_xy_ = this->get_parameter("max_velocity_xy").as_double();
    max_velocity_z_ = this->get_parameter("max_velocity_z").as_double();
    deadband_pixels_ = this->get_parameter("deadband_pixels").as_double();
    target_tag_id_ = this->get_parameter("target_tag_id").as_int();
    detection_timeout_ = this->get_parameter("detection_timeout").as_double();

    // Configure QoS profile
    qos_profile_ = qos_profile_.best_effort()
                              .transient_local()
                              .keep_last(1);

    // Initialize publishers, subscribers, and timer
    initializePublishers();
    initializeSubscribers();
    initializeTimer();

    RCLCPP_INFO(get_logger(), "AprilTag Follower initialized");
    RCLCPP_INFO(get_logger(), "Following enabled: %s", following_enabled_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "Target altitude: %.2f m", target_altitude_);
    RCLCPP_INFO(get_logger(), "Target tag ID: %d (0 = any tag)", target_tag_id_);
}

AprilTagFollower::~AprilTagFollower()
{
}

void AprilTagFollower::initializePublishers()
{
    offboard_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos_profile_);

    trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos_profile_);

    visual_offset_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "apriltag_visual_offset", 10);
}

void AprilTagFollower::initializeSubscribers()
{
    apriltag_subscriber_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections",
        10,
        std::bind(&AprilTagFollower::apriltagCallback, this, std::placeholders::_1));

    camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info",
        qos_profile_,
        std::bind(&AprilTagFollower::cameraInfoCallback, this, std::placeholders::_1));

    vehicle_local_pos_subscriber_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position",
        qos_profile_,
        std::bind(&AprilTagFollower::vehicleLocalPosCallback, this, std::placeholders::_1));
}

void AprilTagFollower::initializeTimer()
{
    const double control_frequency = 20.0; // Hz
    const std::chrono::nanoseconds timer_period{static_cast<int64_t>(1e9/control_frequency)};

    control_timer_ = create_wall_timer(
        timer_period,
        std::bind(&AprilTagFollower::controlLoop, this));
}

void AprilTagFollower::apriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    if (msg->detections.empty()) {
        tag_detected_ = false;
        current_tag_frame_ = "";
        return;
    }

    // Find the target tag (either specific ID or first detected tag)
    for (const auto& detection : msg->detections) {
        // If target_tag_id is 0, follow any tag; otherwise match specific ID
        if (target_tag_id_ == 0 || detection.id == target_tag_id_) {
            tag_detected_ = true;
            last_detection_time_ = this->now();

            // Get tag center in image coordinates
            tag_pixel_x_ = detection.centre.x;
            tag_pixel_y_ = detection.centre.y;

            // Construct tag frame name (standard apriltag_ros convention)
            current_tag_frame_ = detection.family + ":" + std::to_string(detection.id);

            // Get camera frame from message header
            if (!msg->header.frame_id.empty()) {
                camera_frame_id_ = msg->header.frame_id;
            }

            // Try to get pose from TF
            try {
                // Use latest available transform instead of tf2::TimePointZero
                geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                    camera_frame_id_, current_tag_frame_,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.1));

                // Extract position
                double x = transform.transform.translation.x;
                double y = transform.transform.translation.y;
                double z = transform.transform.translation.z;

                // Calculate distance from camera
                tag_distance_ = std::sqrt(x*x + y*y + z*z);

                // Extract yaw from quaternion
                double qx = transform.transform.rotation.x;
                double qy = transform.transform.rotation.y;
                double qz = transform.transform.rotation.z;
                double qw = transform.transform.rotation.w;

                // Calculate yaw (rotation around Z-axis)
                tag_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

                RCLCPP_INFO(get_logger(), "Tag %d detected! Pixel: (%.0f, %.0f), Distance: %.2fm, Yaw: %.2frad",
                            detection.id, tag_pixel_x_, tag_pixel_y_, tag_distance_, tag_yaw_);
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(get_logger(),
                    "Could not get transform from '%s' to '%s': %s",
                    camera_frame_id_.c_str(), current_tag_frame_.c_str(), ex.what());
                tag_detected_ = false;
            }

            // Only process first matching tag
            break;
        }
    }
}

void AprilTagFollower::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!camera_info_received_) {
        camera_fx_ = msg->k[0];  // K[0,0]
        camera_fy_ = msg->k[4];  // K[1,1]
        camera_cx_ = msg->k[2];  // K[0,2]
        camera_cy_ = msg->k[5];  // K[1,2]
        camera_width_ = msg->width;
        camera_height_ = msg->height;
        camera_info_received_ = true;

        RCLCPP_INFO(get_logger(), "Camera info received: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f, size=%dx%d",
                   camera_fx_, camera_fy_, camera_cx_, camera_cy_,
                   static_cast<int>(camera_width_), static_cast<int>(camera_height_));
    }
}

void AprilTagFollower::vehicleLocalPosCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    drone_x_ = msg->x;
    drone_y_ = msg->y;
    drone_z_ = msg->z;
    drone_heading_ = msg->heading;
    drone_position_valid_ = true;
}

void AprilTagFollower::controlLoop()
{
    // Always publish offboard control mode heartbeat (required for PX4)
    px4_msgs::msg::OffboardControlMode offboard_mode{};
    offboard_mode.timestamp = getTimestamp();
    offboard_mode.position = true;
    offboard_mode.velocity = false;
    offboard_mode.acceleration = false;
    offboard_mode.attitude = false;
    offboard_mode.body_rate = false;
    offboard_mode_publisher_->publish(offboard_mode);

    // Only run active control if following is enabled
    if (!following_enabled_) {
        // Publish a basic setpoint to maintain heartbeat (hold current position)
        px4_msgs::msg::TrajectorySetpoint setpoint{};
        setpoint.timestamp = getTimestamp();
        setpoint.position = {static_cast<float>(drone_x_),
                            static_cast<float>(drone_y_),
                            static_cast<float>(drone_z_)};
        setpoint.yaw = static_cast<float>(drone_heading_);
        trajectory_setpoint_publisher_->publish(setpoint);
        return;
    }

    // Check if we have all required data
    if (!camera_info_received_ || !drone_position_valid_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Waiting for camera info or vehicle position...");
        // Publish hold position setpoint to maintain heartbeat
        px4_msgs::msg::TrajectorySetpoint setpoint{};
        setpoint.timestamp = getTimestamp();
        setpoint.position = {static_cast<float>(drone_x_),
                            static_cast<float>(drone_y_),
                            static_cast<float>(drone_z_)};
        setpoint.yaw = static_cast<float>(drone_heading_);
        trajectory_setpoint_publisher_->publish(setpoint);
        return;
    }

    // Check if tag detection is recent
    auto time_since_detection = (this->now() - last_detection_time_).seconds();
    if (!tag_detected_ || time_since_detection > detection_timeout_) {
        // Tag lost - hover in place
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Tag lost or not detected - hovering in place");
        // Reset altitude lock so it re-initializes on next detection
        altitude_initialized_ = false;
        // Publish hold position setpoint
        px4_msgs::msg::TrajectorySetpoint setpoint{};
        setpoint.timestamp = getTimestamp();
        setpoint.position = {static_cast<float>(drone_x_),
                            static_cast<float>(drone_y_),
                            static_cast<float>(drone_z_)};
        setpoint.yaw = static_cast<float>(drone_heading_);
        trajectory_setpoint_publisher_->publish(setpoint);
        return;
    }

    computeAndPublishSetpoint();
}

void AprilTagFollower::computeAndPublishSetpoint()
{
    // Initialize altitude and yaw on first tag detection
    if (!altitude_initialized_) {
        initial_altitude_ = drone_z_;
        altitude_initialized_ = true;
        RCLCPP_INFO(get_logger(), "Locked altitude at %.2fm", -initial_altitude_);
    }

    // Continuously update target yaw to track tag rotation
    // This allows following a rotating tag/object
    target_yaw_ = drone_heading_ + tag_yaw_;

    // Compute pixel offset from image center
    double offset_x_pixels = tag_pixel_x_ - camera_cx_;
    double offset_y_pixels = tag_pixel_y_ - camera_cy_;

    // Publish visual offset for debugging/visualization
    geometry_msgs::msg::PointStamped offset_msg;
    offset_msg.header.stamp = this->now();
    offset_msg.header.frame_id = "camera";
    offset_msg.point.x = offset_x_pixels;
    offset_msg.point.y = offset_y_pixels;
    offset_msg.point.z = tag_distance_;
    visual_offset_publisher_->publish(offset_msg);

    // Check if tag is already centered (within deadband)
    if (std::abs(offset_x_pixels) < deadband_pixels_ &&
        std::abs(offset_y_pixels) < deadband_pixels_) {
        RCLCPP_DEBUG(get_logger(), "Tag centered - holding position");
    }

    // Convert pixel offsets to angular errors (radians)
    // Positive offset_x means tag is to the right in image
    // Positive offset_y means tag is down in image
    double angular_error_x = std::atan2(offset_x_pixels, camera_fx_);
    double angular_error_y = std::atan2(offset_y_pixels, camera_fy_);

    // For downward-facing camera in NED frame:
    // Pixel coordinates: u=0 is LEFT, v=0 is TOP (increases downward)
    // - Tag at TOP of image (negative offset_y) = Tag AHEAD → Move FORWARD (positive vel_x)
    // - Tag at BOTTOM of image (positive offset_y) = Tag BEHIND → Move BACKWARD (negative vel_x)
    // - Tag at RIGHT of image (positive offset_x) = Tag RIGHT → Move RIGHT (positive vel_y)
    // - Tag at LEFT of image (negative offset_x) = Tag LEFT → Move LEFT (negative vel_y)

    // Compute velocity commands in drone body frame (NED)
    double velocity_y = proportional_gain_xy_ * angular_error_x * tag_distance_;  // RIGHT is positive
    double velocity_x = -proportional_gain_xy_ * angular_error_y * tag_distance_; // FORWARD is positive (invert image Y)

    // Clamp velocities
    velocity_x = std::clamp(velocity_x, -max_velocity_xy_, max_velocity_xy_);
    velocity_y = std::clamp(velocity_y, -max_velocity_xy_, max_velocity_xy_);

    // Altitude control: keep current altitude (no Z velocity)
    double velocity_z = 0.0;

    // Convert body frame velocities to world frame using current heading
    double cos_heading = std::cos(drone_heading_);
    double sin_heading = std::sin(drone_heading_);

    double velocity_north = velocity_x * cos_heading - velocity_y * sin_heading;
    double velocity_east = velocity_x * sin_heading + velocity_y * cos_heading;

    // Compute target position (current + velocity * dt, using small dt for smooth control)
    const double dt = 0.2;  // Time horizon for position command
    double target_x = drone_x_ + velocity_north * dt;
    double target_y = drone_y_ + velocity_east * dt;
    double target_z = initial_altitude_;  // Lock altitude to initial height

    // Publish trajectory setpoint
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = getTimestamp();
    setpoint.position = {static_cast<float>(target_x),
                        static_cast<float>(target_y),
                        static_cast<float>(target_z)};
    setpoint.yaw = static_cast<float>(target_yaw_);  // Continuously track tag rotation
    trajectory_setpoint_publisher_->publish(setpoint);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "Tag offset: (%.0f, %.0f)px | Vel: (%.2f, %.2f)m/s | Tag yaw: %.2frad → Target: %.2frad | Alt: %.2fm",
                offset_x_pixels, offset_y_pixels, velocity_x, velocity_y,
                tag_yaw_, target_yaw_, -initial_altitude_);
}

bool AprilTagFollower::isTagCentered()
{
    double offset_x_pixels = tag_pixel_x_ - camera_cx_;
    double offset_y_pixels = tag_pixel_y_ - camera_cy_;

    return std::abs(offset_x_pixels) < deadband_pixels_ &&
           std::abs(offset_y_pixels) < deadband_pixels_;
}

uint64_t AprilTagFollower::getTimestamp()
{
    return static_cast<uint64_t>(this->now().nanoseconds() / 1000);
}

// Main function
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
