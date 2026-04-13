#include "dexi_apriltag/apriltag_odometry.hpp"
#include <tf2/exceptions.h>
#include <limits>
#include <algorithm>
#include <numeric>

using namespace std::chrono_literals;

AprilTagOdometry::AprilTagOdometry(const rclcpp::NodeOptions & options)
: Node("apriltag_odometry", options)
{
    // Declare parameters
    this->declare_parameter("tag_family", "tag36h11");
    this->declare_parameter("target_tag_id", 0);
    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("position_variance", std::vector<double>{50.0, 50.0, 100.0});
    this->declare_parameter("orientation_variance", std::vector<double>{0.01, 0.01, 0.01});
    this->declare_parameter("filter_length", 5);
    this->declare_parameter("dry_run", false);
    this->declare_parameter("origin_lock_delay", 5.0);
    this->declare_parameter("use_frd", false);
    this->declare_parameter("require_ekf_sensors", true);
    this->declare_parameter("tag_map_ids", std::vector<int64_t>{});
    this->declare_parameter("tag_map_x", std::vector<double>{});
    this->declare_parameter("tag_map_y", std::vector<double>{});

    // Read parameters
    tag_family_ = this->get_parameter("tag_family").as_string();
    target_tag_id_ = this->get_parameter("target_tag_id").as_int();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    position_variance_ = this->get_parameter("position_variance").as_double_array();
    orientation_variance_ = this->get_parameter("orientation_variance").as_double_array();
    filter_length_ = this->get_parameter("filter_length").as_int();
    dry_run_ = this->get_parameter("dry_run").as_bool();
    origin_lock_delay_ = this->get_parameter("origin_lock_delay").as_double();
    use_frd_ = this->get_parameter("use_frd").as_bool();
    require_ekf_sensors_ = this->get_parameter("require_ekf_sensors").as_bool();

    // Build tag map
    use_tag_map_ = false;
    auto map_ids = this->get_parameter("tag_map_ids").as_integer_array();
    auto map_x = this->get_parameter("tag_map_x").as_double_array();
    auto map_y = this->get_parameter("tag_map_y").as_double_array();
    if (!map_ids.empty() && map_ids.size() == map_x.size() && map_ids.size() == map_y.size()) {
        for (size_t i = 0; i < map_ids.size(); ++i) {
            tag_map_[static_cast<int>(map_ids[i])] = {map_x[i], map_y[i]};
        }
        use_tag_map_ = true;
    }

    // QoS for PX4
    auto px4_qos = rclcpp::QoS(1)
        .best_effort()
        .transient_local();

    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher
    odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        "/fmu/in/vehicle_visual_odometry", px4_qos);

    // Subscribe to local position for heading (used once at origin lock)
    local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", px4_qos,
        std::bind(&AprilTagOdometry::localPositionCallback, this, std::placeholders::_1));

    // Subscribe to detections
    detection_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/apriltag_detections", 10,
        std::bind(&AprilTagOdometry::detectionCallback, this, std::placeholders::_1));

    // Subscribe to estimator status flags — wait for flow+range before origin lock
    estimator_sub_ = this->create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
        "/fmu/out/estimator_status_flags", px4_qos,
        [this](const px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg) {
            ekf_ev_pos_active_ = msg->cs_ev_pos;
            ekf_opt_flow_active_ = msg->cs_opt_flow;
            ekf_rng_hgt_active_ = msg->cs_rng_hgt;
        });

    // Publish timer
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&AprilTagOdometry::publishOdometry, this));

    last_detection_time_ = std::chrono::steady_clock::now();
    last_log_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "AprilTag Odometry (C++) initialized");
    if (use_frd_) {
        RCLCPP_INFO(get_logger(), "FRD mode — body-frame position, PX4 handles NED rotation");
    } else {
        RCLCPP_INFO(get_logger(), "NED mode — image-derived heading");
    }
    if (use_tag_map_) {
        RCLCPP_INFO(get_logger(), "Tag map mode: %zu tags", tag_map_.size());
        for (const auto & [tid, pos] : tag_map_) {
            RCLCPP_INFO(get_logger(), "  Tag %d: (%.2f, %.2f)", tid, pos.first, pos.second);
        }
    } else {
        RCLCPP_INFO(get_logger(), "Single tag mode: %s:%d", tag_family_.c_str(), target_tag_id_);
    }
    RCLCPP_INFO(get_logger(), "Filter: %d-sample moving average", filter_length_);
    if (dry_run_) {
        RCLCPP_WARN(get_logger(), "DRY RUN MODE - logging only, NOT publishing to EKF2");
    } else {
        RCLCPP_INFO(get_logger(), "Publishing at %.1f Hz to /fmu/in/vehicle_visual_odometry", publish_rate_);
    }
}

void AprilTagOdometry::detectionCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - last_detection_time_).count();
    if (elapsed < 0.1) return;  // Throttle to 10Hz

    detected_tag_ids_.clear();
    for (const auto & det : msg->detections) {
        detected_tag_ids_.insert(det.id);
    }
    if (!detected_tag_ids_.empty()) {
        last_detection_time_ = now;
    }

    // Compute heading from tag corners in the image (2D — no gimbal lock).
    // The angle of the tag's top edge (corners[0]→corners[1]) in pixel coordinates
    // directly encodes the drone's yaw relative to the tag.
    // Average top and bottom edges for noise reduction.
    for (const auto & det : msg->detections) {
        if (det.corners.size() >= 4) {
            // Top edge: corners[0] → corners[1]
            double dx_top = det.corners[1].x - det.corners[0].x;
            double dy_top = det.corners[1].y - det.corners[0].y;
            // Bottom edge: corners[3] → corners[2] (same direction)
            double dx_bot = det.corners[2].x - det.corners[3].x;
            double dy_bot = det.corners[2].y - det.corners[3].y;
            // Average
            double dx = (dx_top + dx_bot) * 0.5;
            double dy = (dy_top + dy_bot) * 0.5;
            latest_image_yaw_ = std::atan2(dy, dx);
            image_yaw_valid_ = true;
            break;  // Use first detected tag's corners
        }
    }
}

void AprilTagOdometry::localPositionCallback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    drone_heading_ = msg->heading;
    drone_x_ = msg->x;
    drone_y_ = msg->y;
    drone_z_ = msg->z;
    heading_valid_ = true;
    position_valid_ = true;
}

void AprilTagOdometry::attitudeCallback(
    const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    // Not used in fixed-heading mode
    (void)msg;
}

AprilTagOdometry::TagTransformResult AprilTagOdometry::lookupTagTf(int tag_id)
{
    std::string tag_frame = tag_family_ + ":" + std::to_string(tag_id);
    try {
        auto transform = tf_buffer_->lookupTransform(
            "base_link", tag_frame, tf2::TimePointZero,
            tf2::durationFromSec(0.005));

        double tx = transform.transform.translation.x;
        double ty = transform.transform.translation.y;
        double tz = transform.transform.translation.z;

        // Extract yaw from the tag's quaternion
        // The TF rotation encodes how the tag is oriented relative to the drone.
        // For a flat tag on the floor with camera pointing down, the yaw component
        // tells us the drone's heading relative to the tag's orientation.
        auto & q = transform.transform.rotation;
        // Yaw (rotation about z-axis) from quaternion
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double tag_yaw = std::atan2(siny_cosp, cosy_cosp);

        // Stale detection
        double tf_stamp = transform.header.stamp.sec +
            transform.header.stamp.nanosec / 1e9;
        double last_stamp = tf_stamps_.count(tag_id) ? tf_stamps_[tag_id] : 0.0;
        if (tf_stamp == last_stamp) {
            stale_counts_[tag_id]++;
            if (stale_counts_[tag_id] > 5) {
                return {false, -1, 0, 0, 0, 0};
            }
        } else {
            tf_stamps_[tag_id] = tf_stamp;
            stale_counts_[tag_id] = 0;
        }

        tf_timestamp_us_ = static_cast<uint64_t>(
            transform.header.stamp.sec * 1e6 +
            transform.header.stamp.nanosec / 1e3);

        return {true, tag_id, tx, ty, tz, tag_yaw};
    } catch (const tf2::TransformException &) {
        return {false, -1, 0, 0, 0, 0};
    }
}

AprilTagOdometry::TagTransformResult AprilTagOdometry::getTagTransform()
{
    if (use_tag_map_) {
        auto now = std::chrono::steady_clock::now();
        double detection_age = std::chrono::duration<double>(now - last_detection_time_).count();
        if (detection_age > 0.5) {
            current_tag_id_ = -1;
            return {false, -1, 0, 0, 0, 0};
        }

        // Find candidates: detected AND in our map
        std::vector<int> candidates;
        for (int id : detected_tag_ids_) {
            if (tag_map_.count(id)) {
                candidates.push_back(id);
            }
        }

        // Tag lock with hysteresis: stay on current tag even if it drops
        // from a few detection frames. Only switch after 3+ misses.
        int chosen_id;
        if (current_tag_id_ >= 0 &&
            std::find(candidates.begin(), candidates.end(), current_tag_id_) != candidates.end()) {
            // Current tag still visible — reset miss counter
            chosen_id = current_tag_id_;
            tag_miss_count_ = 0;
        } else if (current_tag_id_ >= 0 && tag_miss_count_ < 3) {
            // Current tag missing but give it a few frames to reappear
            tag_miss_count_++;
            // Try TF lookup for current tag anyway (TF buffer has history)
            auto result = lookupTagTf(current_tag_id_);
            if (result.success) {
                return result;
            }
            // TF also stale — fall through to pick new tag if candidates exist
            if (candidates.empty()) {
                return {false, -1, 0, 0, 0, 0};
            }
            chosen_id = *std::min_element(candidates.begin(), candidates.end());
        } else if (!candidates.empty()) {
            // Current tag gone for too long or no current tag — pick lowest ID
            chosen_id = *std::min_element(candidates.begin(), candidates.end());
            tag_miss_count_ = 0;
        } else {
            current_tag_id_ = -1;
            return {false, -1, 0, 0, 0, 0};
        }

        auto result = lookupTagTf(chosen_id);
        if (!result.success) {
            return result;
        }
        current_tag_id_ = chosen_id;
        return result;

    } else {
        // Single tag mode
        return lookupTagTf(target_tag_id_);
    }
}

void AprilTagOdometry::publishOdometry()
{
    if (!heading_valid_ || !position_valid_) {
        return;
    }

    auto result = getTagTransform();

    if (!result.success) {
        if (tag_visible_) {
            RCLCPP_INFO(get_logger(), "Tag lost - stopping odometry publishing");
            tag_visible_ = false;
            last_visible_tag_id_ = -1;
            consecutive_readings_ = 0;
        }
        return;
    }

    int tag_id = result.tag_id;
    double tx = result.tx, ty = result.ty, tz = result.tz;
    double tag_yaw = result.yaw;

    if (tag_id != last_visible_tag_id_) {
        if (last_visible_tag_id_ >= 0) {
            RCLCPP_INFO(get_logger(), "Switched to tag %d (reset_counter=%d)", tag_id, reset_counter_ + 1);
            north_buffer_.clear();
            east_buffer_.clear();
            down_buffer_.clear();
            reset_counter_++;  // Tell PX4 the reference point changed
        }
        last_visible_tag_id_ = tag_id;
        consecutive_readings_ = 0;
    }

    consecutive_readings_++;

    // Stability gate (2 readings = 0.2s at 10Hz)
    if (consecutive_readings_ < 2) {
        if (consecutive_readings_ == 1) {
            RCLCPP_INFO(get_logger(), "Tag %d detected - stabilizing...", tag_id);
        }
        return;
    }

    if (!tag_visible_) {
        RCLCPP_INFO(get_logger(), "Tag %d stable - starting odometry publishing", tag_id);
        tag_visible_ = true;
    }

    // Transform from TF frame to body frame
    double body_forward = -ty;
    double body_right = -tz;
    double body_down = -tx;

    // Wait for EKF sensors before publishing anything
    if (!origin_locked_ && !use_frd_) {
        if (require_ekf_sensors_ && !ekf_opt_flow_active_ && !ekf_rng_hgt_active_) {
            if (!first_tag_seen_) {
                first_tag_seen_ = true;
                RCLCPP_INFO(get_logger(),
                    "Tag %d visible — waiting for EKF flow+range before origin lock...", tag_id);
            }
            return;
        }

        auto now_lock = std::chrono::steady_clock::now();
        if (first_tag_seen_ && !origin_locked_ &&
            std::chrono::duration<double>(now_lock - first_tag_time_).count() < origin_lock_delay_) {
            return;
        }
        if (!first_tag_seen_) {
            first_tag_seen_ = true;
            first_tag_time_ = now_lock;
            RCLCPP_INFO(get_logger(),
                "Tag %d visible, EKF sensors active — waiting %.1fs before origin lock...",
                tag_id, origin_lock_delay_);
            return;
        }

        if (!image_yaw_valid_) return;

        image_yaw_offset_ = drone_heading_ - latest_image_yaw_;
        offset_north_ = drone_x_;
        offset_east_ = drone_y_;
        offset_down_ = drone_z_ - body_down;
        origin_locked_ = true;
        north_buffer_.clear();
        east_buffer_.clear();
        down_buffer_.clear();

        double hdg_deg = drone_heading_ * 180.0 / M_PI;
        if (hdg_deg < 0) hdg_deg += 360.0;
        RCLCPP_INFO(get_logger(),
            "Origin locked on tag %d — heading: %.1f° — offset: [%.2f, %.2f, %.2f]",
            tag_id, hdg_deg, offset_north_, offset_east_, offset_down_);
    }

    double pos_x, pos_y, pos_z;
    uint8_t pose_frame;

    if (use_frd_) {
        // FRD mode: publish body-frame position directly.
        // No heading needed — PX4 handles the NED rotation internally.
        pos_x = body_forward;
        pos_y = body_right;
        pos_z = body_down;
        pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    } else {
        // NED mode: rotate body frame to NED using image-derived heading
        double heading = latest_image_yaw_ + image_yaw_offset_;
        double cos_h = std::cos(heading);
        double sin_h = std::sin(heading);
        double tag_rel_north = body_forward * cos_h - body_right * sin_h;
        double tag_rel_east = body_forward * sin_h + body_right * cos_h;

        if (use_tag_map_) {
            auto & [tag_world_north, tag_world_east] = tag_map_[tag_id];
            pos_x = tag_world_north + tag_rel_north + offset_north_;
            pos_y = tag_world_east + tag_rel_east + offset_east_;
        } else {
            pos_x = tag_rel_north + offset_north_;
            pos_y = tag_rel_east + offset_east_;
        }
        pos_z = body_down + offset_down_;
        pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    }

    // Moving average filter
    north_buffer_.push_back(pos_x);
    east_buffer_.push_back(pos_y);
    down_buffer_.push_back(pos_z);
    while (static_cast<int>(north_buffer_.size()) > filter_length_) {
        north_buffer_.pop_front();
        east_buffer_.pop_front();
        down_buffer_.pop_front();
    }

    double avg_x = std::accumulate(north_buffer_.begin(), north_buffer_.end(), 0.0) / north_buffer_.size();
    double avg_y = std::accumulate(east_buffer_.begin(), east_buffer_.end(), 0.0) / east_buffer_.size();
    double avg_z = std::accumulate(down_buffer_.begin(), down_buffer_.end(), 0.0) / down_buffer_.size();

    // Build message
    auto msg = px4_msgs::msg::VehicleOdometry();
    msg.timestamp = tf_timestamp_us_;
    msg.timestamp_sample = tf_timestamp_us_;
    msg.pose_frame = pose_frame;
    msg.position = {static_cast<float>(avg_x), static_cast<float>(avg_y), static_cast<float>(avg_z)};

    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    msg.q = {nan, nan, nan, nan};
    msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    msg.velocity = {nan, nan, nan};
    msg.angular_velocity = {nan, nan, nan};
    msg.position_variance = {
        static_cast<float>(position_variance_[0]),
        static_cast<float>(position_variance_[1]),
        static_cast<float>(position_variance_[2])
    };
    msg.orientation_variance = {
        static_cast<float>(orientation_variance_[0]),
        static_cast<float>(orientation_variance_[1]),
        static_cast<float>(orientation_variance_[2])
    };
    msg.velocity_variance = {nan, nan, nan};
    msg.reset_counter = reset_counter_;
    msg.quality = 100;

    if (!dry_run_) {
        odom_pub_->publish(msg);
    }

    // Log periodically
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_log_time_).count() > 1.0) {
        if (use_frd_) {
            RCLCPP_INFO(get_logger(),
                "Tag %d | FRD: [%.2f, %.2f, %.2f]",
                tag_id, avg_x, avg_y, avg_z);
        } else {
            double hdg_deg = (latest_image_yaw_ + image_yaw_offset_) * 180.0 / M_PI;
            if (hdg_deg < 0) hdg_deg += 360.0;
            RCLCPP_INFO(get_logger(),
                "Tag %d | Body: [%.2f, %.2f] | NED: [%.2f, %.2f] | ImgHdg: %.0f°",
                tag_id, body_forward, body_right, avg_x, avg_y, hdg_deg);
        }
        last_log_time_ = now;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagOdometry>());
    rclcpp::shutdown();
    return 0;
}
