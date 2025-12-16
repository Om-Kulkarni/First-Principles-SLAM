#include "vio_frontend/feature_tracker_ros.hpp"
#include <opencv2/opencv.hpp>

namespace vio_frontend {

FeatureTrackerROS::FeatureTrackerROS(rclcpp::Node* node, FeaturesCallback callback)
    : callback_(callback), logger_(node->get_logger()) {
    
    // Load Parameters
    load_parameters(node);

    // Create Subscriber
    sub_img_ = image_transport::create_subscription(node, "cam0/image_raw", 
        std::bind(&FeatureTrackerROS::img_callback, this, std::placeholders::_1), "raw", rmw_qos_profile_sensor_data);
    
    RCLCPP_INFO(logger_, "Feature Tracker ROS Initialized");
}

void FeatureTrackerROS::load_parameters(rclcpp::Node* node) {
    TrackerConfig config;
    
    // Default values if not present
    config.max_features = node->declare_parameter("max_features", 150);
    config.min_dist = node->declare_parameter("min_dist", 30);
    config.use_ransac = node->declare_parameter("use_ransac", true);

    // Camera Intrinsics
    std::vector<double> intrinsics = node->declare_parameter("cam0.intrinsics", std::vector<double>{});
    if (intrinsics.size() == 4) {
        // [fu, fv, cu, cv] -> K
        config.K = (cv::Mat_<double>(3, 3) << 
            intrinsics[0], 0.0, intrinsics[2],
            0.0, intrinsics[1], intrinsics[3],
            0.0, 0.0, 1.0);
    } else {
        RCLCPP_WARN(logger_, "Invalid camera intrinsics. Expected 4, got %lu", intrinsics.size());
    }

    // Camera Distortion
    std::vector<double> distortion = node->declare_parameter("cam0.distortion_coefficients", std::vector<double>{});
    if (distortion.size() >= 4) {
        config.D = cv::Mat(distortion.size(), 1, CV_64F);
        for (size_t i = 0; i < distortion.size(); i++) {
            config.D.at<double>(i) = distortion[i];
        }
    }

    tracker_ = std::make_unique<FeatureTracker>(config);
}

void FeatureTrackerROS::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
        return;
    }

    // Process
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    uint64_t timestamp_ns = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    
    auto features = tracker_->track_features(cv_ptr->image, timestamp_ns);
    
    // Callback to main node
    if (callback_) {
        callback_(features, timestamp);
    }
}

} // namespace vio_frontend
