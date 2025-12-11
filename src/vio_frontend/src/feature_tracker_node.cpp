#include "vio_frontend/feature_tracker_node.hpp"
#include <opencv2/opencv.hpp>

namespace vio_frontend {

FeatureTrackerNode::FeatureTrackerNode(const rclcpp::NodeOptions & options)
    : Node("feature_tracker_node", options) {
    
    // Load Parameters
    load_parameters();

    // Create Subscriber. Use create_subscription to avoid shared_from_this() in constructor issue
    sub_img_ = image_transport::create_subscription(this, "cam0/image_raw", 
        std::bind(&FeatureTrackerNode::img_callback, this, std::placeholders::_1), "raw", rmw_qos_profile_sensor_data);

    // Create Publisher
    pub_feature_ = this->create_publisher<vio_frontend::msg::FeatureMeasurement>("feature_tracker/feature", 10);
    
    RCLCPP_INFO(this->get_logger(), "Feature Tracker Node Initialized");
}

void FeatureTrackerNode::load_parameters() {
    TrackerConfig config;
    
    // Default values if not present
    config.max_features = this->declare_parameter("max_features", 150);
    config.min_dist = this->declare_parameter("min_dist", 30);
    config.use_ransac = this->declare_parameter("use_ransac", true);

    // Camera Intrinsics
    std::vector<double> intrinsics = this->declare_parameter("cam0.intrinsics", std::vector<double>{});
    if (intrinsics.size() == 4) {
        // [fu, fv, cu, cv] -> K
        config.K = (cv::Mat_<double>(3, 3) << 
            intrinsics[0], 0.0, intrinsics[2],
            0.0, intrinsics[1], intrinsics[3],
            0.0, 0.0, 1.0);
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera intrinsics. Expected 4, got %lu", intrinsics.size());
    }

    // Camera Distortion
    std::vector<double> distortion = this->declare_parameter("cam0.distortion_coefficients", std::vector<double>{});
    if (distortion.size() >= 4) {
        config.D = cv::Mat(distortion.size(), 1, CV_64F);
        for (size_t i = 0; i < distortion.size(); i++) {
            config.D.at<double>(i) = distortion[i];
        }
    }

    tracker_ = std::make_unique<FeatureTracker>(config);
}

void FeatureTrackerNode::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Process
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    auto measurement = tracker_->track_features(cv_ptr->image, timestamp);
    
    // Publish
    pub_feature_->publish(measurement);
}

} // namespace vio_frontend

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vio_frontend::FeatureTrackerNode)
