#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "vio_frontend/feature_tracker.hpp"
#include "vio_frontend/msg/vio_update.hpp"

namespace vio_frontend {

// Callback type: vector of features, timestamp
using FeaturesCallback = std::function<void(const std::vector<vio_frontend::msg::Feature>&, double)>;

/**
 * @brief ROS Wrapper for the FeatureTracker
 * 
 * Subscribes to image topics, converts ROS images to OpenCV format,
 * runs the feature tracker, and invokes a callback with the results.
 */
class FeatureTrackerROS {
public:
    /**
     * @brief Construct a new FeatureTrackerROS object
     * 
     * @param node Pointer to the parent ROS node
     * @param callback Function to call when features are tracked
     */
    FeatureTrackerROS(rclcpp::Node* node, FeaturesCallback callback);

private:
    /**
     * @brief Callback for incoming image messages
     * 
     * @param msg ROS image message
     */
    void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    /**
     * @brief Load parameters from ROS parameter server
     * 
     * @param node Pointer to the ROS node
     */
    void load_parameters(rclcpp::Node* node);

    image_transport::Subscriber sub_img_;
    std::unique_ptr<FeatureTracker> tracker_;
    FeaturesCallback callback_;
    rclcpp::Logger logger_;
};

} // namespace vio_frontend
