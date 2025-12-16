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

class FeatureTrackerROS {
public:
    FeatureTrackerROS(rclcpp::Node* node, FeaturesCallback callback);

private:
    void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void load_parameters(rclcpp::Node* node);

    image_transport::Subscriber sub_img_;
    std::unique_ptr<FeatureTracker> tracker_;
    FeaturesCallback callback_;
    rclcpp::Logger logger_;
};

} // namespace vio_frontend
