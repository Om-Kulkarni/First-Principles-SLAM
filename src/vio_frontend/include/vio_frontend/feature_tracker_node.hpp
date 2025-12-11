#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "vio_frontend/feature_tracker.hpp"
#include "vio_frontend/msg/feature_measurement.hpp"

namespace vio_frontend {

class FeatureTrackerNode : public rclcpp::Node {
public:
    explicit FeatureTrackerNode(const rclcpp::NodeOptions & options);

private:
    /**
     * @brief Image callback
     */
    void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    /**
     * @brief Load parameters from config
     */
    void load_parameters();

    image_transport::Subscriber sub_img_;
    rclcpp::Publisher<vio_frontend::msg::FeatureMeasurement>::SharedPtr pub_feature_;
    
    std::unique_ptr<FeatureTracker> tracker_;
};

} // namespace vio_frontend
