#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "vio_frontend/imu_preintegrator.hpp"
#include "vio_frontend/preintegrated_imu_measurement.hpp"

namespace vio_frontend {

/**
 * @brief ROS Wrapper for IMU Preintegration
 * 
 * Subscribes to raw IMU topics, buffers messages, and uses the ImuPreintegrator
 * to accumulate measurements. Safe for multi-threaded access.
 */
class ImuPreintegratorROS {
public:
    /**
     * @brief Construct a new Imu Preintegrator ROS object
     * 
     * @param node Pointer to the parent ROS node
     */
    explicit ImuPreintegratorROS(rclcpp::Node* node);

    /**
     * @brief Get the current preintegrated measurement and reset the integrator (except bias).
     * 
     * This is typically called by the VIO frontend when a camera frame is processed
     * to get the motion constraint between the previous frame and the current one.
     * 
     * @return inertial_frontend::PreintegratedImuMeasurement struct containing delta_p, v, q, etc.
     */
    inertial_frontend::PreintegratedImuMeasurement get_integration_and_reset();

private:
    /**
     * @brief Callback for high-rate IMU messages
     * 
     * @param msg ROS IMU message
     */
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    std::unique_ptr<inertial_frontend::ImuPreintegrator> preintegrator_;
    
    // Thread safety for IMU integration vs Fetching
    std::mutex mutex_;
    
    double last_imu_time_ = -1.0;
    rclcpp::Logger logger_;
};

} // namespace vio_frontend
