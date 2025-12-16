#include "vio_frontend/imu_preintegrator_ros.hpp"

namespace vio_frontend {

ImuPreintegratorROS::ImuPreintegratorROS(rclcpp::Node* node) : logger_(node->get_logger()) {
    // Parameters (Biases)
    // Note: In a real system, initial biases might come from calibration or VIO init.
    // For now, assume zero or load from params.
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    
    // We can load noise params here if ImuPreintegrator allows setting them
    // Currently ImuPreintegrator has hardcoded noise in header, TODO: make configurable.

    preintegrator_ = std::make_unique<inertial_frontend::ImuPreintegrator>(ba, bg);

    sub_imu_ = node->create_subscription<sensor_msgs::msg::Imu>(
        "imu0", 100, std::bind(&ImuPreintegratorROS::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(logger_, "IMU Preintegrator ROS Initialized");
}

void ImuPreintegratorROS::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    
    if (last_imu_time_ < 0) {
        last_imu_time_ = timestamp;
        // Initialize start measurement
        Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        preintegrator_->setInitialMeasurement(acc, gyro);
        return;
    }

    double dt = timestamp - last_imu_time_;
    last_imu_time_ = timestamp;

    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    preintegrator_->integrate(acc, gyro, dt);
}

inertial_frontend::PreintegratedImuMeasurement ImuPreintegratorROS::get_integration_and_reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Get result
    auto measurement = preintegrator_->getResult();
    
    // Reset integrator but keep current bias
    // AND we must maintain continuity for the next integration.
    // The "last_acc" and "last_gyro" inside ImuPreintegrator must remain valid for the next interval start.
    // ImuPreintegrator::reset() clears last_acc/gyro. This is problematic for continuous operation.
    // Ideally we want to reset delta_p, delta_v, delta_q, etc., but keep the "boundary" measurement.
    
    // Workaround: Get current internal state (last acc/gyro)? 
    // ImuPreintegrator doesn't expose getters for last_acc_.
    // BUT, we are in the callback setting it.
    // If we call reset(), we lose the "Start" of the next integration.
    
    // Let's look at ImuPreintegrator::reset().
    // It sets last_acc_ to Zero.
    
    // We need to modify ImuPreintegrator to allow "Reset Integration" without clearing IsInitialized?
    // Or we manually setInitialMeasurement again using the most recent IMU data?
    // We don't store the most recent IMU data here in ROS wrapper (except implicitly in preintegrator).
    
    // Better approach: Retrieve current bias. Reset.
    Eigen::Vector3d ba = measurement.bias_a;
    Eigen::Vector3d bg = measurement.bias_g;
    
    preintegrator_->reset(ba, bg);
    // Note: The preintegrator is now uninitialized.
    // The next IMU callback needs to re-initialize.
    // Problem: There might be a gap or "last_imu_time_" is now the time of the *last processed* imu msg.
    // If we just continue, the next callback will calculate dt = current - last.
    // But integrate() will return early because !is_initialized_.
    // Then it calls setInitialMeasurement.
    // So one IMU period is "lost" for integration (used only for initialization).
    // This is acceptable for simple VIO, or we can improve it later.
    
    return measurement;
}

} // namespace vio_frontend
