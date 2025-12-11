#ifndef INERTIAL_FRONTEND_IMU_PREINTEGRATOR_HPP_
#define INERTIAL_FRONTEND_IMU_PREINTEGRATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace inertial_frontend {

class ImuPreintegrator {
 public:
  ImuPreintegrator();
  ~ImuPreintegrator() = default;

  void addImuMeasurement(const sensor_msgs::msg::Imu& msg);

 private:
  // TODO: Add implementation details
};

}  // namespace inertial_frontend

#endif  // INERTIAL_FRONTEND_IMU_PREINTEGRATOR_HPP_
