#include "inertial_frontend/imu_preintegrator.hpp"

#include <iostream>

namespace inertial_frontend {

ImuPreintegrator::ImuPreintegrator() {
  // TODO: Initialize preintegration
}

void ImuPreintegrator::addImuMeasurement(const sensor_msgs::msg::Imu& msg) {
  // TODO: Implement preintegration
  (void)msg; // Suppress unused variable warning
}

}  // namespace inertial_frontend
