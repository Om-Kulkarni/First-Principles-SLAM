#ifndef INERTIAL_FRONTEND_IMU_PREINTEGRATOR_HPP_
#define INERTIAL_FRONTEND_IMU_PREINTEGRATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <inertial_frontend/preintegrated_imu_measurement.hpp>

namespace inertial_frontend {

class ImuPreintegrator {
 public:
  explicit ImuPreintegrator(const Eigen::Vector3d& bias_acc, const Eigen::Vector3d& bias_gyro);
  ~ImuPreintegrator() = default;

  /**
   * @brief reset the preintegrator with new biases
   */
  void reset(const Eigen::Vector3d& bias_acc, const Eigen::Vector3d& bias_gyro);

  /**
   * @brief Add a new IMU measurement to the preintegration 'bucket'
   * @param acc Raw acceleration
   * @param gyro Raw angular velocity
   * @param dt Time delta since last valid measurement
   */
  void integrate(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt);

  /**
   * @brief Approximate correction of integration result for a new bias
   * @param new_bias_a New accelerometer bias
   * @param new_bias_g New gyroscope bias
   * @return PreintegratedImuMeasurement Corrected result
   */
  PreintegratedImuMeasurement repropagate(const Eigen::Vector3d& new_bias_a,
                                          const Eigen::Vector3d& new_bias_g) const;
  
  /**
   * @brief Get the current integration result (valid for current internal bias)
   */
  const PreintegratedImuMeasurement& getResult() const { return current_measurement_; }

  /**
   * @brief Initialize/Reset the starting measurement for the next interval.
   * Required for midpoint integration.
   */
  void setInitialMeasurement(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
      last_acc_ = acc;
      last_gyro_ = gyro;
      is_initialized_ = true;
  }

 private:
  PreintegratedImuMeasurement current_measurement_;

  // State for midpoint integration
  Eigen::Vector3d last_acc_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_gyro_ = Eigen::Vector3d::Zero();
  bool is_initialized_ = false;

  // Noise parameters
  double acc_noise_std_ = 0.1; // 0.08 in example
  double gyro_noise_std_ = 0.01; // 0.004 in example
  double acc_bias_rw_std_ = 0.001;
  double gyro_bias_rw_std_ = 0.0001;
};

}  // namespace inertial_frontend

#endif  // INERTIAL_FRONTEND_IMU_PREINTEGRATOR_HPP_
