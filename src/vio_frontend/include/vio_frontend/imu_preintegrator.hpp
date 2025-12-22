#ifndef VIO_FRONTEND_IMU_PREINTEGRATOR_HPP_
#define VIO_FRONTEND_IMU_PREINTEGRATOR_HPP_

#include <vio_frontend/preintegrated_imu_measurement.hpp>

namespace inertial_frontend {

/**
 * @brief Handles IMU Preintegration for Visual-Inertial Odometry
 * 
 * Implements the preintegration theory for IMU measurements to propagate
 * state between keyframes without re-integrating at every optimization step.
 * Uses midpoint integration for better accuracy.
 */
class ImuPreintegrator {
 public:
  /**
   * @brief Construct a new Imu Preintegrator object
   * 
   * @param bias_acc Initial accelerometer bias
   * @param bias_gyro Initial gyroscope bias
   */
  explicit ImuPreintegrator(const Eigen::Vector3d& bias_acc, const Eigen::Vector3d& bias_gyro);
  ~ImuPreintegrator() = default;

  /**
   * @brief Reset the preintegrator with new biases
   * 
   * Clears accumulated measurements and updates linearisation point for biases.
   * 
   * @param bias_acc New accelerometer bias
   * @param bias_gyro New gyroscope bias
   */
  void reset(const Eigen::Vector3d& bias_acc, const Eigen::Vector3d& bias_gyro);

  /**
   * @brief Add a new IMU measurement to the preintegration 'bucket'
   * 
   * Propagates the delta state (position, velocity, rotation) using the
   * current measurements and the previous ones (midpoint integration).
   * Also updates the covariance matrix and Jacobians.
   * 
   * @param acc Raw acceleration measurement
   * @param gyro Raw angular velocity measurement
   * @param dt Time delta since last valid measurement
   */
  void integrate(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt);

  /**
   * @brief Approximate correction of integration result for a new bias
   * 
   * Uses the computed Jacobians to linearise the effect of a small change in bias
   * on the preintegrated values, avoiding full re-integration.
   * 
   * @param new_bias_a New accelerometer bias
   * @param new_bias_g New gyroscope bias
   * @return PreintegratedImuMeasurement Corrected measurement struct
   */
  PreintegratedImuMeasurement repropagate(const Eigen::Vector3d& new_bias_a,
                                          const Eigen::Vector3d& new_bias_g) const;
  
  /**
   * @brief Get the current integration result (valid for current internal bias)
   * @return const PreintegratedImuMeasurement& 
   */
  const PreintegratedImuMeasurement& getResult() const { return current_measurement_; }

  /**
   * @brief Initialize/Reset the starting measurement for the next interval.
   * Required for midpoint integration.
   * 
   * @param acc Initial acceleration reading
   * @param gyro Initial gyroscope reading
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
  double acc_noise_std_ = 0.1; ///< Accelerometer white noise density
  double gyro_noise_std_ = 0.01; ///< Gyroscope white noise density
  double acc_bias_rw_std_ = 0.001; ///< Accelerometer bias random walk
  double gyro_bias_rw_std_ = 0.0001; ///< Gyroscope bias random walk
};

}  // namespace inertial_frontend

#endif  // VIO_FRONTEND_IMU_PREINTEGRATOR_HPP_
