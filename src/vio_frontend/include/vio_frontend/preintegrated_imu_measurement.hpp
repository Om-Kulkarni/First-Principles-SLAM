#ifndef VIO_FRONTEND_PREINTEGRATED_IMU_MEASUREMENT_HPP_
#define VIO_FRONTEND_PREINTEGRATED_IMU_MEASUREMENT_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace inertial_frontend {

/**
 * @brief Container for the results of IMU preintegration.
 *
 * This struct holds the changes in position, velocity, and rotation
 * accumulated over a period of time, along with the covariance of these estimations
 * and the Jacobians w.r.t bias changes.
 */
struct PreintegratedImuMeasurement {
  // Preintegrated values
  Eigen::Vector3d delta_p;  ///< Change in position in local frame (dp)
  Eigen::Vector3d delta_v;  ///< Change in velocity in local frame (dv)
  Eigen::Quaterniond delta_q; ///< Change in rotation (local frame) (dq)

  // Integration time
  double delta_t; ///< Total time interval integrated

  /**
   * @brief Covariance matrix of the preintegration error.
   * 
   * Dimensions: 15x15
   * Blocks:
   * - [0-2]: Position error (delta_p)
   * - [3-5]: Velocity error (delta_v)
   * - [6-8]: Orientation error (delta_q)
   * - [9-11]: Accelerometer bias error (b_a)
   * - [12-14]: Gyroscope bias error (b_g)
   */
  Eigen::Matrix<double, 15, 15> covariance;

  /**
   * @brief Linearized Jacobians for bias correction.
   *
   * Stores how the preintegrated values change if the bias estimate changes.
   * Approximation: value approx value_bar + J * (new_bias - old_bias)
   */
  struct Jacobians {
    Eigen::Matrix3d dp_dba; ///< d(delta_p) / d(b_a)
    Eigen::Matrix3d dp_dbg; ///< d(delta_p) / d(b_g)
    Eigen::Matrix3d dv_dba; ///< d(delta_v) / d(b_a)
    Eigen::Matrix3d dv_dbg; ///< d(delta_v) / d(b_g)
    Eigen::Matrix3d dq_dbg; ///< d(delta_q) / d(b_g)
  } jacobians;

  // Biases used during preintegration (needed for repropagation check)
  Eigen::Vector3d bias_a; ///< Accelerometer bias used during integration
  Eigen::Vector3d bias_g; ///< Gyroscope bias used during integration

  PreintegratedImuMeasurement() {
    delta_p.setZero();
    delta_v.setZero();
    delta_q.setIdentity();
    delta_t = 0.0;
    covariance.setZero();
    
    jacobians.dp_dba.setZero();
    jacobians.dp_dbg.setZero();
    jacobians.dv_dba.setZero();
    jacobians.dv_dbg.setZero();
    jacobians.dq_dbg.setZero();

    bias_a.setZero();
    bias_g.setZero();
  }
};

}  // namespace inertial_frontend

#endif  // VIO_FRONTEND_PREINTEGRATED_IMU_MEASUREMENT_HPP_
