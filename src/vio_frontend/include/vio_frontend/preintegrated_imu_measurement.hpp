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
  Eigen::Vector3d delta_p;  ///< Change in position in local frame
  Eigen::Vector3d delta_v;  ///< Change in velocity in local frame
  Eigen::Quaterniond delta_q; ///< Change in rotation (local frame)

  // Integration time
  double delta_t;

  // Covariance matrix (order: delta_p, delta_q, delta_v, bg, ba)
  // Note: Order depends on convention. Standard is often:
  // p, q, v, ba, bg (15x15) OR orientation, velocity, position...
  // User specified 15x15.
  // We will assume order: delta_p, delta_v, delta_q (3 vector part), ba, bg
  // for the Jacobian structure.
  // However, usually we preintegrate the noise, so covariance is 9x9 for p,v,q
  // and we propagate effects of bias.
  // User request: "covariance (The 15x15 uncertainty matrix)"
  Eigen::Matrix<double, 15, 15> covariance;

  /**
   * @brief Linearized Jacobians for bias correction.
   *
   * Stores how the preintegrated values change if the bias estimate changes.
   * Approximation: value approx value_bar + J * (new_bias - old_bias)
   */
  struct Jacobians {
    Eigen::Matrix3d dp_dba; // d(delta_p) / d(b_a)
    Eigen::Matrix3d dp_dbg; // d(delta_p) / d(b_g)
    Eigen::Matrix3d dv_dba; // d(delta_v) / d(b_a)
    Eigen::Matrix3d dv_dbg; // d(delta_v) / d(b_g)
    Eigen::Matrix3d dq_dbg; // d(delta_q) / d(b_g)
    // Note: dq_dba is usually zero because rotation doesn't depend on accel bias in standard formulation
  } jacobians;

  // Biases used during preintegration (needed for repropagation check)
  Eigen::Vector3d bias_a;
  Eigen::Vector3d bias_g;

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
