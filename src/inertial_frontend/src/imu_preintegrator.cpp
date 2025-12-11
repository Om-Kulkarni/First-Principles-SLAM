#include "inertial_frontend/imu_preintegrator.hpp"
#include <iostream>

namespace inertial_frontend {

ImuPreintegrator::ImuPreintegrator(const Eigen::Vector3d& bias_acc, const Eigen::Vector3d& bias_gyro) {
  reset(bias_acc, bias_gyro);
}

void ImuPreintegrator::reset(const Eigen::Vector3d& bias_acc, const Eigen::Vector3d& bias_gyro) {
  current_measurement_ = PreintegratedImuMeasurement();
  current_measurement_.bias_a = bias_acc;
  current_measurement_.bias_g = bias_gyro;
  is_initialized_ = false;
  last_acc_.setZero();
  last_gyro_.setZero();
}

void ImuPreintegrator::integrate(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt) {
  if (dt <= 0) return;
  
  if (!is_initialized_) {
      setInitialMeasurement(acc, gyro);
      return;
  }

  // 0. Constants and current state
  const Eigen::Vector3d& ba = current_measurement_.bias_a;
  const Eigen::Vector3d& bg = current_measurement_.bias_g;
  
  // 1. Get corrected measurements (raw - bias)
  // Indices: 0 (previous), 1 (current/next)
  Eigen::Vector3d un_acc_0 = last_acc_ - ba;
  Eigen::Vector3d un_gyro_0 = last_gyro_ - bg;
  Eigen::Vector3d un_acc_1 = acc - ba;
  Eigen::Vector3d un_gyro_1 = gyro - bg;

  // 2. Mean Rotation (Midpoint)
  // Update rotation: q_{k+1} = q_k * Exp( (w_k + w_{k+1})/2 * dt )
  Eigen::Vector3d avg_gyro = 0.5 * (un_gyro_0 + un_gyro_1);
  Eigen::Vector3d angle_axis = avg_gyro * dt;
  double angle = angle_axis.norm();
  
  Eigen::Vector3d axis;
  if (angle > 1e-8) {
    axis = angle_axis / angle;
  } else {
    axis = Eigen::Vector3d::UnitX();
  }
  
  Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
  // User used explicit quaternion construction approx: Quaterniond(1, w*dt/2...)
  // AngleAxis is exact exp map. Midpoint ensures better accuracy.
  
  Eigen::Quaterniond delta_q_next = current_measurement_.delta_q * dq;
  delta_q_next.normalize();

  // 3. Update Position & Velocity
  // a_local = 0.5 * (R_k * a_0 + R_{k+1} * a_1)
  // Note: Standard midpoint for preintegration usually defines:
  // v_{k+1} = v_k + R_k * a_k * dt (Euler)
  // OR v_{k+1} = v_k + 0.5 * (R_k * a_k + R_{k+1} * a_{k+1}) * dt (Midpoint/Trapezoidal)
  
  Eigen::Matrix3d R_k = current_measurement_.delta_q.toRotationMatrix();
  Eigen::Matrix3d R_next = delta_q_next.toRotationMatrix();
  
  Eigen::Vector3d acc_local_0 = R_k * un_acc_0;
  Eigen::Vector3d acc_local_1 = R_next * un_acc_1;
  Eigen::Vector3d acc_local_avg = 0.5 * (acc_local_0 + acc_local_1);
  
  std::cout << "acc_avg: " << acc_local_avg.transpose() << " last_acc: " << last_acc_.transpose() << std::endl;

  // Position update: p_{k+1} = p_k + v_k * dt + 0.5 * a_local_avg * dt^2
  current_measurement_.delta_p += current_measurement_.delta_v * dt + 0.5 * acc_local_avg * dt * dt;
  current_measurement_.delta_v += acc_local_avg * dt;
  std::cout << "New delta_v: " << current_measurement_.delta_v.transpose() << std::endl;
  current_measurement_.delta_q = delta_q_next;
  current_measurement_.delta_t += dt;

  // 4. Update Covariance and Jacobians (Retained Euler approx for stability/simplicity as noted by user)
  // Updating F and V based on "k" state (Euler approximation is usually sufficient for covariance)
  
  Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 15, 12> V = Eigen::Matrix<double, 15, 12>::Zero();

  Eigen::Matrix3d acc_skew;
  Eigen::Vector3d acc_vec = un_acc_0; // Use start of interval for Jacobian linearization point
  acc_skew << 0, -acc_vec.z(), acc_vec.y(),
              acc_vec.z(), 0, -acc_vec.x(),
              -acc_vec.y(), acc_vec.x(), 0;

  // dP/d...
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(0, 9) = -0.5 * R_k * dt * dt;
  
  // dV/d...
  F.block<3, 3>(3, 6) = -R_k * acc_skew * dt;
  F.block<3, 3>(3, 9) = -R_k * dt;

  // dTheta/d...
  F.block<3, 3>(6, 6) = dq.toRotationMatrix().transpose(); // Approx
  F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

  // Noise components (V)
  V.block<3, 3>(0, 0) = 0.5 * R_k * dt * dt;
  V.block<3, 3>(3, 0) = R_k * dt;
  V.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;
  V.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity() * dt;
  V.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * dt;

  Eigen::Matrix<double, 12, 12> NoiseCov = Eigen::Matrix<double, 12, 12>::Zero();
  NoiseCov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * acc_noise_std_ * acc_noise_std_;
  NoiseCov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * gyro_noise_std_ * gyro_noise_std_;
  NoiseCov.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * acc_bias_rw_std_ * acc_bias_rw_std_;
  NoiseCov.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * gyro_bias_rw_std_ * gyro_bias_rw_std_;

  current_measurement_.covariance = F * current_measurement_.covariance * F.transpose() + V * NoiseCov * V.transpose();

  // 5. Update Jacobians w.r.t Bias
  current_measurement_.jacobians.dp_dba = F.block<3, 3>(0, 0) * current_measurement_.jacobians.dp_dba + F.block<3, 3>(0, 3) * current_measurement_.jacobians.dv_dba + F.block<3, 3>(0, 9);
  current_measurement_.jacobians.dp_dbg = F.block<3, 3>(0, 0) * current_measurement_.jacobians.dp_dbg + F.block<3, 3>(0, 3) * current_measurement_.jacobians.dv_dbg + F.block<3, 3>(0, 6) * current_measurement_.jacobians.dq_dbg;
  
  current_measurement_.jacobians.dv_dba = F.block<3, 3>(3, 3) * current_measurement_.jacobians.dv_dba + F.block<3, 3>(3, 9);
  current_measurement_.jacobians.dv_dbg = F.block<3, 3>(3, 3) * current_measurement_.jacobians.dv_dbg + F.block<3, 3>(3, 6) * current_measurement_.jacobians.dq_dbg;
  
  current_measurement_.jacobians.dq_dbg = F.block<3, 3>(6, 6) * current_measurement_.jacobians.dq_dbg + F.block<3, 3>(6, 12);

  // Store current measurements for next step
  last_acc_ = acc;
  last_gyro_ = gyro;
}

PreintegratedImuMeasurement ImuPreintegrator::repropagate(const Eigen::Vector3d& new_bias_a,
                                                        const Eigen::Vector3d& new_bias_g) const {
  PreintegratedImuMeasurement result = current_measurement_;
  
  Eigen::Vector3d d_ba = new_bias_a - result.bias_a;
  Eigen::Vector3d d_bg = new_bias_g - result.bias_g;

  // First order correction
  result.delta_p += result.jacobians.dp_dba * d_ba + result.jacobians.dp_dbg * d_bg;
  result.delta_v += result.jacobians.dv_dba * d_ba + result.jacobians.dv_dbg * d_bg;
  
  // Rotation correction
  Eigen::Vector3d d_theta = result.jacobians.dq_dbg * d_bg;
  Eigen::Quaterniond dq_corr;
  double angle = d_theta.norm();
  if (angle > 1e-8) {
    dq_corr = Eigen::Quaterniond(Eigen::AngleAxisd(angle, d_theta / angle));
  } else {
    dq_corr = Eigen::Quaterniond::Identity();
  }
  result.delta_q = result.delta_q * dq_corr;
  result.delta_q.normalize();

  // Update biases in the result (conceptually, though they belong to keys usually)
  result.bias_a = new_bias_a;
  result.bias_g = new_bias_g;
  
  return result;
}

}  // namespace inertial_frontend
