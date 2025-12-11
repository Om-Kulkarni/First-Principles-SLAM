#include <gtest/gtest.h>
#include "inertial_frontend/imu_preintegrator.hpp"

using namespace inertial_frontend;

TEST(ImuPreintegratorTest, Initialization) {
  Eigen::Vector3d ba(0, 0, 0);
  Eigen::Vector3d bg(0, 0, 0);
  ImuPreintegrator integrator(ba, bg);
  
  auto result = integrator.getResult();
  EXPECT_TRUE(result.delta_p.isZero());
  EXPECT_TRUE(result.delta_v.isZero());
  EXPECT_EQ(result.delta_t, 0.0);
  EXPECT_TRUE(result.delta_q.isApprox(Eigen::Quaterniond::Identity()));
}

TEST(ImuPreintegratorTest, StaticIntegration) {
  Eigen::Vector3d ba(0, 0, 0);
  Eigen::Vector3d bg(0, 0, 0);
  ImuPreintegrator integrator(ba, bg);

  double dt = 0.01;
  Eigen::Vector3d acc(0, 0, 0); // No gravity removal in preintegration usually? 
                                // WAIT. Standard preintegration assumes we integrate measured - bias.
                                // If simulating static on table: acc = [0,0,g]. 
                                // But preintegrated Delta V should be approx [0,0,g]*t?
                                // If we don't subtract gravity in frontend (which we usually don't),
                                // then yes, it integrates specific force.
  
  Eigen::Vector3d gyro(0, 0, 0);

  // Integrate for 1 second (100 intervals, so 101 measurements)
  for(int i=0; i<=100; ++i) {
    integrator.integrate(acc, gyro, dt);
  }

  auto result = integrator.getResult();
  EXPECT_NEAR(result.delta_t, 1.0, 1e-6);
  EXPECT_TRUE(result.delta_p.isZero(1e-6));
  EXPECT_TRUE(result.delta_v.isZero(1e-6));
  EXPECT_TRUE(result.delta_q.isApprox(Eigen::Quaterniond::Identity()));
}

TEST(ImuPreintegratorTest, ConstantAcceleration) {
  Eigen::Vector3d ba(0, 0, 0);
  Eigen::Vector3d bg(0, 0, 0);
  ImuPreintegrator integrator(ba, bg);

  double dt = 0.1;
  Eigen::Vector3d acc(1.0, 0, 0); // 1 m/s^2 x
  Eigen::Vector3d gyro(0, 0, 0);

  // First step initializes
  integrator.integrate(acc, gyro, dt); 
  
  // Need second step to actually integrate with midpoint using (last=acc, current=acc)
  // Let's integrate for 1 step of dt=0.1.
  integrator.integrate(acc, gyro, dt); 

  // Total time integrated: 0.1s (first call didn't add time, just init)
  // Logic: 
  // Call 1: stores last=acc, returns. t=0.
  // Call 2: integrates from last(acc) to current(acc) over dt=0.1. t=0.1.
  
  // v = a*t = 1*0.1 = 0.1
  // p = 0.5*a*t^2 = 0.5*1*0.01 = 0.005
  
  auto result = integrator.getResult();
  EXPECT_NEAR(result.delta_v.x(), 0.1, 1e-6);
  EXPECT_NEAR(result.delta_p.x(), 0.005, 1e-6);
  
  // Integrate 3rd step (2nd integration interval)
  integrator.integrate(acc, gyro, dt); // t = 0.2
  // v_new = 0.1 + 0.1 = 0.2
  // p_new = 0.005 + 0.1*0.1 + 0.005 = 0.02
  
  result = integrator.getResult();
  EXPECT_NEAR(result.delta_v.x(), 0.2, 1e-6);
  EXPECT_NEAR(result.delta_p.x(), 0.02, 1e-6);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
