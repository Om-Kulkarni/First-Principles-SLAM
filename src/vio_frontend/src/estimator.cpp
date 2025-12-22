#include <cstdio>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Placeholder for the Estimator backend
 * 
 * This node is currently a stub for verifying the build system and will
 * eventually implement the VIO backend optimization (graph/filter).
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  printf("hello world vio_frontend package\n");
  rclcpp::shutdown();
  return 0;
}
