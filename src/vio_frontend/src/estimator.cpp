#include <cstdio>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  printf("hello world vio_frontend package\n");
  rclcpp::shutdown();
  return 0;
}
