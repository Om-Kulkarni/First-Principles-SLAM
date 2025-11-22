

#include "euroc_driver/data_publisher.hpp"

int main(int argc, char ** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  try {
    // Create the data publisher node
    auto node = std::make_shared<euroc_driver::DataPublisher>();
    
    RCLCPP_INFO(node->get_logger(), "Starting EuRoC data publisher...");
    
    // Spin the node
    rclcpp::spin(node);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("euroc_data_publisher"), "Exception in main: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  // Cleanup
  rclcpp::shutdown();
  return 0;
}
