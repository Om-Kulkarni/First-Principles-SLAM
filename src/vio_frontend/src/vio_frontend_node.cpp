#include <rclcpp/rclcpp.hpp>
#include "vio_frontend/feature_tracker_ros.hpp"
#include "vio_frontend/imu_preintegrator_ros.hpp"
#include "vio_frontend/msg/vio_update.hpp"

namespace vio_frontend {

class VioFrontendNode : public rclcpp::Node {
public:
    explicit VioFrontendNode(const rclcpp::NodeOptions & options)
        : Node("vio_frontend_node", options) {
        
        // Initialize Publishers
        pub_vio_update_ = this->create_publisher<vio_frontend::msg::VIOUpdate>("vio_frontend/update", 10);

        // Initialize Components
        // Order matters: Feature calback relies on IMU being ready? 
        // Actually they are independent until the callback fires.
        
        imu_preintegrator_ = std::make_unique<ImuPreintegratorROS>(this);
        
        // Bind callback
        auto callback = std::bind(&VioFrontendNode::feature_callback, this, std::placeholders::_1, std::placeholders::_2);
        feature_tracker_ = std::make_unique<FeatureTrackerROS>(this, callback);

        RCLCPP_INFO(this->get_logger(), "VIO Frontend Node Started");
    }

private:
    void feature_callback(const std::vector<vio_frontend::msg::Feature>& features, double timestamp) {
        // 1. Get IMU Preintegration
        auto imu_measurement = imu_preintegrator_->get_integration_and_reset();
        
        // 2. Construct Message
        vio_frontend::msg::VIOUpdate msg;
        msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(timestamp * 1e9));
        msg.header.frame_id = "cam0"; // Should be parameter
        
        msg.features = features;
        
        // Convert Eigen/Preintegrated types to ROS message
        msg.delta_p.x = imu_measurement.delta_p.x();
        msg.delta_p.y = imu_measurement.delta_p.y();
        msg.delta_p.z = imu_measurement.delta_p.z();
        
        msg.delta_v.x = imu_measurement.delta_v.x();
        msg.delta_v.y = imu_measurement.delta_v.y();
        msg.delta_v.z = imu_measurement.delta_v.z();
        
        msg.delta_q.w = imu_measurement.delta_q.w();
        msg.delta_q.x = imu_measurement.delta_q.x();
        msg.delta_q.y = imu_measurement.delta_q.y();
        msg.delta_q.z = imu_measurement.delta_q.z();
        
        msg.dt = imu_measurement.delta_t;
        
        msg.accel_bias.x = imu_measurement.bias_a.x();
        msg.accel_bias.y = imu_measurement.bias_a.y();
        msg.accel_bias.z = imu_measurement.bias_a.z();

        msg.gyro_bias.x = imu_measurement.bias_g.x();
        msg.gyro_bias.y = imu_measurement.bias_g.y();
        msg.gyro_bias.z = imu_measurement.bias_g.z();
        
        // 3. Publish
        pub_vio_update_->publish(msg);
    }

    std::unique_ptr<FeatureTrackerROS> feature_tracker_;
    std::unique_ptr<ImuPreintegratorROS> imu_preintegrator_;
    
    rclcpp::Publisher<vio_frontend::msg::VIOUpdate>::SharedPtr pub_vio_update_;
};

} // namespace vio_frontend

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vio_frontend::VioFrontendNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vio_frontend::VioFrontendNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
