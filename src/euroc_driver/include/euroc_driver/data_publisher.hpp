#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "euroc_driver/csv_reader.hpp"
#include "euroc_driver/data_parsers.hpp"

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

namespace euroc_driver {

/**
 * @brief Main data publisher node for EuRoC dataset
 */
class DataPublisher : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    explicit DataPublisher();
    
    /**
     * @brief Destructor
     */
    ~DataPublisher();

private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    image_transport::Publisher image_publisher_cam0_;
    image_transport::Publisher image_publisher_cam1_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    
    // CSV readers
    std::unique_ptr<CSVReader> imu_reader_;
    std::unique_ptr<CSVReader> cam0_reader_;
    std::unique_ptr<CSVReader> cam1_reader_;
    std::unique_ptr<CSVReader> pose_reader_;
    
    // Data parsers
    std::unique_ptr<IMUParser> imu_parser_;
    std::unique_ptr<CameraParser> cam0_parser_;
    std::unique_ptr<CameraParser> cam1_parser_;
    std::unique_ptr<PoseParser> pose_parser_;
    
    // Playback Thread
    std::thread playback_thread_;
    
    // Parameters
    std::string dataset_path_;
    double playback_rate_;
    bool loop_playback_;
    bool publish_images_;
    double imu_rate_hz_;
    double camera_rate_hz_;
    
    // State variables
    std::atomic<bool> is_playing_;
    uint64_t start_timestamp_;
    std::chrono::steady_clock::time_point start_time_;
    
    // Buffers for storing the "next" item from each stream
    struct StreamState {
        bool active = false;
        bool has_data = false;
        uint64_t timestamp_ns = 0;
        std::vector<std::string> raw_row;
    };
    
    StreamState state_imu_;
    StreamState state_cam0_;
    StreamState state_cam1_;
    StreamState state_pose_;

    /**
     * @brief Initialize the node parameters
     */
    void initializeParameters();
    
    /**
     * @brief Initialize publishers
     */
    void initializePublishers();
    
    /**
     * @brief Initialize CSV readers and parsers
     */
    bool initializeReaders();

    /**
     * @brief Initialize Clock publisher
     */
    void initializeClock();
    
    /**
     * @brief The main loop that drives playback
     */
    void playbackLoop();
    
    /**
     * @brief Helper to read the next row into a StreamState
     */
    void advanceStream(StreamState& state, CSVReader& reader, DataParser& parser);
    
    /**
     * @brief Convert timestamp from nanoseconds to ROS time
     * @param timestamp_ns Timestamp in nanoseconds
     * @return ROS time stamp
     */
    rclcpp::Time convertTimestamp(uint64_t timestamp_ns);
    
    /**
     * @brief Load and publish image
     */
    void publishImage(const std::string& image_path, uint64_t timestamp_ns, 
                     image_transport::Publisher& publisher, const std::string& frame_id);
    
    /**
     * @brief Reset all readers to beginning of files
     */
    void resetReaders();
};

} // namespace euroc_driver
