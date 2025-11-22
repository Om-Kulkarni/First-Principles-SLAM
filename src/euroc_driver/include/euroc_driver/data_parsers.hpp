#pragma once

#include <string>
#include <vector>
#include <memory>

namespace euroc_driver {

/**
 * @brief Structure to hold IMU data
 */
struct IMUData {
    uint64_t timestamp_ns;
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
    double linear_acceleration_x;
    double linear_acceleration_y;
    double linear_acceleration_z;
};

/**
 * @brief Structure to hold camera data
 */
struct CameraData {
    uint64_t timestamp_ns;
    std::string filename;
};

/**
 * @brief Structure to hold ground truth pose data
 */
struct PoseData {
    uint64_t timestamp_ns;
    double position_x;
    double position_y;
    double position_z;
    double quaternion_w;
    double quaternion_x;
    double quaternion_y;
    double quaternion_z;
    double velocity_x;
    double velocity_y;
    double velocity_z;
    double bias_gyro_x;
    double bias_gyro_y;
    double bias_gyro_z;
    double bias_accel_x;
    double bias_accel_y;
    double bias_accel_z;
};

/**
 * @brief Base class for data parsers
 */
class DataParser {
public:
    virtual ~DataParser() = default;
    
    /**
     * @brief Parse a row of CSV data
     * @param row Vector of string tokens from CSV row
     * @return true if parsing was successful, false otherwise
     */
    virtual bool parseRow(const std::vector<std::string>& row) = 0;
    
    /**
     * @brief Get the timestamp of the last parsed data in nanoseconds
     * @return timestamp in nanoseconds
     */
    virtual uint64_t getTimestamp() const = 0;
};

/**
 * @brief Parser for IMU data
 */
class IMUParser : public DataParser {
public:
    bool parseRow(const std::vector<std::string>& row) override;
    uint64_t getTimestamp() const override;
    const IMUData& getData() const { return data_; }

private:
    IMUData data_;
};

/**
 * @brief Parser for camera data
 */
class CameraParser : public DataParser {
public:
    explicit CameraParser(const std::string& image_base_path);
    bool parseRow(const std::vector<std::string>& row) override;
    uint64_t getTimestamp() const override;
    const CameraData& getData() const { return data_; }

private:
    CameraData data_;
    std::string image_base_path_;
};

/**
 * @brief Parser for ground truth pose data
 */
class PoseParser : public DataParser {
public:
    bool parseRow(const std::vector<std::string>& row) override;
    uint64_t getTimestamp() const override;
    const PoseData& getData() const { return data_; }

private:
    PoseData data_;
};

} // namespace euroc_driver
