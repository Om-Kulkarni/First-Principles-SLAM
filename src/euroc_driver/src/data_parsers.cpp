#include "euroc_driver/data_parsers.hpp"
#include <iostream>
#include <stdexcept>

namespace euroc_driver {

// IMUParser implementation
bool IMUParser::parseRow(const std::vector<std::string>& row) {
    if (row.size() < 7) {
        std::cerr << "Error: IMU row has insufficient columns (" << row.size() << ")" << std::endl;
        return false;
    }
    
    try {
        data_.timestamp_ns = std::stoull(row[0]);
        data_.angular_velocity_x = std::stod(row[1]);
        data_.angular_velocity_y = std::stod(row[2]);
        data_.angular_velocity_z = std::stod(row[3]);
        data_.linear_acceleration_x = std::stod(row[4]);
        data_.linear_acceleration_y = std::stod(row[5]);
        data_.linear_acceleration_z = std::stod(row[6]);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing IMU data: " << e.what() << std::endl;
        return false;
    }
}

uint64_t IMUParser::getTimestamp() const {
    return data_.timestamp_ns;
}

// CameraParser implementation
CameraParser::CameraParser(const std::string& image_base_path) 
    : image_base_path_(image_base_path) {}

bool CameraParser::parseRow(const std::vector<std::string>& row) {
    if (row.size() < 2) {
        std::cerr << "Error: Camera row has insufficient columns (" << row.size() << ")" << std::endl;
        return false;
    }
    
    try {
        data_.timestamp_ns = std::stoull(row[0]);
        data_.filename = image_base_path_ + "/" + row[1];
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing camera data: " << e.what() << std::endl;
        return false;
    }
}

uint64_t CameraParser::getTimestamp() const {
    return data_.timestamp_ns;
}

// PoseParser implementation
bool PoseParser::parseRow(const std::vector<std::string>& row) {
    if (row.size() < 17) {
        std::cerr << "Error: Pose row has insufficient columns (" << row.size() << ")" << std::endl;
        return false;
    }
    
    try {
        data_.timestamp_ns = std::stoull(row[0]);
        data_.position_x = std::stod(row[1]);
        data_.position_y = std::stod(row[2]);
        data_.position_z = std::stod(row[3]);
        data_.quaternion_w = std::stod(row[4]);
        data_.quaternion_x = std::stod(row[5]);
        data_.quaternion_y = std::stod(row[6]);
        data_.quaternion_z = std::stod(row[7]);
        data_.velocity_x = std::stod(row[8]);
        data_.velocity_y = std::stod(row[9]);
        data_.velocity_z = std::stod(row[10]);
        data_.bias_gyro_x = std::stod(row[11]);
        data_.bias_gyro_y = std::stod(row[12]);
        data_.bias_gyro_z = std::stod(row[13]);
        data_.bias_accel_x = std::stod(row[14]);
        data_.bias_accel_y = std::stod(row[15]);
        data_.bias_accel_z = std::stod(row[16]);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing pose data: " << e.what() << std::endl;
        return false;
    }
}

uint64_t PoseParser::getTimestamp() const {
    return data_.timestamp_ns;
}

} // namespace euroc_driver
