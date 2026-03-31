#pragma once

#include <string>

namespace leju {
namespace hw {

namespace path {

/**
 * @brief Configuration directory path
 * @return Path to configuration directory
 */
std::string config_dir();

/**
 * @brief IMU type configuration file path
 * @return Path to IMU type configuration file
 */
std::string imu_type_path();

/**
 * @brief EC master type configuration file path
 * @return Path to EC master type configuration file
 */
std::string ecmaster_type_path();

/**
 * @brief Motor offset configuration file path
 * @return Path to motor offset configuration file
 */
std::string offset_path();

/**
 * @brief Motor revo offset configuration file path
 * @return Path to motor revo offset configuration file
 */
std::string motor_revo_offset_path();

/**
 * @brief CAN bus wiring type configuration file path
 * @return Path to CAN bus wiring type configuration file
 */
std::string canbus_wiring_type_path();

/**
 * @brief CAN bus configuration file path
 * @return Path to CAN bus configuration file
 */
std::string canbus_config_path();

} // namespace utils
} // namespace hw
} // namespace leju