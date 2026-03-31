#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <string>
#include <atomic>
#include <ostream>
#include "lejusdk_hw/hw_types.h"

namespace HighlyDynamic {
class HardwarePlant;
struct HardwareParam;
struct HardwareSettings;
}

namespace leju {
namespace hw {

// IMU type conversion functions
/**
 * @brief Convert IMU type enum to string
 * @param type [in] IMU type enum value
 * @return Corresponding string representation
 */
std::string ImuType2String(ImuType type);

/**
 * @brief Convert string to IMU type enum
 * @param type_str [in] IMU type string
 * @return Corresponding IMU type enum value
 */
ImuType String2ImuType(const std::string& type_str);

/**
 * @brief Hardware interface class providing unified communication with underlying hardware
 *
 * This class uses singleton pattern to manage robot hardware initialization,
 * data reading and control command sending. Including motor control,
 * IMU data acquisition and other functions.
 */
class HardwareInterface {
public:
    HardwareInterface(const HardwareInterface&) = delete;
    HardwareInterface& operator=(const HardwareInterface&) = delete;
    /**
     * @brief Get hardware interface singleton instance
     * @return HardwareInterface singleton reference
     */
    static HardwareInterface& GetInstance();

    /**
     * @brief Get current IMU type
     * @return IMU type enum value
     */
    static ImuType GetImuType();

    /**
     * @brief Initialize hardware interface
     *
     * This method initializes the underlying hardware communication interface,
     * including motor controllers, IMU and other hardware devices.
     * Must be called before using other functions.
     * 
     * @param config_dir [in] Configuration directory path containing hardware configuration files.
     * @return HwErrorType Initialization result status code
     *         - Success: Initialization successful
     *         - NotInitialized: Not initialized
     *         - ImuInitFailed: IMU initialization failed
     *         - ImuUnavailable: IMU unavailable
     *         - ConfigMissing: Configuration file missing
     *         - Failed: Initialization failed
     */
    HwErrorType Initialize(const std::string &config_dir);

    /**
     * @brief Shutdown hardware interface
     *
     * Release hardware resources, disconnect from underlying hardware,
     * clean up internal state and safely shutdown all hardware devices.
     */
    void Shutdown();

    /**
     * @brief Get motor count
     * @return Total number of motors
     */
    uint32_t GetMotorNumber() const;

    /**
     * @brief Get IMU data
     *
     * Read latest attitude and motion data from IMU sensor
     * Including acceleration, angular velocity, attitude quaternion and other information
     *
     * @param imu_data [out] Reference to structure storing IMU data
     * @return HwErrorType Read result status code
     *         - Success: Read successful
     *         - NotInitialized: Interface not initialized
     *         - ImuUnavailable: IMU unavailable
     *         - ReadSensorFailed: Sensor read failed
     *         - Failed: Read failed
     */
    HwErrorType GetImuData(ImuData_t &imu_data);

    /**
     * @brief Set joint control command
     *
     * Send position, velocity and torque control commands to all joints
     * The commands will be sent to the underlying motor controller for execution
     *
     * @param joint_command [in] Joint command structure containing target position, velocity, torque and other information for all joints
     * @return HwErrorType Set result status code
     *         - Success: Command set successfully
     *         - NotInitialized: Interface not initialized
     *         - DimensionMismatch: Data dimension mismatch
     *         - Failed: Command send failed
     */
    HwErrorType SetJointCommand(const JointCommand_t &joint_command);

    /**
     * @brief Get joint state data
     *
     * Read current state information of all joints
     * Including current position, velocity, torque, temperature and other state data
     *
     * @param joint_data [out] Reference to structure storing joint state data
     * @return HwErrorType Read result status code
     *         - Success: Read successful
     *         - NotInitialized: Interface not initialized
     *         - ReadSensorFailed: Sensor read failed
     *         - Failed: Read failed
     */
    HwErrorType GetJointState(JointData_t &joint_data);


    /**
     * @brief Move all joints to specified target positions with specified speed
     *
     * @param goal_pos [in] Target positions for all joints in radians
     *                     The vector size must match the number of joints in the robot
     * @param speed [in] degrees/second
     * @param dt [in] Time step for trajectory interpolation in seconds
     * @param current_limit [in] Maximum current limit for all joints during movement
     *                         Default: -1 (no current limit).
     * @return HwErrorType Command result status code
     *         - Success: Movement command executed successfully
     *         - NotInitialized: Hardware interface not initialized
     */
    HwErrorType JointMoveTo(const std::vector<double> &goal_pos, double speed, double dt = 1e-3, double current_limit=-1);
    
    // TODO(ohh): Implement these methods
    // HwErrorType SetMotorCommand(const MotorCommand_t &motor_command);
    // TODO(ohh): Implement these methods
    // HwErrorType GetMotorState(MotorData_t &motor_data);
private:
    /**
     * @brief Private constructor implementing singleton pattern
     */
    HardwareInterface() = default;

    /**
     * @brief Private destructor
     */
    ~HardwareInterface() = default;

    /// Initialization status flag, true indicates initialized
    std::atomic<bool> initialized_{false};

    /// Underlying hardware control object pointer
    HighlyDynamic::HardwarePlant* hardware_plant_ = nullptr;
    HighlyDynamic::HardwareSettings* hardware_settings_ = nullptr;
};

} // hw
} // namespace leju

/**
 * @brief Output stream operator for JointData_t
 *
 * Allows direct output of JointData_t to std::ostream
 *
 * @param os [in/out] Output stream reference
 * @param joint_data [in] Joint data to output
 * @return Reference to output stream for chaining
 */
std::ostream& operator<<(std::ostream& os, const leju::hw::JointData_t& joint_data);

/**
 * @brief Output stream operator for ImuData_t
 *
 * Allows direct output of ImuData_t to std::ostream
 *
 * @param os [in/out] Output stream reference
 * @param imu_data [in] IMU data to output
 * @return Reference to output stream for chaining
 */
std::ostream& operator<<(std::ostream& os, const leju::hw::ImuData_t& imu_data);

/**
 * @brief Output stream operator for JointCommand_t
 *
 * Allows direct output of JointCommand_t to std::ostream
 *
 * @param os [in/out] Output stream reference
 * @param joint_command [in] Joint command data to output
 * @return Reference to output stream for chaining
 */
std::ostream& operator<<(std::ostream& os, const leju::hw::JointCommand_t& joint_command);