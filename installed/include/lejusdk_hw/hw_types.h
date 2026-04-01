#pragma once
#include <array>
#include <string>
#include <vector>

namespace leju {
namespace hw {

enum class ImuType {
    None,
    Xsens,
    Hipnuc
};

enum class HwErrorType : int32_t {
    Success = 0,
    NotInitialized,
    ImuInitFailed,
    ImuUnavailable,
    DimensionMismatch,
    PathNotFound,
    ConfigMissing,
    ReadSensorFailed,
    Failed
};

struct ImuData_t {
    std::array<double, 3> gyro;    // 角速度 [rad/s] - [x, y, z]
    std::array<double, 3> acc;     // 加速度 [m/s²] - [x, y, z]
    std::array<double, 3> free_acc;// 无重力加速度 [m/s²] - [x, y, z]
    std::array<double, 4> quat;    // 姿态四元数 [w, x, y, z]
    double timestamp;              // 时间戳 [s]
  
    // 默认构造函数
    ImuData_t() {
        reset();
    }

    // 重置函数
    void reset() {
        gyro.fill(0.0);
        acc.fill(0.0);
        free_acc.fill(0.0);
        quat.fill(0.0);
        quat[0] = 1.0;  // w = 1, identity quaternion
        timestamp = 0.0;
    }
};

// TODO(ohh): 添加 MotorCommand_t 和 MotorData_t 结构体定义
// struct MotorCommand_t {
//     std::vector<double> position;    // 目标位置 [rad]
//     std::vector<double> velocity;    // 目标速度 [rad/s]
//     std::vector<double> torque;      // 目标力矩 [N·m]
//     std::vector<double> kp;          // 位置增益 [N·m/rad]
//     std::vector<double> kd;          // 速度增益 [N·m·s/rad]
    
//     MotorCommand_t() {
//         // vectors will be empty by default
//     }
// };

// struct MotorData_t {
//     std::vector<double> q;          // 关节位置 [rad]
//     std::vector<double> v;          // 关节速度 [rad/s]
//     std::vector<double> current;    // 关节电流 [A]
// };
////////////////////////////////////////////////////////////

struct JointData_t {
    std::vector<double> q;      // 关节位置 [rad]
    std::vector<double> v;      // 关节速度 [rad/s]
    std::vector<double> vd;     // 关节加速度 [rad/s²]
    std::vector<double> tau;    // 关节力矩 [N·m]
    JointData_t() {}
    JointData_t(size_t num) {
        resize(num);
    }
    
    void resize(size_t num) {
        q.assign(num, 0.0);
        v.assign(num, 0.0);
        vd.assign(num, 0.0);
        tau.assign(num, 0.0);
    }

    // 检查所有字段维度是否一致且不为空
    bool isValid() const {
        size_t size = q.size();
        return (size != 0 &&
                v.size() == size &&
                vd.size() == size &&
                tau.size() == size);
    }
};

struct JointCommand_t {
    std::vector<double> q;      // 目标关节位置 [rad]
    std::vector<double> v;      // 目标关节速度 [rad/s]
    std::vector<double> tau;     // 目标关节力矩 [N·m]
    std::vector<double> kp;         // 位置增益 [N·m/rad]
    std::vector<double> kd;         // 速度增益 [N·m·s/rad]
    std::vector<double> tau_max;    // 目标关节力矩最大值 [N·m]
    std::vector<double> tau_ratio;  // 目标关节力矩比例 [N·m/N·m]
    std::vector<uint8_t> modes;     // 关节控制模式

    JointCommand_t()  {}
    JointCommand_t(size_t num) {
        resize(num);
    }

    void resize(size_t num) {
        q.assign(num, 0.0);
        v.assign(num, 0.0);
        tau.assign(num, 0.0);
        kp.assign(num, 0.0);
        kd.assign(num, 0.0);
        tau_max.assign(num, 0.0);
        tau_ratio.assign(num, 1.0);
        modes.assign(num, static_cast<uint8_t>(0));
    }

    bool isValid() const {
        size_t size = q.size();
        return (size != 0 &&
            v.size() == size &&
            tau.size() == size &&
            kp.size() == size &&
            kd.size() == size &&
            tau_max.size() == size &&
            tau_ratio.size() == size &&
            modes.size() == size);
    }
};

} // hw
} // namespace leju