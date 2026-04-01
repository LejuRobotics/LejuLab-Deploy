#pragma once

#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace leju {

using scalar_t = double;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using array_t = Eigen::Array<scalar_t, Eigen::Dynamic, 1>;
using array_i = Eigen::Array<int, Eigen::Dynamic, 1>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

/**
 * @brief Numeric bounds representing a closed interval [lower, upper]
 */
struct Bounds {
  scalar_t lower;  ///< Lower bound (inclusive)
  scalar_t upper;  ///< Upper bound (inclusive)

  /// Clamp value to within bounds
  scalar_t clamp(scalar_t v) const { return std::clamp(v, lower, upper); }
  /// Check if value lies within bounds
  bool contains(scalar_t v) const { return v >= lower && v <= upper; }
  /// Interval length (upper - lower)
  scalar_t size() const { return upper - lower; }
};

/**
 * @brief Velocity command in robot base frame
 *
 * Coordinate convention (right-hand rule):
 *   - X: forward (positive = forward)
 *   - Y: left (positive = left)
 *   - Z: up (positive = counter-clockwise when viewed from above)
 */
struct VelocityCommand {
  scalar_t linear_x = 0.0;   ///< Forward velocity [m/s], positive = forward
  scalar_t linear_y = 0.0;   ///< Lateral velocity [m/s], positive = left
  scalar_t angular_z = 0.0;  ///< Yaw rate [rad/s], positive = counter-clockwise

  void setZero() { linear_x = linear_y = angular_z = 0.0; }
};

/**
 * @brief Controller lifecycle state
 */
enum class ControllerState {
  kUninitialized = 0,  ///< Not yet initialized
  kRunning,            ///< Actively running
  kPaused,             ///< Temporarily paused
  kStopped,            ///< Stopped and idle
  kError,              ///< Error state
};

/**
 * @brief 观测历史堆叠顺序
 *
 * 假设 2 个 term (A, B), history_length=3:
 *
 *   kIsaaclab: [A_t-2, A_t-1, A_t, B_t-2, B_t-1, B_t]
 *               |---- term A ----|  |---- term B ----|
 *
 *   kClassic:  [A_t-2, B_t-2, A_t-1, B_t-1, A_t, B_t]
 *               |-- t-2 --|  |-- t-1 --|  |-- t --|
 */
enum class StackOrder {
  kIsaaclab,  ///< 按 term 优先
  kClassic,   ///< 按 time 优先
};

/**
 * @brief 观测项配置
 */
struct ObsTermConfig {
  std::string name;
  double scale = 1.0;
  Bounds clip = {-100.0, 100.0};  ///< clip 范围
};

}  // namespace leju
