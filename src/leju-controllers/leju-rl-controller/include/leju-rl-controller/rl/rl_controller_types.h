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
 */
struct VelocityCommand {
  scalar_t linear_x = 0.0;
  scalar_t linear_y = 0.0;
  scalar_t angular_z = 0.0;

  void setZero() { linear_x = linear_y = angular_z = 0.0; }
};

/**
 * @brief Controller lifecycle state
 */
enum class ControllerState {
  kUninitialized = 0,
  kRunning,
  kPaused,
  kStopped,
  kError,
};

/**
 * @brief 观测历史堆叠顺序
 */
enum class StackOrder {
  kIsaaclab,
  kClassic,
};

/**
 * @brief 观测项配置
 */
struct ObsTermConfig {
  std::string name;
  double scale = 1.0;
  Bounds clip = {-100.0, 100.0};
};

}  // namespace leju
