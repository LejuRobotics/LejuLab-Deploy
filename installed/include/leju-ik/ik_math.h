/**
 * @file ik_math.h
 * @brief Math utilities for IK module (fhan filter, quaternion limiting)
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace leju {
namespace ik {

// ============ Quaternion limiting (from leju_utils) ============
inline Eigen::Quaterniond limitQuaternionAngleEulerZYX(const Eigen::Quaterniond& q_input,
                                                        const Eigen::Vector3d& zyxLimits,
                                                        const Eigen::Vector3d scale = Eigen::Vector3d::Ones()) {
  Eigen::Quaterniond q = q_input.normalized();
  if (q.w() < 0) {
    q.coeffs() = -q.coeffs();
  }

  double yaw_z = 0.0, pitch_y = 0.0, roll_x = 0.0;
  double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll_x = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1.0) {
    pitch_y = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch_y = std::asin(sinp);
  }

  double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw_z = std::atan2(siny_cosp, cosy_cosp);

  yaw_z = std::clamp(yaw_z * scale[0], -zyxLimits[0], zyxLimits[0]);
  pitch_y = std::clamp(pitch_y * scale[1], -zyxLimits[1], zyxLimits[1]);
  roll_x = std::clamp(roll_x * scale[2], -zyxLimits[2], zyxLimits[2]);

  Eigen::AngleAxisd rollAngle(roll_x, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch_y, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw_z, Eigen::Vector3d::UnitZ());
  return (yawAngle * pitchAngle * rollAngle).normalized();
}

// ============ FHAN tracking differentiator (from leju_utils) ============
namespace detail {
inline double sign(double x) {
  const double eps = 1e-6;
  return (x - eps < 0 && x + eps > 0) ? 0.0 : (x >= eps ? 1.0 : -1.0);
}
inline double fsg(double x, double d) { return (sign(x + d) - sign(x - d)) / 2; }
inline double sat(double x, double delta) {
  return (x - delta < 0 && x + delta > 0) ? x : (x >= delta ? delta : -delta);
}
inline double fhan(double x1, double x2, double r, double h0) {
  double d = r * h0 * h0;
  double a0 = h0 * x2;
  double y = x1 + a0;
  double a1 = std::sqrt(d * (d + 8.0 * std::abs(y)));
  double a2 = a0 + sign(y) * (a1 - d) / 2;
  double a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
  return (0 - r) * (a / d) * fsg(a, d) - r * sign(a) * (1 - fsg(a, d));
}
}  // namespace detail

inline void fhanStepForward(double& x1,
                            double& x2,
                            double x1Ref,
                            double r,
                            double h,
                            double h0) {
  double fh = detail::fhan((x1 - x1Ref), x2, r, h0);
  x1 = x1 + h * x2;
  x2 = x2 + h * fh;
}

inline double quaternionAngleRad(const Eigen::Quaterniond& qIn) {
  const Eigen::Quaterniond q = qIn.normalized();
  const double wAbs = std::min(1.0, std::max(0.0, std::abs(q.w())));
  return 2.0 * std::acos(wAbs);
}

}  // namespace ik
}  // namespace leju
