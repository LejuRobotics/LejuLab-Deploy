#pragma once

namespace leju::depth {

// Source ROS MuJoCo applies ff_tau plus a PD term with zero position/velocity
// command. This small pure helper is shared by the controller contract tests.
inline double sourceMujocoControl(double actuation, double q, double v,
                                  double kp, double kd) {
  return actuation + kp * (0.0 - q) + kd * (0.0 - v);
}

inline double targetTorqueControl(double actuation) { return actuation; }

inline bool sourceMujocoExecutorEnabled(bool configured, bool action_ready) {
  return configured && action_ready;
}

// RobotCmd::modes only defines CST/CSV/CSP (0/1/2).  Mode 3 is a private
// MuJoCo-only escape hatch and must never be enabled on a real hardware path.
inline bool sourceMujocoExecutorEnabled(bool configured, bool action_ready,
                                        bool mujoco_runtime_authorized) {
  return configured && action_ready && mujoco_runtime_authorized;
}

}  // namespace leju::depth
