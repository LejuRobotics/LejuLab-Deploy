#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "leju-rl-controller/rl/rl_controller_types.h"

namespace leju {

/**
 * @brief Motion 轨迹数据
 *
 * 从 CSV 文件加载 motion 数据，按语义存储：
 * - joint_pos: 关节位置 (num_frames, num_joints)
 * - joint_vel: 关节速度 (num_frames, num_joints)
 * - body_pos:  基座位置 (num_frames, 3)
 * - body_quat: 基座四元数 (num_frames, 4) [w, x, y, z]
 *
 * CSV 列名格式（自动解析）：
 *   body_pos_x, body_pos_y, body_pos_z,
 *   body_quat_w, body_quat_x, body_quat_y, body_quat_z,
 *   joint_pos00, joint_pos01, ..., joint_pos20,
 *   joint_vel_00, joint_vel_01, ..., joint_vel_20
 */
class MotionTrajectory {
 public:
  MotionTrajectory() = default;
  ~MotionTrajectory() = default;

  /**
   * @brief 从 CSV 文件加载
   * @param file_path 文件路径
   * @return 是否加载成功
   */
  bool load(const std::string& file_path);

  /**
   * @brief 获取当前帧的关节位置
   */
  array_t getJointPos() const;

  /**
   * @brief 获取当前帧的关节速度
   */
  array_t getJointVel() const;

  /**
   * @brief 获取当前帧的基座位置
   */
  Eigen::Vector3d getBodyPos() const;

  /**
   * @brief 获取当前帧的基座四元数
   */
  Eigen::Quaterniond getBodyQuat() const;

  /**
   * @brief 获取当前帧的命令（joint_pos + joint_vel 拼接）
   */
  Eigen::VectorXd getCurrentCommand() const;

  /**
   * @brief 前进一帧（到达末尾后停止）
   */
  void next();

  /**
   * @brief 重置到起始帧
   */
  void reset();

  /**
   * @brief 是否还有下一帧
   */
  bool hasNext() const { return current_frame_ < num_frames_ - 1; }

  /**
   * @brief 获取当前帧索引
   */
  int getCurrentFrame() const { return current_frame_; }

  /**
   * @brief 获取总帧数
   */
  int getNumFrames() const { return num_frames_; }


  /**
   * @brief 是否已加载
   */
  bool isLoaded() const { return loaded_; }

  /**
   * @brief 获取参考偏航角（第一帧的 yaw）
   */
  double getReferenceYaw() const { return reference_yaw_; }

 private:
  bool parseHeader(const std::string& header_line, char delimiter);
  double quaternionToYaw(const Eigen::Quaterniond& q) const;

  bool loaded_ = false;
  int num_frames_ = 0;
  int num_joints_ = 0;
  int current_frame_ = 0;
  double reference_yaw_ = 0.0;

  // 按语义存储的数据
  Eigen::MatrixXd joint_pos_;   // (num_frames, num_joints)
  Eigen::MatrixXd joint_vel_;   // (num_frames, num_joints)
  Eigen::MatrixXd body_pos_;    // (num_frames, 3)
  Eigen::MatrixXd body_quat_;   // (num_frames, 4) [w, x, y, z]
};

}  // namespace leju
