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
   * @brief 获取首帧的关节位置（不依赖 current_frame_）
   *
   * 用于查询 motion 的"起始姿态"，例如控制器切换时计算插值目标。
   * 与 getJointPos() 不同：本方法始终返回第一帧，即使 motion 已播放完毕。
   */
  array_t getFirstFrameJointPos() const;

  /**
   * @brief 获取当前帧的关节速度（末帧保持时返回零速度）
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

  /// @brief 获取相对当前帧指定偏移处的数据（越界时钳制到首/末帧）
  array_t getJointPosAtOffset(int frame_offset) const;
  /// @brief 获取偏移帧速度；钳制到末帧时返回零速度（旧静态保持语义）
  array_t getJointVelAtOffset(int frame_offset) const;
  /// @brief 获取 CSV 中记录的偏移帧速度；钳制到末帧时保留末帧记录值
  array_t getRecordedJointVelAtOffset(int frame_offset) const;
  Eigen::Vector3d getBodyPosAtOffset(int frame_offset) const;
  Eigen::Vector3d getBodyLinVelAtOffset(int frame_offset,
                                        double frame_dt) const;
  /// @brief 由相邻姿态估计偏移帧角速度；末帧使用后向差分
  Eigen::Vector3d getBodyAngVelAtOffset(int frame_offset,
                                        double frame_dt) const;
  Eigen::Quaterniond getBodyQuatAtOffset(int frame_offset) const;

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

  array_t getFrameJointPos(int frame) const;
  array_t getFrameJointVel(int frame) const;
  Eigen::Vector3d getFrameBodyPos(int frame) const;
  Eigen::Quaterniond getFrameBodyQuat(int frame) const;

 private:
  bool parseHeader(const std::string& header_line, char delimiter);
  double quaternionToYaw(const Eigen::Quaterniond& q) const;

  int clampFrame(int frame) const {
    if (frame < 0) return 0;
    if (frame >= num_frames_) return num_frames_ - 1;
    return frame;
  }

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
