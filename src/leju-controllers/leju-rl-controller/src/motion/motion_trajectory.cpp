#include "leju-rl-controller/motion/motion_trajectory.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

namespace leju {

bool MotionTrajectory::load(const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "[ERROR] [MotionTrajectory::load] "
              << "Failed to open file: " << file_path << std::endl;
    return false;
  }

  // 读取第一行（表头）
  std::string header_line;
  if (!std::getline(file, header_line)) {
    std::cerr << "[ERROR] [MotionTrajectory::load] "
              << "Failed to read header line" << std::endl;
    return false;
  }

  // 自动检测分隔符：优先制表符，否则用逗号
  char delimiter = (header_line.find('\t') != std::string::npos) ? '\t' : ',';

  // 解析表头，获取列索引
  if (!parseHeader(header_line, delimiter)) {
    return false;
  }

  // 读取所有数据行
  std::vector<std::vector<double>> all_data;
  std::string line;
  int line_num = 1;

  while (std::getline(file, line)) {
    line_num++;
    if (line.empty()) continue;

    std::vector<double> row;
    std::stringstream ss(line);
    std::string value;

    while (std::getline(ss, value, delimiter)) {
      if (value.empty()) continue;
      try {
        row.push_back(std::stod(value));
      } catch (const std::exception& e) {
        std::cerr << "[ERROR] [MotionTrajectory::load] "
                  << "Failed to parse value '" << value << "' at line " << line_num
                  << std::endl;
        return false;
      }
    }

    if (!row.empty()) {
      all_data.push_back(std::move(row));
    }
  }

  if (all_data.empty()) {
    std::cerr << "[ERROR] [MotionTrajectory::load] "
              << "No data rows found" << std::endl;
    return false;
  }

  // 验证列数：body_pos(3) + body_quat(4) + joint_pos(N) + joint_vel(N) = 7 + 2*N
  int num_cols = static_cast<int>(all_data[0].size());
  if ((num_cols - 7) % 2 != 0) {
    std::cerr << "[ERROR] [MotionTrajectory::load] "
              << "Invalid column count: " << num_cols
              << " (expected 7 + 2*num_joints)" << std::endl;
    return false;
  }

  num_joints_ = (num_cols - 7) / 2;
  num_frames_ = static_cast<int>(all_data.size());

  // 分配矩阵
  body_pos_.resize(num_frames_, 3);
  body_quat_.resize(num_frames_, 4);
  joint_pos_.resize(num_frames_, num_joints_);
  joint_vel_.resize(num_frames_, num_joints_);

  // 填充数据（按语义存储）
  // CSV格式: body_pos(3), body_quat(4), joint_pos(N), joint_vel(N)
  for (int i = 0; i < num_frames_; ++i) {
    const auto& row = all_data[i];
    if (static_cast<int>(row.size()) != num_cols) {
      std::cerr << "[ERROR] [MotionTrajectory::load] "
                << "Row " << i << " has " << row.size()
                << " columns, expected " << num_cols << std::endl;
      return false;
    }

    // body_pos: columns 0-2
    body_pos_(i, 0) = row[0];
    body_pos_(i, 1) = row[1];
    body_pos_(i, 2) = row[2];

    // body_quat: columns 3-6 [w, x, y, z]
    body_quat_(i, 0) = row[3];  // w
    body_quat_(i, 1) = row[4];  // x
    body_quat_(i, 2) = row[5];  // y
    body_quat_(i, 3) = row[6];  // z

    // joint_pos: columns 7 to 7+N-1
    for (int j = 0; j < num_joints_; ++j) {
      joint_pos_(i, j) = row[7 + j];
    }

    // joint_vel: columns 7+N to 7+2*N-1
    for (int j = 0; j < num_joints_; ++j) {
      joint_vel_(i, j) = row[7 + num_joints_ + j];
    }
  }

  // 计算参考偏航角（第一帧的 yaw）
  Eigen::Quaterniond first_quat(body_quat_(0, 0), body_quat_(0, 1),
                                body_quat_(0, 2), body_quat_(0, 3));
  reference_yaw_ = quaternionToYaw(first_quat);

  current_frame_ = 0;
  loaded_ = true;

  std::cout << "[INFO] [MotionTrajectory::load] "
            << "Loaded " << num_frames_ << " frames"
            << ", reference_yaw=" << reference_yaw_ << " rad"
            << std::endl;
  return true;
}

bool MotionTrajectory::parseHeader(const std::string& header_line, char delimiter) {
  // 解析表头，验证格式
  // 预期: body_pos_x, body_pos_y, body_pos_z, body_quat_w, body_quat_x, body_quat_y, body_quat_z,
  //       joint_pos00, ..., joint_vel_00, ...
  std::vector<std::string> columns;
  std::stringstream ss(header_line);
  std::string col;

  while (std::getline(ss, col, delimiter)) {
    if (!col.empty()) {
      columns.push_back(col);
    }
  }

  if (columns.size() < 7) {
    std::cerr << "[ERROR] [MotionTrajectory::parseHeader] "
              << "Header has too few columns: " << columns.size() << std::endl;
    return false;
  }

  // 验证前7列
  const std::vector<std::string> expected_prefix = {
      "body_pos_x", "body_pos_y", "body_pos_z",
      "body_quat_w", "body_quat_x", "body_quat_y", "body_quat_z"};

  for (size_t i = 0; i < expected_prefix.size(); ++i) {
    if (columns[i] != expected_prefix[i]) {
      std::cerr << "[WARN] [MotionTrajectory::parseHeader] "
                << "Column " << i << " is '" << columns[i]
                << "', expected '" << expected_prefix[i] << "'" << std::endl;
      // 不作为错误，仅警告
    }
  }

  return true;
}

double MotionTrajectory::quaternionToYaw(const Eigen::Quaterniond& q) const {
  // 从旋转矩阵第3列提取 yaw（与 kuavo-rl 保持一致）
  Eigen::Matrix3d rot = q.toRotationMatrix();
  return std::atan2(rot(1, 2), rot(0, 2));
}

array_t MotionTrajectory::getJointPos() const {
  if (!loaded_) {
    return array_t();
  }
  return joint_pos_.row(current_frame_);
}

array_t MotionTrajectory::getFirstFrameJointPos() const {
  if (!loaded_ || num_frames_ <= 0) {
    return array_t();
  }
  return joint_pos_.row(0);
}

array_t MotionTrajectory::getJointVel() const {
  if (!loaded_) {
    return array_t();
  }
  return joint_vel_.row(current_frame_);
}

Eigen::Vector3d MotionTrajectory::getBodyPos() const {
  if (!loaded_) {
    return Eigen::Vector3d::Zero();
  }
  return body_pos_.row(current_frame_).transpose();
}

Eigen::Quaterniond MotionTrajectory::getBodyQuat() const {
  if (!loaded_) {
    return Eigen::Quaterniond::Identity();
  }
  // [w, x, y, z]
  return Eigen::Quaterniond(body_quat_(current_frame_, 0),
                            body_quat_(current_frame_, 1),
                            body_quat_(current_frame_, 2),
                            body_quat_(current_frame_, 3));
}

array_t MotionTrajectory::getJointPosAtOffset(int frame_offset) const {
  if (!loaded_) return array_t();
  const int frame = std::clamp(current_frame_ + frame_offset, 0, num_frames_ - 1);
  return joint_pos_.row(frame);
}

array_t MotionTrajectory::getJointVelAtOffset(int frame_offset) const {
  if (!loaded_) return array_t();
  const int frame = std::clamp(current_frame_ + frame_offset, 0, num_frames_ - 1);
  if (frame == num_frames_ - 1) return array_t::Zero(num_joints_);
  return joint_vel_.row(frame);
}

array_t MotionTrajectory::getRecordedJointVelAtOffset(int frame_offset) const {
  if (!loaded_) return array_t();
  const int frame = std::clamp(current_frame_ + frame_offset, 0, num_frames_ - 1);
  return joint_vel_.row(frame);
}

Eigen::Vector3d MotionTrajectory::getBodyPosAtOffset(int frame_offset) const {
  if (!loaded_) return Eigen::Vector3d::Zero();
  const int frame = std::clamp(current_frame_ + frame_offset, 0, num_frames_ - 1);
  return body_pos_.row(frame).transpose();
}

Eigen::Vector3d MotionTrajectory::getBodyLinVelAtOffset(
    int frame_offset, double frame_dt) const {
  if (!loaded_ || num_frames_ <= 1 || frame_dt <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  const int frame = std::clamp(current_frame_ + frame_offset, 0, num_frames_ - 1);
  if (frame == 0) {
    return (body_pos_.row(1) - body_pos_.row(0)).transpose() / frame_dt;
  }
  if (frame == num_frames_ - 1) {
    return (body_pos_.row(frame) - body_pos_.row(frame - 1)).transpose() / frame_dt;
  }
  return (body_pos_.row(frame + 1) - body_pos_.row(frame - 1)).transpose() /
         (2.0 * frame_dt);
}

Eigen::Vector3d MotionTrajectory::getBodyAngVelAtOffset(
    int frame_offset, double frame_dt) const {
  if (!loaded_ || num_frames_ <= 1 || frame_dt <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  const int frame = std::clamp(current_frame_ + frame_offset, 0, num_frames_ - 1);
  const int frame0 = (frame == num_frames_ - 1) ? frame - 1 : frame;
  const int frame1 = frame0 + 1;
  Eigen::Quaterniond q0(body_quat_(frame0, 0), body_quat_(frame0, 1),
                         body_quat_(frame0, 2), body_quat_(frame0, 3));
  Eigen::Quaterniond q1(body_quat_(frame1, 0), body_quat_(frame1, 1),
                         body_quat_(frame1, 2), body_quat_(frame1, 3));
  Eigen::Quaterniond delta = (q0.normalized().inverse() * q1.normalized()).normalized();
  if (delta.w() < 0.0) delta.coeffs() *= -1.0;
  return (2.0 / frame_dt) * delta.vec();
}

Eigen::Quaterniond MotionTrajectory::getBodyQuatAtOffset(int frame_offset) const {
  if (!loaded_) return Eigen::Quaterniond::Identity();
  const int frame = std::clamp(current_frame_ + frame_offset, 0, num_frames_ - 1);
  return Eigen::Quaterniond(body_quat_(frame, 0), body_quat_(frame, 1),
                            body_quat_(frame, 2), body_quat_(frame, 3));
}

Eigen::VectorXd MotionTrajectory::getCurrentCommand() const {
  if (!loaded_) {
    return Eigen::VectorXd();
  }
  // 拼接 joint_pos + joint_vel
  Eigen::VectorXd cmd(num_joints_ * 2);
  cmd.head(num_joints_) = joint_pos_.row(current_frame_);
  cmd.tail(num_joints_) = joint_vel_.row(current_frame_);
  return cmd;
}

void MotionTrajectory::next() {
  if (loaded_ && current_frame_ < num_frames_ - 1) {
    current_frame_++;
  }
}

void MotionTrajectory::reset() {
  current_frame_ = 0;
}

array_t MotionTrajectory::getFrameJointPos(int frame) const {
  if (!loaded_ || num_frames_ <= 0) {
    return array_t();
  }
  return joint_pos_.row(clampFrame(frame));
}

array_t MotionTrajectory::getFrameJointVel(int frame) const {
  if (!loaded_ || num_frames_ <= 0) {
    return array_t();
  }
  return joint_vel_.row(clampFrame(frame));
}

Eigen::Vector3d MotionTrajectory::getFrameBodyPos(int frame) const {
  if (!loaded_ || num_frames_ <= 0) {
    return Eigen::Vector3d::Zero();
  }
  return body_pos_.row(clampFrame(frame)).transpose();
}

Eigen::Quaterniond MotionTrajectory::getFrameBodyQuat(int frame) const {
  if (!loaded_ || num_frames_ <= 0) {
    return Eigen::Quaterniond::Identity();
  }
  const int f = clampFrame(frame);
  // [w, x, y, z]
  return Eigen::Quaterniond(body_quat_(f, 0), body_quat_(f, 1),
                            body_quat_(f, 2), body_quat_(f, 3));
}

}  // namespace leju
