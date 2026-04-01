#pragma once

#include <atomic>
#include <functional>
#include <mutex>

#include "lejusdk-lowlevel/leju_sdk.h"

namespace leju {

/**
 * @brief 机器人数据管理
 *
 * 负责订阅 lejusdk 数据并提供统一访问接口：
 * - 订阅 RobotState、ImuData
 * - 线程安全缓存
 * - 提供给 ControllerManager 使用
 */
class RobotData {
 public:
  RobotData() = default;
  ~RobotData() = default;

  // 禁止拷贝
  RobotData(const RobotData&) = delete;
  RobotData& operator=(const RobotData&) = delete;

  // ==================== 初始化 ====================
  /**
   * @brief 初始化并开始订阅数据
   * @return 是否初始化成功
   */
  bool initialize();

  /**
   * @brief 停止订阅
   */
  void shutdown();

  // ==================== 数据获取 ====================
  /**
   * @brief 获取最新的机器人状态
   * @param state 输出参数
   * @return 是否获取成功（数据是否有效）
   */
  bool getRobotState(RobotState& state) const;

  /**
   * @brief 获取最新的 IMU 数据
   * @param imu 输出参数
   * @return 是否获取成功（数据是否有效）
   */
  bool getImuData(ImuData& imu) const;

  // ==================== 状态查询 ====================
  /**
   * @brief 检查数据是否就绪
   * @return 是否已收到有效数据
   */
  bool isDataReady() const;

  /**
   * @brief 检查 RobotState 是否有效
   */
  bool hasRobotState() const { return robot_state_valid_.load(); }

  /**
   * @brief 检查 ImuData 是否有效
   */
  bool hasImuData() const { return imu_data_valid_.load(); }

  /**
   * @brief 获取硬件状态
   * @return 当前硬件状态
   */
  HardwareState getHardwareState() const;

  /**
   * @brief 检查硬件状态是否有效
   */
  bool hasHardwareState() const { return hw_state_valid_.load(); }

  /**
   * @brief 检查硬件是否就绪（READY_OK 状态）
   */
  bool isHardwareReady() const;

 private:
  // ==================== 回调函数 ====================
  void onRobotState(const RobotStateConstPtr& state);
  void onImuData(const ImuDataConstPtr& imu);
  void onHardwareState(const StringDataConstPtr& hw_state);

  // ==================== 成员变量 ====================
  mutable std::mutex state_mutex_;
  mutable std::mutex imu_mutex_;
  mutable std::mutex hw_state_mutex_;

  RobotState robot_state_;
  ImuData imu_data_;
  HardwareState hw_state_ = HardwareState::UNKNOWN;

  std::atomic<bool> robot_state_valid_{false};
  std::atomic<bool> imu_data_valid_{false};
  std::atomic<bool> hw_state_valid_{false};

  bool initialized_ = false;
};

}  // namespace leju
