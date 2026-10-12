#pragma once

#include <mutex>
#include <string>

// DDS
#include <dds/dds.hpp>
#include "lejusdk-dds-idl/Float64Types.hpp"
#include "lejusdk-dds-idl/StringData.hpp"
#include "lejusdk-topic-pubsub/topic_names.h"
#include "lejusdk-topic-pubsub/topic_publisher.hpp"
#include "lejusdk-topic-pubsub/topic_subscriber.hpp"

namespace leju {

class ControllerManager;

namespace runtime {

/**
 * @brief 倒地起身命令字符串
 *
 * DDS /rt/fall_stand_command 的 data 字段期望值（对齐研杨 FallStandCommand service，
 * RESET 在研杨 joy 侧是死代码，不实现）：
 *   PREPARE   FALL_DOWN(瘫软) -> 插值到起身轨迹首帧（INTERPOLATING -> READY_FOR_STAND_UP）
 *   STAND_UP  READY_FOR_STAND_UP -> 起播起身轨迹（STAND_UP -> STANDING）
 * 进入/离开 mimic_fall_stand 不走本通道：由调度层（手柄 Scheduler 或外部 DDS 客户端）
 * requestSwitch；本类只做阶段门控命令与状态发布。
 */
struct FallStandCommand {
  static constexpr const char* kPrepare = "PREPARE";
  static constexpr const char* kStandUp = "STAND_UP";
};

/**
 * @brief 倒地起身协调器（运控层，零调度）
 *
 * 对齐研杨 FallStandController 的独立命令/状态通道（独立于搬运模式通道）：
 * - 订阅 /rt/fall_stand_command（PREPARE/STAND_UP），tick() 按阶段门控消费：
 *   PREPARE 仅 FALL_DOWN(0，瘫软) 接受 -> prepareToMotionStart；
 *   STAND_UP 仅 READY_FOR_STAND_UP(2，首帧保持) 接受 -> startMotion。
 *   命令时机由调度层掌握，运控层不做任何等待/编排。
 * - 发布 /rt/fall_stand_state（0~4，仅控制器激活期间发布，按 kHeartbeatPeriod 心跳）。
 * - 起身完成后不再自动切 amp：回切时机由 TransportFallStandScheduler 或外部客户端编排。
 *
 * 线程安全：DDS 回调在独立线程写 pending_command_（mutex 保护）；
 * tick() 在主控制线程由 ControlLoop 调用。
 */
class FallStandCoordinator {
 public:
  FallStandCoordinator() = default;
  ~FallStandCoordinator() = default;

  /**
   * @brief 初始化 DDS 发布/订阅
   * @param participant 共享 DDS participant（由组装层创建并保证生命周期）
   * @return 是否成功
   */
  bool initialize(dds::domain::DomainParticipant& participant);

  /**
   * @brief 每控制周期调用：消费命令 + 发布阶段
   * @param controller_manager 控制器管理器（非拥有）
   * @param control_output_allowed Lifecycle 是否放行控制输出
   *        （Running 或 START 前独立起身；状态发布不受限）
   * @param now 当前时间戳 [s]
   */
  void tick(ControllerManager& controller_manager, bool control_output_allowed,
            double now);

  /// 进程内命令入口：校验后缓存为 pending，tick 按阶段门控消费
  void sendCommand(const std::string& command) { onCommand(command); }

  /// 取消调度链时清理尚未被 tick 消费的命令。
  void clearPendingCommand();

  /// 倒地起身阶段；-1 表示 mimic_fall_stand 未激活
  int fallStandState() const { return last_fs_state_; }

 private:
  void onCommand(const std::string& command);
  std::string takePendingCommand();
  void publishState(int state, double now);

  // DDS
  std::unique_ptr<leju::dds_common::TopicPublisher<leju::msgs::Float64>> state_pub_;
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::StringData>> command_sub_;

  std::mutex pending_mutex_;
  std::string pending_command_;  ///< 待处理命令（多条到达时 last wins）

  int last_published_state_ = -1;
  double last_publish_time_ = -1.0;

  int last_fs_state_ = -1;
};

}  // namespace runtime
}  // namespace leju
