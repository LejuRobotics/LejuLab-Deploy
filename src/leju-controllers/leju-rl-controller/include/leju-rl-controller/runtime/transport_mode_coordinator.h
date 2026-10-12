#pragma once

#include <atomic>
#include <memory>
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
 * @brief 搬运模式状态
 *
 * 与研杨版 TransportModeState 对齐：
 *   0 = INACTIVE      未激活
 *   1 = INTERPOLATING 正在插值到搬运姿态（amp 运控 + 手臂到零位）
 *   2 = READY         已到位，等待 LOCK
 *   3 = ACTIVE        全身位控锁死（hold_pose 控制器），可被搬运
 *   4 = HANDING_OVER  挂起移交态：运控层不做任何动作，
 *                     回切 amp / 手臂复位 / EXIT 时机均由调度层编排
 */
enum class TransportModeState : int {
  kInactive = 0,
  kInterpolating = 1,
  kReady = 2,
  kActive = 3,
  kHandingOver = 4,
};

/**
 * @brief 搬运模式命令字符串
 *
 * DDS /rt/transport_mode_command 的 data 字段期望值（对齐研杨运控层语义：
 * 每条命令 = 合法性检查 + 原子副作用，无任何编排）：
 *   ENTER     INACTIVE -> INTERPOLATING；门控前置（对齐研杨基线）：
 *             接受时同步校验 amp+站立，不满足直接拒绝、不进入 INTERPOLATING；
 *             原子副作用：手臂切 kExternal 并 min-jerk 到零位（时长见 kTransportArmMoveDuration），到位后自动 READY
 *   LOCK      READY -> （pending）-> ACTIVE；门控前置：接受时校验站立 +
 *             hold_pose 可用，不满足直接拒绝；原子副作用：instant 切 hold_pose
 *             （跳过混合，首拍抓传感器反馈位冻住），提交后翻 ACTIVE
 *   HAND_OVER READY/ACTIVE -> HANDING_OVER；零副作用，并取消未完成的 LOCK/FALL_DOWN
 *   FALL_DOWN ACTIVE -> （pending）-> HANDING_OVER；门控前置：接受时校验站立 +
 *             mimic_fall_stand 可用，不满足直接拒绝；原子副作用：
 *             instant 切 mimic_fall_stand（进瘫软，不起播）
 *   EXIT      HANDING_OVER -> INACTIVE；无条件接受（对齐研杨，
 *             回切 amp + 手臂复位由调度层在 EXIT 前完成）
 */
struct TransportModeCommand {
  static constexpr const char* kEnter = "ENTER";
  static constexpr const char* kLock = "LOCK";
  static constexpr const char* kHandOver = "HAND_OVER";
  static constexpr const char* kFallDown = "FALL_DOWN";
  static constexpr const char* kExit = "EXIT";
};

/**
 * @brief 搬运模式协调器（运控层，零调度）
 *
 * 职责（对齐研杨 humanoidController 的搬运命令语义）：
 * - 订阅 /rt/transport_mode_command，命令合法性检查 + 状态机维护
 * - ENTER 门控前置：接受时（DDS 回调线程）同步校验 amp+站立，移动中直接拒绝
 * - 每条命令的原子副作用在 tick() 主控制线程执行（手臂归零、
 *   LOCK 切 hold_pose、FALL_DOWN 切 mimic_fall_stand）
 * - 发布 /rt/transport_mode_state（非 Running 也发布）
 * - 对 ControlLogic 提供门控查询（isInputGating/isFallProtectionSuppressed）
 *
 * 不做：回切 amp、手臂复位、EXIT 时机、一切等待/重试——全部由调度层
 * （TransportFallStandScheduler 或外部 DDS 客户端）编排。
 *
 * 线程安全：DDS 回调在独立线程触发 onCommand()（原子变量/pending 标志）；
 * tick() 在主控制线程由 ControlLoop 调用。
 */
class TransportModeCoordinator {
 public:
  TransportModeCoordinator() = default;
  ~TransportModeCoordinator() = default;

  /**
   * @brief 初始化 DDS 发布/订阅
   * @param participant 共享 DDS participant（由组装层创建并保证生命周期）
   * @return 是否成功
   */
  bool initialize(dds::domain::DomainParticipant& participant);

  /// @brief 注入控制器管理器（非拥有；仅命令门控查询用，其内部方法自带锁）
  void setControllerManager(ControllerManager* controller_manager) {
    controller_manager_ = controller_manager;
  }

  /**
   * @brief 每控制周期调用：发布状态 + 执行 pending 原子副作用
   * @param controller_manager 控制器管理器（非拥有）
   * @param running Lifecycle 是否 Running（副作用仅 Running 时执行；状态发布不受限）
   * @param now 当前时间戳 [s]
   */
  void tick(ControllerManager& controller_manager, bool running, double now);

  TransportModeState state() const {
    return static_cast<TransportModeState>(state_.load());
  }

  /// @brief 搬运模式进行中（非 INACTIVE）：外部输入需被门控（ControlLogic 查询用）
  bool isInputGating() const { return state() != TransportModeState::kInactive; }

  /// @brief ACTIVE（全身锁死、被搬运）期间 IMU 姿态任意，跌倒保护需豁免
  bool isFallProtectionSuppressed() const { return state() == TransportModeState::kActive; }

  /// 进程内命令入口：校验后置 pending，tick 在主控制线程执行副作用
  void sendCommand(const std::string& command) { onCommand(command); }

 private:
  void onCommand(const std::string& command);
  void setState(TransportModeState new_state);
  std::string stateToString(TransportModeState state) const;

  // tick 内部分状态原子副作用
  void handleInterpolating(ControllerManager& controller_manager);
  void handleLockPending(ControllerManager& controller_manager, double now);
  void handleFallDownPending(ControllerManager& controller_manager, double now);

  // DDS
  std::unique_ptr<leju::dds_common::TopicPublisher<leju::msgs::Float64>> state_pub_;
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::StringData>> command_sub_;

  std::atomic<int> state_{static_cast<int>(TransportModeState::kInactive)};
  std::atomic<bool> lock_pending_{false};       ///< LOCK 已接受，待 tick 执行切换
  std::atomic<bool> fall_down_pending_{false};  ///< FALL_DOWN 已接受，待 tick 执行切换

  // 主控制线程内部状态（仅 tick 访问）
  TransportModeState last_state_ = TransportModeState::kInactive;
  bool arm_move_requested_ = false;  ///< INTERPOLATING 手臂归零已下发（只下一次）

  ControllerManager* controller_manager_ = nullptr;  ///< 非拥有；命令门控查询用
};

}  // namespace runtime
}  // namespace leju
