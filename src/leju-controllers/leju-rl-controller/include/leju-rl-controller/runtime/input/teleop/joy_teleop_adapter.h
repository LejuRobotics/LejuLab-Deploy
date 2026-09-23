/**
 * @file joy_teleop_adapter.h
 * @brief Joy 手柄输入适配器（配置驱动组合键）
 *
 * 将原始 JoyData 转换为 CommandBuffer，支持配置驱动的组合键绑定。
 * 内部直接订阅 SDK 手柄数据，对外提供统一的初始化和关闭接口。
 *
 * 配置示例（YAML）：
 * @code{.yaml}
 * joy_bindings:
 *   - combo: [LB, X]
 *     type: controller_action
 *     controller_id: dance_controller
 *     action_id: dance_a
 *   - combo: [RB, A]
 *     type: arm_mode
 *     arm_mode: auto
 * @endcode
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "leju-rl-controller/runtime/input/teleop/teleop_adapter_base.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding.hpp"
#include "lejusdk-lowlevel/data_types.h"

namespace leju {

namespace runtime {

// 前向声明
class TriggerBuffer;

/**
 * @brief Joy 手柄输入适配器
 *
 * 继承自 TeleopAdapterBase，实现 Joy 手柄特定的输入处理。
 * 采用主动订阅 SDK 模式，内部管理手柄数据订阅生命周期。
 */
class JoyTeleopAdapter : public TeleopAdapterBase<JoyTeleopAdapter, JoyData> {
public:
  /**
   * @brief 构造函数
   * @param trigger_buffer 触发器缓冲区指针（非拥有，用于输出离散事件）
   */
  explicit JoyTeleopAdapter(TriggerBuffer* trigger_buffer);
  ~JoyTeleopAdapter();

  // 禁止拷贝
  JoyTeleopAdapter(const JoyTeleopAdapter&) = delete;
  JoyTeleopAdapter& operator=(const JoyTeleopAdapter&) = delete;

  // 允许移动
  JoyTeleopAdapter(JoyTeleopAdapter&&) noexcept = default;
  JoyTeleopAdapter& operator=(JoyTeleopAdapter&&) noexcept = default;

  /**
   * @brief 初始化并订阅手柄数据
   * @return 是否成功
   */
  bool initialize();

  /**
   * @brief 关闭并停止手柄数据订阅
   */
  void shutdown();

  /**
   * @brief 检查是否已初始化
   */
  bool isInitialized() const;

private:
  friend class TeleopAdapterBase<JoyTeleopAdapter, JoyData>;

  // ========================================================================
  // InputSource 接口实现
  // ========================================================================

  InputPriority getPriority() const override {
    return InputPriority::kJoy;
  }

  const char* getName() const override {
    return "Joy";
  }

  // ========================================================================
  // CRTP 钩子方法实现
  // ========================================================================

  /**
   * @brief 获取设备绑定配置（Joy 配置）
   * @return Joy 设备绑定配置
   */
  const DeviceBindingConfig& getDeviceConfig() const {
    return binding_config_.getJoyConfig();
  }

  /**
   * @brief 识别当前组合键
   * @param joy 当前帧手柄数据
   * @return 当前按下的组合键
   */
  ComboKey detectCurrentComboImpl(const JoyData& joy) const;

  /**
   * @brief 处理摇杆 → 速度指令（写入 CommandBuffer）
   * @param joy 手柄数据
   * @param buffer 指令缓冲区
   * @param current_time 当前时间戳 [s]
   */
  void processVelocityImpl(const JoyData& joy, CommandBuffer& buffer, double current_time);
  void processAmpHandPostureAxis(float left_y, float left_x, float right_x,
                                 float right_y, double max_linear_x_forward,
                                 double max_linear_x_backward,
                                 MotionCommand& cmd);
  void handleCmdVelLineXLimitDpad(const JoyData& joy,
                                  const JoyData::Buttons& prev);
  double getEffectiveMaxLinearXForward() const;
  double getEffectiveMaxLinearXBackward() const;
  double resolveMaxLinearX(double signed_axis) const;
  double getCmdVelLineXLimitForLevel(int level) const;

  // ========================================================================
  // 内部方法
  // ========================================================================

  /**
   * @brief 处理 Joy 数据回调
   * @param joy 当前帧手柄数据
   * @param prev 上一帧手柄按钮状态
   */
  void onJoyData(const JoyData& joy, const JoyData::Buttons& prev);

  /**
   * @brief 处理系统级按钮（START/BACK）
   * @param joy 当前帧手柄数据
   * @param prev 上一帧按钮状态
   * @param out_triggers 动作触发器列表
   */
  void processSystemButtons(const JoyData& joy,
                            const JoyData::Buttons& prev,
                            std::vector<ActionTrigger>& out_triggers);

  /**
   * @brief 判断当前帧是否处于「走路」状态（遥杆或方向键产生运动）
   *
   * 仅依据原始输入轴/方向键，与输入屏蔽状态解耦，供 block_when_walking 门控使用。
   */
  bool isWalking(const JoyData& joy) const;

  /**
   * @brief 处理组合键边沿，按 press/release 与走路门控产出触发器
   * @param prev_combo 上一帧组合键
   * @param current_combo 当前帧组合键
   * @param walking 当前是否走路
   * @param out_triggers 输出触发器
   */
  void processComboEdges(const ComboKey& prev_combo,
                         const ComboKey& current_combo,
                         bool walking,
                         std::vector<ActionTrigger>& out_triggers);

  /**
   * @brief 拦截并消费 SetInputMask 触发器，更新内部屏蔽状态
   *
   * SetInputMask 为 adapter 内部语义（屏蔽遥杆/方向键），不下发到 TriggerBuffer。
   * @param triggers 触发器列表（原地移除已消费的 SetInputMask）
   */
  void consumeInputMaskTriggers(std::vector<ActionTrigger>& triggers);

  /// START 首次入场或切换到 AMP 时启动真实 Joy 帧回中保护。
  void armVelocityNeutralGuard(const std::vector<ActionTrigger>& triggers);

  /// 回中保护激活时更新真实帧计数；返回 true 表示本帧速度必须清零。
  bool shouldHoldVelocityForNeutral(const JoyData& joy);

  /**
   * @brief 长按绑定检测（hold_duration > 0 的绑定）
   *
   * 组合键持续匹配满 hold_duration 秒触发一次；组合键变化即重置计时。
   * @param current_combo 当前帧组合键
   * @param walking 当前是否走路
   * @param current_time 当前时间戳 [s]
   * @param out_triggers 输出触发器
   */
  void processHoldBinding(const ComboKey& current_combo,
                          bool walking,
                          double current_time,
                          std::vector<ActionTrigger>& out_triggers);

  /**
   * @brief LB+B 手臂控制模式 toggle（走路摆臂 auto ↔ 固定 keep_pose）
   *
   * 需要交替状态，无法用静态 yaml 绑定表达，故由 adapter 内部处理，
   * 复用已实现的 SetArmMode 下游接口产出触发器。
   * @param current_combo 当前帧组合键
   * @param out_triggers 动作触发器列表
   */
  void handleArmModeToggle(const ComboKey& current_combo,
                           std::vector<ActionTrigger>& out_triggers);

  /**
  /**
   * @brief LB+Y 手臂模式轮转（auto → external → keep_pose → auto 循环）
   *
   * 组合键无法用静态 yaml 绑定表达，由 adapter 内部处理，
   * 复用已实现的 SetArmMode 下游接口产出触发器。
   * @param current_combo 当前帧组合键
   * @param out_triggers 动作触发器列表
   */
  void handleArmUnlock(const ComboKey& current_combo,
                       std::vector<ActionTrigger>& out_triggers);

  /**
   * @brief LB+X 灵巧手抓握 toggle（切换6指全闭合 ↔ 全张开）
   *
   * 组合键无法用静态 yaml 绑定表达，由 adapter 内部直接 publishHandCmd。
   * @param current_combo 当前帧组合键
   */
  void handleHandGripToggle(const ComboKey& current_combo);

private:
  // SDK 订阅状态
  std::atomic<bool> running_{false};        ///< 是否正在运行
  JoyData::Buttons prev_buttons_{};         ///< 上一帧按钮状态（用于边缘检测）
  bool input_masked_ = false;               ///< 是否屏蔽遥杆与方向键输入（LB+A 切换）
  bool posture_control_mode_ = false;       ///< amp_hand 右摇杆姿态模式
  int cmd_vel_line_x_limit_level_ = 1;      ///< 0=low, 1=default, 2=up
  bool hand_grip_toggled_ = false;          ///< LB+X 灵巧手抓握 toggle 状态
  std::atomic<bool> hand_presence_resolved_{false};  ///< 是否已收到首帧灵巧手状态
  std::atomic<bool> dexterous_hands_available_{false};  ///< 双侧灵巧手是否有效
  bool rt_was_active_ = false;              ///< 上一帧 RT 是否按下（上升沿检测）
  double frozen_squat_cmd_ = 0.0;           ///< RT 按住期间冻结的下蹲高度命令
  double last_published_angular_z_ = 0.0;   ///< 上一帧 Adapter 下发的 angular_z
  bool velocity_neutral_guard_active_ = false;  ///< 是否等待真实 Joy 帧回中
  bool start_neutral_guard_consumed_ = false;   ///< 首次 START 已启动过保护
  bool neutral_guard_nonzero_logged_ = false;   ///< 本次等待已记录非零轴
  int velocity_neutral_frames_ = 0;             ///< 连续回中的真实 Joy 帧数
  // 长按绑定状态（仅 joy 回调线程访问）
  std::string hold_combo_key_;              ///< 正在计时的长按组合 key（空=无）
  double hold_start_time_ = 0.0;            ///< 长按计时起点 [s]
  bool hold_fired_ = false;                 ///< 本轮长按是否已触发
};

} // namespace runtime
} // namespace leju
