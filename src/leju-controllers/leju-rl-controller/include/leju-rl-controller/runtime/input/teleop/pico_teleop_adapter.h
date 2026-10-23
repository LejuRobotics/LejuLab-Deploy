/**
 * @file pico_teleop_adapter.h
 * @brief Pico VR 输入适配器（配置驱动组合键）
 *
 * 直接订阅 DDS /rt/pico/joysticks（leju::msgs::QuestJoysticks 类型，由
 * leju_pico_gmr 前端发布），复用 Quest 的数据布局与组合键框架。
 * 不依赖 lejusdk-vr 的 VR API。
 */

#pragma once

#include <atomic>
#include <memory>

#include "leju-rl-controller/runtime/input/teleop/teleop_adapter_base.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding.hpp"
#include "lejusdk-vr/data_types.h"

namespace leju {
namespace runtime {

class TriggerBuffer;

// 复用 Quest 的数据结构（字段布局一致）
using PicoJoystickData = vr::QuestJoystickData;

/**
 * @brief Pico VR 输入适配器
 *
 * 数据流：leju_pico_gmr (Python) → DDS /rt/pico/joysticks →
 *        PicoTeleopAdapter → CommandBuffer / TriggerBuffer
 *
 * 校准（RT+B / LT+B）在前端内部处理；
 * RT+Y / RT+X：前端做 yaw 连续性，本适配器经 HeftStreamFreezeGate 通知
 * C++ LiveHeftReferenceProvider 冻结/恢复消费。
 * 本适配器另负责摇杆速度、pico_bindings、LT/RT 灵巧手整体抓握，
 * 以及 LT+Y / LT+X 触发 HEFT policy 50Hz CSV 录制。
 */
class PicoTeleopAdapter
    : public TeleopAdapterBase<PicoTeleopAdapter, PicoJoystickData> {
 public:
  explicit PicoTeleopAdapter(TriggerBuffer* trigger_buffer = nullptr);
  ~PicoTeleopAdapter();

  PicoTeleopAdapter(const PicoTeleopAdapter&) = delete;
  PicoTeleopAdapter& operator=(const PicoTeleopAdapter&) = delete;

  /// 初始化并订阅 DDS Pico 手柄数据
  bool initialize();

  /// 停止订阅
  void shutdown();

  bool isInitialized() const;

 private:
  friend class TeleopAdapterBase<PicoTeleopAdapter, PicoJoystickData>;

  // ==================== InputSource 接口 ====================
  InputPriority getPriority() const override { return InputPriority::kPico; }
  const char* getName() const override { return "Pico"; }

  // ==================== CRTP 钩子 ====================
  const DeviceBindingConfig& getDeviceConfig() const {
    return binding_config_.getPicoConfig();
  }

  ComboKey detectCurrentComboImpl(const PicoJoystickData& data) const;

  void processVelocityImpl(const PicoJoystickData& data,
                           CommandBuffer& buffer,
                           double current_time);

  /// 组合变化即触发（支持按住 RG 再按 A 的时序）
  bool shouldTriggerCombo(const ComboKey& prev, const ComboKey& current) const;

  // ==================== 内部 ====================
  void onPicoData(const PicoJoystickData& data);

  /// LT/RT 整体抓握；LG/RG 侧扳机驱动左/右 ThumbAux（index 1，50%）。
  /// 每帧 writeHandTarget，由 ControlLogic 100Hz 重发 /rt/hand_cmd。
  void handleHandGripFromTriggers(const PicoJoystickData& data);

  void handleCsvRecordCombos(const PicoJoystickData& data);
  void handleStreamFreezeCombos(const PicoJoystickData& data);

  std::atomic<bool> running_{false};

  /// 前端占用组合按下时，对应手保持上一有效目标。
  double last_left_hand_cmd_ = 0.0;
  double last_right_hand_cmd_ = 0.0;
  bool last_left_thumb_curl_ = false;
  bool last_right_thumb_curl_ = false;
  bool prev_left_y_ = false;
  bool prev_left_x_ = false;

  // DDS 实体（pimpl 隔离 dds 头依赖）
  struct DdsImpl;
  std::unique_ptr<DdsImpl> dds_;
};

}  // namespace runtime
}  // namespace leju
