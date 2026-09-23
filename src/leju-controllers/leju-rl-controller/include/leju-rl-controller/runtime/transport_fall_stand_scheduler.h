#pragma once

#include <functional>
#include <string>

namespace leju {

class ControllerManager;
struct ImuData;

namespace runtime {

class TransportModeCoordinator;
class FallStandCoordinator;

/// 手柄搬运/倒地起身调度：把手柄 TransportFallStand 事件展开成 coordinator 原语序列。
/// 仅服务手柄路径；外部 DDS 直接打 coordinator 话题自行编排。
class TransportFallStandScheduler {
 public:
  TransportFallStandScheduler() = default;

  void bind(TransportModeCoordinator* transport, FallStandCoordinator* fall_stand,
            ControllerManager* controller_manager) {
    transport_ = transport;
    fall_stand_ = fall_stand;
    controller_manager_ = controller_manager;
  }

  /// 复用组装层的 /rt/audio_play_file 发布器，避免调度层持有 DDS 实体。
  void setAudioPlayer(std::function<void(const std::string&)> cb) {
    audio_player_ = std::move(cb);
  }

  /// 注入灵巧手开合命令；true=双手闭合，false=双手张开。
  /// 返回 false 只告警，不阻断搬运状态机。
  void setHandGripSetter(std::function<bool(bool)> cb) {
    hand_grip_setter_ = std::move(cb);
  }

  /// 注入握拳前的拇指回位命令：双手 [0,0,0,0,0,0]。
  void setHandPreGripSetter(std::function<bool()> cb) {
    hand_pre_grip_setter_ = std::move(cb);
  }

  /// 注入四指闭合命令：拇指保持 0，其余四指到 100。
  void setFourFingerGripSetter(std::function<bool()> cb) {
    four_finger_grip_setter_ = std::move(cb);
  }

  /// 注入本会话灵巧手存在查询。未注入时保持历史行为，视为存在。
  void setHandsAvailableChecker(std::function<bool()> cb) {
    hands_available_checker_ = std::move(cb);
  }

  /// 单个手柄调度事件；同周期内可能连续到达多个，按到达顺序处理
  void onEvent(const std::string& name, const ImuData& imu, double now);

  /// 每控制周期调用：搬运状态边沿驱动 + 保护窗自解 + 起身链重发 + 退出编排
  void tick(const ImuData& imu, double now);

 private:
  // 搬运
  void onTransportEdge(int ts, double now);
  void handleTransportEvent(const std::string& event, const ImuData& imu, int ts);
  void setHeadPitch(double pitch_rad);
  void playVoice(const std::string& file_name) const;
  bool areHandsAvailable() const;
  bool setHandsClosed(bool closed) const;
  bool setHandsPreGrip() const;
  bool setFourFingersClosed() const;
  void beginHandGripSequence(double now);
  void tickHandGripSequencer(double now);

  // 起身
  void onFallStandEdge(int fs, double now);
  void driveHardChain(const ImuData& imu, int ts, int fs, double now);
  void driveSoftChain(const ImuData& imu, int ts, int fs, double now);
  void onStandUpEvent(const ImuData& imu, int ts, int fs, double now);
  void resetChains();
  void resetSoloChain();

  // 退出编排
  // 需切控制器：切 amp → 延时 → 等插值完 → arm auto → EXIT
  // READY 取消（已在 amp）：arm auto → EXIT
  enum class ExitStage {
    kIdle,
    kPostSwitchDelay,  // 已 requestSwitch(amp)，等 switchDuration+slack 后再查插值
    kWaitBlend,        // amp 且非 transitioning → setArmMode(auto)
    kSendExit,         // 仍 HANDING_OVER 时发 EXIT
  };
  void beginNormalExit(double now);
  void beginFallPathExit(double now);
  void requestAmpAndDelay(double now);
  void tickExitSequencer(double now);

  TransportModeCoordinator* transport_ = nullptr;
  FallStandCoordinator* fall_stand_ = nullptr;
  ControllerManager* controller_manager_ = nullptr;
  std::function<void(const std::string&)> audio_player_;
  std::function<bool(bool)> hand_grip_setter_;
  std::function<bool()> hand_pre_grip_setter_;
  std::function<bool()> four_finger_grip_setter_;
  std::function<bool()> hands_available_checker_;

  // 搬运边沿
  int prev_transport_state_ = 0;
  bool normal_exit_pending_ = false;
  bool exit_from_ready_ = false;  // exit 来自 READY（控制器仍是 amp）
  // 待播退出语音（空=无）：事件里登记，统一在 HandingOver 发 kExit 时播放
  std::string pending_exit_voice_;
  enum class HandGripStage {
    kIdle,
    kWaitFourFingers,
    kWaitThumbs,
  };
  HandGripStage hand_grip_stage_ = HandGripStage::kIdle;
  double hand_grip_stage_at_ = 0.0;

  // 保护窗
  bool protection_active_ = false;
  double protection_start_ = 0.0;
  double protection_deadline_ = 0.0;

  // 起身边沿
  int prev_fall_stand_state_ = 0;
  bool saw_standup_progress_ = false;

  // 硬起身链
  bool hard_chain_active_ = false;
  bool hard_chain_prepare_done_ = false;
  bool hard_chain_standup_sent_ = false;
  double hard_next_drive_ = 0.0;

  // 软起身链
  int soft_stage_ = 0;
  double soft_next_drive_ = 0.0;

  // 独立两按起身已发起；仅用于起身完成后的退出编排。
  bool solo_chain_active_ = false;

  // 退出编排
  ExitStage exit_stage_ = ExitStage::kIdle;
  double stage_until_ = 0.0;
  double next_wait_log_ = 0.0;
};

}  // namespace runtime
}  // namespace leju
