#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "leju-rl-controller/controllers/generic_rl_controller.h"
#include "leju-rl-controller/motion/heft_reference_source.h"
#include "leju-rl-controller/utils/obs_ring_buffer.h"

namespace leju {

class LiveHeftReferenceProvider;
class HeftPolicyCsvRecorder;

/// HEFT student 跟踪控制器
class HeftRLController : public GenericRLController {
 public:
  HeftRLController(const RobotVersion& version, const std::string& name);
  ~HeftRLController() override;

  bool initialize() override;
  void reset() override;

  /// 切走控制器时停录并落盘（后台线程写），避免两段时间被串缝拼接
  void pause() override;

  /// 激活就绪：live 源需已校准/缓冲/新鲜；csv 源恒可用
  bool isActivationReady(std::string& reason) const override;

  /// 数据流退化时请求回退（live 源丢流 -> 回退 amp）
  bool requestsFallback(std::string& to) const override;

  /// 是否 live 参考模式（Pico 流）
  bool isLiveReference() const { return reference_source_mode_ == "live"; }

 protected:
  bool loadConfig(const std::string& config_path) override;
  bool loadPolicy(const std::string& policy_path) override;
  void initObsHistory() override;
  void resetObsHistory() override;
  void computeObservation() override;
  bool inferActions(const array_t& observation,
                    array_t& inferred_actions) override;
  array_t getActionsSnapshot() const override;
  array_t getMotorActionsSnapshot() const override;
  void updateRobotCmd(RobotCmd& cmd) override;
  void initializeDummyWorldYaw() override;

 private:
  void primeHistory();
  void pushCurrentState();
  void resetActionFilter();
  Eigen::Quaterniond getAlignedBaseQuat() const;
  /// freejoint（去掉腰偏）+ dummy_world 航向对齐；live 参考为 base_link 时使用。
  Eigen::Quaterniond getFreejointAlignedQuat() const;
  int expectedObsDim() const;
  array_t getHeftJointPos() const;
  void initializeDummyWorldYawFromLive();

  std::vector<int> future_steps_;
  std::vector<int> history_taps_;
  int prev_action_steps_ = 8;
  double boot_indicator_max_ = 25.0;
  double action_clip_ = 10.0;
  double action_alpha_ = 0.9;  // applied = (1-a)*applied + a*raw
  std::string output_name_ = "action";
  int expected_obs_dim_ = 0;  // >0 时与 computed dim 交叉检查

  double boot_counter_ = 25.0;
  bool history_primed_ = false;
  bool action_filter_initialized_ = false;
  mutable std::mutex action_filter_mutex_;  ///< 保护 filtered_actions_
  array_t filtered_actions_;  ///< 仅电机目标；actions_/prev_action 保持 raw

  // ==================== 参考源 ====================
  std::string reference_source_mode_ = "csv";  ///< csv | live
  std::string fallback_controller_ = "amp";    ///< live 丢流回退目标
  std::unique_ptr<HeftReferenceSource> reference_source_;
  LiveHeftReferenceProvider* live_provider_ = nullptr;  ///< 非拥有（同上对象）
  bool live_yaw_initialized_ = false;
  std::unique_ptr<HeftPolicyCsvRecorder> policy_csv_recorder_;

  ObsRingBuffer angvel_buf_;
  ObsRingBuffer projg_buf_;
  ObsRingBuffer jpos_buf_;
  ObsRingBuffer jvel_buf_;
  ObsRingBuffer action_buf_;
};

}  // namespace leju
