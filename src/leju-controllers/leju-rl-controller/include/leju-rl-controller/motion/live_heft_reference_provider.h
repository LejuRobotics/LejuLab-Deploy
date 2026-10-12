#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "leju-rl-controller/motion/heft_reference_source.h"

namespace leju {

/**
 * @brief 实时 HEFT 参考提供者（Pico/GMR DDS 流）
 *
 * 订阅 /rt/pico/heft_reference（100Hz, leju::msgs::HeftReference），
 * 在 DDS 回调中仅做校验+入队（绝不阻塞 1kHz 控制回路），控制器在
 * policy tick 采样时按"最新帧时刻 - cursor_delay"的延迟游标重采样到
 * 精确的 20ms policy 时间栅格：
 *   - 位置/关节 线性插值
 *   - 四元数 slerp（符号连续）
 *   - 越界处夹取到最旧/最新帧（与 MotionTrajectory::clampFrame 语义一致）
 *
 * 就绪条件：已校准 && 缓冲时间跨度覆盖 [cursor - 8*dt, newest] && 新鲜。
 * 短暂丢包：返回最后一致窗口（绝不清零观测）。
 * 持续丢失（> hold_timeout）：degraded() 置位，管理器平滑回退。
 * 校准 epoch 变化 / 重连：清空缓冲，重新预热。
 *
 * RT+Y 锁定 frozen_frame_。RT+X 等待前端确认 yaw 修正帧已发布后，
 * 再执行原有的 prefill + discontinuity 接入。
 */
class LiveHeftReferenceProvider final : public HeftReferenceSource {
 public:
  struct Config {
    std::string topic = "/rt/pico/heft_reference";
    std::string expected_robot_version = "roban_s17";
    uint32_t expected_schema_version = 1;
    int num_joints = 21;
    double policy_dt = 0.02;          ///< policy 帧周期 [s]
    double cursor_delay = 0.32;       ///< 游标延迟 = 最大 future 偏移 16*20ms [s]
    double history_margin = 0.18;     ///< 额外历史需求 (>= 8*20ms=0.16) [s]
    double fresh_timeout = 0.15;      ///< 最新帧本地接收龄超过则不新鲜 [s]
    double hold_timeout = 0.5;        ///< 不新鲜持续超过则 degraded [s]
    double buffer_capacity_sec = 2.0; ///< 环形缓冲容量 [s]
  };

  explicit LiveHeftReferenceProvider(const Config& config);
  ~LiveHeftReferenceProvider() override;

  LiveHeftReferenceProvider(const LiveHeftReferenceProvider&) = delete;
  LiveHeftReferenceProvider& operator=(const LiveHeftReferenceProvider&) = delete;

  /// 启动 DDS 订阅（构造后调用一次；失败返回 false）
  bool startSubscription();

  // ==================== HeftReferenceSource ====================
  bool ready() const override;
  bool sampleWindow(const std::vector<int>& offsets,
                    std::vector<HeftRefFrame>& out) override;
  void onPolicyTick() override {}
  void onActivate() override;
  Eigen::Quaterniond currentRootQuat() const override;
  bool degraded() const override;
  double progressFrame() const override;

  /// 诊断：最新帧接收龄 [s]（无数据时为一个大数）
  double newestAgeSec() const;
  /// 诊断：当前缓冲帧数
  size_t bufferedFrames() const;
  /// 诊断/测试：直接注入一帧（绕过 DDS，golden test 用）
  void injectFrame(double recv_time_sec, bool valid, bool calibrated,
                   uint32_t epoch, uint64_t seq,
                   const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
                   const array_t& jpos, const array_t& jvel = array_t());

  /// RT+Y：锁定当前参考为 frozen_frame_（消费侧冻结；DDS 仍可更新 buffer）
  void setStreamFrozen(bool frozen);
  /// RT+X：等待前端 yaw-corrected frame ready 后再解除冻结
  void requestStreamResume();
  bool streamFrozen() const;

  /// RT+X 等参考突变后由控制器消费一次，用于重填观测历史
  bool consumeReferenceDiscontinuity();

  /// 诊断/测试：注入前端 resume-ready（绕过 DDS）
  void injectResumeReady();

 private:
  struct TimedFrame {
    double recv_time = 0.0;  ///< 本地 steady 时间 [s]
    HeftRefFrame frame;
  };

  // DDS 回调路径（仅校验+入队）
  void handleFrame(double recv_time_sec, bool valid, bool calibrated,
                   uint32_t epoch, uint64_t seq,
                   const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
                   const array_t& jpos, const array_t& jvel = array_t());
  void noteRejected();
  void handleResumeReady();

  bool coverageOkLocked(double now_sec) const;
  bool freshLocked(double now_sec) const;
  HeftRefFrame interpolateLocked(double t) const;
  void prefillBufferLocked(double recv_time_sec, const HeftRefFrame& frame);
  bool captureFrozenFrameLocked();
  bool tryCompleteResumeLocked();

  Config config_;

  mutable std::mutex mutex_;
  std::deque<TimedFrame> buffer_;
  bool calibrated_ = false;
  uint32_t epoch_ = 0;
  uint64_t last_seq_ = 0;
  double last_fresh_time_ = 0.0;   ///< 最近一次"新鲜"判定成立的时刻
  bool had_fresh_data_ = false;

  // 采样保持：最后一致窗口（offsets 尺寸变化时重建）
  std::vector<HeftRefFrame> last_window_;
  std::vector<int> last_window_offsets_;

  // 统计
  uint64_t accepted_frames_ = 0;
  uint64_t rejected_frames_ = 0;
  bool reference_discontinuity_pending_ = false;

  // RT+Y / RT+X（kuavo 对齐：控制器侧冻结消费）
  bool stream_frozen_ = false;
  bool has_frozen_frame_ = false;
  HeftRefFrame frozen_frame_;
  bool resume_pending_ = false;
  uint64_t resume_min_seq_ = 0;
  bool resume_ready_ = false;

  // DDS 实体（pimpl 避免头文件引入 dds 依赖）
  struct DdsImpl;
  std::unique_ptr<DdsImpl> dds_;
};

}  // namespace leju
