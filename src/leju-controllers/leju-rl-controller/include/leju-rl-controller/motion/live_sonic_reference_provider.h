#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "leju-rl-controller/motion/sonic_reference_source.h"

namespace leju {

/**
 * @brief 实时 Sonic 参考提供者（Pico/GMR DDS 流）
 *
 * 订阅 /rt/pico/sonic_reference（100Hz, leju::msgs::SonicReference），
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
 */
class LiveSonicReferenceProvider final : public SonicReferenceSource {
 public:
  struct Config {
    std::string topic = "/rt/pico/sonic_reference";
    std::string expected_robot_version = "roban_s17";
    uint32_t expected_schema_version = 1;
    int num_joints = 21;
    double policy_dt = 0.02;          ///< policy 帧周期 [s]
    double cursor_delay = 0.2;        ///< 游标延迟 = 最大 future 偏移 10*20ms [s]
    double history_margin = 0.18;     ///< 额外历史需求 (>= 8*20ms=0.16) [s]
    double fresh_timeout = 0.15;      ///< 最新帧本地接收龄超过则不新鲜 [s]
    double hold_timeout = 0.5;        ///< 不新鲜持续超过则 degraded [s]
    double buffer_capacity_sec = 2.0; ///< 环形缓冲容量 [s]
  };

  explicit LiveSonicReferenceProvider(const Config& config);
  ~LiveSonicReferenceProvider() override;

  LiveSonicReferenceProvider(const LiveSonicReferenceProvider&) = delete;
  LiveSonicReferenceProvider& operator=(const LiveSonicReferenceProvider&) = delete;

  /// 启动 DDS 订阅（构造后调用一次；失败返回 false）
  bool startSubscription();

  // ==================== SonicReferenceSource ====================
  bool ready() const override;
  bool sampleWindow(const std::vector<int>& offsets,
                    std::vector<SonicRefFrame>& out) override;
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

 private:
  struct TimedFrame {
    double recv_time = 0.0;  ///< 本地 steady 时间 [s]
    SonicRefFrame frame;
  };

  // DDS 回调路径（仅校验+入队）
  void handleFrame(double recv_time_sec, bool valid, bool calibrated,
                   uint32_t epoch, uint64_t seq,
                   const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
                   const array_t& jpos, const array_t& jvel = array_t());
  void noteRejected();

  bool coverageOkLocked(double now_sec) const;
  bool freshLocked(double now_sec) const;
  SonicRefFrame interpolateLocked(double t) const;

  Config config_;

  mutable std::mutex mutex_;
  std::deque<TimedFrame> buffer_;
  bool calibrated_ = false;
  uint32_t epoch_ = 0;
  uint64_t last_seq_ = 0;
  double last_fresh_time_ = 0.0;   ///< 最近一次"新鲜"判定成立的时刻
  bool had_fresh_data_ = false;

  // 采样保持：最后一致窗口（offsets 尺寸变化时重建）
  std::vector<SonicRefFrame> last_window_;
  std::vector<int> last_window_offsets_;

  // 统计
  uint64_t accepted_frames_ = 0;
  uint64_t rejected_frames_ = 0;

  // DDS 实体（pimpl 避免头文件引入 dds 依赖）
  struct DdsImpl;
  std::unique_ptr<DdsImpl> dds_;
};

}  // namespace leju
