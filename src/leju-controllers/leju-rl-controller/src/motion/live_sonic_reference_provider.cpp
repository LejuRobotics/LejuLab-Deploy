#include "leju-rl-controller/motion/live_sonic_reference_provider.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/SonicReference.hpp>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>

#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace {

double SteadySeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool isFinite(const Eigen::Vector3d& v) {
  return v.allFinite();
}

}  // namespace

// ============================================================================
// DDS pimpl
// ============================================================================
struct LiveSonicReferenceProvider::DdsImpl {
  explicit DdsImpl(LiveSonicReferenceProvider* owner, const Config& config)
      : participant(0),
        subscriber(participant, config.topic,
                   [owner, config](const msgs::SonicReference& msg) {
                     // 回调线程：仅校验+入队，不做任何阻塞工作
                     const auto& q = msg.q();
                     if (static_cast<int>(q.size()) != config.num_joints) {
                       owner->noteRejected();
                       return;
                     }
                     if (msg.schema_version() != config.expected_schema_version ||
                         msg.robot_version() != config.expected_robot_version) {
                       owner->noteRejected();
                       return;
                     }
                     Eigen::Vector3d pos(msg.root_pos()[0], msg.root_pos()[1],
                                         msg.root_pos()[2]);
                     // IDL root_quat 为 wxyz
                     Eigen::Quaterniond quat(msg.root_quat()[0], msg.root_quat()[1],
                                             msg.root_quat()[2], msg.root_quat()[3]);
                     array_t jpos(config.num_joints);
                     for (int i = 0; i < config.num_joints; ++i) {
                       jpos[i] = q[i];
                     }
                     array_t jvel;
                     const auto& qd = msg.qd();
                     if (static_cast<int>(qd.size()) == config.num_joints) {
                       jvel.resize(config.num_joints);
                       for (int i = 0; i < config.num_joints; ++i) {
                         jvel[i] = qd[i];
                       }
                     }
                     owner->handleFrame(SteadySeconds(), msg.valid(),
                                        msg.calibrated(), msg.calibration_epoch(),
                                        msg.seq(), pos, quat, jpos, jvel);
                   }) {}

  dds::domain::DomainParticipant participant;
  dds_common::TopicSubscriber<msgs::SonicReference> subscriber;
};

// ============================================================================
// LiveSonicReferenceProvider
// ============================================================================
LiveSonicReferenceProvider::LiveSonicReferenceProvider(const Config& config)
    : config_(config) {}

LiveSonicReferenceProvider::~LiveSonicReferenceProvider() = default;

bool LiveSonicReferenceProvider::startSubscription() {
  if (dds_) {
    return true;
  }
  try {
    dds_ = std::make_unique<DdsImpl>(this, config_);
    RL_LOG_SUCCESS("LiveSonicReferenceProvider subscribed: %s",
                   config_.topic.c_str());
    return true;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("LiveSonicReferenceProvider DDS init failed: %s", e.what());
    dds_.reset();
    return false;
  } catch (...) {
    RL_LOG_FAILURE("LiveSonicReferenceProvider DDS init failed (unknown)");
    dds_.reset();
    return false;
  }
}

void LiveSonicReferenceProvider::noteRejected() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (++rejected_frames_ <= 5) {
    RL_LOG_WARNING("Sonic live reference: rejected frame (%llu total)",
                   static_cast<unsigned long long>(rejected_frames_));
  }
}

void LiveSonicReferenceProvider::handleFrame(
    double recv_time_sec, bool valid, bool calibrated, uint32_t epoch,
    uint64_t seq, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
    const array_t& jpos, const array_t& jvel) {
  // 数值校验（回调线程）
  if (!valid || !isFinite(pos) || !jpos.allFinite() ||
      std::abs(quat.norm() - 1.0) > 1e-2) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++rejected_frames_;
    return;
  }
  if (jvel.size() != 0 &&
      (jvel.size() != jpos.size() || !jvel.allFinite())) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++rejected_frames_;
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // 校准 epoch 变化：清缓冲，重新预热
  if (epoch != epoch_) {
    RL_LOG_INFO("Sonic live reference: calibration epoch %u -> %u, buffer reset",
                epoch_, epoch);
    buffer_.clear();
    last_seq_ = 0;
    epoch_ = epoch;
  }
  calibrated_ = calibrated;

  // 乱序/重复帧拒绝
  if (last_seq_ != 0 && seq <= last_seq_) {
    ++rejected_frames_;
    return;
  }
  last_seq_ = seq;

  TimedFrame tf;
  tf.recv_time = recv_time_sec;
  tf.frame.pos = pos;
  tf.frame.quat = quat.normalized();
  tf.frame.jpos = jpos;
  tf.frame.jvel = jvel;
  buffer_.push_back(std::move(tf));
  ++accepted_frames_;

  // 限制缓冲容量（按时间）
  const double horizon = config_.buffer_capacity_sec;
  while (!buffer_.empty() &&
         recv_time_sec - buffer_.front().recv_time > horizon) {
    buffer_.pop_front();
  }

  last_fresh_time_ = recv_time_sec;
  had_fresh_data_ = true;
}

bool LiveSonicReferenceProvider::freshLocked(double now_sec) const {
  if (buffer_.empty()) {
    return false;
  }
  return (now_sec - buffer_.back().recv_time) <= config_.fresh_timeout;
}

bool LiveSonicReferenceProvider::coverageOkLocked(double now_sec) const {
  if (buffer_.size() < 2) {
    return false;
  }
  const double newest = buffer_.back().recv_time;
  const double oldest = buffer_.front().recv_time;
  // 需要覆盖 [cursor - history_margin, newest]:
  //   cursor = newest - cursor_delay
  const double required_span = config_.cursor_delay + config_.history_margin;
  (void)now_sec;
  return (newest - oldest) >= required_span;
}

bool LiveSonicReferenceProvider::ready() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = SteadySeconds();
  return calibrated_ && freshLocked(now) && coverageOkLocked(now);
}

bool LiveSonicReferenceProvider::degraded() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!had_fresh_data_) {
    return false;  // 从未有数据：尚未激活语义，不触发回退
  }
  const double now = SteadySeconds();
  if (freshLocked(now)) {
    return false;
  }
  return (now - last_fresh_time_) > config_.hold_timeout;
}

bool LiveSonicReferenceProvider::sampleWindow(const std::vector<int>& offsets,
                                             std::vector<SonicRefFrame>& out) {
  std::lock_guard<std::mutex> lock(mutex_);

  const double now = SteadySeconds();
  const bool fresh = freshLocked(now);

  if (buffer_.size() < 2 || (!fresh && !last_window_.empty() &&
                             last_window_offsets_ == offsets)) {
    // 短暂丢包：保持最后一致窗口（绝不清零）
    if (!last_window_.empty() && last_window_offsets_ == offsets) {
      out = last_window_;
      return true;
    }
    return false;
  }

  // 游标 = 最新帧接收时刻 - 延迟；每个偏移映射到绝对时刻后线性插值
  const double cursor = buffer_.back().recv_time - config_.cursor_delay;

  out.resize(offsets.size());
  for (size_t i = 0; i < offsets.size(); ++i) {
    const double t = cursor + offsets[i] * config_.policy_dt;
    out[i] = interpolateLocked(t);
  }

  last_window_ = out;
  last_window_offsets_ = offsets;
  return true;
}

SonicRefFrame LiveSonicReferenceProvider::interpolateLocked(double t) const {
  // 前置条件：buffer_.size() >= 2，mutex_ 已持有
  // 夹取语义与 MotionTrajectory::clampFrame 一致
  if (t <= buffer_.front().recv_time) {
    SonicRefFrame f = buffer_.front().frame;
    if (f.jvel.size() != f.jpos.size()) {
      f.jvel = array_t::Zero(f.jpos.size());
    }
    return f;
  }
  if (t >= buffer_.back().recv_time) {
    SonicRefFrame f = buffer_.back().frame;
    if (f.jvel.size() != f.jpos.size()) {
      f.jvel = array_t::Zero(f.jpos.size());
    }
    return f;
  }

  // 二分查找第一个 recv_time >= t 的帧
  const auto cmp = [](const TimedFrame& f, double value) {
    return f.recv_time < value;
  };
  const auto hi_it = std::lower_bound(buffer_.begin(), buffer_.end(), t, cmp);
  const auto lo_it = std::prev(hi_it);

  const TimedFrame& lo = *lo_it;
  const TimedFrame& hi = *hi_it;
  const double span = hi.recv_time - lo.recv_time;
  const double alpha =
      (span > 1e-9) ? std::clamp((t - lo.recv_time) / span, 0.0, 1.0) : 1.0;

  SonicRefFrame result;
  result.pos = (1.0 - alpha) * lo.frame.pos + alpha * hi.frame.pos;
  result.jpos = (1.0 - alpha) * lo.frame.jpos + alpha * hi.frame.jpos;

  // 关节速度：优先线性插值两端 qd；否则用相邻帧差分
  if (lo.frame.jvel.size() == lo.frame.jpos.size() &&
      hi.frame.jvel.size() == hi.frame.jpos.size()) {
    result.jvel =
        (1.0 - alpha) * lo.frame.jvel + alpha * hi.frame.jvel;
  } else if (span > 1e-9) {
    result.jvel = (hi.frame.jpos - lo.frame.jpos) / span;
  } else {
    result.jvel = array_t::Zero(lo.frame.jpos.size());
  }

  // slerp 符号连续
  Eigen::Quaterniond q_hi = hi.frame.quat;
  if (lo.frame.quat.dot(q_hi) < 0.0) {
    q_hi.coeffs() = -q_hi.coeffs();
  }
  result.quat = lo.frame.quat.slerp(alpha, q_hi).normalized();
  return result;
}

void LiveSonicReferenceProvider::onActivate() {
  std::lock_guard<std::mutex> lock(mutex_);
  last_window_.clear();
  last_window_offsets_.clear();
  // 保留缓冲（数据本身仍有效），但要求重新新鲜/覆盖判定通过后 ready
}

Eigen::Quaterniond LiveSonicReferenceProvider::currentRootQuat() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_.empty()) {
    return Eigen::Quaterniond::Identity();
  }
  // 最新帧的参考根姿态（dummy_world_yaw 初始化只取 yaw 分量，滞后 0.32s 影响可忽略）
  return buffer_.back().frame.quat;
}

double LiveSonicReferenceProvider::progressFrame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_.empty()) {
    return 0.0;
  }
  return static_cast<double>(accepted_frames_);
}

double LiveSonicReferenceProvider::newestAgeSec() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_.empty()) {
    return 1e9;
  }
  return SteadySeconds() - buffer_.back().recv_time;
}

size_t LiveSonicReferenceProvider::bufferedFrames() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_.size();
}

void LiveSonicReferenceProvider::injectFrame(
    double recv_time_sec, bool valid, bool calibrated, uint32_t epoch,
    uint64_t seq, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
    const array_t& jpos, const array_t& jvel) {
  handleFrame(recv_time_sec, valid, calibrated, epoch, seq, pos, quat, jpos,
              jvel);
}

}  // namespace leju
