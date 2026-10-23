#include "leju-rl-controller/motion/live_heft_reference_provider.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/HeftReference.hpp>
#include <lejusdk-dds-idl/StringData.hpp>
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
struct LiveHeftReferenceProvider::DdsImpl {
  explicit DdsImpl(LiveHeftReferenceProvider* owner, const Config& config)
      : participant(0),
        subscriber(participant, config.topic,
                   [owner, config](const msgs::HeftReference& msg) {
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
                   }),
        resume_ready_subscriber(
            participant, "/rt/pico/heft_resume_ready",
            [owner](const msgs::StringData&) {
              owner->handleResumeReady();
            }) {}

  dds::domain::DomainParticipant participant;
  dds_common::TopicSubscriber<msgs::HeftReference> subscriber;
  dds_common::TopicSubscriber<msgs::StringData> resume_ready_subscriber;
};

// ============================================================================
// LiveHeftReferenceProvider
// ============================================================================
LiveHeftReferenceProvider::LiveHeftReferenceProvider(const Config& config)
    : config_(config) {}

LiveHeftReferenceProvider::~LiveHeftReferenceProvider() = default;

bool LiveHeftReferenceProvider::startSubscription() {
  if (dds_) {
    return true;
  }
  try {
    dds_ = std::make_unique<DdsImpl>(this, config_);
    RL_LOG_SUCCESS("LiveHeftReferenceProvider subscribed: %s",
                   config_.topic.c_str());
    return true;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("LiveHeftReferenceProvider DDS init failed: %s", e.what());
    dds_.reset();
    return false;
  } catch (...) {
    RL_LOG_FAILURE("LiveHeftReferenceProvider DDS init failed (unknown)");
    dds_.reset();
    return false;
  }
}

void LiveHeftReferenceProvider::noteRejected() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (++rejected_frames_ <= 5) {
    RL_LOG_WARNING("HEFT live reference: rejected frame (%llu total)",
                   static_cast<unsigned long long>(rejected_frames_));
  }
}

void LiveHeftReferenceProvider::handleFrame(
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
    RL_LOG_INFO("HEFT live reference: calibration epoch %u -> %u, buffer reset",
                epoch_, epoch);
    buffer_.clear();
    last_seq_ = 0;
    epoch_ = epoch;
    resume_pending_ = false;
    resume_min_seq_ = 0;
    resume_ready_ = false;
  }
  calibrated_ = calibrated;

  // 乱序/重复帧拒绝
  if (last_seq_ != 0 && seq <= last_seq_) {
    ++rejected_frames_;
    return;
  }
  last_seq_ = seq;

  HeftRefFrame incoming;
  incoming.pos = pos;
  incoming.quat = quat.normalized();
  incoming.jpos = jpos;
  incoming.jvel = jvel;

  TimedFrame tf;
  tf.recv_time = recv_time_sec;
  tf.frame = std::move(incoming);
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

  // ready 可能先于对应参考帧到达；新帧入队后再尝试完成接入。
  tryCompleteResumeLocked();
}

bool LiveHeftReferenceProvider::freshLocked(double now_sec) const {
  if (buffer_.empty()) {
    return false;
  }
  return (now_sec - buffer_.back().recv_time) <= config_.fresh_timeout;
}

bool LiveHeftReferenceProvider::coverageOkLocked(double now_sec) const {
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

bool LiveHeftReferenceProvider::ready() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = SteadySeconds();
  return calibrated_ && freshLocked(now) && coverageOkLocked(now);
}

bool LiveHeftReferenceProvider::degraded() const {
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

bool LiveHeftReferenceProvider::sampleWindow(const std::vector<int>& offsets,
                                             std::vector<HeftRefFrame>& out) {
  std::lock_guard<std::mutex> lock(mutex_);

  // RT+Y：控制器侧冻结消费——所有 offset 返回同一 frozen_frame_
  if (stream_frozen_ && has_frozen_frame_) {
    out.assign(offsets.size(), frozen_frame_);
    last_window_ = out;
    last_window_offsets_ = offsets;
    return !offsets.empty();
  }

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

HeftRefFrame LiveHeftReferenceProvider::interpolateLocked(double t) const {
  // 前置条件：buffer_.size() >= 2，mutex_ 已持有
  // 夹取语义与 MotionTrajectory::clampFrame 一致
  if (t <= buffer_.front().recv_time) {
    HeftRefFrame f = buffer_.front().frame;
    if (f.jvel.size() != f.jpos.size()) {
      f.jvel = array_t::Zero(f.jpos.size());
    }
    return f;
  }
  if (t >= buffer_.back().recv_time) {
    HeftRefFrame f = buffer_.back().frame;
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

  HeftRefFrame result;
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

void LiveHeftReferenceProvider::onActivate() {
  std::lock_guard<std::mutex> lock(mutex_);
  last_window_.clear();
  last_window_offsets_.clear();
  reference_discontinuity_pending_ = false;
  stream_frozen_ = false;
  has_frozen_frame_ = false;
  resume_pending_ = false;
  resume_min_seq_ = 0;
  resume_ready_ = false;
  // 保留缓冲（数据本身仍有效），但要求重新新鲜/覆盖判定通过后 ready
}

bool LiveHeftReferenceProvider::captureFrozenFrameLocked() {
  if (!last_window_.empty()) {
    frozen_frame_ = last_window_.front();
    has_frozen_frame_ = true;
    return true;
  }
  if (!buffer_.empty()) {
    frozen_frame_ = buffer_.back().frame;
    if (frozen_frame_.jvel.size() != frozen_frame_.jpos.size()) {
      frozen_frame_.jvel = array_t::Zero(frozen_frame_.jpos.size());
    }
    has_frozen_frame_ = true;
    return true;
  }
  return false;
}

void LiveHeftReferenceProvider::setStreamFrozen(bool frozen) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frozen) {
    resume_pending_ = false;
    resume_min_seq_ = 0;
    resume_ready_ = false;
    if (stream_frozen_) {
      return;
    }
    if (!captureFrozenFrameLocked()) {
      RL_LOG_WARNING("HEFT live reference: freeze ignored (no frame yet)");
      return;
    }
    stream_frozen_ = true;
    RL_LOG_INFO("HEFT live reference: stream FROZEN (controller hold)");
    return;
  }
  resume_pending_ = false;
  resume_min_seq_ = 0;
  resume_ready_ = false;
  stream_frozen_ = false;
  has_frozen_frame_ = false;
}

void LiveHeftReferenceProvider::requestStreamResume() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stream_frozen_) {
    return;
  }
  resume_pending_ = true;
  resume_min_seq_ = last_seq_;
  resume_ready_ = false;
  RL_LOG_INFO("HEFT live reference: resume pending, waiting for corrected frame");
}

bool LiveHeftReferenceProvider::streamFrozen() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stream_frozen_;
}

void LiveHeftReferenceProvider::handleResumeReady() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!resume_pending_) {
    return;
  }
  resume_ready_ = true;
  tryCompleteResumeLocked();
}

bool LiveHeftReferenceProvider::tryCompleteResumeLocked() {
  if (!resume_pending_ || !stream_frozen_ || !resume_ready_ ||
      last_seq_ <= resume_min_seq_ || buffer_.empty()) {
    return false;
  }

  HeftRefFrame resume_frame = buffer_.back().frame;
  if (resume_frame.jvel.size() != resume_frame.jpos.size()) {
    resume_frame.jvel = array_t::Zero(resume_frame.jpos.size());
  }
  const double recv_time = buffer_.back().recv_time;

  resume_pending_ = false;
  resume_min_seq_ = 0;
  resume_ready_ = false;
  stream_frozen_ = false;
  has_frozen_frame_ = false;
  prefillBufferLocked(recv_time, resume_frame);
  RL_LOG_INFO("HEFT live reference: corrected frame received; stream RESUMED");
  return true;
}

void LiveHeftReferenceProvider::prefillBufferLocked(
    double recv_time_sec, const HeftRefFrame& frame) {
  buffer_.clear();
  last_window_.clear();
  last_window_offsets_.clear();

  // 与 kuavo VMP prefillRetargetedReferenceBuffers 一致：突变后用同一姿态
  // 填满历史窗口，避免 future_steps 混有冻结帧与 live 帧导致观测抖动。
  constexpr double kSourceDt = 0.01;  // leju_pico_gmr 100Hz
  const double span = config_.cursor_delay + config_.history_margin;
  const int n =
      std::max(2, static_cast<int>(std::ceil(span / kSourceDt)) + 2);
  for (int i = 0; i < n; ++i) {
    TimedFrame tf;
    tf.recv_time =
        recv_time_sec - static_cast<double>(n - 1 - i) * kSourceDt;
    tf.frame = frame;
    buffer_.push_back(std::move(tf));
  }
  last_fresh_time_ = recv_time_sec;
  had_fresh_data_ = true;
  reference_discontinuity_pending_ = true;
  RL_LOG_INFO("HEFT live reference: resume prefilled %d frames", n);
}

bool LiveHeftReferenceProvider::consumeReferenceDiscontinuity() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!reference_discontinuity_pending_) {
    return false;
  }
  reference_discontinuity_pending_ = false;
  return true;
}

void LiveHeftReferenceProvider::injectResumeReady() {
  handleResumeReady();
}

Eigen::Quaterniond LiveHeftReferenceProvider::currentRootQuat() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stream_frozen_ && has_frozen_frame_) {
    return frozen_frame_.quat;
  }
  if (buffer_.empty()) {
    return Eigen::Quaterniond::Identity();
  }
  // 最新帧的参考根姿态（dummy_world_yaw 初始化只取 yaw 分量，滞后 0.32s 影响可忽略）
  return buffer_.back().frame.quat;
}

double LiveHeftReferenceProvider::progressFrame() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_.empty()) {
    return 0.0;
  }
  return static_cast<double>(accepted_frames_);
}

double LiveHeftReferenceProvider::newestAgeSec() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_.empty()) {
    return 1e9;
  }
  return SteadySeconds() - buffer_.back().recv_time;
}

size_t LiveHeftReferenceProvider::bufferedFrames() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_.size();
}

void LiveHeftReferenceProvider::injectFrame(
    double recv_time_sec, bool valid, bool calibrated, uint32_t epoch,
    uint64_t seq, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat,
    const array_t& jpos, const array_t& jvel) {
  handleFrame(recv_time_sec, valid, calibrated, epoch, seq, pos, quat, jpos,
              jvel);
}

}  // namespace leju
