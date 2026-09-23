#include "leju-rl-controller/motion/bezier_interpolator.h"

#include <algorithm>
#include <cmath>

namespace leju {
namespace vr {
namespace tact_player {
namespace {

constexpr double kKeyframeToSec = 0.01;
constexpr double kEps = 1e-9;
constexpr double kPi = 3.14159265358979323846;

}  // namespace

double BezierInterpolator::DegToRad(double deg) {
  return deg * kPi / 180.0;
}

double BezierInterpolator::RadToDeg(double rad) {
  return rad * 180.0 / kPi;
}

bool BezierInterpolator::Build(const TactAction& action,
                               const InterpolateOptions& options,
                               std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  if (options.arm_dof == 0) {
    *error_message = "arm_dof must be > 0";
    return false;
  }
  if (options.speed_scale <= 0.0) {
    *error_message = "speed_scale must be > 0";
    return false;
  }
  if (action.frames.size() < 2) {
    *error_message = "tact frames must contain at least 2 keyframes";
    return false;
  }

  arm_dof_ = options.arm_dof;
  first_sec_ = action.first_sec;
  finish_sec_ = action.finish_sec;
  speed_scale_ = options.speed_scale;
  has_waist_ = options.enable_waist && action.has_waist;
  has_hand_ = options.enable_hand && action.has_hand;

  tracks_.assign(arm_dof_, {});
  waist_track_.clear();
  hand_tracks_.clear();

  for (std::size_t joint = 0; joint < arm_dof_; ++joint) {
    auto& track = tracks_[joint];
    const std::size_t n = action.frames.size();

    // 收集各关键帧的时间、弧度位置、以及 CP 是否为 AUTO（角度偏移=0）
    std::vector<double> times(n), positions(n);
    std::vector<bool> right_cp_auto(n, true);   // right_cp[1] == 0 → AUTO
    std::vector<bool> left_cp_auto(n, true);     // left_cp[1] == 0 → AUTO
    for (std::size_t i = 0; i < n; ++i) {
      times[i] = action.frames[i].keyframe * kKeyframeToSec;
      positions[i] = DegToRad(action.frames[i].servos_deg[joint]);
      const auto& attr = action.frames[i].attributes[joint];
      left_cp_auto[i] = (std::abs(attr.cp[0][1]) <= kEps);
      right_cp_auto[i] = (std::abs(attr.cp[1][1]) <= kEps);
    }

    // Catmull-Rom 切线（仅用于 AUTO CP 的帧）
    std::vector<double> tangents(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
      if (i == 0) {
        double dt = times[1] - times[0];
        tangents[0] = (dt > kEps) ? (positions[1] - positions[0]) / dt : 0.0;
      } else if (i == n - 1) {
        double dt = times[n - 1] - times[n - 2];
        tangents[n - 1] = (dt > kEps) ? (positions[n - 1] - positions[n - 2]) / dt : 0.0;
      } else {
        double dt_prev = times[i] - times[i - 1];
        double dt_next = times[i + 1] - times[i];
        double slope_prev = (dt_prev > kEps) ? (positions[i] - positions[i - 1]) / dt_prev : 0.0;
        double slope_next = (dt_next > kEps) ? (positions[i + 1] - positions[i]) / dt_next : 0.0;
        tangents[i] = (slope_prev + slope_next) * 0.5;
      }
    }

    // 构建贝塞尔段：CP 偏移量为 0 时用 Catmull-Rom，否则保留原始 CP
    for (std::size_t i = 1; i < n; ++i) {
      double dt = times[i] - times[i - 1];
      if (dt <= kEps) continue;

      CubicBezierSegment seg;
      seg.t0 = times[i - 1];
      seg.t1 = times[i];
      seg.y0 = positions[i - 1];
      seg.y3 = positions[i];

      // 出控制点 y1：前一帧的 right_cp
      if (right_cp_auto[i - 1]) {
        seg.y1 = positions[i - 1] + tangents[i - 1] * dt / 3.0;
      } else {
        double prev_deg = action.frames[i - 1].servos_deg[joint];
        seg.y1 = DegToRad(prev_deg + action.frames[i - 1].attributes[joint].cp[1][1]);
      }

      // 入控制点 y2：当前帧的 left_cp
      if (left_cp_auto[i]) {
        seg.y2 = positions[i] - tangents[i] * dt / 3.0;
      } else {
        double curr_deg = action.frames[i].servos_deg[joint];
        seg.y2 = DegToRad(curr_deg + action.frames[i].attributes[joint].cp[0][1]);
      }

      track.push_back(seg);
    }

    if (track.empty()) {
      *error_message = "joint " + std::to_string(joint) + " has no valid segment";
      return false;
    }
  }

  // Build waist track if enabled
  if (has_waist_) {
    // 收集有腰部数据的关键帧时间和位置
    std::vector<double> waist_times, waist_positions;
    for (std::size_t i = 0; i < action.frames.size(); ++i) {
      if (action.frames[i].has_waist) {
        waist_times.push_back(action.frames[i].keyframe * kKeyframeToSec);
        waist_positions.push_back(DegToRad(action.frames[i].waist_deg));
      }
    }

    const std::size_t wn = waist_times.size();
    if (wn >= 2) {
      // Catmull-Rom 切线
      std::vector<double> waist_tangents(wn, 0.0);
      for (std::size_t i = 0; i < wn; ++i) {
        if (i == 0) {
          double dt = waist_times[1] - waist_times[0];
          waist_tangents[0] = (dt > kEps) ? (waist_positions[1] - waist_positions[0]) / dt : 0.0;
        } else if (i == wn - 1) {
          double dt = waist_times[wn - 1] - waist_times[wn - 2];
          waist_tangents[wn - 1] = (dt > kEps) ? (waist_positions[wn - 1] - waist_positions[wn - 2]) / dt : 0.0;
        } else {
          double dt_prev = waist_times[i] - waist_times[i - 1];
          double dt_next = waist_times[i + 1] - waist_times[i];
          double slope_prev = (dt_prev > kEps) ? (waist_positions[i] - waist_positions[i - 1]) / dt_prev : 0.0;
          double slope_next = (dt_next > kEps) ? (waist_positions[i + 1] - waist_positions[i]) / dt_next : 0.0;
          waist_tangents[i] = (slope_prev + slope_next) * 0.5;
        }
      }

      // 按 C1 切线构建腰部贝塞尔段
      for (std::size_t i = 1; i < wn; ++i) {
        double dt = waist_times[i] - waist_times[i - 1];
        if (dt <= kEps) continue;

        CubicBezierSegment seg;
        seg.t0 = waist_times[i - 1];
        seg.t1 = waist_times[i];
        seg.y0 = waist_positions[i - 1];
        seg.y3 = waist_positions[i];
        seg.y1 = waist_positions[i - 1] + waist_tangents[i - 1] * dt / 3.0;
        seg.y2 = waist_positions[i] - waist_tangents[i] * dt / 3.0;
        waist_track_.push_back(seg);
      }
    }

    if (waist_track_.empty()) {
      // No valid waist segments, disable waist
      has_waist_ = false;
    }
  }


  // Build hand tracks if enabled. Hand values are percent-like [0,100] in tact files;
  // reuse the bezier segment math by storing them as radians internally and converting
  // back to [0,100] on output.
  if (has_hand_) {
    hand_tracks_.assign(12, {});
    for (std::size_t finger = 0; finger < 12; ++finger) {
      auto& track = hand_tracks_[finger];
      for (std::size_t i = 1; i < action.frames.size(); ++i) {
        const auto& prev = action.frames[i - 1];
        const auto& curr = action.frames[i];
        if (!prev.has_hand || !curr.has_hand ||
            prev.hand_positions.size() <= finger || curr.hand_positions.size() <= finger ||
            prev.hand_attributes.size() <= finger || curr.hand_attributes.size() <= finger) {
          continue;
        }

        const double t0 = prev.keyframe * kKeyframeToSec;
        const double t1 = curr.keyframe * kKeyframeToSec;
        if (t1 - t0 <= kEps) {
          continue;
        }

        CubicBezierSegment seg;
        seg.t0 = t0;
        seg.t1 = t1;

        const double prev_pos = prev.hand_positions[finger];
        const double curr_pos = curr.hand_positions[finger];
        const auto& prev_right_cp = prev.hand_attributes[finger].cp[1];
        const auto& curr_left_cp = curr.hand_attributes[finger].cp[0];

        seg.y0 = DegToRad(prev_pos);
        seg.y1 = DegToRad(prev_pos + prev_right_cp[1]);
        seg.y2 = DegToRad(curr_pos + curr_left_cp[1]);
        seg.y3 = DegToRad(curr_pos);
        track.push_back(seg);
      }

      if (track.empty()) {
        has_hand_ = false;
        hand_tracks_.clear();
        break;
      }
    }
  }

  if (finish_sec_ <= first_sec_) {
    finish_sec_ = action.frames.back().keyframe * kKeyframeToSec;
  }
  finish_sec_ = std::max(finish_sec_, tracks_.front().back().t1);
  first_sec_ = std::max(first_sec_, tracks_.front().front().t0);

  if (finish_sec_ - first_sec_ <= kEps) {
    *error_message = "invalid play range, finish <= first";
    return false;
  }

  duration_sec_ = (finish_sec_ - first_sec_) / speed_scale_;
  return true;
}

double BezierInterpolator::EvalPosition(const CubicBezierSegment& seg, double u) {
  const double one_minus_u = 1.0 - u;
  return one_minus_u * one_minus_u * one_minus_u * seg.y0 +
         3.0 * one_minus_u * one_minus_u * u * seg.y1 +
         3.0 * one_minus_u * u * u * seg.y2 +
         u * u * u * seg.y3;
}

double BezierInterpolator::EvalVelocity(const CubicBezierSegment& seg, double u) {
  const double dt = seg.t1 - seg.t0;
  const double one_minus_u = 1.0 - u;
  const double dy_du = 3.0 * one_minus_u * one_minus_u * (seg.y1 - seg.y0) +
                       6.0 * one_minus_u * u * (seg.y2 - seg.y1) +
                       3.0 * u * u * (seg.y3 - seg.y2);
  return dy_du / dt;
}

double BezierInterpolator::EvalAcceleration(const CubicBezierSegment& seg, double u) {
  const double dt = seg.t1 - seg.t0;
  const double d2y_du2 =
      6.0 * (1.0 - u) * (seg.y2 - 2.0 * seg.y1 + seg.y0) +
      6.0 * u * (seg.y3 - 2.0 * seg.y2 + seg.y1);
  return d2y_du2 / (dt * dt);
}

JointTrajectorySample BezierInterpolator::Evaluate(double elapsed_sec) const {
  JointTrajectorySample out;
  out.q.assign(arm_dof_, 0.0);
  out.v.assign(arm_dof_, 0.0);
  out.acc.assign(arm_dof_, 0.0);

  const double clamped_elapsed = std::clamp(elapsed_sec, 0.0, duration_sec_);
  const double query_time = std::clamp(first_sec_ + clamped_elapsed * speed_scale_, first_sec_, finish_sec_);

  for (std::size_t joint = 0; joint < arm_dof_; ++joint) {
    const auto& track = tracks_[joint];
    auto it = std::find_if(track.begin(), track.end(), [query_time](const CubicBezierSegment& seg) {
      return query_time <= seg.t1;
    });

    if (it == track.end()) {
      it = std::prev(track.end());
    }

    const auto& seg = *it;
    const double dt = seg.t1 - seg.t0;
    if (dt <= kEps) {
      out.q[joint] = seg.y3;
      out.v[joint] = 0.0;
      out.acc[joint] = 0.0;
      continue;
    }

    const double u = std::clamp((query_time - seg.t0) / dt, 0.0, 1.0);
    out.q[joint] = EvalPosition(seg, u);
    out.v[joint] = EvalVelocity(seg, u);
    out.acc[joint] = EvalAcceleration(seg, u);
  }

  // Evaluate waist if enabled
  if (has_waist_ && !waist_track_.empty()) {
    out.has_waist = true;

    auto it = std::find_if(waist_track_.begin(), waist_track_.end(),
                           [query_time](const CubicBezierSegment& seg) {
                             return query_time <= seg.t1;
                           });

    if (it == waist_track_.end()) {
      it = std::prev(waist_track_.end());
    }

    const auto& seg = *it;
    const double dt = seg.t1 - seg.t0;
    if (dt <= kEps) {
      out.waist_q = seg.y3;
      out.waist_v = 0.0;
      out.waist_acc = 0.0;
    } else {
      const double u = std::clamp((query_time - seg.t0) / dt, 0.0, 1.0);
      out.waist_q = EvalPosition(seg, u);
      out.waist_v = EvalVelocity(seg, u);
      out.waist_acc = EvalAcceleration(seg, u);
    }
  }


  // Evaluate hand if enabled
  if (has_hand_ && hand_tracks_.size() == 12) {
    out.has_hand = true;
    out.hand_position.assign(12, 0.0);

    for (std::size_t finger = 0; finger < 12; ++finger) {
      const auto& track = hand_tracks_[finger];
      auto it = std::find_if(track.begin(), track.end(),
                             [query_time](const CubicBezierSegment& seg) {
                               return query_time <= seg.t1;
                             });
      if (it == track.end()) {
        it = std::prev(track.end());
      }

      const auto& seg = *it;
      const double dt = seg.t1 - seg.t0;
      double pos = seg.y3;
      if (dt > kEps) {
        const double u = std::clamp((query_time - seg.t0) / dt, 0.0, 1.0);
        pos = EvalPosition(seg, u);
      }
      out.hand_position[finger] = std::clamp(RadToDeg(pos), 0.0, 100.0);
    }
  }

  return out;
}

}  // namespace tact_player
}  // namespace vr
}  // namespace leju
