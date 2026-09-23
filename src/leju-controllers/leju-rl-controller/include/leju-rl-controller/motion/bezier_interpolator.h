// Copyright 2024 Leju Robotics. All rights reserved.
//
// 贝塞尔曲线插值器，用于将 tact 关键帧数据插值为连续轨迹。
// 支持手臂关节和腰部关节的三次贝塞尔曲线插值。

#ifndef LEJU_RL_CONTROLLER_MOTION_BEZIER_INTERPOLATOR_H_
#define LEJU_RL_CONTROLLER_MOTION_BEZIER_INTERPOLATOR_H_

#include <string>
#include <vector>

#include "leju-rl-controller/motion/tact_types.h"

namespace leju {
namespace vr {
namespace tact_player {

// 插值器配置选项。
struct InterpolateOptions {
  std::size_t arm_dof = 14;    // 手臂关节自由度
  double speed_scale = 1.0;    // 播放速度倍率（1.0 = 原速）
  bool enable_waist = false;   // 是否启用腰部关节插值
  bool enable_hand = false;    // 是否启用手部插值
};

// 贝塞尔曲线轨迹插值器。
// 根据 tact 文件中的关键帧和控制点，构建三次贝塞尔曲线，
// 并在任意时刻计算关节的位置、速度和加速度。
class BezierInterpolator {
 public:
  // 根据 tact 动作数据构建插值器。
  // @param action 解析后的 tact 动作数据
  // @param options 插值选项
  // @param error_message 错误信息输出
  // @return 成功返回 true，失败返回 false
  bool Build(const TactAction& action,
             const InterpolateOptions& options,
             std::string* error_message);

  // 在指定时刻计算轨迹采样点。
  // @param elapsed_sec 从轨迹开始经过的时间（秒）
  // @return 包含位置、速度、加速度的采样点
  JointTrajectorySample Evaluate(double elapsed_sec) const;

  // 获取手臂关节自由度。
  std::size_t ArmDof() const { return arm_dof_; }

  // 获取轨迹总时长（秒）。
  double Duration() const { return duration_sec_; }

  // 是否包含腰部关节数据。
  bool HasWaist() const { return has_waist_; }

 private:
  // 角度转弧度。
  static double DegToRad(double deg);
  static double RadToDeg(double rad);

  // 计算贝塞尔曲线在参数 u 处的位置。
  static double EvalPosition(const CubicBezierSegment& seg, double u);

  // 计算贝塞尔曲线在参数 u 处的速度（一阶导数）。
  static double EvalVelocity(const CubicBezierSegment& seg, double u);

  // 计算贝塞尔曲线在参数 u 处的加速度（二阶导数）。
  static double EvalAcceleration(const CubicBezierSegment& seg, double u);

  std::size_t arm_dof_ = 0;       // 手臂关节数
  double first_sec_ = 0.0;        // 轨迹起始时间
  double finish_sec_ = 0.0;       // 轨迹结束时间
  double duration_sec_ = 0.0;     // 轨迹总时长
  double speed_scale_ = 1.0;      // 播放速度倍率
  std::vector<std::vector<CubicBezierSegment>> tracks_;  // 各关节的曲线段

  // 腰部关节插值
  bool has_waist_ = false;                        // 是否有腰部数据
  std::vector<CubicBezierSegment> waist_track_;   // 腰部曲线段

  bool has_hand_ = false;
  std::vector<std::vector<CubicBezierSegment>> hand_tracks_;
};

}  // namespace tact_player
}  // namespace vr
}  // namespace leju

#endif  // LEJU_RL_CONTROLLER_MOTION_BEZIER_INTERPOLATOR_H_
