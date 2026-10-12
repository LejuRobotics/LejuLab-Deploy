// Copyright 2024 Leju Robotics. All rights reserved.
//
// tact 动作文件播放器的数据类型定义。
// 包含 tact 文件解析结果、贝塞尔曲线段和轨迹采样点等结构体。

#ifndef LEJU_RL_CONTROLLER_MOTION_TACT_TYPES_H_
#define LEJU_RL_CONTROLLER_MOTION_TACT_TYPES_H_

#include <array>
#include <string>
#include <vector>

namespace leju {
namespace vr {
namespace tact_player {

// 关键帧属性，包含贝塞尔曲线控制点信息。
// 每个关节在每个关键帧都有对应的属性。
struct FrameAttribute {
  // 贝塞尔曲线控制点，cp[0] 为左控制点，cp[1] 为右控制点。
  // 每个控制点包含 [时间偏移(ms), 角度偏移(deg)]。
  std::array<std::array<double, 2>, 2> cp{{{0.0, 0.0}, {0.0, 0.0}}};

  // 控制点类型，可选 "AUTO" 或 "MANUAL"。
  std::array<std::string, 2> cp_type{{"AUTO", "AUTO"}};
};

// tact 文件中的单个关键帧数据。
struct TactFrame {
  // 各关节角度数组，单位：度（degree）。
  // 数组长度取决于机器人类型（Roban: 23, Kuavo 4.x: 28）。
  std::vector<double> servos_deg;

  // 关键帧时间戳，单位：毫秒（ms）。
  double keyframe = 0.0;

  // 各关节的贝塞尔曲线属性，与 servos_deg 一一对应。
  std::vector<FrameAttribute> attributes;

  // 腰部关节数据（可选，仅 Roban 支持）
  bool has_waist = false;       // 是否包含腰部数据
  double waist_deg = 0.0;       // 腰部角度，单位：度
  FrameAttribute waist_attribute;  // 腰部贝塞尔曲线属性

  // 灵巧手数据（可选，Roban tact: servos[8:20]，左手6维 + 右手6维）
  bool has_hand = false;
  std::vector<double> hand_positions;
  std::vector<FrameAttribute> hand_attributes;
};

// tact 动作文件的完整解析结果。
struct TactAction {
  // 机器人类型标识（如 45 表示 Kuavo 4.5，11 表示 Roban）。
  int robot_type = 0;

  // 动作起始时间，单位：秒。
  double first_sec = 0.0;

  // 动作结束时间，单位：秒。
  double finish_sec = 0.0;

  // 所有关键帧数据，按时间排序。
  std::vector<TactFrame> frames;

  // 是否包含腰部关节数据。
  bool has_waist = false;

  // 是否包含手部数据。
  bool has_hand = false;
};

// 三次贝塞尔曲线段，用于关节轨迹插值。
// 曲线方程：B(u) = (1-u)³·y0 + 3(1-u)²u·y1 + 3(1-u)u²·y2 + u³·y3
// 其中 u = (t - t0) / (t1 - t0)，t ∈ [t0, t1]
struct CubicBezierSegment {
  double t0 = 0.0;  // 起始时间，单位：秒
  double t1 = 0.0;  // 结束时间，单位：秒
  double y0 = 0.0;  // 起点位置，单位：弧度
  double y1 = 0.0;  // 控制点1（影响起点切线）
  double y2 = 0.0;  // 控制点2（影响终点切线）
  double y3 = 0.0;  // 终点位置，单位：弧度
};

// 关节轨迹采样点，包含位置、速度和加速度。
// 用于发布给控制器执行。
struct JointTrajectorySample {
  std::vector<double> q;    // 各关节位置，单位：弧度
  std::vector<double> v;    // 各关节速度，单位：弧度/秒
  std::vector<double> acc;  // 各关节加速度，单位：弧度/秒²

  // 腰部关节数据（单关节）
  bool has_waist = false;     // 是否包含腰部数据
  double waist_q = 0.0;       // 腰部位置，单位：弧度
  double waist_v = 0.0;       // 腰部速度，单位：弧度/秒
  double waist_acc = 0.0;     // 腰部加速度，单位：弧度/秒²

  bool has_hand = false;              // 是否包含手部数据
  std::vector<double> hand_position;  // 双手目标位置，左手6维 + 右手6维，范围 [0,100]
};

}  // namespace tact_player
}  // namespace vr
}  // namespace leju

#endif  // LEJU_RL_CONTROLLER_MOTION_TACT_TYPES_H_
