// Copyright 2024 Leju Robotics. All rights reserved.
//
// tact 动作文件解析器。
// 解析 JSON 格式的 .tact 文件，提取关键帧、关节角度和贝塞尔控制点。

#ifndef LEJU_RL_CONTROLLER_MOTION_TACT_PARSER_H_
#define LEJU_RL_CONTROLLER_MOTION_TACT_PARSER_H_

#include <string>

#include "leju-rl-controller/motion/tact_types.h"

namespace leju {
namespace vr {
namespace tact_player {

// tact 文件解析选项。
struct ParseOptions {
  // 手臂关节自由度（Kuavo: 14, Roban: 8）。
  std::size_t arm_dof = 14;

  // 是否在 t=0 处添加初始帧（如果第一帧不在 t=0）。
  bool add_init_frame = true;

  // 初始手臂位置（度），为空时使用第一帧的值。
  std::vector<double> init_arm_pos;

  // 初始腰部位置（度），为空时使用第一帧的值。
  std::vector<double> init_waist_pos;

  // 腰部关节在 servos 数组中的索引，-1 表示无腰部。
  // Roban: 22, Kuavo 5.x: 28, Kuavo 4.x: -1
  int waist_index = -1;

  // 手部数据在 servos 数组中的起始索引，-1 表示无手部。Roban: 8。
  int hand_start_index = -1;
};

// 加载并解析 tact 文件。
// @param path tact 文件路径
// @param options 解析选项
// @param action 解析结果输出
// @param error_message 错误信息输出
// @return 成功返回 true，失败返回 false
bool LoadTactFile(const std::string& path,
                  const ParseOptions& options,
                  TactAction* action,
                  std::string* error_message);

}  // namespace tact_player
}  // namespace vr
}  // namespace leju

#endif  // LEJU_RL_CONTROLLER_MOTION_TACT_PARSER_H_
