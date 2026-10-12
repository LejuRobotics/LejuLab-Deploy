#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "leju-rl-controller/rl/rl_controller_types.h"

namespace leju {

/// 单帧 Sonic 参考（与 MotionTrajectory 帧语义一致：训练/MuJoCo 坐标）
struct SonicRefFrame {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();  ///< wxyz 语义
  array_t jpos;  ///< 21 关节位置 (Sonic 输入源顺序)
  array_t jvel;  ///< 21 关节速度 (Sonic 输入源顺序)
};

/**
 * @brief Sonic 参考源抽象
 *
 * GenericRLController 的 Sonic 多输入策略通过该接口按 future_steps 偏移
 * 采样一个不可变的 Pico 实时参考窗口。
 */
class SonicReferenceSource {
 public:
  virtual ~SonicReferenceSource() = default;

  /// 参考数据是否就绪（已加载 / 已缓冲足够窗口且新鲜）
  virtual bool ready() const = 0;

  /**
   * @brief 采样参考窗口
   * @param offsets 相对当前游标的帧偏移（policy 帧, 20ms/帧, 可负）
   * @param[out] out 与 offsets 等长的参考帧
   * @return 成功返回 true；失败时 out 内容未定义
   */
  virtual bool sampleWindow(const std::vector<int>& offsets,
                            std::vector<SonicRefFrame>& out) = 0;

  /// 每个 policy tick（decimation 边界）调用一次，用于推进游标（CSV: next()）
  virtual void onPolicyTick() = 0;

  /// 控制器 reset/激活时调用：清游标/历史，要求重新就绪
  virtual void onActivate() = 0;

  /// 当前参考根四元数（initializeDummyWorldYaw 用）；不可用时返回单位四元数
  virtual Eigen::Quaterniond currentRootQuat() const = 0;

  /// 数据流退化（持续丢失，应回退到安全控制器）。CSV 恒为 false。
  virtual bool degraded() const { return false; }

  /// 当前进度（日志用）：CSV=帧号，Live=游标时刻相对最新帧的滞后秒数*(-50)
  virtual double progressFrame() const { return 0.0; }
};

}  // namespace leju
