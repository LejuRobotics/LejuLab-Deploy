#pragma once

#include <functional>

#include "leju-rl-controller/motion/heft_reference_source.h"
#include "leju-rl-controller/motion/motion_trajectory.h"

namespace leju {

/**
 * @brief CSV 参考源：包装现有 MotionTrajectory 播放语义
 *
 * 通过回调获取"当前 motion"，保持 GenericRLController 的
 * motion 切换 / motion_playing_ 行为完全不变。帧推进仍由
 * GenericRLController::updateImpl 的 loader->next() 完成，
 * 因此 onPolicyTick() 为空实现（避免双重推进）。
 */
class CsvHeftReferenceSource final : public HeftReferenceSource {
 public:
  using MotionGetter = std::function<MotionTrajectory*()>;

  explicit CsvHeftReferenceSource(MotionGetter getter)
      : getter_(std::move(getter)) {}

  bool ready() const override {
    MotionTrajectory* motion = getter_();
    return motion != nullptr && motion->isLoaded();
  }

  bool sampleWindow(const std::vector<int>& offsets,
                    std::vector<HeftRefFrame>& out) override {
    MotionTrajectory* motion = getter_();
    if (!motion || !motion->isLoaded()) {
      return false;
    }
    const int t = motion->getCurrentFrame();
    out.resize(offsets.size());
    for (size_t i = 0; i < offsets.size(); ++i) {
      const int frame = t + offsets[i];
      out[i].pos = motion->getFrameBodyPos(frame);
      out[i].quat = motion->getFrameBodyQuat(frame);
      out[i].jpos = motion->getFrameJointPos(frame);
      out[i].jvel = motion->getFrameJointVel(frame);
    }
    return true;
  }

  // 帧推进由 GenericRLController::updateImpl 的 loader->next() 负责
  void onPolicyTick() override {}

  void onActivate() override {}

  Eigen::Quaterniond currentRootQuat() const override {
    MotionTrajectory* motion = getter_();
    if (!motion || !motion->isLoaded()) {
      return Eigen::Quaterniond::Identity();
    }
    return motion->getBodyQuat();
  }

  double progressFrame() const override {
    MotionTrajectory* motion = getter_();
    return motion ? static_cast<double>(motion->getCurrentFrame()) : 0.0;
  }

 private:
  MotionGetter getter_;
};

}  // namespace leju
