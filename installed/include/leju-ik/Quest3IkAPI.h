#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "leju-ik/ik_types.h"
#include "leju-ik/Quest3ArmInfoTransformer.h"
#include "leju-ik/TwoStageTorsoIK.h"

namespace leju {
namespace ik {

class Quest3IkAPI {
 public:
  using ArmJointCallback = std::function<void(const std::vector<double>& q_rad)>;
  using HeadBodyPoseCallback = std::function<void(const HeadBodyPose&)>;

  Quest3IkAPI();
  ~Quest3IkAPI();

  /** Initialize with full URDF path and JSON config */
  bool init(const std::string& urdf_path, const nlohmann::json& config);

  /**
   * Initialize with assets path and RobotVersion.
   * Builds URDF path: assets_path + "/models/biped_s" + version_str + "/urdf/drake/biped_v3_arm.urdf"
   */
  bool init(const std::string& assets_path, uint16_t major, uint16_t minor,
            const nlohmann::json& config = nlohmann::json());

  /**
   * Initialize with separate models_base and config_base paths.
   * URDF: models_base + "/models/biped_s" + version_str + "/urdf/drake/biped_v3_arm.urdf"
   * Config: config_base + "/config/kuavo_v" + version_str + "/kuavo.json"
   */
  bool init(const std::string& models_base, const std::string& config_base,
            uint16_t major, uint16_t minor,
            const nlohmann::json& config = nlohmann::json());

  void onBonePoses(const PoseInfoList& bone_poses);
  void onJoystick(const JoyStickData& joystick);

  void setArmJointCallback(ArmJointCallback cb) { armJointCallback_ = cb; }
  void setHeadBodyPoseCallback(HeadBodyPoseCallback cb) { headBodyPoseCallback_ = cb; }

  /** @brief 设置手臂控制模式，对齐 kuavo：0=KeepPose, 1=Auto, 2=External；仅 mode=2 时 IK 运行 */
  void setArmMode(int mode);

  void setPublishRate(double hz) { publishRate_ = hz; }
  void runOnce();

  bool isRunning() const { return transformer_ && transformer_->isRunning(); }

 private:
  bool buildPlantAndIK(const std::string& urdf_path, const nlohmann::json& config);
  std::vector<std::string> loadFrameNamesFromConfig(const nlohmann::json& config);

  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagramContext_;
  drake::multibody::MultibodyPlant<double>* plant_ = nullptr;
  std::unique_ptr<Quest3ArmInfoTransformer> transformer_;
  std::unique_ptr<TwoStageTorsoIK> twoStageIk_;

  ArmJointCallback armJointCallback_;
  HeadBodyPoseCallback headBodyPoseCallback_;
  double publishRate_ = 100.0;
  ArmIdx ctrlArmIdx_ = ArmIdx::BOTH;

  std::mutex bonePosesMutex_;
  std::mutex joystickMutex_;
  PoseInfoList latestBonePoses_;
  JoyStickData latestJoystick_;
  bool hasBonePoses_ = false;
};

}  // namespace ik
}  // namespace leju
