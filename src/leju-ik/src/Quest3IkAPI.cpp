#include "leju-ik/Quest3IkAPI.h"

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace leju {
namespace ik {

Quest3IkAPI::Quest3IkAPI() = default;

Quest3IkAPI::~Quest3IkAPI() = default;

bool Quest3IkAPI::init(const std::string& urdf_path, const nlohmann::json& config) {
  return buildPlantAndIK(urdf_path, config);
}

bool Quest3IkAPI::init(const std::string& assets_path, uint16_t major, uint16_t minor,
                       const nlohmann::json& config) {
  return init(assets_path, assets_path, major, minor, config);
}

bool Quest3IkAPI::init(const std::string& models_base, const std::string& config_base,
                       uint16_t major, uint16_t minor, const nlohmann::json& config) {
  std::string version_str = std::to_string(major) + std::to_string(minor);
  std::string urdf_path =
      models_base + "/models/biped_s" + version_str + "/urdf/drake/biped_v3_arm.urdf";
  nlohmann::json merged_config = config;
  if (config.empty()) {
    std::string config_path =
        config_base + "/config/kuavo_v" + version_str + "/kuavo.json";
    std::ifstream f(config_path);
    if (f.good()) {
      merged_config = nlohmann::json::parse(f);
    }
  }
  return buildPlantAndIK(urdf_path, merged_config);
}

std::vector<std::string> Quest3IkAPI::loadFrameNamesFromConfig(const nlohmann::json& config) {
  if (config.contains("end_frames_name_ik") && config["end_frames_name_ik"].is_array()) {
    return config["end_frames_name_ik"].get<std::vector<std::string>>();
  }
  return {"base_link", "zarm_l7_end_effector", "zarm_r7_end_effector", "zarm_l4_link",
          "zarm_r4_link"};
}

bool Quest3IkAPI::buildPlantAndIK(const std::string& urdf_path, const nlohmann::json& config) {
  auto diagramBuilder = std::make_unique<drake::systems::DiagramBuilder<double>>();
  auto [plant, sceneGraph] =
      drake::multibody::AddMultibodyPlantSceneGraph(diagramBuilder.get(), 0.0);

  drake::multibody::Parser parser(&plant);
  parser.AddModelFromFile(urdf_path);

  const auto& baseFrame = plant.GetFrameByName("base_link");
  plant.WeldFrames(plant.world_frame(), baseFrame);
  plant.Finalize();

  diagram_ = diagramBuilder->Build();
  diagramContext_ = diagram_->CreateDefaultContext();

  std::vector<std::string> frameNames = loadFrameNamesFromConfig(config);
  transformer_ = std::make_unique<Quest3ArmInfoTransformer>("kuavo_45");
  if (config.contains("upper_arm_length")) {
    transformer_->updateUpperArmLength(config["upper_arm_length"].get<double>());
  }
  if (config.contains("lower_arm_length")) {
    transformer_->updateLowerArmLength(config["lower_arm_length"].get<double>());
  }
  if (config.contains("shoulder_width")) {
    transformer_->updateShoulderWidth(config["shoulder_width"].get<double>());
  }
  if (config.contains("base_height_offset")) {
    transformer_->updateBaseHeightOffset(config["base_height_offset"].get<double>());
  }

  plant_ = &plant;
  twoStageIk_ = std::make_unique<TwoStageTorsoIK>(
      &plant, frameNames, 1.0e-8, 1.0e-6, 100, ArmIdx::BOTH, true);
  // 使用 plant.CreateDefaultContext() 创建根 context，不能 clone 子 context
  twoStageIk_->setPlantContext(plant.CreateDefaultContext());

  return true;
}

void Quest3IkAPI::onBonePoses(const PoseInfoList& bone_poses) {
  std::lock_guard<std::mutex> lock(bonePosesMutex_);
  latestBonePoses_ = bone_poses;
  hasBonePoses_ = true;
}

void Quest3IkAPI::onJoystick(const JoyStickData& joystick) {
  std::lock_guard<std::mutex> lock(joystickMutex_);
  latestJoystick_ = joystick;
  if (transformer_) {
    transformer_->updateJoystickData(joystick.left_trigger, joystick.left_grip,
                                     joystick.right_trigger, joystick.right_grip);
  }
}

void Quest3IkAPI::setArmMode(int mode) {
  if (transformer_) {
    transformer_->setRunning(mode == 2);
  }
}

void Quest3IkAPI::runOnce() {
  if (!transformer_ || !twoStageIk_) return;

  PoseInfoList bonePoses;
  {
    std::lock_guard<std::mutex> lock(bonePosesMutex_);
    if (!hasBonePoses_) return;
    bonePoses = latestBonePoses_;
  }

  if (headBodyPoseCallback_ && transformer_->isRunning()) {
    headBodyPoseCallback_(transformer_->getHeadBodyPose());
  }

  if (!transformer_->isRunning()) return;

  PoseInfoList handElbowOutput;
  if (!transformer_->updateHandPoseAndElbowPosition(bonePoses, handElbowOutput)) return;
  if (handElbowOutput.poses.size() < 4) return;

  std::vector<PoseData> poseDataList(POSE_DATA_LIST_SIZE);
  const auto& chestPose = bonePoses.poses[POSE_INDEX_CHEST];
  poseDataList[POSE_DATA_LIST_INDEX_CHEST].position = chestPose.position;
  poseDataList[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix =
      chestPose.orientation.toRotationMatrix();

  const auto& lHand = handElbowOutput.poses[0];
  poseDataList[POSE_DATA_LIST_INDEX_LEFT_HAND].position = lHand.position;
  poseDataList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix =
      lHand.orientation.toRotationMatrix();

  const auto& rHand = handElbowOutput.poses[2];
  poseDataList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = rHand.position;
  poseDataList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix =
      rHand.orientation.toRotationMatrix();

  const auto& lElbow = handElbowOutput.poses[1];
  poseDataList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = lElbow.position;
  poseDataList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].rotation_matrix =
      Eigen::Matrix3d::Identity();

  const auto& rElbow = handElbowOutput.poses[3];
  poseDataList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = rElbow.position;
  poseDataList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].rotation_matrix =
      Eigen::Matrix3d::Identity();

  auto [success, q] = twoStageIk_->solveTwoStageIK(poseDataList, ctrlArmIdx_);
  if (success && armJointCallback_ && q.size() >= 14) {
    std::vector<double> qVec(q.data(), q.data() + 14);
    armJointCallback_(qVec);
  }
}

}  // namespace ik
}  // namespace leju
