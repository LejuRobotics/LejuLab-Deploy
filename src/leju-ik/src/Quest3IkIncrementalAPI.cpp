#include "leju-ik/Quest3IkIncrementalAPI.h"

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <tuple>

namespace leju {
namespace ik {

namespace {

bool shouldLogIk(const char* key, std::chrono::seconds interval = std::chrono::seconds(1)) {
  static std::map<std::string, std::chrono::steady_clock::time_point> lastLog;
  const auto now = std::chrono::steady_clock::now();
  auto it = lastLog.find(key);
  if (it == lastLog.end() || now - it->second >= interval) {
    lastLog[key] = now;
    return true;
  }
  return false;
}

void mergeVectorNode(const YAML::Node& src, nlohmann::json& dst, const char* key) {
  if (!src[key] || !src[key].IsSequence()) {
    return;
  }
  std::vector<double> values;
  for (size_t i = 0; i < src[key].size(); ++i) {
    values.push_back(src[key][i].as<double>());
  }
  dst[key] = values;
}

void mergeScalarNode(const YAML::Node& src, nlohmann::json& dst, const char* key, bool asBool = false) {
  if (!src[key] || !src[key].IsScalar()) {
    return;
  }
  if (asBool) {
    dst[key] = src[key].as<bool>();
  } else {
    dst[key] = src[key].as<double>();
  }
}

void mergeQuest3YamlIntoConfig(const std::string& yamlPath, nlohmann::json& config) {
  try {
    const YAML::Node yaml = YAML::LoadFile(yamlPath);
    const YAML::Node q3 = yaml["quest3"];
    if (!q3) {
      return;
    }

    mergeScalarNode(q3, config, "fhan_r");
    mergeScalarNode(q3, config, "fhan_kh0");
    mergeVectorNode(q3, config, "delta_scale");
    mergeScalarNode(q3, config, "max_pos_diff");
    mergeScalarNode(q3, config, "arm_move_threshold");
    mergeScalarNode(q3, config, "use_incremental_hand_orientation", true);
    mergeVectorNode(q3, config, "zyx_limits_final");
    mergeVectorNode(q3, config, "zyx_limits_ee");
    mergeVectorNode(q3, config, "zyx_limits_link4");
    mergeScalarNode(q3, config, "sphere_radius_limit");
    mergeScalarNode(q3, config, "min_reachable_distance");
    mergeScalarNode(q3, config, "elbow_min_distance");
    mergeScalarNode(q3, config, "elbow_max_distance");
    mergeVectorNode(q3, config, "box_min_bound");
    mergeVectorNode(q3, config, "box_max_bound");
    mergeScalarNode(q3, config, "chest_offset_y_ax");
    mergeVectorNode(q3, config, "left_center");
    mergeVectorNode(q3, config, "right_center");
    mergeVectorNode(q3, config, "default_left_hand_pos_on_exit");
    mergeVectorNode(q3, config, "default_right_hand_pos_on_exit");
    mergeScalarNode(q3, config, "hand_changing_mode_threshold");

    // Keep the base incremental parameters in YAML, but do not let YAML override
    // Drake runtime tuning when kuavo.json already provides it. This keeps the
    // runtime data path aligned with kuavo-ros-control.
  } catch (const std::exception& e) {
    std::cerr << "[Quest3IkIncrementalAPI] Failed to load quest3_incremental_ik.yaml: "
              << e.what() << std::endl;
  }
}

Eigen::Vector3d jsonVector3OrDefault(const nlohmann::json& config,
                                     const char* key,
                                     const Eigen::Vector3d& fallback) {
  if (!config.contains(key) || !config[key].is_array() || config[key].size() < 3) {
    return fallback;
  }
  return Eigen::Vector3d(config[key][0].get<double>(),
                         config[key][1].get<double>(),
                         config[key][2].get<double>());
}

}  // namespace

Quest3IkIncrementalAPI::Quest3IkIncrementalAPI() = default;

Quest3IkIncrementalAPI::~Quest3IkIncrementalAPI() = default;

bool Quest3IkIncrementalAPI::init(const std::string& urdf_path, const nlohmann::json& config) {
  return buildPlantAndIK(urdf_path, config, "kuavo_45");
}

bool Quest3IkIncrementalAPI::init(const std::string& assets_path,
                                  uint16_t major,
                                  uint16_t minor,
                                  const nlohmann::json& config) {
  return init(assets_path, assets_path, major, minor, config, std::string());
}

bool Quest3IkIncrementalAPI::init(const std::string& models_base,
                                  const std::string& config_base,
                                  uint16_t major,
                                  uint16_t minor,
                                  const nlohmann::json& config,
                                  const std::string& quest3_yaml_path) {
  const std::string version_str = std::to_string(major) + std::to_string(minor);
  const std::string urdf_path =
      models_base + "/models/biped_s" + version_str + "/urdf/drake/biped_v3_arm.urdf";

  nlohmann::json mergedConfig = config;
  if (config.empty()) {
    const std::string config_path = config_base + "/config/kuavo_v" + version_str + "/kuavo.json";
    std::ifstream f(config_path);
    if (f.good()) {
      mergedConfig = nlohmann::json::parse(f);
    }
  }

  const std::string yaml_path = quest3_yaml_path.empty()
                                    ? (config_base + "/config/quest3_incremental_ik.yaml")
                                    : quest3_yaml_path;
  mergeQuest3YamlIntoConfig(yaml_path, mergedConfig);
  return buildPlantAndIK(urdf_path, mergedConfig, "kuavo_" + version_str);
}

std::vector<std::string> Quest3IkIncrementalAPI::loadFrameNamesFromConfig(const nlohmann::json& config) {
  if (config.contains("end_frames_name_ik") && config["end_frames_name_ik"].is_array()) {
    return config["end_frames_name_ik"].get<std::vector<std::string>>();
  }
  return {"base_link", "zarm_l7_end_effector", "zarm_r7_end_effector", "zarm_l4_link", "zarm_r4_link"};
}

DrakeVelocityIKWeightConfig Quest3IkIncrementalAPI::loadDrakeVelocityIKWeightsFromJson(
    const nlohmann::json& config) {
  DrakeVelocityIKWeightConfig weights = DrakeVelocityIKWeights::HandPriority;
  weights.name = "JSON_Loaded";
  if (config.contains("drake_velocity_ik_weights") && config["drake_velocity_ik_weights"].is_object()) {
    const auto& node = config["drake_velocity_ik_weights"];
    if (node.contains("q11")) weights.q11 = node["q11"].get<double>();
    if (node.contains("q12")) weights.q12 = node["q12"].get<double>();
    if (node.contains("q2")) weights.q2 = node["q2"].get<double>();
    if (node.contains("qv1")) weights.qv1 = node["qv1"].get<double>();
    if (node.contains("qv2")) weights.qv2 = node["qv2"].get<double>();
  }
  return weights;
}

DrakeVelocityIKBoundsConfig Quest3IkIncrementalAPI::loadDrakeVelocityIKBoundsFromJson(
    const nlohmann::json& config) {
  DrakeVelocityIKBoundsConfig bounds;
  if (config.contains("drake_velocity_ik_bounds") && config["drake_velocity_ik_bounds"].is_object()) {
    const auto& node = config["drake_velocity_ik_bounds"];
    if (node.contains("x_upper_offset")) bounds.xUpperOffset = node["x_upper_offset"].get<double>();
    if (node.contains("z_lower")) bounds.zLower = node["z_lower"].get<double>();
    if (node.contains("z_upper_offset")) bounds.zUpperOffset = node["z_upper_offset"].get<double>();
    if (node.contains("left_y_lower")) bounds.leftYLower = node["left_y_lower"].get<double>();
    if (node.contains("left_y_upper_offset")) {
      bounds.leftYUpperOffset = node["left_y_upper_offset"].get<double>();
    }
    if (node.contains("right_y_lower_offset")) {
      bounds.rightYLowerOffset = node["right_y_lower_offset"].get<double>();
    }
    if (node.contains("right_y_upper")) bounds.rightYUpper = node["right_y_upper"].get<double>();
  }
  return bounds;
}

PointTrackIKSolverConfig Quest3IkIncrementalAPI::loadPointTrackIKSolverConfigFromJson(
    const nlohmann::json& config) {
  PointTrackIKSolverConfig pointTrackConfig;
  if (!config.contains("point_track_ik_solver_config") ||
      !config["point_track_ik_solver_config"].is_object()) {
    return pointTrackConfig;
  }

  const auto& ikConfig = config["point_track_ik_solver_config"];
  if (ikConfig.contains("historyBufferSize")) {
    pointTrackConfig.historyBufferSize = ikConfig["historyBufferSize"].get<int>();
  }
  if (ikConfig.contains("eeTrackingWeight")) {
    pointTrackConfig.eeTrackingWeight = ikConfig["eeTrackingWeight"].get<double>();
  }
  if (ikConfig.contains("elbowTrackingWeight")) {
    pointTrackConfig.elbowTrackingWeight = ikConfig["elbowTrackingWeight"].get<double>();
  }
  if (ikConfig.contains("link6TrackingWeight")) {
    pointTrackConfig.link6TrackingWeight = ikConfig["link6TrackingWeight"].get<double>();
  }
  if (ikConfig.contains("virtualThumbTrackingWeight")) {
    pointTrackConfig.virtualThumbTrackingWeight = ikConfig["virtualThumbTrackingWeight"].get<double>();
  }
  if (ikConfig.contains("constraintTolerance")) {
    pointTrackConfig.constraintTolerance = ikConfig["constraintTolerance"].get<double>();
  }
  if (ikConfig.contains("solverTolerance")) {
    pointTrackConfig.solverTolerance = ikConfig["solverTolerance"].get<double>();
  }
  if (ikConfig.contains("maxIterations")) {
    pointTrackConfig.maxIterations = ikConfig["maxIterations"].get<int>();
  }
  if (ikConfig.contains("controlArmIndex")) {
    const int armIdxInt = ikConfig["controlArmIndex"].get<int>();
    if (armIdxInt == 0) {
      pointTrackConfig.controlArmIndex = ArmIdx::LEFT;
    } else if (armIdxInt == 1) {
      pointTrackConfig.controlArmIndex = ArmIdx::RIGHT;
    } else {
      pointTrackConfig.controlArmIndex = ArmIdx::BOTH;
    }
  }
  if (ikConfig.contains("isWeldBaseLink")) {
    pointTrackConfig.isWeldBaseLink = ikConfig["isWeldBaseLink"].get<bool>();
  }
  if (ikConfig.contains("useJointLimits")) {
    pointTrackConfig.useJointLimits = ikConfig["useJointLimits"].get<bool>();
  }
  if (ikConfig.contains("jointSmoothWeightDefault")) {
    pointTrackConfig.jointSmoothWeightDefault = ikConfig["jointSmoothWeightDefault"].get<double>();
  }
  if (ikConfig.contains("jointSmoothWeight0")) {
    pointTrackConfig.jointSmoothWeight0 = ikConfig["jointSmoothWeight0"].get<double>();
  }
  if (ikConfig.contains("jointSmoothWeight1")) {
    pointTrackConfig.jointSmoothWeight1 = ikConfig["jointSmoothWeight1"].get<double>();
  }
  if (ikConfig.contains("jointSmoothWeight2")) {
    pointTrackConfig.jointSmoothWeight2 = ikConfig["jointSmoothWeight2"].get<double>();
  }
  if (ikConfig.contains("jointSmoothWeight3")) {
    pointTrackConfig.jointSmoothWeight3 = ikConfig["jointSmoothWeight3"].get<double>();
  }
  if (ikConfig.contains("jointSmoothWeight4")) {
    pointTrackConfig.jointSmoothWeight4 = ikConfig["jointSmoothWeight4"].get<double>();
  }
  if (ikConfig.contains("jointSmoothWeight5")) {
    pointTrackConfig.jointSmoothWeight5 = ikConfig["jointSmoothWeight5"].get<double>();
  }
  if (ikConfig.contains("jointSmoothWeight6")) {
    pointTrackConfig.jointSmoothWeight6 = ikConfig["jointSmoothWeight6"].get<double>();
  }
  return pointTrackConfig;
}

void Quest3IkIncrementalAPI::loadDrakeVelocityIKGeometryFromJson(const nlohmann::json& config) {
  l1_ = 0.2837;
  l2_ = 0.2335;
  robotLeftFixedShoulderPos_ = Eigen::Vector3d(-0.017499853, 0.2927, 0.4245);
  robotRightFixedShoulderPos_ = Eigen::Vector3d(-0.017499853, -0.2927, 0.4245);

  if (config.contains("drake_velocity_ik_geometry") && config["drake_velocity_ik_geometry"].is_object()) {
    const auto& node = config["drake_velocity_ik_geometry"];
    if (node.contains("l1")) l1_ = node["l1"].get<double>();
    if (node.contains("l2")) l2_ = node["l2"].get<double>();
    if (node.contains("left_p0") && node["left_p0"].is_array() && node["left_p0"].size() == 3) {
      robotLeftFixedShoulderPos_ = Eigen::Vector3d(node["left_p0"][0].get<double>(),
                                                   node["left_p0"][1].get<double>(),
                                                   node["left_p0"][2].get<double>());
    }
    if (node.contains("right_p0") && node["right_p0"].is_array() && node["right_p0"].size() == 3) {
      robotRightFixedShoulderPos_ = Eigen::Vector3d(node["right_p0"][0].get<double>(),
                                                    node["right_p0"][1].get<double>(),
                                                    node["right_p0"][2].get<double>());
    }
  }
}

bool Quest3IkIncrementalAPI::buildPlantAndIK(const std::string& urdf_path,
                                             const nlohmann::json& config,
                                             const std::string& robotModel) {
  auto diagramBuilder = std::make_unique<drake::systems::DiagramBuilder<double>>();
  auto [plant, sceneGraph] =
      drake::multibody::AddMultibodyPlantSceneGraph(diagramBuilder.get(), 0.0);

  drake::multibody::Parser parser(&plant);
  parser.AddModelFromFile(urdf_path);

  std::string base_frame_name = "base_link";
  try {
    plant.GetFrameByName(base_frame_name);
  } catch (const std::logic_error&) {
    base_frame_name = "torso";
  }
  const auto& baseFrame = plant.GetFrameByName(base_frame_name);
  plant.WeldFrames(plant.world_frame(), baseFrame);
  plant.Finalize();

  diagram_ = diagramBuilder->Build();
  diagramContext_ = diagram_->CreateDefaultContext();
  plant_ = &plant;

  std::vector<std::string> frameNames = loadFrameNamesFromConfig(config);
  if (!frameNames.empty() && frameNames[0] == "base_link" && base_frame_name == "torso") {
    frameNames[0] = "torso";
  }

  auto resolveFrame = [&plant](const std::string& preferred, const std::string& fallback) {
    try {
      plant.GetFrameByName(preferred);
      return preferred;
    } catch (const std::logic_error&) {
      return fallback;
    }
  };
  if (frameNames.size() > 1) {
    frameNames[1] = resolveFrame("zarm_l7_end_effector", "zarm_l7_link");
  }
  if (frameNames.size() > 2) {
    frameNames[2] = resolveFrame("zarm_r7_end_effector", "zarm_r7_link");
  }

  transformer_ = std::make_unique<Quest3ArmInfoTransformer>(robotModel);
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

  PointTrackIKSolverConfig pointTrackConfig = loadPointTrackIKSolverConfigFromJson(config);
  oneStageIk_ = std::make_unique<OneStageIKEndEffector>(&plant, frameNames, pointTrackConfig);

  IncrementalControlConfig incrementalConfig;
  incrementalConfig.publishRate = publishRate_;
  if (config.contains("fhan_r")) incrementalConfig.fhanR = config["fhan_r"].get<double>();
  if (config.contains("fhan_kh0")) incrementalConfig.fhanKh0 = config["fhan_kh0"].get<double>();
  if (config.contains("delta_scale") && config["delta_scale"].is_array() && config["delta_scale"].size() >= 3) {
    incrementalConfig.deltaScale << config["delta_scale"][0].get<double>(),
        config["delta_scale"][1].get<double>(), config["delta_scale"][2].get<double>();
  }
  if (config.contains("max_pos_diff")) incrementalConfig.maxPosDiff = config["max_pos_diff"].get<double>();
  if (config.contains("arm_move_threshold")) {
    incrementalConfig.armMoveThreshold = config["arm_move_threshold"].get<double>();
  }
  if (config.contains("use_incremental_hand_orientation")) {
    incrementalConfig.usePythonIncrementalOrientation =
        config["use_incremental_hand_orientation"].get<bool>();
  }
  if (config.contains("zyx_limits_final") && config["zyx_limits_final"].is_array() &&
      config["zyx_limits_final"].size() >= 3) {
    incrementalConfig.zyxLimitsFinal << config["zyx_limits_final"][0].get<double>(),
        config["zyx_limits_final"][1].get<double>(),
        config["zyx_limits_final"][2].get<double>();
  }
  if (config.contains("zyx_limits_ee") && config["zyx_limits_ee"].is_array() &&
      config["zyx_limits_ee"].size() >= 3) {
    incrementalConfig.zyxLimitsEE << config["zyx_limits_ee"][0].get<double>(),
        config["zyx_limits_ee"][1].get<double>(), config["zyx_limits_ee"][2].get<double>();
  }
  if (config.contains("zyx_limits_link4") && config["zyx_limits_link4"].is_array() &&
      config["zyx_limits_link4"].size() >= 3) {
    incrementalConfig.zyxLimitsLink4 << config["zyx_limits_link4"][0].get<double>(),
        config["zyx_limits_link4"][1].get<double>(),
        config["zyx_limits_link4"][2].get<double>();
  }

  incrementalController_ = std::make_unique<IncrementalControlModule>(incrementalConfig);
  transformer_->setDeltaScale(incrementalConfig.deltaScale);
  useIncrementalHandOrientation_ = incrementalConfig.usePythonIncrementalOrientation;

  defaultLeftHandPosOnExit_ =
      jsonVector3OrDefault(config, "default_left_hand_pos_on_exit", defaultLeftHandPosOnExit_);
  defaultRightHandPosOnExit_ =
      jsonVector3OrDefault(config, "default_right_hand_pos_on_exit", defaultRightHandPosOnExit_);
  if (config.contains("hand_changing_mode_threshold") &&
      config["hand_changing_mode_threshold"].is_number()) {
    handChangingModeThreshold_ = config["hand_changing_mode_threshold"].get<double>();
  }
  leftHandSmoother_ =
      std::make_unique<HandSmoother>("左臂", "zarm_l6_link", defaultLeftHandPosOnExit_);
  rightHandSmoother_ =
      std::make_unique<HandSmoother>("右臂", "zarm_r6_link", defaultRightHandPosOnExit_);

  loadDrakeVelocityIKGeometryFromJson(config);
  const DrakeVelocityIKWeightConfig drakeWeights = loadDrakeVelocityIKWeightsFromJson(config);
  const DrakeVelocityIKBoundsConfig drakeBounds = loadDrakeVelocityIKBoundsFromJson(config);

  const Eigen::Vector3d leftLb(robotLeftFixedShoulderPos_.x(),
                               drakeBounds.leftYLower,
                               drakeBounds.zLower);
  const Eigen::Vector3d leftUb(robotLeftFixedShoulderPos_.x() + drakeBounds.xUpperOffset,
                               robotLeftFixedShoulderPos_.y() + drakeBounds.leftYUpperOffset,
                               robotLeftFixedShoulderPos_.z() + drakeBounds.zUpperOffset);
  leftVelocityIkSolverPtr_ = std::make_unique<DrakeVelocityIKSolver>(robotLeftFixedShoulderPos_, l1_, l2_,
                                                                     leftLb, leftUb);
  leftVelocityIkSolverPtr_->setWeights(drakeWeights);

  const Eigen::Vector3d rightLb(robotRightFixedShoulderPos_.x(),
                                robotRightFixedShoulderPos_.y() + drakeBounds.rightYLowerOffset,
                                drakeBounds.zLower);
  const Eigen::Vector3d rightUb(robotRightFixedShoulderPos_.x() + drakeBounds.xUpperOffset,
                                drakeBounds.rightYUpper,
                                robotRightFixedShoulderPos_.z() + drakeBounds.zUpperOffset);
  rightVelocityIkSolverPtr_ = std::make_unique<DrakeVelocityIKSolver>(robotRightFixedShoulderPos_, l1_, l2_,
                                                                      rightLb, rightUb);
  rightVelocityIkSolverPtr_->setWeights(drakeWeights);

  Eigen::VectorXd armJoints = Eigen::VectorXd::Zero(14);
  auto [leftLink6Pos, leftLink6Quat] = oneStageIk_->FK(armJoints, "zarm_l6_link", 14);
  auto [rightLink6Pos, rightLink6Quat] = oneStageIk_->FK(armJoints, "zarm_r6_link", 14);
  auto [leftEEPos, leftEEQuat] = oneStageIk_->FK(armJoints, frameNames[1], 14);
  auto [rightEEPos, rightEEQuat] = oneStageIk_->FK(armJoints, frameNames[2], 14);

  leftEE2Link6Offset_ = leftEEPos - leftLink6Pos;
  rightEE2Link6Offset_ = rightEEPos - rightLink6Pos;
  leftThumb2Link6Offset_ = Eigen::Vector3d(0.15, 0.0, 0.0);
  rightThumb2Link6Offset_ = Eigen::Vector3d(0.15, 0.0, 0.0);

  initializePoseConstraintList();
  sensorArmJoints_ = Eigen::VectorXd::Zero(14);
  filteredSensorArmJoints_ = Eigen::VectorXd::Zero(14);
  jointMidValues_.resize(0);
  incrementalController_->setHandQuatSeeds(Eigen::Quaterniond::Identity(),
                                           Eigen::Quaterniond::Identity(),
                                           useIncrementalHandOrientation_);
  return true;
}

void Quest3IkIncrementalAPI::initializePoseConstraintList() {
  latestPoseConstraintList_.assign(POSE_DATA_LIST_SIZE_PLUS, PoseData());
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = Eigen::Vector3d::Zero();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = Eigen::Matrix3d::Identity();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = defaultLeftHandPosOnExit_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = Eigen::Matrix3d::Identity();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = defaultRightHandPosOnExit_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = Eigen::Matrix3d::Identity();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = leftElbowFixedPoint_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].rotation_matrix = Eigen::Matrix3d::Identity();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = rightElbowFixedPoint_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].rotation_matrix = Eigen::Matrix3d::Identity();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_LINK6].position = defaultLeftHandPosOnExit_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_LINK6].rotation_matrix = Eigen::Matrix3d::Identity();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position = defaultRightHandPosOnExit_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_LINK6].rotation_matrix = Eigen::Matrix3d::Identity();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].position =
      defaultLeftHandPosOnExit_ + leftThumb2Link6Offset_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].rotation_matrix =
      Eigen::Matrix3d::Identity();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].position =
      defaultRightHandPosOnExit_ + rightThumb2Link6Offset_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].rotation_matrix =
      Eigen::Matrix3d::Identity();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position =
      defaultLeftHandPosOnExit_ + leftEE2Link6Offset_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].rotation_matrix =
      Eigen::Matrix3d::Identity();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].position =
      defaultRightHandPosOnExit_ + rightEE2Link6Offset_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].rotation_matrix =
      Eigen::Matrix3d::Identity();

  leftLink6Position_ = defaultLeftHandPosOnExit_;
  rightLink6Position_ = defaultRightHandPosOnExit_;
  leftEndEffectorPosition_ = defaultLeftHandPosOnExit_ + leftEE2Link6Offset_;
  rightEndEffectorPosition_ = defaultRightHandPosOnExit_ + rightEE2Link6Offset_;
  leftVirtualThumbPosition_ = defaultLeftHandPosOnExit_ + leftThumb2Link6Offset_;
  rightVirtualThumbPosition_ = defaultRightHandPosOnExit_ + rightThumb2Link6Offset_;
  latestHumanLeftElbowPos_ = leftElbowFixedPoint_;
  latestHumanRightElbowPos_ = rightElbowFixedPoint_;
  latestRobotLeftElbowPos_ = leftElbowFixedPoint_;
  latestRobotRightElbowPos_ = rightElbowFixedPoint_;
}

void Quest3IkIncrementalAPI::onBonePoses(const PoseInfoList& bone_poses) {
  std::lock_guard<std::mutex> lock(bonePosesMutex_);
  latestBonePoses_ = bone_poses;
  hasBonePoses_ = true;
}

void Quest3IkIncrementalAPI::onJoystick(const JoyStickData& joystick) {
  std::lock_guard<std::mutex> lock(joystickMutex_);
  latestJoystick_ = joystick;
  if (transformer_) {
    transformer_->updateJoystickData(joystick.left_trigger, joystick.left_grip,
                                     joystick.right_trigger, joystick.right_grip);
  }
}

void Quest3IkIncrementalAPI::onArmCtrlModeState(const ArmControlModeState& state) {
  std::lock_guard<std::mutex> lock(armCtrlModeMutex_);
  latestArmCtrlModeState_ = state;
  leftArmCtrlModeActive_ = state.left_active;
  rightArmCtrlModeActive_ = state.right_active;
  if (state.left_changed) {
    pendingLeftArmCtrlModeChanged_ = true;
  }
  if (state.right_changed) {
    pendingRightArmCtrlModeChanged_ = true;
  }
}

void Quest3IkIncrementalAPI::onSensorArmJoints(const std::vector<double>& q_rad) {
  std::lock_guard<std::mutex> lock(sensorJointsMutex_);
  if (q_rad.size() < 14) {
    return;
  }
  sensorArmJoints_.resize(14);
  if (filteredSensorArmJoints_.size() != 14) {
    filteredSensorArmJoints_.resize(14);
    filteredSensorArmJoints_.setZero();
  }
  for (int i = 0; i < 14; ++i) {
    sensorArmJoints_(i) = q_rad[i];
  }
  if (!hasSensorJoints_) {
    filteredSensorArmJoints_ = sensorArmJoints_;
  } else {
    static constexpr double kKeep = 0.92;
    static constexpr double kNew = 0.08;
    filteredSensorArmJoints_ = kKeep * filteredSensorArmJoints_ + kNew * sensorArmJoints_;
  }
  hasSensorJoints_ = true;
}

void Quest3IkIncrementalAPI::setArmMode(int mode) {
  int previousMode = 0;
  int currentMode = 0;
  bool pendingEnter = false;
  bool pendingExit = false;
  {
    std::lock_guard<std::mutex> lock(modeMutex_);
    if (mode == currentArmMode_) {
      if (transformer_) {
        transformer_->setRunning(mode == 2);
      }
      return;
    }

    previousArmMode_ = currentArmMode_;
    currentArmMode_ = mode;
    if (previousArmMode_ != 2 && currentArmMode_ == 2) {
      pendingEnterMode2_ = true;
      pendingExitMode2_ = false;
    } else if (previousArmMode_ == 2 && currentArmMode_ != 2) {
      pendingExitMode2_ = true;
      pendingEnterMode2_ = false;
    }

    previousMode = previousArmMode_;
    currentMode = currentArmMode_;
    pendingEnter = pendingEnterMode2_;
    pendingExit = pendingExitMode2_;
  }

  if (transformer_) {
    transformer_->setRunning(mode == 2);
  }

  std::cout << "[IK] setArmMode: previous=" << previousMode
            << " current=" << currentMode
            << " pendingEnterMode2=" << (pendingEnter ? "1" : "0")
            << " pendingExitMode2=" << (pendingExit ? "1" : "0") << std::endl;
}

bool Quest3IkIncrementalAPI::hasValidSensorJoints(Eigen::VectorXd& qForFK) {
  std::lock_guard<std::mutex> lock(sensorJointsMutex_);
  if (hasSensorJoints_ && filteredSensorArmJoints_.size() == 14) {
    qForFK = filteredSensorArmJoints_;
    return true;
  }
  if (hasSensorJoints_ && sensorArmJoints_.size() == 14) {
    qForFK = sensorArmJoints_;
    return true;
  }
  qForFK.resize(0);
  return false;
}

bool Quest3IkIncrementalAPI::computeLeftEndEffectorFK(const Eigen::VectorXd& q,
                                                      Eigen::Vector3d& pOut,
                                                      Eigen::Quaterniond& qOut) {
  if (q.size() != 14) {
    return false;
  }
  std::tie(pOut, qOut) = oneStageIk_->FK(q, "zarm_l7_end_effector", 14);
  return true;
}

bool Quest3IkIncrementalAPI::computeRightEndEffectorFK(const Eigen::VectorXd& q,
                                                       Eigen::Vector3d& pOut,
                                                       Eigen::Quaterniond& qOut) {
  if (q.size() != 14) {
    return false;
  }
  std::tie(pOut, qOut) = oneStageIk_->FK(q, "zarm_r7_end_effector", 14);
  return true;
}

bool Quest3IkIncrementalAPI::computeLeftLink4FK(const Eigen::VectorXd& q,
                                                Eigen::Vector3d& pOut,
                                                Eigen::Quaterniond& qOut) {
  if (q.size() != 14) {
    return false;
  }
  std::tie(pOut, qOut) = oneStageIk_->FK(q, "zarm_l4_link", 14);
  return true;
}

bool Quest3IkIncrementalAPI::computeRightLink4FK(const Eigen::VectorXd& q,
                                                 Eigen::Vector3d& pOut,
                                                 Eigen::Quaterniond& qOut) {
  if (q.size() != 14) {
    return false;
  }
  std::tie(pOut, qOut) = oneStageIk_->FK(q, "zarm_r4_link", 14);
  return true;
}

bool Quest3IkIncrementalAPI::computeLeftLink6FK(const Eigen::VectorXd& q,
                                                Eigen::Vector3d& pOut,
                                                Eigen::Quaterniond& qOut) {
  if (q.size() != 14) {
    return false;
  }
  std::tie(pOut, qOut) = oneStageIk_->FK(q, "zarm_l6_link", 14);
  return true;
}

bool Quest3IkIncrementalAPI::computeRightLink6FK(const Eigen::VectorXd& q,
                                                 Eigen::Vector3d& pOut,
                                                 Eigen::Quaterniond& qOut) {
  if (q.size() != 14) {
    return false;
  }
  std::tie(pOut, qOut) = oneStageIk_->FK(q, "zarm_r6_link", 14);
  return true;
}

void Quest3IkIncrementalAPI::syncRuntimeStateFromCurrentFk(const Eigen::Vector3d& leftLink4Pos,
                                                           const Eigen::Quaterniond& leftLink4Quat,
                                                           const Eigen::Vector3d& rightLink4Pos,
                                                           const Eigen::Quaterniond& rightLink4Quat,
                                                           const Eigen::Vector3d& leftLink6Pos,
                                                           const Eigen::Quaterniond& leftLink6Quat,
                                                           const Eigen::Vector3d& rightLink6Pos,
                                                           const Eigen::Quaterniond& rightLink6Quat,
                                                           const Eigen::Vector3d& leftEEPos,
                                                           const Eigen::Quaterniond& leftEEQuat,
                                                           const Eigen::Vector3d& rightEEPos,
                                                           const Eigen::Quaterniond& rightEEQuat) {
  leftLink6Position_ = leftLink6Pos;
  rightLink6Position_ = rightLink6Pos;
  leftEndEffectorPosition_ = leftEEPos;
  rightEndEffectorPosition_ = rightEEPos;
  leftVirtualThumbPosition_ = leftLink6Pos + leftLink6Quat * leftThumb2Link6Offset_;
  rightVirtualThumbPosition_ = rightLink6Pos + rightLink6Quat * rightThumb2Link6Offset_;

  leftLink4Quat_ = leftLink4Quat;
  rightLink4Quat_ = rightLink4Quat;
  leftLink6Quat_ = leftLink6Quat;
  rightLink6Quat_ = rightLink6Quat;
  leftEndEffectorQuat_ = leftEEQuat;
  rightEndEffectorQuat_ = rightEEQuat;

  latestHumanLeftElbowPos_ = leftLink4Pos;
  latestHumanRightElbowPos_ = rightLink4Pos;
  latestRobotLeftElbowPos_ = leftLink4Pos;
  latestRobotRightElbowPos_ = rightLink4Pos;
}

void Quest3IkIncrementalAPI::rebuildPoseConstraintListFromCurrentFk(
    const Eigen::Vector3d& leftLink4Pos,
    const Eigen::Quaterniond& leftLink4Quat,
    const Eigen::Vector3d& rightLink4Pos,
    const Eigen::Quaterniond& rightLink4Quat,
    const Eigen::Vector3d& leftLink6Pos,
    const Eigen::Quaterniond& leftLink6Quat,
    const Eigen::Vector3d& rightLink6Pos,
    const Eigen::Quaterniond& rightLink6Quat,
    const Eigen::Vector3d& leftEEPos,
    const Eigen::Quaterniond& leftEEQuat,
    const Eigen::Vector3d& rightEEPos,
    const Eigen::Quaterniond& rightEEQuat) {
  if (latestPoseConstraintList_.size() < POSE_DATA_LIST_SIZE_PLUS) {
    initializePoseConstraintList();
  }

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].position = Eigen::Vector3d::Zero();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = Eigen::Matrix3d::Identity();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = leftLink6Pos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix =
      leftLink6Quat.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = rightLink6Pos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix =
      rightLink6Quat.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = leftLink4Pos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].rotation_matrix =
      leftLink4Quat.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = rightLink4Pos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].rotation_matrix =
      rightLink4Quat.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_LINK6].position = leftLink6Pos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_LINK6].rotation_matrix =
      leftLink6Quat.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position = rightLink6Pos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_LINK6].rotation_matrix =
      rightLink6Quat.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].position =
      leftLink6Pos + leftLink6Quat * leftThumb2Link6Offset_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].rotation_matrix =
      leftLink6Quat.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].position =
      rightLink6Pos + rightLink6Quat * rightThumb2Link6Offset_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].rotation_matrix =
      rightLink6Quat.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position = leftEEPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].rotation_matrix =
      leftEEQuat.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].position = rightEEPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].rotation_matrix =
      rightEEQuat.toRotationMatrix();

  syncRuntimeStateFromCurrentFk(leftLink4Pos, leftLink4Quat, rightLink4Pos, rightLink4Quat,
                                leftLink6Pos, leftLink6Quat, rightLink6Pos, rightLink6Quat,
                                leftEEPos, leftEEQuat, rightEEPos, rightEEQuat);
}

void Quest3IkIncrementalAPI::updateLeftConstraintList(const Eigen::Vector3d& leftHandPos,
                                                      const Eigen::Quaterniond& leftHandQuat,
                                                      const Eigen::Vector3d& leftElbowPos) {
  if (latestPoseConstraintList_.size() < POSE_DATA_LIST_SIZE_PLUS) {
    initializePoseConstraintList();
  }

  leftLink6Quat_ = leftHandQuat.normalized();
  leftEndEffectorQuat_ = leftHandQuat.normalized();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position = leftHandPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix =
      leftLink6Quat_.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = leftElbowPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_ELBOW].rotation_matrix =
      leftLink4Quat_.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_LINK6].position = leftLink6Position_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_LINK6].rotation_matrix =
      leftLink6Quat_.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].position = leftVirtualThumbPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].rotation_matrix =
      leftLink6Quat_.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position = leftEndEffectorPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].rotation_matrix =
      leftEndEffectorQuat_.toRotationMatrix();
}

void Quest3IkIncrementalAPI::updateRightConstraintList(const Eigen::Vector3d& rightHandPos,
                                                       const Eigen::Quaterniond& rightHandQuat,
                                                       const Eigen::Vector3d& rightElbowPos) {
  if (latestPoseConstraintList_.size() < POSE_DATA_LIST_SIZE_PLUS) {
    initializePoseConstraintList();
  }

  rightLink6Quat_ = rightHandQuat.normalized();
  rightEndEffectorQuat_ = rightHandQuat.normalized();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = rightHandPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix =
      rightLink6Quat_.toRotationMatrix();
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = rightElbowPos;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].rotation_matrix =
      rightLink4Quat_.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position = rightLink6Position_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_LINK6].rotation_matrix =
      rightLink6Quat_.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].position = rightVirtualThumbPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].rotation_matrix =
      rightLink6Quat_.toRotationMatrix();

  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].position = rightEndEffectorPosition_;
  latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].rotation_matrix =
      rightEndEffectorQuat_.toRotationMatrix();
}

void Quest3IkIncrementalAPI::buildPoseConstraintListFromIncrementalResult(
    const IncrementalPoseResult& incResult,
    std::vector<PoseData>& out) {
  if (out.size() < POSE_DATA_LIST_SIZE_PLUS) {
    out.resize(POSE_DATA_LIST_SIZE_PLUS, PoseData());
  }

  auto [leftQuat, rightQuat, leftPos, rightPos] =
      incResult.getLatestIncrementalHandPose(true, useIncrementalHandOrientation_, true);
  leftQuat.normalize();
  rightQuat.normalize();

  out[POSE_DATA_LIST_INDEX_CHEST].position = Eigen::Vector3d::Zero();
  out[POSE_DATA_LIST_INDEX_CHEST].rotation_matrix = Eigen::Matrix3d::Identity();

  out[POSE_DATA_LIST_INDEX_LEFT_HAND].position = leftPos;
  out[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = leftQuat.toRotationMatrix();
  out[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = rightPos;
  out[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = rightQuat.toRotationMatrix();

  out[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position = leftElbowFixedPoint_;
  out[POSE_DATA_LIST_INDEX_LEFT_ELBOW].rotation_matrix = Eigen::Matrix3d::Identity();
  out[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position = rightElbowFixedPoint_;
  out[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].rotation_matrix = Eigen::Matrix3d::Identity();

  out[POSE_DATA_LIST_INDEX_LEFT_LINK6].position = leftPos;
  out[POSE_DATA_LIST_INDEX_LEFT_LINK6].rotation_matrix = leftQuat.toRotationMatrix();
  out[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position = rightPos;
  out[POSE_DATA_LIST_INDEX_RIGHT_LINK6].rotation_matrix = rightQuat.toRotationMatrix();

  out[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].position = leftPos + leftQuat * leftThumb2Link6Offset_;
  out[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].rotation_matrix = leftQuat.toRotationMatrix();
  out[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].position =
      rightPos + rightQuat * rightThumb2Link6Offset_;
  out[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].rotation_matrix = rightQuat.toRotationMatrix();

  out[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position = leftPos + leftQuat * leftEE2Link6Offset_;
  out[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].rotation_matrix = leftQuat.toRotationMatrix();
  out[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].position = rightPos + rightQuat * rightEE2Link6Offset_;
  out[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].rotation_matrix = rightQuat.toRotationMatrix();
}

bool Quest3IkIncrementalAPI::detectLeftArmMove(const ArmPose& vrLeftPose) {
  if (incrementalController_->hasLeftArmMoved()) {
    return true;
  }
  return incrementalController_->detectLeftArmMove(vrLeftPose.position);
}

bool Quest3IkIncrementalAPI::detectRightArmMove(const ArmPose& vrRightPose) {
  if (incrementalController_->hasRightArmMoved()) {
    return true;
  }
  return incrementalController_->detectRightArmMove(vrRightPose.position);
}

bool Quest3IkIncrementalAPI::updateLatestIncrementalResult(const ArmPose& vrLeftPose,
                                                           const ArmPose& vrRightPose,
                                                           bool leftCanProcess,
                                                           bool rightCanProcess,
                                                           const Eigen::Quaterniond& qLeftEndEffector,
                                                           const Eigen::Quaterniond& qRightEndEffector) {
  if (shouldLogIk("update_incremental_state")) {
    std::cout << "[IK] updateLatestIncrementalResult: leftCanProcess="
              << (leftCanProcess ? "1" : "0")
              << " rightCanProcess=" << (rightCanProcess ? "1" : "0") << std::endl;
  }

  if (!leftCanProcess && !rightCanProcess) {
    return false;
  }

  if (leftCanProcess) {
    latestIncrementalResult_ =
        incrementalController_->computeIncrementalPoseLeftArm(vrLeftPose, true, qLeftEndEffector);
  }
  if (rightCanProcess) {
    latestIncrementalResult_ =
        incrementalController_->computeIncrementalPoseRightArm(vrRightPose, true, qRightEndEffector);
  }

  latestIncrementalResult_ = incrementalController_->getLatestIncrementalResult();
  return latestIncrementalResult_.isValid();
}

bool Quest3IkIncrementalAPI::processChangingDataLeftArm(bool leftHandCtrlModeChanged) {
  if (!leftHandSmoother_) {
    return false;
  }

  Eigen::Vector3d scaledLeftHandPos =
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
  const Eigen::Vector3d scaledRightHandPos =
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
  (void)scaledRightHandPos;
  const bool isLeftArmCtrlModeActive = leftArmCtrlModeActive_;

  if (leftHandCtrlModeChanged && isLeftArmCtrlModeActive) {
    auto [leftMaintain, leftInstant] = leftHandSmoother_->getModeChangingState();
    bool leftInstantCopy = leftInstant;
    leftHandSmoother_->processActiveModeInterpolation(
        scaledLeftHandPos, leftInstantCopy, leftHandSmoother_->getDefaultPosOnExit(), "左臂");
    leftHandSmoother_->setModeChangingState(leftMaintain, leftInstantCopy);
  }

  if (leftHandCtrlModeChanged && !isLeftArmCtrlModeActive) {
    auto [leftMaintain, leftInstant] = leftHandSmoother_->getModeChangingState();
    bool leftInstantCopy = leftInstant;
    leftHandSmoother_->processInactiveModeInterpolation(
        scaledLeftHandPos, leftInstantCopy, leftHandSmoother_->getDefaultPosOnExit(), "左臂");
    leftHandSmoother_->setModeChangingState(leftMaintain, leftInstantCopy);
  }

  Eigen::Quaterniond latchedLeftQuat(
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix);
  if (latchedLeftQuat.norm() < 1e-6) {
    latchedLeftQuat = leftLink6Quat_;
  } else {
    latchedLeftQuat.normalize();
  }

  const Eigen::Vector3d eeToWristVec = leftLink6Position_ - leftEndEffectorPosition_;
  if (eeToWristVec.norm() > 1e-6) {
    latestHumanLeftElbowPos_ = leftLink6Position_ + eeToWristVec.normalized() * l2_;
  }
  const auto [p1Optimized, p2Optimized] =
      leftVelocityIkSolverPtr_->solve(latestHumanLeftElbowPos_, leftElbowFixedPoint_, scaledLeftHandPos);

  latestRobotLeftElbowPos_ = p1Optimized;
  leftLink6Position_ = p2Optimized;
  leftLink6Quat_ = latchedLeftQuat;
  leftEndEffectorQuat_ = latchedLeftQuat;
  leftEndEffectorPosition_ = p2Optimized + leftLink6Quat_ * leftEE2Link6Offset_;
  leftVirtualThumbPosition_ = p2Optimized + leftLink6Quat_ * leftThumb2Link6Offset_;
  updateLeftConstraintList(p2Optimized, latchedLeftQuat, p1Optimized);
  std::cout << "[IK] processChangingDataLeftArm" << std::endl;
  return true;
}

bool Quest3IkIncrementalAPI::processChangingDataRightArm(bool rightHandCtrlModeChanged) {
  if (!rightHandSmoother_) {
    return false;
  }

  const Eigen::Vector3d scaledLeftHandPos =
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
  Eigen::Vector3d scaledRightHandPos =
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
  (void)scaledLeftHandPos;
  const bool isRightArmCtrlModeActive = rightArmCtrlModeActive_;

  if (rightHandCtrlModeChanged && isRightArmCtrlModeActive) {
    auto [rightMaintain, rightInstant] = rightHandSmoother_->getModeChangingState();
    bool rightInstantCopy = rightInstant;
    rightHandSmoother_->processActiveModeInterpolation(
        scaledRightHandPos, rightInstantCopy, rightHandSmoother_->getDefaultPosOnExit(), "右臂");
    rightHandSmoother_->setModeChangingState(rightMaintain, rightInstantCopy);
  }

  if (rightHandCtrlModeChanged && !isRightArmCtrlModeActive) {
    auto [rightMaintain, rightInstant] = rightHandSmoother_->getModeChangingState();
    bool rightInstantCopy = rightInstant;
    rightHandSmoother_->processInactiveModeInterpolation(
        scaledRightHandPos, rightInstantCopy, rightHandSmoother_->getDefaultPosOnExit(), "右臂");
    rightHandSmoother_->setModeChangingState(rightMaintain, rightInstantCopy);
  }

  Eigen::Quaterniond latchedRightQuat(
      latestPoseConstraintList_[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix);
  if (latchedRightQuat.norm() < 1e-6) {
    latchedRightQuat = rightLink6Quat_;
  } else {
    latchedRightQuat.normalize();
  }

  const Eigen::Vector3d eeToWristVec = rightLink6Position_ - rightEndEffectorPosition_;
  if (eeToWristVec.norm() > 1e-6) {
    latestHumanRightElbowPos_ = rightLink6Position_ + eeToWristVec.normalized() * l2_;
  }
  const auto [p1Optimized, p2Optimized] =
      rightVelocityIkSolverPtr_->solve(latestHumanRightElbowPos_, rightElbowFixedPoint_, scaledRightHandPos);

  latestRobotRightElbowPos_ = p1Optimized;
  rightLink6Position_ = p2Optimized;
  rightLink6Quat_ = latchedRightQuat;
  rightEndEffectorQuat_ = latchedRightQuat;
  rightEndEffectorPosition_ = p2Optimized + rightLink6Quat_ * rightEE2Link6Offset_;
  rightVirtualThumbPosition_ = p2Optimized + rightLink6Quat_ * rightThumb2Link6Offset_;
  updateRightConstraintList(p2Optimized, latchedRightQuat, p1Optimized);
  std::cout << "[IK] processChangingDataRightArm" << std::endl;
  return true;
}

bool Quest3IkIncrementalAPI::processDataLeftArm() {
  std::vector<PoseData> targetConstraints;
  buildPoseConstraintListFromIncrementalResult(latestIncrementalResult_, targetConstraints);

  if (shouldLogIk("left_passive_latched")) {
    std::cout << "[IK] passive arm target latched from latestPoseConstraintList: right" << std::endl;
  }

  const Eigen::Quaterniond leftQuat(
      targetConstraints[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix);
  leftLink6Position_ = targetConstraints[POSE_DATA_LIST_INDEX_LEFT_LINK6].position;
  leftLink6Quat_ = leftQuat.normalized();
  leftEndEffectorPosition_ = targetConstraints[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position;
  leftEndEffectorQuat_ = leftQuat.normalized();
  leftVirtualThumbPosition_ = targetConstraints[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].position;
  latestHumanLeftElbowPos_ = leftElbowFixedPoint_;
  latestRobotLeftElbowPos_ = leftElbowFixedPoint_;

  updateLeftConstraintList(targetConstraints[POSE_DATA_LIST_INDEX_LEFT_HAND].position,
                           leftLink6Quat_,
                           leftElbowFixedPoint_);
  if (shouldLogIk("process_data_left_arm")) {
    std::cout << "[IK] processDataLeftArm" << std::endl;
  }
  return true;
}

bool Quest3IkIncrementalAPI::processDataRightArm() {
  std::vector<PoseData> targetConstraints;
  buildPoseConstraintListFromIncrementalResult(latestIncrementalResult_, targetConstraints);

  if (shouldLogIk("right_passive_latched")) {
    std::cout << "[IK] passive arm target latched from latestPoseConstraintList: left" << std::endl;
  }

  const Eigen::Quaterniond rightQuat(
      targetConstraints[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix);
  rightLink6Position_ = targetConstraints[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position;
  rightLink6Quat_ = rightQuat.normalized();
  rightEndEffectorPosition_ = targetConstraints[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].position;
  rightEndEffectorQuat_ = rightQuat.normalized();
  rightVirtualThumbPosition_ = targetConstraints[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].position;
  latestHumanRightElbowPos_ = rightElbowFixedPoint_;
  latestRobotRightElbowPos_ = rightElbowFixedPoint_;

  updateRightConstraintList(targetConstraints[POSE_DATA_LIST_INDEX_RIGHT_HAND].position,
                            rightLink6Quat_,
                            rightElbowFixedPoint_);
  if (shouldLogIk("process_data_right_arm")) {
    std::cout << "[IK] processDataRightArm" << std::endl;
  }
  return true;
}

bool Quest3IkIncrementalAPI::fsmChange(const ArmPose& vrLeftPose,
                                       const ArmPose& vrRightPose,
                                       const Eigen::Quaterniond& qLeftEndEffector,
                                       const Eigen::Quaterniond& qRightEndEffector,
                                       bool leftHandCtrlModeChanged,
                                       bool rightHandCtrlModeChanged) {
  if (!incrementalController_ || !incrementalController_->isIncrementalMode()) {
    return false;
  }

  bool leftChangingMaintainUpdated = false;
  bool rightChangingMaintainUpdated = false;
  if (leftHandSmoother_) {
    std::tie(leftChangingMaintainUpdated, std::ignore) =
        leftHandSmoother_->updateModeChangingStateIfNeeded(leftHandCtrlModeChanged);
  }
  if (rightHandSmoother_) {
    std::tie(rightChangingMaintainUpdated, std::ignore) =
        rightHandSmoother_->updateModeChangingStateIfNeeded(rightHandCtrlModeChanged);
  }

  if (!leftChangingMaintainUpdated && !rightChangingMaintainUpdated) {
    return false;
  }

  if (shouldLogIk("fsm_change")) {
    std::cout << "[IK] fsmChange: leftChangingActive="
              << (leftChangingMaintainUpdated ? "1" : "0")
              << " rightChangingActive="
              << (rightChangingMaintainUpdated ? "1" : "0") << std::endl;
  }

  bool leftCanProcess = false;
  bool rightCanProcess = false;
  leftCanProcess = leftArmCtrlModeActive_ &&
                   incrementalController_->isIncrementalModeLeftArm() &&
                   currentLeftGripPressed_;
  rightCanProcess = rightArmCtrlModeActive_ &&
                    incrementalController_->isIncrementalModeRightArm() &&
                    currentRightGripPressed_;
  if (leftCanProcess) {
    leftCanProcess = detectLeftArmMove(vrLeftPose);
  }
  if (rightCanProcess) {
    rightCanProcess = detectRightArmMove(vrRightPose);
  }

  const bool hasIncrementalResult =
      updateLatestIncrementalResult(vrLeftPose, vrRightPose, leftCanProcess, rightCanProcess,
                                    qLeftEndEffector, qRightEndEffector);
  if (!hasIncrementalResult) {
    return false;
  }

  bool leftProcessed = false;
  bool rightProcessed = false;
  if (leftChangingMaintainUpdated) {
    leftProcessed = processChangingDataLeftArm(leftHandCtrlModeChanged);
  }
  if (rightChangingMaintainUpdated) {
    rightProcessed = processChangingDataRightArm(rightHandCtrlModeChanged);
  }

  return leftProcessed || rightProcessed;
}

bool Quest3IkIncrementalAPI::fsmProcess(const ArmPose& vrLeftPose,
                                        const ArmPose& vrRightPose,
                                        const Eigen::Quaterniond& qLeftEndEffector,
                                        const Eigen::Quaterniond& qRightEndEffector,
                                        bool& hasIncrementalResult) {
  hasIncrementalResult = false;
  if (!incrementalController_ || !incrementalController_->isIncrementalMode()) {
    return false;
  }

  bool leftMaintainProcess = false;
  bool rightMaintainProcess = false;
  if (leftHandSmoother_) {
    std::tie(leftMaintainProcess, std::ignore) = leftHandSmoother_->getModeChangingState();
  }
  if (rightHandSmoother_) {
    std::tie(rightMaintainProcess, std::ignore) = rightHandSmoother_->getModeChangingState();
  }

  bool leftCanProcess = leftArmCtrlModeActive_ &&
                        !leftMaintainProcess &&
                        incrementalController_->isIncrementalModeLeftArm() &&
                        currentLeftGripPressed_;
  bool rightCanProcess = rightArmCtrlModeActive_ &&
                         !rightMaintainProcess &&
                         incrementalController_->isIncrementalModeRightArm() &&
                         currentRightGripPressed_;
  if (leftCanProcess) {
    leftCanProcess = detectLeftArmMove(vrLeftPose);
  }
  if (rightCanProcess) {
    rightCanProcess = detectRightArmMove(vrRightPose);
  }
  if (!leftCanProcess && !rightCanProcess) {
    hasIncrementalResult = false;
    return false;
  }

  hasIncrementalResult =
      updateLatestIncrementalResult(vrLeftPose, vrRightPose, leftCanProcess, rightCanProcess,
                                    qLeftEndEffector, qRightEndEffector);
  if (!hasIncrementalResult) {
    return false;
  }

  if (shouldLogIk("fsm_process")) {
    std::cout << "[IK] fsmProcess: leftCanProcess=" << (leftCanProcess ? "1" : "0")
              << " rightCanProcess=" << (rightCanProcess ? "1" : "0") << std::endl;
  }

  bool leftProcessed = false;
  bool rightProcessed = false;
  if (leftCanProcess) {
    leftProcessed = processDataLeftArm();
  }
  if (rightCanProcess) {
    rightProcessed = processDataRightArm();
  }

  return leftProcessed || rightProcessed;
}

bool Quest3IkIncrementalAPI::updateLeftHandChangingMode(const Eigen::Vector3d& leftTargetPos) {
  if (!leftHandSmoother_ || !oneStageIk_) {
    return false;
  }
  if (jointMidValues_.size() != 14) {
    return false;
  }

  return leftHandSmoother_->updateChangingMode(leftTargetPos,
                                               oneStageIk_.get(),
                                               jointMidValues_,
                                               14,
                                               handChangingModeThreshold_);
}

bool Quest3IkIncrementalAPI::updateRightHandChangingMode(const Eigen::Vector3d& rightTargetPos) {
  if (!rightHandSmoother_ || !oneStageIk_) {
    return false;
  }
  if (jointMidValues_.size() != 14) {
    return false;
  }

  return rightHandSmoother_->updateChangingMode(rightTargetPos,
                                                oneStageIk_.get(),
                                                jointMidValues_,
                                                14,
                                                handChangingModeThreshold_);
}

void Quest3IkIncrementalAPI::solveIk() {
  auto ikResult = oneStageIk_->solveIK(latestPoseConstraintList_, ArmIdx::BOTH, jointMidValues_);
  if (!ikResult.isSuccess) {
    if (shouldLogIk("ik_fail")) {
      std::cout << "[IK] solveIk: control=BOTH failed" << std::endl;
    }
    return;
  }

  if (armJointCallback_ && ikResult.solution.size() >= 14) {
    std::vector<double> qVec(ikResult.solution.data(), ikResult.solution.data() + 14);
    armJointCallback_(qVec);
  }

  if (shouldLogIk("ik_solve_both")) {
    std::cout << "[IK] solveIk: control=BOTH" << std::endl;
  }
}

void Quest3IkIncrementalAPI::runOnce() {
  if (!transformer_ || !oneStageIk_ || !incrementalController_) {
    return;
  }

  PoseInfoList bonePoses;
  JoyStickData joystick;
  {
    std::lock_guard<std::mutex> lock(bonePosesMutex_);
    if (!hasBonePoses_) {
      if (shouldLogIk("no_bone_poses")) {
        std::cout << "[IK] 未收到骨骼数据 hasBonePoses_=false" << std::endl;
      }
      return;
    }
    bonePoses = latestBonePoses_;
  }
  {
    std::lock_guard<std::mutex> lock(joystickMutex_);
    joystick = latestJoystick_;
  }

  currentLeftGripPressed_ = joystick.left_grip > 0.5f;
  currentRightGripPressed_ = joystick.right_grip > 0.5f;
  bool leftGripRising = !prevLeftGripPressed_ && currentLeftGripPressed_;
  bool rightGripRising = !prevRightGripPressed_ && currentRightGripPressed_;
  auto finalizeGripState = [&]() {
    prevLeftGripPressed_ = currentLeftGripPressed_;
    prevRightGripPressed_ = currentRightGripPressed_;
  };
  bool leftArmCtrlModeChanged = false;
  bool rightArmCtrlModeChanged = false;
  {
    std::lock_guard<std::mutex> lock(armCtrlModeMutex_);
    leftArmCtrlModeChanged = pendingLeftArmCtrlModeChanged_;
    rightArmCtrlModeChanged = pendingRightArmCtrlModeChanged_;
    pendingLeftArmCtrlModeChanged_ = false;
    pendingRightArmCtrlModeChanged_ = false;
  }

  if (headBodyPoseCallback_ && transformer_->isRunning()) {
    headBodyPoseCallback_(transformer_->getHeadBodyPose());
  }

  PoseInfoList handElbowOutput;
  if (!transformer_->updateHandPoseAndElbowPosition(bonePoses, handElbowOutput)) {
    if (shouldLogIk("update_hand_fail")) {
      std::cout << "[IK] updateHandPoseAndElbowPosition 失败" << std::endl;
    }
    finalizeGripState();
    return;
  }
  if (handElbowOutput.poses.size() < 4) {
    if (shouldLogIk("hand_elbow_size")) {
      std::cout << "[IK] handElbowOutput.poses.size()=" << handElbowOutput.poses.size()
                << " < 4" << std::endl;
    }
    finalizeGripState();
    return;
  }

  const auto& leftHandPose = transformer_->getLeftHandPose();
  const auto& rightHandPose = transformer_->getRightHandPose();
  const ArmPose vrLeftPose(leftHandPose.position, leftHandPose.quaternion);
  const ArmPose vrRightPose(rightHandPose.position, rightHandPose.quaternion);

  Eigen::VectorXd qForFK;
  const bool hasValidFk = hasValidSensorJoints(qForFK);
  jointMidValues_ = hasValidFk ? qForFK : Eigen::VectorXd();

  Eigen::Vector3d leftLink4Pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightLink4Pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d leftLink6Pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightLink6Pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d leftEEPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightEEPos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond leftLink4Quat = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond rightLink4Quat = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond leftLink6Quat = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond rightLink6Quat = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond leftEEQuat = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond rightEEQuat = Eigen::Quaterniond::Identity();

  bool fkReady = false;
  if (hasValidFk) {
    fkReady = computeLeftLink4FK(qForFK, leftLink4Pos, leftLink4Quat) &&
              computeRightLink4FK(qForFK, rightLink4Pos, rightLink4Quat) &&
              computeLeftLink6FK(qForFK, leftLink6Pos, leftLink6Quat) &&
              computeRightLink6FK(qForFK, rightLink6Pos, rightLink6Quat) &&
              computeLeftEndEffectorFK(qForFK, leftEEPos, leftEEQuat) &&
              computeRightEndEffectorFK(qForFK, rightEEPos, rightEEQuat);
  }

  bool pendingEnterMode2 = false;
  bool pendingExitMode2 = false;
  {
    std::lock_guard<std::mutex> lock(modeMutex_);
    pendingEnterMode2 = pendingEnterMode2_;
    pendingExitMode2 = pendingExitMode2_;
    pendingEnterMode2_ = false;
    pendingExitMode2_ = false;
  }

  if (pendingEnterMode2) {
    if (leftHandSmoother_) {
      leftHandSmoother_->cancelMaintain();
    }
    if (rightHandSmoother_) {
      rightHandSmoother_->cancelMaintain();
    }
    if (fkReady) {
      rebuildPoseConstraintListFromCurrentFk(leftLink4Pos, leftLink4Quat, rightLink4Pos, rightLink4Quat,
                                             leftLink6Pos, leftLink6Quat, rightLink6Pos, rightLink6Quat,
                                             leftEEPos, leftEEQuat, rightEEPos, rightEEQuat);
      std::cout << "[IK] mode enter rebuild" << std::endl;
    } else if (shouldLogIk("mode_enter_no_sensor")) {
      std::cout << "[IK] mode enter rebuild: 缺少有效 sensor joints，跳过重建" << std::endl;
    }
    prevLeftGripPressed_ = false;
    prevRightGripPressed_ = false;
    leftGripRising = currentLeftGripPressed_;
    rightGripRising = currentRightGripPressed_;
  }

  if (pendingExitMode2) {
    if (leftHandSmoother_) {
      leftHandSmoother_->cancelMaintain();
    }
    if (rightHandSmoother_) {
      rightHandSmoother_->cancelMaintain();
    }
    if (fkReady) {
      rebuildPoseConstraintListFromCurrentFk(leftLink4Pos, leftLink4Quat, rightLink4Pos, rightLink4Quat,
                                             leftLink6Pos, leftLink6Quat, rightLink6Pos, rightLink6Quat,
                                             leftEEPos, leftEEQuat, rightEEPos, rightEEQuat);
      std::cout << "[IK] mode exit rebuild" << std::endl;
    } else if (shouldLogIk("mode_exit_no_sensor")) {
      std::cout << "[IK] mode exit rebuild: 缺少有效 sensor joints，跳过重建" << std::endl;
    }

    if (incrementalController_->isIncrementalModeLeftArm()) {
      incrementalController_->exitIncrementalModeLeftArm(vrLeftPose, latestPoseConstraintList_,
                                                         leftEndEffectorPosition_, leftEndEffectorQuat_,
                                                         leftLink4Quat_);
    }
    if (incrementalController_->isIncrementalModeRightArm()) {
      incrementalController_->exitIncrementalModeRightArm(vrRightPose, latestPoseConstraintList_,
                                                          rightEndEffectorPosition_, rightEndEffectorQuat_,
                                                          rightLink4Quat_);
    }
    prevLeftGripPressed_ = false;
    prevRightGripPressed_ = false;
  }

  if (!transformer_->isRunning()) {
    if (shouldLogIk("not_running")) {
      std::cout << "[IK] 未激活 isRunning=false，按 X+A 切换至 External 模式" << std::endl;
    }
    finalizeGripState();
    return;
  }

  const bool leftMaintainActive = leftHandSmoother_ && leftHandSmoother_->isMaintaining();
  const bool rightMaintainActive = rightHandSmoother_ && rightHandSmoother_->isMaintaining();

  if (leftGripRising && !leftMaintainActive) {
    if (fkReady) {
      const auto [p1Optimized, p2Optimized] =
          leftVelocityIkSolverPtr_->solve(leftLink4Pos, leftElbowFixedPoint_, leftLink6Pos);
      latestHumanLeftElbowPos_ = leftLink4Pos;
      latestRobotLeftElbowPos_ = p1Optimized;
      leftLink6Position_ = p2Optimized;
      leftLink6Quat_ = leftLink6Quat;
      leftEndEffectorPosition_ = p2Optimized + leftLink6Quat * leftEE2Link6Offset_;
      leftEndEffectorQuat_ = leftEEQuat;
      leftVirtualThumbPosition_ = p2Optimized + leftLink6Quat * leftThumb2Link6Offset_;
      leftLink4Quat_ = leftLink4Quat;
      updateLeftConstraintList(p2Optimized, leftLink6Quat, p1Optimized);
      std::cout << "[IK] grip rising rebuild: left" << std::endl;
    } else if (shouldLogIk("left_grip_no_sensor")) {
      std::cout << "[IK] grip rising rebuild: 左臂缺少有效 sensor joints，沿用现有基准" << std::endl;
    }
  }

  if (rightGripRising && !rightMaintainActive) {
    if (fkReady) {
      const auto [p1Optimized, p2Optimized] =
          rightVelocityIkSolverPtr_->solve(rightLink4Pos, rightElbowFixedPoint_, rightLink6Pos);
      latestHumanRightElbowPos_ = rightLink4Pos;
      latestRobotRightElbowPos_ = p1Optimized;
      rightLink6Position_ = p2Optimized;
      rightLink6Quat_ = rightLink6Quat;
      rightEndEffectorPosition_ = p2Optimized + rightLink6Quat * rightEE2Link6Offset_;
      rightEndEffectorQuat_ = rightEEQuat;
      rightVirtualThumbPosition_ = p2Optimized + rightLink6Quat * rightThumb2Link6Offset_;
      rightLink4Quat_ = rightLink4Quat;
      updateRightConstraintList(p2Optimized, rightLink6Quat, p1Optimized);
      std::cout << "[IK] grip rising rebuild: right" << std::endl;
    } else if (shouldLogIk("right_grip_no_sensor")) {
      std::cout << "[IK] grip rising rebuild: 右臂缺少有效 sensor joints，沿用现有基准" << std::endl;
    }
  }

  if (incrementalController_->shouldEnterIncrementalModeLeftArm(currentLeftGripPressed_)) {
    if (!fkReady) {
      if (shouldLogIk("left_enter_no_sensor")) {
        std::cout << "[IK] 左臂进入增量模式失败: 缺少有效 sensor joints" << std::endl;
      }
      finalizeGripState();
      return;
    }
    incrementalController_->enterIncrementalModeLeftArm(vrLeftPose, latestPoseConstraintList_,
                                                        leftEEPos, leftEEQuat, leftLink4Quat);
    std::cout << "[IK] 左臂进入增量模式"
              << (leftGripRising ? " (grip rising rebuild)" : " (reuse incremental state)")
              << std::endl;
  }

  if (incrementalController_->shouldEnterIncrementalModeRightArm(currentRightGripPressed_)) {
    if (!fkReady) {
      if (shouldLogIk("right_enter_no_sensor")) {
        std::cout << "[IK] 右臂进入增量模式失败: 缺少有效 sensor joints" << std::endl;
      }
      finalizeGripState();
      return;
    }
    incrementalController_->enterIncrementalModeRightArm(vrRightPose, latestPoseConstraintList_,
                                                         rightEEPos, rightEEQuat, rightLink4Quat);
    std::cout << "[IK] 右臂进入增量模式"
              << (rightGripRising ? " (grip rising rebuild)" : " (reuse incremental state)")
              << std::endl;
  }

  if (incrementalController_->shouldExitIncrementalModeLeftArm(currentLeftGripPressed_)) {
    incrementalController_->exitIncrementalModeLeftArm(vrLeftPose, latestPoseConstraintList_,
                                                       leftEndEffectorPosition_, leftEndEffectorQuat_,
                                                       leftLink4Quat_);
    std::cout << "[IK] 左臂退出增量模式" << std::endl;
  }
  if (incrementalController_->shouldExitIncrementalModeRightArm(currentRightGripPressed_)) {
    incrementalController_->exitIncrementalModeRightArm(vrRightPose, latestPoseConstraintList_,
                                                        rightEndEffectorPosition_, rightEndEffectorQuat_,
                                                        rightLink4Quat_);
    std::cout << "[IK] 右臂退出增量模式" << std::endl;
  }

  if (!incrementalController_->isIncrementalMode()) {
    if (shouldLogIk("not_incremental")) {
      std::cout << "[IK] 未进入增量模式 isIncrementalMode=false，需按住对应手臂的握把(grip)"
                << std::endl;
    }
    finalizeGripState();
    return;
  }

  const bool changingProcessed =
      fsmChange(vrLeftPose, vrRightPose, leftEndEffectorQuat_, rightEndEffectorQuat_,
                leftArmCtrlModeChanged, rightArmCtrlModeChanged);
  bool hasIncrementalResult = false;
  const bool processProcessed =
      fsmProcess(vrLeftPose, vrRightPose, leftEndEffectorQuat_, rightEndEffectorQuat_,
                 hasIncrementalResult);

  if (!hasIncrementalResult && !changingProcessed && !processProcessed) {
    if (shouldLogIk("wait_for_arm_move")) {
      std::cout << "[IK] grip 已按下，但未检测到手臂实际移动，保持当前姿态等待" << std::endl;
    }
    finalizeGripState();
    return;
  }

  if (changingProcessed || processProcessed) {
    solveIk();
    if (changingProcessed) {
      updateLeftHandChangingMode(defaultLeftHandPosOnExit_);
      updateRightHandChangingMode(defaultRightHandPosOnExit_);
    }
  }

  finalizeGripState();
}

}  // namespace ik
}  // namespace leju
