/**
 * @file ik_types.h
 * @brief Pure C++ data types for IK module (ROS-free)
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace leju {
namespace ik {

// ============ Equivalent to noitom_hi5_hand_udp_python::PoseInfo ============
struct PoseInfo {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;  // x, y, z, w

  PoseInfo() : orientation(Eigen::Quaterniond::Identity()) {}
};

// ============ Equivalent to noitom_hi5_hand_udp_python::PoseInfoList ============
struct PoseInfoList {
  int64_t timestamp_ms = 0;
  bool is_high_confidence = false;
  bool is_hand_tracking = false;
  std::vector<PoseInfo> poses;
};

// ============ Equivalent to noitom_hi5_hand_udp_python::JoySticks ============
struct JoyStickData {
  float left_x = 0.0f;
  float left_y = 0.0f;
  float left_trigger = 0.0f;
  float left_grip = 0.0f;
  float right_x = 0.0f;
  float right_y = 0.0f;
  float right_trigger = 0.0f;
  float right_grip = 0.0f;
};

struct ArmControlModeState {
  bool left_active = false;
  bool right_active = false;
  bool left_changed = false;
  bool right_changed = false;
};

// ============ From leju_utils (ROS-free) ============
struct PoseData {
  Eigen::Matrix3d rotation_matrix;
  Eigen::Vector3d position;

  PoseData() {
    rotation_matrix = Eigen::Matrix3d::Identity();
    position = Eigen::Vector3d::Zero();
  }

  PoseData(const Eigen::Matrix3d& rot_mat, const Eigen::Vector3d& pos)
      : rotation_matrix(rot_mat), position(pos) {}
};

enum class ArmIdx { LEFT = 0, RIGHT = 1, BOTH = 2 };

struct HeadBodyPose {
  double head_pitch = 0.0;
  double head_yaw = 0.0;
  double body_yaw = 0.0;
  double body_x = 0.0;
  double body_y = 0.0;
  double body_roll = 0.0;
  double body_pitch = 6.0 * M_PI / 180.0;
  double body_height = 0.74;
};

struct ArmPose {
  Eigen::Vector3d position;
  Eigen::Quaterniond quaternion;

  ArmPose() {
    position = Eigen::Vector3d::Zero();
    quaternion = Eigen::Quaterniond::Identity();
  }

  ArmPose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat)
      : position(pos), quaternion(quat) {}

  ArmPose(const std::string& robotModel, bool isLeftHand) {
    if (robotModel == "kuavo_45") {
      if (isLeftHand) {
        position << 0.073, 0.25, -0.26;
        quaternion = Eigen::Quaterniond::Identity();
      } else {
        position << 0.073, -0.25, -0.26;
        quaternion = Eigen::Quaterniond::Identity();
      }
    } else {
      position = Eigen::Vector3d::Zero();
      quaternion = Eigen::Quaterniond::Identity();
    }
  }

  bool isValid() const {
    return position.allFinite() && quaternion.coeffs().allFinite() &&
           std::abs(quaternion.norm() - 1.0) < 1e-4;
  }
};

struct ArmData {
  Eigen::Quaterniond& handQuatInW;
  ArmPose& handPose;
  ArmPose& elbowPose;
  int elbowIndex;
  int shoulderIndex;
  int handIndex;

  ArmData(Eigen::Quaterniond& handQuat, ArmPose& hand, ArmPose& elbow,
          int elbowIdx, int shoulderIdx, int handIdx)
      : handQuatInW(handQuat),
        handPose(hand),
        elbowPose(elbow),
        elbowIndex(elbowIdx),
        shoulderIndex(shoulderIdx),
        handIndex(handIdx) {}
};

// ============ Pose index constants (from leju_utils) ============
#define POSE_INDEX_LEFT_HAND 4
#define POSE_INDEX_LEFT_ELBOW 1
#define POSE_INDEX_RIGHT_HAND 5
#define POSE_INDEX_RIGHT_ELBOW 3
#define POSE_INDEX_CHEST 23
#define POSE_INDEX_LEFT_ARM_UPPER 0
#define POSE_INDEX_RIGHT_ARM_UPPER 2

#define POSE_DATA_LIST_INDEX_CHEST 0
#define POSE_DATA_LIST_INDEX_LEFT_HAND 1
#define POSE_DATA_LIST_INDEX_RIGHT_HAND 2
#define POSE_DATA_LIST_INDEX_LEFT_ELBOW 3
#define POSE_DATA_LIST_INDEX_RIGHT_ELBOW 4
#define POSE_DATA_LIST_SIZE 5
#define POSE_DATA_LIST_SIZE_PLUS 11

#define POSE_DATA_LIST_INDEX_LEFT_LINK6 5
#define POSE_DATA_LIST_INDEX_RIGHT_LINK6 6
#define POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB 7
#define POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB 8
#define POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR 9
#define POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR 10

// ============ Output type (DDS JointTrajectoryPoint compatible) ============
struct ArmJointOutput {
  std::vector<double> q;  // 14 joints, rad
};

// ============ IK solver types (for OneStageIKEndEffector, incremental IK) ============
struct IKSolveResult {
  bool isSuccess = false;
  Eigen::VectorXd solution;
  std::chrono::milliseconds solveDuration{0};
  std::string solverLog;

  IKSolveResult() = default;
  IKSolveResult(const Eigen::VectorXd& sol, const std::chrono::milliseconds& duration)
      : isSuccess(true), solution(sol), solveDuration(duration), solverLog("ok") {}
  IKSolveResult(int nq, const std::string& errorMsg = "")
      : isSuccess(false), solution(Eigen::VectorXd::Zero(nq)), solveDuration(0), solverLog(errorMsg) {}
};

struct IKSolverConfig {
  double constraintTolerance = 1e-8;
  double solverTolerance = 1e-6;
  int maxIterations = 3000;
  ArmIdx controlArmIndex = ArmIdx::BOTH;
  bool isWeldBaseLink = true;
  bool useJointLimits = true;
  Eigen::VectorXd jointLowerBounds;
  Eigen::VectorXd jointUpperBounds;

  IKSolverConfig() = default;
  IKSolverConfig(double constraintTol, double solverTol, int maxIter, ArmIdx armIdx, bool weldBase)
      : constraintTolerance(constraintTol),
        solverTolerance(solverTol),
        maxIterations(maxIter),
        controlArmIndex(armIdx),
        isWeldBaseLink(weldBase),
        useJointLimits(true) {}
};

// PointTrack config for OneStageIKEndEffector
struct PointTrackIKSolverConfig : IKSolverConfig {
  int historyBufferSize = 10;
  double eeTrackingWeight = 4e3;
  double elbowTrackingWeight = 4e2;
  double link6TrackingWeight = 4e3;
  double virtualThumbTrackingWeight = 4e3;
  double jointSmoothWeightDefault = 5e1;
  double jointSmoothWeight0 = 5e1, jointSmoothWeight1 = 5e1, jointSmoothWeight2 = 5e1;
  double jointSmoothWeight3 = 1e1, jointSmoothWeight4 = 1e-3, jointSmoothWeight5 = 1e-3, jointSmoothWeight6 = 1e-3;
};

}  // namespace ik
}  // namespace leju
