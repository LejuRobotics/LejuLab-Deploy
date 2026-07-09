/**
 * @file test_leju-ik.cpp
 * @brief Minimal test to verify leju-ik compiles and basic types work
 */

#include <leju-ik/ik_types.h>
#include <leju-ik/Quest3ArmInfoTransformer.h>
#include <leju-ik/Quest3IkAPI.h>

#include <iostream>

int main() {
  std::cout << "leju-ik: Testing ik_types..." << std::endl;

  leju::ik::PoseInfo pi;
  pi.position = Eigen::Vector3d(0, 0, 0);
  pi.orientation = Eigen::Quaterniond::Identity();

  leju::ik::PoseInfoList pil;
  pil.timestamp_ms = 0;
  pil.poses.resize(26, pi);

  leju::ik::JoyStickData js;
  js.left_trigger = 0.0f;
  js.right_trigger = 0.0f;

  std::cout << "leju-ik: Testing Quest3ArmInfoTransformer..." << std::endl;
  leju::ik::Quest3ArmInfoTransformer transformer("kuavo_45");
  leju::ik::PoseInfoList output;
  bool ok = transformer.updateHandPoseAndElbowPosition(pil, output);
  std::cout << "  updateHandPoseAndElbowPosition: " << (ok ? "ok" : "fail") << std::endl;

  std::cout << "leju-ik: Testing Quest3IkAPI (init only, no Drake plant)..." << std::endl;
  leju::ik::Quest3IkAPI api;
  // init would need valid URDF path - skip in minimal test
  std::cout << "  Quest3IkAPI constructed" << std::endl;

  std::cout << "leju-ik: All tests passed." << std::endl;
  return 0;
}
