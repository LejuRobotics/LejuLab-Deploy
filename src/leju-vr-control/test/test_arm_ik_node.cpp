/**
 * @file test_arm_ik_node.cpp
 * @brief 独立 IK 测试节点：合成 VR 末端位置 → ArmAbsoluteIK → 写入状态文件
 *
 * 无需 Quest3 设备，生成左右手正弦运动轨迹作为 IK 输入，
 * 求解后将关节角写入 /tmp/vr_ik_state.json（供 mujoco_viewer.py 读取）。
 *
 * 环境变量：
 *   LEJU_ASSETS_PATH   leju_assets 根目录
 *   ROBOT_VERSION      与 quest_vr_abs_control_node 相同，默认 "17"，用于 URDF 路径与 QuestVrCalibration
 *   VR_STATE_FILE      状态文件路径（默认 /tmp/vr_ik_state.json）
 *   VR_IK_HZ           循环频率（默认 10 Hz）
 *
 * 合成目标在躯干坐标系下满足硬编码臂长（与 URDF/MJCF 连杆尺度一致）：
 *   upper_arm_length = 0.20533 m（肩 → 肘）
 *   lower_arm_length = 0.2175 m（肘 → 手末端）
 */

#include <leju-ik/ArmAbsoluteIK.h>
#include <leju-ik/QuestVrCalibration.h>
#include <nlohmann/json.hpp>

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

static std::atomic<bool> g_running{true};
void signalHandler(int) { g_running = false; }

static std::string getEnv(const char* name, const std::string& def = "") {
  const char* v = std::getenv(name);
  return (v && v[0] != '\0') ? std::string(v) : def;
}

static double getEnvDouble(const char* name, double def) {
  std::string s = getEnv(name);
  if (s.empty()) return def;
  try { return std::stod(s); } catch (...) { return def; }
}

/// 原子性写入状态 JSON（先写 .tmp 再 rename，避免读端读到半写文件）
static void writeStateFile(
    const std::string& path,
    const Eigen::VectorXd& joints,
    const Eigen::Vector3d& l_hand,
    const Eigen::Vector3d& r_hand,
    const Eigen::Vector3d& l_elbow,
    const Eigen::Vector3d& r_elbow,
    bool ik_success) {
  using json = nlohmann::json;

  json j;
  j["timestamp"] = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  j["ik_success"] = ik_success;

  if (ik_success && joints.size() > 0) {
    j["arm_joints_rad"] = std::vector<double>(joints.data(), joints.data() + joints.size());
  } else {
    j["arm_joints_rad"] = std::vector<double>(8, 0.0);
  }

  j["left_hand_target"]  = {l_hand[0],  l_hand[1],  l_hand[2]};
  j["right_hand_target"] = {r_hand[0],  r_hand[1],  r_hand[2]};
  j["left_elbow_target"] = {l_elbow[0], l_elbow[1], l_elbow[2]};
  j["right_elbow_target"]= {r_elbow[0], r_elbow[1], r_elbow[2]};

  const std::string tmp_path = path + ".tmp";
  {
    std::ofstream ofs(tmp_path);
    if (!ofs) {
      std::cerr << "[TestIK] Cannot write to " << tmp_path << std::endl;
      return;
    }
    ofs << j.dump(2);
  }
  std::rename(tmp_path.c_str(), path.c_str());
}

int main(int argc, char** argv) {
  std::signal(SIGINT,  signalHandler);
  std::signal(SIGTERM, signalHandler);

  // ------- URDF 与标定（与 quest_vr_abs_control_node 相同逻辑）-------
  const std::string robot_version_str = getEnv("ROBOT_VERSION", "17");
  const leju::ik::QuestVrCalibration cal =
      leju::ik::calibrationForRobotVersion(robot_version_str);

  const std::string assets_path = getEnv("LEJU_ASSETS_PATH");
  std::string urdf_path;
  if (!assets_path.empty()) {
    urdf_path = assets_path + "/models/biped_s" + robot_version_str +
                "/urdf/drake/biped_v3_arm.urdf";
  } else {
    std::cerr << "[VrAbsCtrl] LEJU_ASSETS_PATH not set. " << std::endl;
    return 1;
  }
  const std::string state_file = getEnv("VR_STATE_FILE", "/tmp/vr_ik_state.json");
  const double hz              = getEnvDouble("VR_IK_HZ", 100.0);

  // 合成轨迹用固定臂长（与 JSON 配置一致，独立于 QuestVrCalibration）
  constexpr double kUpperArmLength = 0.20533;  // 肩 → 肘
  constexpr double kLowerArmLength = 0.2175;   // 肘 → 手末端

  std::cout << "[TestIK] robot_version=" << robot_version_str << std::endl;
  std::cout << "[TestIK] calibration: upper=" << cal.upper_arm_length
            << " lower=" << cal.lower_arm_length
            << " shoulder_w=" << cal.shoulder_width
            << " base_h=" << cal.base_height_offset
            << " base_chest_x=" << cal.base_chest_offset_x
            << " eef_z_bias=" << cal.eef_z_bias << std::endl;
  std::cout << "[TestIK] URDF   : " << urdf_path   << std::endl;
  std::cout << "[TestIK] State  : " << state_file  << std::endl;
  std::cout << "[TestIK] Rate   : " << hz           << " Hz" << std::endl;
  std::cout << "[TestIK] Test arm lengths (hardcoded): upper=" << kUpperArmLength
            << " m  lower=" << kLowerArmLength << " m" << std::endl;

  leju::ik::ArmAbsoluteIKConfig ik_cfg;
  ik_cfg.eef_z_bias = cal.eef_z_bias;
  leju::ik::ArmAbsoluteIK arm_ik(urdf_path, ik_cfg);
  arm_ik.setTorsoState(0.0, 0.0);

  const int nq = arm_ik.numPositions();
  std::cout << "[TestIK] nq=" << nq << "  (expected 8 for biped_s17)" << std::endl;

  // 躯干系下近似肩位置（与 biped zarm_l1 附近一致，仅用于生成测试轨迹）
  const Eigen::Vector3d l_shoulder(0.0, 0.181, 0.24);
  const Eigen::Vector3d r_shoulder(0.0, -0.181, 0.24);

  const double omega = 0.5;  // rad/s

  auto unitFromSpherical = [](double theta, double phi) -> Eigen::Vector3d {
    const double st = std::sin(theta);
    return Eigen::Vector3d(st * std::cos(phi), st * std::sin(phi), std::cos(theta));
  };

  const auto loop_dt = std::chrono::duration<double>(1.0 / hz);
  auto t0 = std::chrono::steady_clock::now();

  int run_cnt = 0, fail_cnt = 0;

  std::cout << "[TestIK] Starting IK loop. Ctrl-C to stop." << std::endl;

  while (g_running) {
    const auto loop_start = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(loop_start - t0).count();

    // ------- 合成目标：|肘-肩|=upper，|手-肘|=lower -------
    const double theta_e = 0.5 * M_PI * 0.55 + 0.4 * std::sin(omega * t);
    const double phi_e = omega * t * 0.85;
    const Eigen::Vector3d u_el_l = unitFromSpherical(theta_e, phi_e);
    const Eigen::Vector3d u_el_r = unitFromSpherical(theta_e, -phi_e);  // 右手 y 镜像

    Eigen::Vector3d l_elbow = l_shoulder + kUpperArmLength * u_el_l;
    Eigen::Vector3d r_elbow = r_shoulder + kUpperArmLength * u_el_r;

    const double theta_f = theta_e + 0.35 * std::sin(omega * t * 1.05);
    const double phi_f = phi_e + 0.5 * std::sin(omega * t * 0.9);
    const Eigen::Vector3d u_fa_l = unitFromSpherical(theta_f, phi_f);
    const Eigen::Vector3d u_fa_r = unitFromSpherical(theta_f, -phi_f);

    Eigen::Vector3d l_hand = l_elbow + kLowerArmLength * u_fa_l;
    Eigen::Vector3d r_hand = r_elbow + kLowerArmLength * u_fa_r;

    // ------- IK 求解 -------
    const auto t_ik0 = std::chrono::steady_clock::now();
    Eigen::VectorXd q = arm_ik.computeIK(l_hand, r_hand, l_elbow, r_elbow);
    const double ik_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_ik0).count();

    const bool ok = (q.size() == static_cast<Eigen::Index>(nq));
    run_cnt++;
    if (!ok) fail_cnt++;

    // ------- 写状态文件 -------
    writeStateFile(state_file, q, l_hand, r_hand, l_elbow, r_elbow, ok);

    if (run_cnt % 20 == 0) {
      const double succ_rate = 100.0 * (1.0 - static_cast<double>(fail_cnt) / run_cnt);
      std::cout << "\r[TestIK] t=" << std::fixed << std::setprecision(1) << t
                << "s  IK=" << (ok ? "OK " : "FAIL")
                << "  " << std::setprecision(1) << ik_ms << "ms"
                << "  success=" << succ_rate << "%   q=" << q.transpose() << " r_elbow=" << r_elbow.transpose() << " r_hand=" << r_hand.transpose() << std::flush;
    }

    const auto elapsed = std::chrono::steady_clock::now() - loop_start;
    if (elapsed < loop_dt) std::this_thread::sleep_for(loop_dt - elapsed);
  }

  std::cout << "\n[TestIK] Done." << std::endl;
  return 0;
}
