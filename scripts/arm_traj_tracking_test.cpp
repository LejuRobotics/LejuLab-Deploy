/**
 * @file arm_traj_tracking_test.cpp
 * @brief ROBOT_VERSION=17 手臂 8 关节 /rt/arm_trajectory 稳态跟踪误差测试
 *
 * 外部控制下把 zarm_l1..l4、zarm_r1..r4 从当前位姿移开一小段并保持，
 * 统计电机指令与反馈的稳态误差，以及当时下发的 kp/kd。
 * 用来对照调节手臂 PD 后，位置执行误差有没有变小。
 *
 * 速度前馈非零，控制器直通目标。误差是电机拿到目标之后没跟上的角度，
 * 不含接入插值和 5Hz 滤波。
 *
 * 步进角和保持时间写在 arm_traj_tracking.yaml。开源仓库里这个文件和本程序放在 scripts/。
 * 控制器不读这个文件，也不需要改控制器源码。
 *
 * 用法（控制器与硬件节点已在跑，机器人站稳、手臂可小幅活动）：
 *   ROBOT_VERSION=17 ./arm_traj_tracking_test
 *
 * 左臂四个步进角写在 left_delta_deg，单位 deg。
 * 右臂镜像：pitch/elbow 同号，roll/yaw 取反。测完插值回起点并切回 Auto。
 * v17 手臂 joint_direction 为 +1，轨迹 q 与电机 q 同向。
 */

#include <lejusdk-vr/lejusdk_vr.h>

#include <lejusdk-dds-idl/JointCmd.hpp>
#include <lejusdk-dds-idl/JointState.hpp>
#include <lejusdk-topic-pubsub/topic_names.h>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>
#include <lejusdk-utils/robot_version.hpp>

#include <dds/dds.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace leju::vr;

namespace {

constexpr int kArmDof = 8;
constexpr int kArmMotorBegin = 13;  // 腿 12 + 腰 1，随后是 8 个手臂关节
constexpr double kFeedforwardVel = 1e-3;
constexpr double kMaxStepDeg = 50.0;
constexpr double kRad2Deg = 180.0 / M_PI;

const char* kArmNames[kArmDof] = {
    "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
    "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
};

// 右臂相对左臂：pitch/elbow 同号，roll/yaw 取反。
constexpr int kRightMirror[4] = {1, -1, -1, 1};

volatile sig_atomic_t g_stop = 0;

void signalHandler(int) { g_stop = 1; }

struct ArmVec {
  std::array<double, kArmDof> v{};
  bool ok = false;
};

struct CmdSnap {
  ArmVec q;
  ArmVec kp;
  ArmVec kd;
  ArmVec mode;
  bool ok = false;
};

struct Shared {
  std::mutex mu;
  CmdSnap joint_cmd;
  CmdSnap motor_cmd;
  ArmVec motor_state;
  ArmVec joint_state;
};

bool sliceArm(const std::vector<double>& src, ArmVec* out) {
  if (static_cast<int>(src.size()) < kArmMotorBegin + kArmDof) {
    return false;
  }
  for (int i = 0; i < kArmDof; ++i) {
    out->v[i] = src[kArmMotorBegin + i];
  }
  out->ok = true;
  return true;
}

CmdSnap snapCmd(const leju::msgs::JointCmd& msg) {
  CmdSnap snap;
  sliceArm(msg.q(), &snap.q);
  sliceArm(msg.kp(), &snap.kp);
  sliceArm(msg.kd(), &snap.kd);
  if (static_cast<int>(msg.modes().size()) >= kArmMotorBegin + kArmDof) {
    for (int i = 0; i < kArmDof; ++i) {
      snap.mode.v[i] = msg.modes()[kArmMotorBegin + i];
    }
    snap.mode.ok = true;
  }
  snap.ok = snap.q.ok;
  return snap;
}

void onJointCmd(Shared& shared, const leju::msgs::JointCmd& msg) {
  CmdSnap snap = snapCmd(msg);
  if (!snap.ok) {
    return;
  }
  std::lock_guard<std::mutex> lock(shared.mu);
  shared.joint_cmd = snap;
}

void onMotorCmd(Shared& shared, const leju::msgs::JointCmd& msg) {
  CmdSnap snap = snapCmd(msg);
  if (!snap.ok) {
    return;
  }
  std::lock_guard<std::mutex> lock(shared.mu);
  shared.motor_cmd = snap;
}

void onMotorState(Shared& shared, const leju::msgs::JointState& msg) {
  ArmVec q;
  if (!sliceArm(msg.q(), &q)) {
    return;
  }
  std::lock_guard<std::mutex> lock(shared.mu);
  shared.motor_state = q;
}

void onJointState(Shared& shared, const leju::msgs::JointState& msg) {
  ArmVec q;
  if (!sliceArm(msg.q(), &q)) {
    return;
  }
  std::lock_guard<std::mutex> lock(shared.mu);
  shared.joint_state = q;
}

bool publishArm(VRBaseAPI& api, const std::array<double, kArmDof>& q,
                const std::array<double, kArmDof>& v) {
  JointTrajectoryPoint cmd;
  cmd.q.assign(q.begin(), q.end());
  cmd.v.assign(v.begin(), v.end());
  cmd.acc.assign(kArmDof, 0.0);
  return api.publishArmJointCmd(cmd);
}

bool publishHold(VRBaseAPI& api, const std::array<double, kArmDof>& q) {
  std::array<double, kArmDof> v;
  v.fill(kFeedforwardVel);
  return publishArm(api, q, v);
}

void sleepUntil(std::chrono::steady_clock::time_point wake) {
  const auto now = std::chrono::steady_clock::now();
  if (wake > now) {
    std::this_thread::sleep_until(wake);
  }
}

bool waitJointState(Shared& shared, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline && !g_stop) {
    {
      std::lock_guard<std::mutex> lock(shared.mu);
      if (shared.joint_state.ok) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

bool holdPose(VRBaseAPI& api, const std::array<double, kArmDof>& q, int duration_ms, double rate_hz) {
  const auto dt = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / rate_hz));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_ms);
  auto next = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < deadline && !g_stop) {
    if (!publishHold(api, q)) {
      return false;
    }
    next += dt;
    sleepUntil(next);
  }
  return !g_stop;
}

bool moveTo(VRBaseAPI& api, const std::array<double, kArmDof>& q0,
            const std::array<double, kArmDof>& q1, double duration_sec, double rate_hz) {
  const auto dt = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / rate_hz));
  const auto t0 = std::chrono::steady_clock::now();
  auto next = t0;
  while (!g_stop) {
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (t >= duration_sec) {
      break;
    }
    const double s = 0.5 * (1.0 - std::cos(M_PI * t / duration_sec));
    const double ds = 0.5 * M_PI / duration_sec * std::sin(M_PI * t / duration_sec);
    std::array<double, kArmDof> q{};
    std::array<double, kArmDof> v{};
    for (int i = 0; i < kArmDof; ++i) {
      const double dq = q1[i] - q0[i];
      q[i] = q0[i] + dq * s;
      const double vi = dq * ds;
      v[i] = std::abs(vi) > kFeedforwardVel ? vi : (vi >= 0.0 ? kFeedforwardVel : -kFeedforwardVel);
    }
    if (!publishArm(api, q, v)) {
      return false;
    }
    next += dt;
    sleepUntil(next);
  }
  return !g_stop && publishHold(api, q1);
}

struct Accum {
  int n = 0;
  double sum_cmd = 0.0;
  double sum_fb = 0.0;
  double sum_kp = 0.0;
  double sum_kd = 0.0;
  double sum_abs = 0.0;
  double max_abs = 0.0;
  int mode = -1;
};

struct Options {
  int settle_ms = 300;
  double move_sec = 1.5;
  int hold_ms = 2000;
  int skip_ms = 500;
  double rate_hz = 200.0;
  std::string config;
};

bool parseArgs(int argc, char** argv, Options* opt) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "缺少参数: " << name << std::endl;
        return nullptr;
      }
      return argv[++i];
    };
    if (arg == "--settle-ms") {
      const char* v = need("--settle-ms");
      if (!v) return false;
      opt->settle_ms = std::atoi(v);
    } else if (arg == "--move-sec") {
      const char* v = need("--move-sec");
      if (!v) return false;
      opt->move_sec = std::atof(v);
    } else if (arg == "--hold-ms") {
      const char* v = need("--hold-ms");
      if (!v) return false;
      opt->hold_ms = std::atoi(v);
    } else if (arg == "--skip-ms") {
      const char* v = need("--skip-ms");
      if (!v) return false;
      opt->skip_ms = std::atoi(v);
    } else if (arg == "--rate") {
      const char* v = need("--rate");
      if (!v) return false;
      opt->rate_hz = std::atof(v);
    } else if (arg == "--config") {
      const char* v = need("--config");
      if (!v) return false;
      opt->config = v;
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "用法: ROBOT_VERSION=17 arm_traj_tracking_test [--config PATH]\n"
          << "步进角、保持时间在 arm_traj_tracking.yaml。\n"
          << "误差 = 电机指令角 - 反馈角，取保持段后半的均值。正值表示反馈没跟上指令。\n";
      return false;
    } else {
      std::cerr << "未知参数: " << arg << std::endl;
      return false;
    }
  }
  if (opt->move_sec < 0.3 || opt->hold_ms < 200 || opt->skip_ms < 0 || opt->skip_ms >= opt->hold_ms ||
      opt->settle_ms < 50 || opt->rate_hz < 20.0) {
    std::cerr << "move-sec>=0.3，hold-ms>=200 且大于 skip-ms，settle-ms>=50，rate>=20" << std::endl;
    return false;
  }
  return true;
}

std::string resolveConfig(const char* argv0, const std::string& given, std::string* error) {
  namespace fs = std::filesystem;
  if (!given.empty()) {
    if (!fs::exists(given)) {
      *error = "找不到 --config: " + given;
      return {};
    }
    return fs::absolute(given).string();
  }
  std::error_code ec;
  const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
  if (!ec) {
    const fs::path beside = exe.parent_path() / "arm_traj_tracking.yaml";
    if (fs::exists(beside)) {
      return fs::absolute(beside).string();
    }
  }
  if (argv0 != nullptr && argv0[0] != '\0') {
    const fs::path beside = fs::absolute(argv0).parent_path() / "arm_traj_tracking.yaml";
    if (fs::exists(beside)) {
      return beside.string();
    }
  }
  const char* candidates[] = {
      "scripts/arm_traj_tracking.yaml",
      "src/lejusdk/lejusdk-vr/examples/arm_traj_tracking.yaml",
  };
  fs::path dir = fs::current_path();
  for (int i = 0; i < 8; ++i) {
    for (const char* rel : candidates) {
      const fs::path candidate = dir / rel;
      if (fs::exists(candidate)) {
        return fs::absolute(candidate).string();
      }
    }
    if (!dir.has_parent_path() || dir.parent_path() == dir) {
      break;
    }
    dir = dir.parent_path();
  }
  *error = "找不到 arm_traj_tracking.yaml，请放在可执行文件同目录或用 --config 指定";
  return {};
}

bool loadLeftDeltaDeg(const std::string& path, std::array<double, 4>* deg, Options* opt, std::string* error) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    *error = std::string("读取 PD 配置失败: ") + e.what();
    return false;
  }
  const YAML::Node node = root["left_delta_deg"];
  if (!node || !node.IsSequence() || node.size() != 4) {
    *error = "arm_traj_tracking.yaml 需要 left_delta_deg: [l1, l2, l3, l4]，单位 deg";
    return false;
  }
  try {
    for (int i = 0; i < 4; ++i) {
      const double value = node[i].as<double>();
      if (std::abs(value) > kMaxStepDeg) {
        *error = "left_delta_deg 单个绝对值不能超过 " + std::to_string(static_cast<int>(kMaxStepDeg)) + " deg";
        return false;
      }
      (*deg)[i] = value;
    }
    if (root["move_sec"]) {
      opt->move_sec = root["move_sec"].as<double>();
    }
    if (root["hold_ms"]) {
      opt->hold_ms = root["hold_ms"].as<int>();
    }
    if (root["skip_ms"]) {
      opt->skip_ms = root["skip_ms"].as<int>();
    }
    if (root["settle_ms"]) {
      opt->settle_ms = root["settle_ms"].as<int>();
    }
    if (root["rate_hz"]) {
      opt->rate_hz = root["rate_hz"].as<double>();
    }
  } catch (const std::exception& e) {
    *error = std::string("测试配置解析失败: ") + e.what();
    return false;
  }
  if (opt->move_sec < 0.3 || opt->hold_ms < 200 || opt->skip_ms < 0 || opt->skip_ms >= opt->hold_ms ||
      opt->settle_ms < 50 || opt->rate_hz < 20.0) {
    *error = "move_sec>=0.3，hold_ms>=200 且大于 skip_ms，settle_ms>=50，rate_hz>=20";
    return false;
  }
  return true;
}

struct PinocchioProbe {
  bool installed = false;
  std::string version;
  std::string header;
};

std::string queryDebVersion(const char* package) {
  const std::string cmd = std::string("dpkg-query -W -f='${Version}' ") + package + " 2>/dev/null";
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    return {};
  }
  char buf[128] = {};
  const bool got = fgets(buf, sizeof(buf), pipe) != nullptr;
  pclose(pipe);
  if (!got || buf[0] == '\0') {
    return {};
  }
  std::string version(buf);
  while (!version.empty() && (version.back() == '\n' || version.back() == '\r')) {
    version.pop_back();
  }
  return version;
}

bool pinocchioHeader(const std::filesystem::path& prefix, std::string* header) {
  const std::filesystem::path path = prefix / "include/pinocchio/fwd.hpp";
  if (!std::filesystem::exists(path)) {
    return false;
  }
  *header = path.string();
  return true;
}

void searchPrefixList(const char* env_value, std::string* header) {
  if (!env_value || header->empty() == false) {
    return;
  }
  std::string prefixes(env_value);
  std::size_t begin = 0;
  while (begin <= prefixes.size()) {
    const std::size_t colon = prefixes.find(':', begin);
    const std::string item = prefixes.substr(begin, colon == std::string::npos ? std::string::npos : colon - begin);
    if (!item.empty() && pinocchioHeader(item, header)) {
      return;
    }
    if (colon == std::string::npos) {
      break;
    }
    begin = colon + 1;
  }
}

PinocchioProbe probePinocchio() {
  PinocchioProbe probe;
  probe.version = queryDebVersion("ros-noetic-pinocchio");
  const char* prefixes[] = {"/opt/ros/noetic", "/opt/openrobots", "/usr/local", "/usr"};
  for (const char* prefix : prefixes) {
    if (pinocchioHeader(prefix, &probe.header)) {
      break;
    }
  }
  searchPrefixList(std::getenv("CMAKE_PREFIX_PATH"), &probe.header);
  probe.installed = !probe.version.empty() || !probe.header.empty();
  return probe;
}

void printPinocchioStatus() {
  const PinocchioProbe probe = probePinocchio();
  if (!probe.installed) {
    std::cout << "Pinocchio: 未安装。控制器不会算手臂前馈力矩，跟踪误差会偏大。\n";
    return;
  }
  std::cout << "Pinocchio: 已安装";
  if (!probe.version.empty()) {
    std::cout << " ros-noetic-pinocchio " << probe.version;
  }
  if (!probe.header.empty()) {
    std::cout << " (" << probe.header << ")";
  }
  std::cout << "\n";
  std::cout << "控制器需以 -DENABLE_PINOCCHIO=ON 编译才会用它算手臂前馈力矩；"
               "没打开时力矩为 0，跟踪误差会偏大。\n";
}

std::array<double, kArmDof> mirrorArmDelta(const std::array<double, 4>& left_deg) {
  std::array<double, kArmDof> delta{};
  for (int i = 0; i < 4; ++i) {
    delta[i] = left_deg[i];
    delta[4 + i] = kRightMirror[i] * left_deg[i];
  }
  return delta;
}

void printReport(const std::array<Accum, kArmDof>& stats, const char* cmd_topic, const char* state_topic,
                 const std::array<double, kArmDof>& delta_deg) {
  std::cout << "\n稳态误差（" << cmd_topic << " − " << state_topic << "）\n";
  std::cout << std::fixed << std::setprecision(1);
  std::cout << "左臂步进 deg: " << delta_deg[0] << " " << delta_deg[1] << " " << delta_deg[2] << " "
            << delta_deg[3] << "\n";
  std::cout << "右臂镜像 deg: " << delta_deg[4] << " " << delta_deg[5] << " " << delta_deg[6] << " "
            << delta_deg[7] << "\n\n";
  std::cout << std::left << std::setw(16) << "关节" << std::right << std::setw(10) << "指令°"
            << std::setw(10) << "反馈°" << std::setw(10) << "误差°" << std::setw(10) << "|误差|°"
            << std::setw(10) << "kp" << std::setw(10) << "kd" << std::setw(8) << "mode"
            << "\n";

  double worst = 0.0;
  const char* worst_name = kArmNames[0];
  int counted = 0;
  for (int i = 0; i < kArmDof; ++i) {
    const Accum& s = stats[i];
    std::cout << std::left << std::setw(16) << kArmNames[i] << std::right;
    if (s.n <= 0) {
      std::cout << std::setw(10) << "-" << std::setw(10) << "-" << std::setw(10) << "-" << std::setw(10)
                << "-" << std::setw(10) << "-" << std::setw(10) << "-" << std::setw(8) << "-"
                << "\n";
      continue;
    }
    const double cmd_deg = (s.sum_cmd / s.n) * kRad2Deg;
    const double fb_deg = (s.sum_fb / s.n) * kRad2Deg;
    const double err_deg = cmd_deg - fb_deg;
    const double abs_deg = (s.sum_abs / s.n) * kRad2Deg;
    const double kp = s.sum_kp / s.n;
    const double kd = s.sum_kd / s.n;
    std::cout << std::fixed << std::setprecision(2) << std::setw(10) << cmd_deg << std::setw(10) << fb_deg
              << std::setw(10) << err_deg << std::setw(10) << abs_deg << std::setprecision(3) << std::setw(10)
              << kp << std::setw(10) << kd << std::setw(8) << s.mode << "\n";
    if (abs_deg > worst) {
      worst = abs_deg;
      worst_name = kArmNames[i];
    }
    ++counted;
  }
  if (counted == 0) {
    std::cout << "\n保持段没有采到指令和反馈。确认 hardware_node 在发电机话题，手臂已切到外部控制。\n";
    return;
  }
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "\n最大平均 |误差|  " << worst_name << "  " << worst << " deg"
            << "  （样本关节数 " << counted << "）\n";
  std::cout << "mode=2 为 CSP。误差大且 kp 偏小、或 mode 不是 2 时，位置环没有按这组 PD 在跟。\n";
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parseArgs(argc, argv, &opt)) {
    return argc > 1 && (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0) ? 0 : 1;
  }

  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  leju::RobotVersion version;
  try {
    version = leju::RobotVersion::from_env();
  } catch (const std::exception& e) {
    std::cerr << "读取 ROBOT_VERSION 失败: " << e.what() << std::endl;
    return 1;
  }
  if (version.to_string() != "17") {
    std::cerr << "本测试只覆盖 ROBOT_VERSION=17 的 8 关节手臂，当前为 " << version.to_string() << std::endl;
    return 1;
  }

  std::string path_error;
  const std::string config_path = resolveConfig(argv[0], opt.config, &path_error);
  if (config_path.empty()) {
    std::cerr << path_error << std::endl;
    return 1;
  }
  std::array<double, 4> left_delta{};
  if (!loadLeftDeltaDeg(config_path, &left_delta, &opt, &path_error)) {
    std::cerr << path_error << std::endl;
    return 1;
  }
  const std::array<double, kArmDof> delta_deg = mirrorArmDelta(left_delta);

  std::cout << "=== /rt/arm_trajectory 手臂稳态误差 (ROBOT_VERSION=17) ===\n";
  printPinocchioStatus();
  std::cout << "测试配置: " << config_path << "\n";
  std::cout << std::fixed << std::setprecision(1);
  std::cout << "左臂步进 deg: " << delta_deg[0] << " " << delta_deg[1] << " " << delta_deg[2] << " "
            << delta_deg[3] << "\n";
  std::cout << "右臂镜像 deg: " << delta_deg[4] << " " << delta_deg[5] << " " << delta_deg[6] << " "
            << delta_deg[7] << "\n";
  std::cout << "移动 " << opt.move_sec << " s，保持 " << opt.hold_ms << " ms\n";

  Shared shared;
  dds::domain::DomainParticipant participant(0);
  dds::sub::qos::DataReaderQos reader_qos;
  reader_qos << dds::core::policy::Reliability::BestEffort();

  leju::dds_common::TopicSubscriber<leju::msgs::JointCmd> joint_cmd_sub(
      participant, leju::dds_topics::kJointCmd,
      [&](const leju::msgs::JointCmd& msg) { onJointCmd(shared, msg); }, nullptr, reader_qos);
  leju::dds_common::TopicSubscriber<leju::msgs::JointCmd> motor_cmd_sub(
      participant, leju::dds_topics::kMotorCmd,
      [&](const leju::msgs::JointCmd& msg) { onMotorCmd(shared, msg); }, nullptr, reader_qos);
  leju::dds_common::TopicSubscriber<leju::msgs::JointState> motor_state_sub(
      participant, leju::dds_topics::kMotorState,
      [&](const leju::msgs::JointState& msg) { onMotorState(shared, msg); }, nullptr, reader_qos);
  leju::dds_common::TopicSubscriber<leju::msgs::JointState> joint_state_sub(
      participant, leju::dds_topics::kJointState,
      [&](const leju::msgs::JointState& msg) { onJointState(shared, msg); }, nullptr, reader_qos);

  RobanVRAPI api(version);
  if (!api.initialize()) {
    std::cerr << "VR API 初始化失败" << std::endl;
    return 1;
  }

  std::cout << "等待 /rt/joint_state ...\n";
  if (!waitJointState(shared, 3000)) {
    std::cerr << "3s 内没有收到含手臂的 /rt/joint_state" << std::endl;
    api.shutdown();
    return 1;
  }

  std::array<double, kArmDof> q_hold{};
  {
    std::lock_guard<std::mutex> lock(shared.mu);
    q_hold = shared.joint_state.v;
  }

  std::cout << "切换手臂到外部控制...\n";
  if (!api.setArmMode(ControlMode::kExternal)) {
    std::cerr << "setArmMode(kExternal) 失败。控制器需要处于 Running。" << std::endl;
    api.shutdown();
    return 1;
  }

  std::cout << "保持当前位姿 " << opt.settle_ms << " ms，打开指令直通...\n";
  if (!holdPose(api, q_hold, opt.settle_ms, opt.rate_hz)) {
    std::cout << "保持阶段被中断，恢复 Auto\n";
    api.setArmMode(ControlMode::kAuto);
    api.shutdown();
    return 1;
  }

  std::array<double, kArmDof> q_target = q_hold;
  for (int i = 0; i < kArmDof; ++i) {
    q_target[i] = q_hold[i] + delta_deg[i] / kRad2Deg;
  }

  std::cout << "移动到目标...\n";
  if (!moveTo(api, q_hold, q_target, opt.move_sec, opt.rate_hz)) {
    std::cout << "移动被中断，尝试回到起点\n";
    moveTo(api, q_target, q_hold, opt.move_sec, opt.rate_hz);
    api.setArmMode(ControlMode::kAuto);
    api.shutdown();
    return 1;
  }

  std::cout << "保持目标并采样（前 " << opt.skip_ms << " ms 不计入）...\n";
  const auto dt = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / opt.rate_hz));
  const auto t_hold = std::chrono::steady_clock::now();
  const auto deadline = t_hold + std::chrono::milliseconds(opt.hold_ms);
  auto next = t_hold;
  std::array<Accum, kArmDof> stats{};
  int source = 0;  // 1=电机话题，2=关节话题
  while (std::chrono::steady_clock::now() < deadline && !g_stop) {
    if (!publishHold(api, q_target)) {
      std::cerr << "发布 /rt/arm_trajectory 失败" << std::endl;
      break;
    }
    const double held_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_hold).count();
    if (held_ms >= opt.skip_ms) {
      CmdSnap cmd;
      ArmVec state;
      {
        std::lock_guard<std::mutex> lock(shared.mu);
        if (source == 0) {
          if (shared.motor_cmd.ok && shared.motor_state.ok) {
            source = 1;
          } else if (shared.joint_cmd.ok && shared.joint_state.ok) {
            source = 2;
          }
        }
        if (source == 1) {
          cmd = shared.motor_cmd;
          state = shared.motor_state;
        } else if (source == 2) {
          cmd = shared.joint_cmd;
          state = shared.joint_state;
        }
      }
      if (cmd.ok && state.ok) {
        for (int i = 0; i < kArmDof; ++i) {
          const double err = cmd.q.v[i] - state.v[i];
          Accum& s = stats[i];
          s.n += 1;
          s.sum_cmd += cmd.q.v[i];
          s.sum_fb += state.v[i];
          s.sum_abs += std::abs(err);
          s.max_abs = std::max(s.max_abs, std::abs(err));
          if (cmd.kp.ok) {
            s.sum_kp += cmd.kp.v[i];
          }
          if (cmd.kd.ok) {
            s.sum_kd += cmd.kd.v[i];
          }
          if (cmd.mode.ok) {
            s.mode = static_cast<int>(cmd.mode.v[i]);
          }
        }
      }
    }
    next += dt;
    sleepUntil(next);
  }

  const char* cmd_topic = source == 1 ? "/rt/motor_cmd" : "/rt/joint_cmd";
  const char* state_topic = source == 1 ? "/rt/motor_state" : "/rt/joint_state";
  if (source != 1) {
    std::cout << "电机话题不齐，本段用 " << cmd_topic << " 与 " << state_topic << "\n";
  }
  printReport(stats, cmd_topic, state_topic, delta_deg);

  if (!g_stop) {
    std::cout << "\n回到阶跃前位姿...\n";
    moveTo(api, q_target, q_hold, opt.move_sec, opt.rate_hz);
    holdPose(api, q_hold, 200, opt.rate_hz);
  }

  std::cout << "恢复手臂到 Auto...\n";
  api.setArmMode(ControlMode::kAuto);
  api.shutdown();
  std::cout << "结束\n";
  return 0;
}
