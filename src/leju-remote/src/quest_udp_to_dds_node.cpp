// Quest3 UDP to DDS bridge
// Receives LejuHandPoseEvent protobuf from Quest3 via UDP, publishes QuestBonePoses and QuestJoysticks via lejusdk-vr
//
// 与 motion_capture_ik / ik_ros_uni.py 对齐说明：
// - ik_ros_uni 里 Quest3ArmInfoTransformer 从 PoseInfoList 按 bone 名取 Chest / *ArmUpper / *ArmLower / *HandPalm（见 quest3_utils.py bone_names）。
// - 本节点将 protobuf 中第 i 个 pose 按 initBoneNames() 顺序填入 DDS（与 Python bone_names 前 24 项一致；本节点多含 Neck、Head，Chest 仍为索引 23）。
// - 默认对每帧位置/四元数做「Quest 左手系 → 右手系」重映射（见 processPoseData 内 rx=-lz 等）。若与旧 ROS 桥（未做重映射、由下游解释）不一致，会导致与 ik_ros_uni 行为不同。
// - 环境变量 QUEST_POSE_COORD_MODE=passthrough 时跳过该重映射，直接使用 protobuf 的 x,y,z 与 qx,qy,qz,qw（用于与 ROS 侧逐帧对比调试）。

#include <hand_pose.pb.h>
#include <robot_info.pb.h>

using namespace protos;

#include <lejusdk-vr/lejusdk_vr.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ifaddrs.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int QUEST3_DEFAULT_DATA_PORT = 10019;
constexpr int QUEST3_DISCOVERY_START_PORT = 11000;
constexpr int QUEST3_DISCOVERY_END_PORT = 11010;
constexpr int ROBOT_INFO_START_PORT = 11050;
constexpr int ROBOT_INFO_END_PORT = 11060;

std::atomic<bool> g_running{true};

void signalHandler(int) { g_running = false; }

std::vector<std::string> getLocalBroadcastIps() {
  std::vector<std::string> result;
  std::set<std::string> unique;

  struct ifaddrs* ifaddr = nullptr;
  if (getifaddrs(&ifaddr) == -1) {
    std::cerr << "getifaddrs failed: " << strerror(errno) << std::endl;
    return result;
  }

  for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET)
      continue;

    std::string name(ifa->ifa_name ? ifa->ifa_name : "");
    if (name.rfind("docker", 0) == 0 || name.rfind("br-", 0) == 0 || name.rfind("veth", 0) == 0)
      continue;

    struct sockaddr_in* addr = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_broadaddr);
    if (!addr)
      continue;

    char buf[INET_ADDRSTRLEN] = {0};
    if (!inet_ntop(AF_INET, &addr->sin_addr, buf, sizeof(buf)))
      continue;

    std::string ip(buf);
    if (!ip.empty() && unique.insert(ip).second)
      result.push_back(ip);
  }

  freeifaddrs(ifaddr);
  std::sort(result.begin(), result.end());
  return result;
}

}  // namespace

class QuestUdpToDdsNode {
 public:
  QuestUdpToDdsNode()
      : data_socket_(-1),
        listening_udp_ports_cnt_(0),
        exit_listen_thread_(false) {
    const char* coord = std::getenv("QUEST_POSE_COORD_MODE");
    if (coord && std::strcmp(coord, "passthrough") == 0) {
      pose_coord_passthrough_ = true;
      std::cout << "[quest_udp_to_dds] QUEST_POSE_COORD_MODE=passthrough — 不重映射骨骼位姿，与默认 remux 二选一用于对齐 ik_ros_uni/ROS 桥\n";
    } else {
      std::cout << "[quest_udp_to_dds] QUEST_POSE_COORD_MODE=remux (default) — Quest 左手系→右手系 rx=-lz,ry=-lx,rz=ly\n";
    }
    initBoneNames();
    if (!vr_api_.initialize()) {
      std::cerr << "Failed to initialize lejusdk-vr" << std::endl;
    }
  }

  ~QuestUdpToDdsNode() { shutdown(); }

  void shutdown() {
    exit_listen_thread_ = true;
    if (data_socket_ >= 0) {
      close(data_socket_);
      data_socket_ = -1;
    }
    vr_api_.shutdown();
  }

  void updateBroadcastIps(const std::vector<std::string>& ips) {
    std::set<std::string> unique(ips.begin(), ips.end());
    broadcast_ips_.assign(unique.begin(), unique.end());
    std::sort(broadcast_ips_.begin(), broadcast_ips_.end());
  }

  bool setupSocket(const std::string& server_address, int port) {
    if (data_socket_ >= 0) {
      std::cout << "Socket already established, skipping creation." << std::endl;
      return true;
    }

    data_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (data_socket_ < 0) {
      std::cerr << "Failed to create UDP socket: " << strerror(errno) << std::endl;
      return false;
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port);

    if (inet_pton(AF_INET, server_address.c_str(), &server_addr_.sin_addr) <= 0) {
      std::cerr << "Invalid server address: " << server_address << std::endl;
      ::close(data_socket_);
      data_socket_ = -1;
      return false;
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    if (setsockopt(data_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
      std::cerr << "Failed to set socket timeout: " << strerror(errno) << std::endl;
    }

    server_ip_ = server_address;
    server_port_ = port;
    return true;
  }

  bool sendInitialMessage() {
    if (data_socket_ < 0) {
      std::cerr << "Data socket is not initialized." << std::endl;
      return false;
    }

    const char* msg = "hi";
    const int max_retries = 200;
    char buffer[1024];

    for (int attempt = 0; attempt < max_retries && g_running; ++attempt) {
      ssize_t sent = sendto(data_socket_, msg, strlen(msg), 0,
                            reinterpret_cast<struct sockaddr*>(&server_addr_),
                            sizeof(server_addr_));
      if (sent < 0) {
        std::cerr << "Failed to send 'hi' to Quest3: " << strerror(errno) << std::endl;
      }

      struct sockaddr_in from_addr;
      socklen_t from_len = sizeof(from_addr);
      ssize_t recvd = recvfrom(data_socket_, buffer, sizeof(buffer), 0,
                               reinterpret_cast<struct sockaddr*>(&from_addr), &from_len);

      if (recvd > 0) {
        std::cout << "\033[92mAcknowledgment From Quest3 received on attempt " << (attempt + 1)
                  << ", start to receiving data...\033[0m" << std::endl;
        return true;
      } else {
        std::cout << "\033[91mQuest3_timeout: Attempt " << (attempt + 1) << " timed out. Retrying...\033[0m"
                  << std::endl;
      }
    }

    std::cout << "Failed to send message after 200 attempts." << std::endl;
    return false;
  }

  bool restartSocket() {
    std::cout << "Restarting socket connection..." << std::endl;
    if (data_socket_ >= 0) {
      ::close(data_socket_);
      data_socket_ = -1;
    }

    if (!setupSocket(server_ip_, server_port_)) {
      std::cout << "Failed to recreate socket." << std::endl;
      return false;
    }

    if (!sendInitialMessage()) {
      std::cout << "Failed to restart socket connection." << std::endl;
      return false;
    }

    std::cout << "Socket connection restarted successfully." << std::endl;
    return true;
  }

  void broadcastRobotInfoAndWaitForQuest3() {
    std::thread periodic_broadcaster(&QuestUdpToDdsNode::periodicRobotInfoBroadcaster, this);

    std::vector<std::thread> threads;
    for (int port = QUEST3_DISCOVERY_START_PORT; port <= QUEST3_DISCOVERY_END_PORT; ++port) {
      threads.emplace_back(&QuestUdpToDdsNode::listenForQuest3Broadcasts, this, port);
    }

    if (listening_udp_ports_cnt_ == 0) {
      std::cerr << "\033[91mAll UDP broadcast ports are occupied. Please check with 'lsof -i :11000-11010'.\033[0m"
                << std::endl;
    }

    for (auto& t : threads) {
      if (t.joinable())
        t.join();
    }

    if (periodic_broadcaster.joinable())
      periodic_broadcaster.join();

    if (!server_ip_.empty()) {
      std::cout << "\033[92mQuest3 device found at IP: " << server_ip_ << "\033[0m" << std::endl;
      std::cout << "\033[92mReceived Quest3 Broadcast, starting to connect.\033[0m" << std::endl;
    }
  }

  void run() {
    constexpr double rate_hz = 100.0;
    auto interval = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / rate_hz));

    while (g_running) {
      uint8_t buf[4096];
      struct sockaddr_in from_addr;
      socklen_t from_len = sizeof(from_addr);

      ssize_t recvd = recvfrom(data_socket_, buf, sizeof(buf), 0,
                               reinterpret_cast<struct sockaddr*>(&from_addr), &from_len);

      if (recvd < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::cout << "Timeout occurred, no data received. Restarting socket..." << std::endl;
          if (!restartSocket()) {
            break;
          }
          continue;
        } else {
          std::cerr << "recvfrom error: " << strerror(errno) << std::endl;
          if (!restartSocket()) {
            break;
          }
          continue;
        }
      }

      LejuHandPoseEvent event;
      if (!event.ParseFromArray(buf, static_cast<int>(recvd))) {
        std::cerr << "Failed to parse LejuHandPoseEvent protobuf." << std::endl;
        continue;
      }

      auto now = std::chrono::system_clock::now();
      auto sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
      auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() % 1000000000;

      // Publish joystick data (always) via lejusdk-vr
      leju::vr::QuestJoystickData joy_msg;
      processJoystickData(event, joy_msg);
      joy_msg.header_sec = static_cast<int32_t>(sec);
      joy_msg.header_nanosec = static_cast<uint32_t>(nsec);
      vr_api_.publishQuestJoystickData(joy_msg);
      // Publish first, then stop: ensure vr-control receives this X+Y frame.
      if (joy_msg.left_first_button_pressed && joy_msg.left_second_button_pressed) {
        std::cout << "[REMOTE] X+Y forwarded, stopping remote bridge..." << std::endl;
        g_running = false;
      }

      // Publish bone poses (only when high confidence) via lejusdk-vr
      if (event.isdatahighconfidence()) {
        leju::vr::QuestBonePosesData poses_msg;
        processPoseData(event, poses_msg);
        poses_msg.header_sec = static_cast<int32_t>(sec);
        poses_msg.header_nanosec = static_cast<uint32_t>(nsec);
        poses_msg.timestamp_ms = event.timestamp();
        poses_msg.is_high_confidence = event.isdatahighconfidence();
        poses_msg.is_hand_tracking = event.ishandtracking();
        vr_api_.publishQuestBonePoses(poses_msg);
      }

      std::this_thread::sleep_for(interval);
    }
  }

 private:
  void initBoneNames() {
    bone_names_ = {"LeftArmUpper",  "LeftArmLower", "RightArmUpper", "RightArmLower",
                   "LeftHandPalm",  "RightHandPalm", "LeftHandThumbMetacarpal",
                   "LeftHandThumbProximal", "LeftHandThumbDistal", "LeftHandThumbTip",
                   "LeftHandIndexTip", "LeftHandMiddleTip", "LeftHandRingTip", "LeftHandLittleTip",
                   "RightHandThumbMetacarpal", "RightHandThumbProximal", "RightHandThumbDistal",
                   "RightHandThumbTip", "RightHandIndexTip", "RightHandMiddleTip", "RightHandRingTip",
                   "RightHandLittleTip", "Root", "Chest", "Neck", "Head"};

    for (size_t i = 0; i < bone_names_.size(); ++i) {
      bone_name_to_index_[bone_names_[i]] = static_cast<int>(i);
      index_to_bone_name_[static_cast<int>(i)] = bone_names_[i];
    }
  }

  void processJoystickData(const LejuHandPoseEvent& event, leju::vr::QuestJoystickData& msg) {
    const auto& left = event.left_joystick();
    const auto& right = event.right_joystick();

    msg.left_x = left.x();
    msg.left_y = left.y();
    msg.left_trigger = left.trigger();
    msg.left_grip = left.grip();
    msg.left_first_button_pressed = left.firstbuttonpressed();
    msg.left_second_button_pressed = left.secondbuttonpressed();
    msg.left_first_button_touched = left.firstbuttontouched();
    msg.left_second_button_touched = left.secondbuttontouched();

    msg.right_x = right.x();
    msg.right_y = right.y();
    msg.right_trigger = right.trigger();
    msg.right_grip = right.grip();
    msg.right_first_button_pressed = right.firstbuttonpressed();
    msg.right_second_button_pressed = right.secondbuttonpressed();
    msg.right_first_button_touched = right.firstbuttontouched();
    msg.right_second_button_touched = right.secondbuttontouched();
  }

  void processPoseData(const LejuHandPoseEvent& event, leju::vr::QuestBonePosesData& poses_msg) {
    std::vector<leju::vr::Pose> poses;
    const int n = std::min(static_cast<int>(event.poses_size()), static_cast<int>(bone_names_.size()));

    for (int i = 0; i < n; ++i) {
      const auto& pose = event.poses(i);

      double lx = pose.position().x();
      double ly = pose.position().y();
      double lz = pose.position().z();

      double rx, ry, rz;
      double qx = pose.quaternion().x();
      double qy = pose.quaternion().y();
      double qz = pose.quaternion().z();
      double qw = pose.quaternion().w();
      double rqx, rqy, rqz, rqw;

      if (pose_coord_passthrough_) {
        rx = lx;
        ry = ly;
        rz = lz;
        rqx = qx;
        rqy = qy;
        rqz = qz;
        rqw = qw;
      } else {
        // Convert from Quest3 left-handed to right-handed frame（与历史 UDP 桥一致）
        rx = -lz;
        ry = -lx;
        rz = ly;
        rqx = -qz;
        rqy = -qx;
        rqz = qy;
        rqw = qw;
      }

      leju::vr::Pose p;
      p.x = static_cast<float>(rx);
      p.y = static_cast<float>(ry);
      p.z = static_cast<float>(rz);
      p.qx = static_cast<float>(rqx);
      p.qy = static_cast<float>(rqy);
      p.qz = static_cast<float>(rqz);
      p.qw = static_cast<float>(rqw);
      poses.push_back(p);
    }

    poses_msg.poses = poses;
  }

  void sendRobotInfoOnBroadcastIps(const std::string& robot_name, int robot_version, int start_port,
                                   int end_port) {
    if (broadcast_ips_.empty())
      return;

    RobotDescription desc;
    desc.set_robot_name(robot_name);
    desc.set_robot_version(robot_version);
    std::string serialized = desc.SerializeAsString();

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
      return;

    int yes = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)) < 0) {
      ::close(sock);
      return;
    }

    for (const auto& ip : broadcast_ips_) {
      for (int p = start_port; p <= end_port; ++p) {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(p);
        if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0)
          continue;
        sendto(sock, serialized.data(), serialized.size(), 0,
               reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
      }
    }
    ::close(sock);
  }

  void periodicRobotInfoBroadcaster() {
    std::cout << "Starting periodic robot info broadcaster (kuavo, ports 11050-11060)." << std::endl;
    int robot_version = 45;

    while (!exit_listen_thread_ && g_running) {
      sendRobotInfoOnBroadcastIps("kuavo", robot_version, ROBOT_INFO_START_PORT, ROBOT_INFO_END_PORT);
      for (int i = 0; i < 10; ++i) {
        if (exit_listen_thread_ || !g_running)
          break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  void listenForQuest3Broadcasts(int port) {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
      return;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      ::close(sock);
      return;
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ++listening_udp_ports_cnt_;

    char buf[1024];
    while (!exit_listen_thread_ && g_running) {
      struct sockaddr_in from_addr;
      socklen_t from_len = sizeof(from_addr);
      ssize_t recvd = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                               reinterpret_cast<struct sockaddr*>(&from_addr), &from_len);

      if (recvd < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
          continue;
        continue;
      }

      buf[recvd] = '\0';
      char ip_str[INET_ADDRSTRLEN] = {0};
      inet_ntop(AF_INET, &from_addr.sin_addr, ip_str, sizeof(ip_str));

      std::cout << "Received from Quest3: " << buf << " from " << ip_str << " on port " << port
                << " - Setting up connection" << std::endl;

      exit_listen_thread_ = true;
      server_ip_ = ip_str;
      setupSocket(server_ip_, QUEST3_DEFAULT_DATA_PORT);
      break;
    }

    ::close(sock);
  }

  leju::vr::KuavoVRAPI vr_api_;

  std::vector<std::string> bone_names_;
  std::map<std::string, int> bone_name_to_index_;
  std::map<int, std::string> index_to_bone_name_;

  int data_socket_;
  struct sockaddr_in server_addr_;
  std::string server_ip_;
  int server_port_{QUEST3_DEFAULT_DATA_PORT};

  std::vector<std::string> broadcast_ips_;
  std::atomic<int> listening_udp_ports_cnt_;
  std::atomic<bool> exit_listen_thread_;

  /// false：默认 remux；true：QUEST_POSE_COORD_MODE=passthrough，与 ik_ros_uni 若使用「未重映射」的 Pose 时对齐
  bool pose_coord_passthrough_{false};
};

int main(int argc, char** argv) {
  std::signal(SIGINT, signalHandler);

  QuestUdpToDdsNode node;

  auto broadcast_ips = getLocalBroadcastIps();
  std::cout << "Local broadcast IPs: ";
  for (const auto& ip : broadcast_ips)
    std::cout << ip << " ";
  std::cout << std::endl;

  node.updateBroadcastIps(broadcast_ips);

  std::string server_address;
  int port = QUEST3_DEFAULT_DATA_PORT;

  if (argc >= 2 && std::string(argv[1]).find('.') != std::string::npos) {
    std::string arg = argv[1];
    auto pos = arg.find(':');
    if (pos != std::string::npos) {
      server_address = arg.substr(0, pos);
      try {
        port = std::stoi(arg.substr(pos + 1));
      } catch (const std::exception& e) {
        std::cerr << "Invalid quest_ip format: port '" << arg.substr(pos + 1)
                  << "' is not a number. Use quest_ip:=IP or quest_ip:=IP:port"
                  << " (e.g. quest_ip:=10.10.31.15)" << std::endl;
        return 1;
      }
    } else {
      server_address = arg;
    }

    // 校验 IP 格式，避免 inet_pton 静默失败（如 10.10.31.15- 尾随非法字符）
    struct in_addr addr;
    if (inet_pton(AF_INET, server_address.c_str(), &addr) <= 0) {
      std::cerr << "Invalid quest_ip: '" << server_address
                << "' is not a valid IPv4 address. Use e.g. quest_ip:=10.10.31.15"
                << std::endl;
      return 1;
    }

    if (!node.setupSocket(server_address, port)) {
      std::cerr << "Failed to setup socket for Quest3." << std::endl;
      return 1;
    }
  } else {
    std::cout << "IP not specified. Waiting for Quest3 to connect." << std::endl;
    node.broadcastRobotInfoAndWaitForQuest3();
  }

  if (node.sendInitialMessage()) {
    node.run();
  } else {
    std::cout << "Failed to establish initial connection." << std::endl;
  }

  return 0;
}
