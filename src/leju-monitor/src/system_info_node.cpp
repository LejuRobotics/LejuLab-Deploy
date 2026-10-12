// 系统监控节点：5Hz 发布 /monitor/system_info/* 调试话题。
// 从 kuavo-ros-control system_info_publisher.py (ROS/psutil) 迁移到
// RK3588 C++/CycloneDDS，全部数据直接读 /proc 与 /sys，无外部依赖。
//
// 话题（类型见 topic_names.h 注释）：
//  - cpu_usage / cpu_temperature / cpu_frequency / memory
//  - rl_cpu_core: 外部扫 /proc/<pid>/task/<tid>/stat 第 39 字段(processor)，
//    零侵入拿到 run_rl_controller 各线程当前所在核心（验证大核绑定用）
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/Float64Types.hpp>
#include <lejusdk-topic-pubsub/topic_names.h>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>
#include <lejusdk-utils/cpu_affinity.hpp>

namespace {

constexpr int kDefaultDomainId = 0;
constexpr int kPublishHz = 5;

std::atomic<bool> g_running{true};
void signalHandler(int) { g_running = false; }

std::string envOr(const char* name, const std::string& def) {
  const char* v = std::getenv(name);
  return v && *v ? std::string(v) : def;
}

bool readFileFirstLine(const std::string& path, std::string& out) {
  std::ifstream f(path);
  if (!f.is_open()) return false;
  return static_cast<bool>(std::getline(f, out));
}

// ── CPU 使用率：/proc/stat 两次采样差分 ─────────────────────────────
struct CpuTimes {
  unsigned long long idle = 0;   // idle + iowait
  unsigned long long total = 0;  // 全字段之和
};

std::vector<CpuTimes> readPerCoreCpuTimes() {
  std::vector<CpuTimes> cores;
  std::ifstream f("/proc/stat");
  std::string line;
  while (std::getline(f, line)) {
    // 只取 "cpuN ..." 行，跳过汇总行 "cpu "
    if (line.compare(0, 3, "cpu") != 0 || !std::isdigit(line[3])) continue;
    std::istringstream iss(line);
    std::string label;
    iss >> label;
    unsigned long long v = 0;
    CpuTimes t;
    int field = 0;
    while (iss >> v) {
      t.total += v;
      if (field == 3 || field == 4) t.idle += v;  // idle, iowait
      ++field;
    }
    cores.push_back(t);
  }
  return cores;
}

std::vector<double> computeCpuUsage(const std::vector<CpuTimes>& prev,
                                    const std::vector<CpuTimes>& curr) {
  std::vector<double> usage;
  const size_t n = std::min(prev.size(), curr.size());
  usage.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const double total_d = static_cast<double>(curr[i].total - prev[i].total);
    const double idle_d = static_cast<double>(curr[i].idle - prev[i].idle);
    usage.push_back(total_d > 0.0 ? 100.0 * (1.0 - idle_d / total_d) : 0.0);
  }
  return usage;
}

// ── 温度：/sys/class/thermal/thermal_zone*/temp (milli-°C) ──────────
std::vector<double> readThermalZoneTemps() {
  std::vector<double> temps;
  for (int zone = 0;; ++zone) {
    std::string line;
    if (!readFileFirstLine("/sys/class/thermal/thermal_zone" +
                               std::to_string(zone) + "/temp",
                           line)) {
      break;
    }
    temps.push_back(std::strtod(line.c_str(), nullptr) / 1000.0);
  }
  return temps;
}

// ── 频率：scaling_cur_freq (kHz → MHz) ──────────────────────────────
std::vector<double> readCpuFrequenciesMhz(int num_cores) {
  std::vector<double> freqs;
  freqs.reserve(num_cores);
  for (int i = 0; i < num_cores; ++i) {
    std::string line;
    if (readFileFirstLine("/sys/devices/system/cpu/cpu" + std::to_string(i) +
                              "/cpufreq/scaling_cur_freq",
                          line)) {
      freqs.push_back(std::strtod(line.c_str(), nullptr) / 1000.0);
    } else {
      freqs.push_back(0.0);  // 无 cpufreq (如部分虚拟机) 占位，保持长度=核数
    }
  }
  return freqs;
}

// ── 内存：/proc/meminfo → [percent, used_GB, total_GB, available_GB] ─
std::vector<double> readMemoryInfo() {
  std::ifstream f("/proc/meminfo");
  std::string key;
  unsigned long long value_kb = 0, total_kb = 0, avail_kb = 0;
  std::string unit;
  while (f >> key >> value_kb >> unit) {
    if (key == "MemTotal:") total_kb = value_kb;
    else if (key == "MemAvailable:") avail_kb = value_kb;
    if (total_kb && avail_kb) break;
  }
  if (total_kb == 0) return {};
  const double kGb = 1024.0 * 1024.0;
  const unsigned long long used_kb = total_kb - avail_kb;
  return {100.0 * static_cast<double>(used_kb) / static_cast<double>(total_kb),
          static_cast<double>(used_kb) / kGb,
          static_cast<double>(total_kb) / kGb,
          static_cast<double>(avail_kb) / kGb};
}

// ── RL controller 线程所在核心 ──────────────────────────────────────
// /proc/<tid>/comm 最长 15 字符，"run_rl_controller" 会被截断为
// "run_rl_controll"，用前缀匹配。
bool commMatches(const std::string& comm, const std::string& target) {
  const std::string truncated = target.substr(0, 15);
  return comm == target || comm == truncated;
}

int findProcessByName(const std::string& name) {
  DIR* dir = opendir("/proc");
  if (!dir) return -1;
  int found = -1;
  while (dirent* ent = readdir(dir)) {
    const char* d = ent->d_name;
    if (!std::isdigit(d[0])) continue;
    std::string comm;
    if (!readFileFirstLine(std::string("/proc/") + d + "/comm", comm)) continue;
    if (commMatches(comm, name)) {
      found = std::atoi(d);
      break;
    }
  }
  closedir(dir);
  return found;
}

// /proc/<pid>/task/<tid>/stat 第 39 字段 processor（comm 可含空格/括号，
// 从最后一个 ')' 之后按空格切分，state 为第 3 字段）。
int readThreadCpu(int pid, int tid) {
  std::string stat;
  if (!readFileFirstLine("/proc/" + std::to_string(pid) + "/task/" +
                             std::to_string(tid) + "/stat",
                         stat)) {
    return -1;
  }
  const size_t rparen = stat.rfind(')');
  if (rparen == std::string::npos) return -1;
  std::istringstream iss(stat.substr(rparen + 1));
  std::string tok;
  for (int field = 3; field <= 39; ++field) {
    if (!(iss >> tok)) return -1;
  }
  return std::atoi(tok.c_str());
}

// [tid0, core0, tid1, core1, ...]，主线程(tid==pid，即 RL 控制线程)在前。
// 进程不在返回空数组。pid 命中后缓存；未找到时 1Hz 降频重扫
// （全量扫 /proc 较贵，5Hz 扫会白耗 ~5% 单核）。
std::vector<double> readRlThreadCores(const std::string& proc_name, int& cached_pid) {
  static int rescan_countdown = 0;
  if (cached_pid > 0 && !std::ifstream("/proc/" + std::to_string(cached_pid) + "/comm").is_open()) {
    cached_pid = -1;  // 进程已退出，重新扫
  }
  if (cached_pid <= 0) {
    if (--rescan_countdown > 0) return {};
    rescan_countdown = kPublishHz;  // 每 1s 才全量扫一次
    cached_pid = findProcessByName(proc_name);
  }
  if (cached_pid <= 0) return {};

  std::vector<double> result;
  const std::string task_dir = "/proc/" + std::to_string(cached_pid) + "/task";
  DIR* dir = opendir(task_dir.c_str());
  if (!dir) {
    cached_pid = -1;
    return {};
  }
  std::vector<int> tids;
  while (dirent* ent = readdir(dir)) {
    if (std::isdigit(ent->d_name[0])) tids.push_back(std::atoi(ent->d_name));
  }
  closedir(dir);
  std::sort(tids.begin(), tids.end());  // 主线程 tid==pid 通常最小

  for (int tid : tids) {
    const int cpu = readThreadCpu(cached_pid, tid);
    if (cpu < 0) continue;
    if (tid == cached_pid) {  // 主线程放最前
      result.insert(result.begin(), {static_cast<double>(tid), static_cast<double>(cpu)});
    } else {
      result.push_back(static_cast<double>(tid));
      result.push_back(static_cast<double>(cpu));
    }
  }
  return result;
}

// ── 发布辅助 ────────────────────────────────────────────────────────
void fillHeader(leju::msgs::Float64Array& msg) {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(now);
  msg.header_sec(static_cast<int32_t>(sec.count()));
  msg.header_nanosec(static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now - sec).count()));
}

void publishArray(leju::dds_common::TopicPublisher<leju::msgs::Float64Array>& pub,
                  const std::vector<double>& data) {
  leju::msgs::Float64Array msg;
  fillHeader(msg);
  msg.data(data);
  pub.publish(msg);
}

}  // namespace

int main() {
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

#ifdef __aarch64__
  // RK3588: 绑到小核 0-3，不与 RL 控制线程(大核 4-7)争抢
  leju::cpu::bindCurrentThreadToCpuRange(0, leju::cpu::kRk3588BigCoreFirst - 1);
#endif

  // 可通过环境变量改被监控进程名（默认 RL 控制器）
  const std::string rl_proc_name = envOr("MONITOR_RL_PROC_NAME", "run_rl_controller");

  dds::domain::DomainParticipant participant(kDefaultDomainId);
  using leju::dds_common::TopicPublisher;
  using leju::msgs::Float64Array;
  namespace topics = leju::dds_topics;

  TopicPublisher<Float64Array> pub_usage(participant, topics::kMonitorCpuUsage);
  TopicPublisher<Float64Array> pub_temp(participant, topics::kMonitorCpuTemperature);
  TopicPublisher<Float64Array> pub_freq(participant, topics::kMonitorCpuFrequency);
  TopicPublisher<Float64Array> pub_mem(participant, topics::kMonitorMemory);
  TopicPublisher<Float64Array> pub_rl_cpu(participant, topics::kMonitorRlCpuCore);

  std::cout << "[INFO] monitoring process: " << rl_proc_name << std::endl;

  std::vector<CpuTimes> prev_times = readPerCoreCpuTimes();
  const int num_cores = static_cast<int>(prev_times.size());
  int rl_pid = -1;

  const auto period = std::chrono::milliseconds(1000 / kPublishHz);
  auto next_wake = std::chrono::steady_clock::now() + period;

  std::cout << "[INFO] system info publisher started (" << kPublishHz
            << " Hz, " << num_cores << " cores)" << std::endl;

  while (g_running) {
    std::this_thread::sleep_until(next_wake);
    next_wake += period;

    // CPU 使用率（差分，首轮起即有效：init 时已采过一次）
    const std::vector<CpuTimes> curr_times = readPerCoreCpuTimes();
    publishArray(pub_usage, computeCpuUsage(prev_times, curr_times));
    prev_times = curr_times;

    // 温度 / 频率 / 内存
    publishArray(pub_temp, readThermalZoneTemps());
    publishArray(pub_freq, readCpuFrequenciesMhz(num_cores));
    publishArray(pub_mem, readMemoryInfo());

    // RL controller 各线程所在核心
    publishArray(pub_rl_cpu, readRlThreadCores(rl_proc_name, rl_pid));
  }

  std::cout << "[INFO] system info publisher exiting" << std::endl;
  return 0;
}
