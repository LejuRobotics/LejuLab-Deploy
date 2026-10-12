// 调试工具：订阅全部 /monitor/system_info/* 话题并打印，
// 用于验收 system_info_node 与上位机链路排查（类比 leju-audio/audio_stream_probe）。
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/Float64Types.hpp>
#include <lejusdk-topic-pubsub/topic_names.h>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>

namespace {
std::atomic<bool> g_running{true};
void signalHandler(int) { g_running = false; }

void printArray(const char* name, const leju::msgs::Float64Array& msg) {
  std::string s;
  char buf[32];
  for (double v : msg.data()) {
    std::snprintf(buf, sizeof(buf), "%.1f ", v);
    s += buf;
  }
  std::printf("[%s] n=%zu: %s\n", name, msg.data().size(), s.c_str());
  std::fflush(stdout);
}
}  // namespace

int main() {
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  dds::domain::DomainParticipant participant(0);
  using leju::dds_common::TopicSubscriber;
  using leju::msgs::Float64Array;
  namespace topics = leju::dds_topics;

  TopicSubscriber<Float64Array> sub_usage(
      participant, topics::kMonitorCpuUsage,
      [](const Float64Array& m) { printArray("cpu_usage", m); });
  TopicSubscriber<Float64Array> sub_temp(
      participant, topics::kMonitorCpuTemperature,
      [](const Float64Array& m) { printArray("cpu_temp", m); });
  TopicSubscriber<Float64Array> sub_freq(
      participant, topics::kMonitorCpuFrequency,
      [](const Float64Array& m) { printArray("cpu_freq", m); });
  TopicSubscriber<Float64Array> sub_mem(
      participant, topics::kMonitorMemory,
      [](const Float64Array& m) { printArray("memory", m); });
  TopicSubscriber<Float64Array> sub_rl(
      participant, topics::kMonitorRlCpuCore,
      [](const Float64Array& m) { printArray("rl_cpu_core", m); });

  std::printf("listening on /monitor/system_info/* ... Ctrl-C to exit\n");
  while (g_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
