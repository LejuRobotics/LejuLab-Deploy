/**
 * @file posture_height_cmd_node.cpp
 * @brief 流式发布 AMP 姿态高度命令到 DDS /rt/posture_height_cmd
 *
 * 控制器侧只认 timeout 内的最新值；本节点需以 ~20Hz 持续发送，停发后机器人自然回站立。
 *
 * 用法:
 *   posture_height_cmd_node --height 0.65 --duration 3
 *   posture_height_cmd_node --sequence "0.65:3,0.55:2"
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/Float64Types.hpp>
#include <lejusdk-topic-pubsub/topic_names.h>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>

namespace {

struct HeightStep {
  double height_m = 0.0;
  double duration_sec = 0.0;
};

constexpr double kDefaultStandingHeight = 0.77;
constexpr double kDefaultSquatDepthMax = 0.22;
constexpr double kDefaultDurationSec = 3.0;
constexpr double kDefaultRateHz = 20.0;

void PrintUsage(const char* prog) {
  std::fprintf(stderr,
               "用法:\n"
               "  %s --height <m> [--duration <sec>] [--rate <hz>]\n"
               "  %s --sequence \"<height>:<sec>[,<height>:<sec>...]\" [--rate <hz>]\n"
               "\n"
               "示例:\n"
               "  %s --height 0.65 --duration 3\n"
               "  %s --sequence \"0.65:3,0.55:2\"\n"
               "\n"
               "高度映射 (standing=%.2f m):\n"
               "  height=0.77 -> 站立; height=0.55 -> 最深蹲 (depth=%.2f m)\n",
               prog, prog, prog, prog, kDefaultStandingHeight, kDefaultSquatDepthMax);
}

bool ParseDoubleArg(int argc, char** argv, int& index, const char* name, double& out) {
  if (index + 1 >= argc) {
    std::fprintf(stderr, "缺少 %s 参数值\n", name);
    return false;
  }
  char* end = nullptr;
  const double value = std::strtod(argv[index + 1], &end);
  if (end == argv[index + 1] || *end != '\0') {
    std::fprintf(stderr, "无效的 %s: %s\n", name, argv[index + 1]);
    return false;
  }
  out = value;
  index += 2;
  return true;
}

bool ParseSequence(const std::string& text, std::vector<HeightStep>& steps) {
  std::size_t start = 0;
  while (start < text.size()) {
    const std::size_t comma = text.find(',', start);
    const std::string token = text.substr(start, comma == std::string::npos ? std::string::npos
                                                                            : comma - start);
    const std::size_t colon = token.find(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= token.size()) {
      std::fprintf(stderr, "无效的 sequence 片段: %s\n", token.c_str());
      return false;
    }

    char* end = nullptr;
    HeightStep step;
    step.height_m = std::strtod(token.c_str(), &end);
    if (end == token.c_str() || *end != ':') {
      std::fprintf(stderr, "无效的高度: %s\n", token.c_str());
      return false;
    }
    step.duration_sec = std::strtod(token.c_str() + colon + 1, &end);
    if (end == token.c_str() + colon + 1 || *end != '\0' || step.duration_sec <= 0.0) {
      std::fprintf(stderr, "无效的时长: %s\n", token.c_str());
      return false;
    }
    steps.push_back(step);

    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }
  return !steps.empty();
}

void PublishHeightForDuration(leju::dds_common::TopicPublisher<leju::msgs::Float64>& pub,
                              double height_m,
                              double duration_sec,
                              double rate_hz,
                              double standing_height) {
  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  const auto end_time = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(duration_sec));

  const double squat_depth = std::max(0.0, standing_height - height_m);
  std::printf("[posture_height_cmd_node] streaming height=%.3f m, squat_depth=%.3f m, "
              "duration=%.2f s, rate=%.1f Hz\n",
              height_m, squat_depth, duration_sec, rate_hz);
  std::fflush(stdout);

  while (std::chrono::steady_clock::now() < end_time) {
    const auto tick_start = std::chrono::steady_clock::now();
    const auto now = std::chrono::system_clock::now();
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
    const auto nsec =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() - sec);

    leju::msgs::Float64 msg(static_cast<int32_t>(sec.count()),
                            static_cast<uint32_t>(nsec.count()),
                            height_m);
    pub.publish(msg);

    const auto next_tick = tick_start + period;
    if (next_tick > end_time) {
      break;
    }
    std::this_thread::sleep_until(next_tick);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 2;
  }

  double height_m = 0.0;
  double duration_sec = kDefaultDurationSec;
  double rate_hz = kDefaultRateHz;
  double standing_height = kDefaultStandingHeight;
  std::string sequence_text;
  bool have_height = false;
  bool have_sequence = false;

  for (int i = 1; i < argc;) {
    if (std::strcmp(argv[i], "--height") == 0) {
      if (!ParseDoubleArg(argc, argv, i, "--height", height_m)) {
        return 2;
      }
      have_height = true;
      continue;
    }
    if (std::strcmp(argv[i], "--duration") == 0) {
      if (!ParseDoubleArg(argc, argv, i, "--duration", duration_sec)) {
        return 2;
      }
      continue;
    }
    if (std::strcmp(argv[i], "--rate") == 0) {
      if (!ParseDoubleArg(argc, argv, i, "--rate", rate_hz)) {
        return 2;
      }
      if (rate_hz <= 0.0) {
        std::fprintf(stderr, "--rate 必须 > 0\n");
        return 2;
      }
      continue;
    }
    if (std::strcmp(argv[i], "--standing-height") == 0) {
      if (!ParseDoubleArg(argc, argv, i, "--standing-height", standing_height)) {
        return 2;
      }
      continue;
    }
    if (std::strcmp(argv[i], "--sequence") == 0) {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "缺少 --sequence 参数值\n");
        return 2;
      }
      sequence_text = argv[i + 1];
      have_sequence = true;
      i += 2;
      continue;
    }
    if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
      PrintUsage(argv[0]);
      return 0;
    }

    std::fprintf(stderr, "未知参数: %s\n", argv[i]);
    PrintUsage(argv[0]);
    return 2;
  }

  if (have_height == have_sequence) {
    std::fprintf(stderr, "必须指定 --height 或 --sequence 之一（不可同时使用）\n");
    PrintUsage(argv[0]);
    return 2;
  }
  if (duration_sec <= 0.0) {
    std::fprintf(stderr, "--duration 必须 > 0\n");
    return 2;
  }

  std::vector<HeightStep> steps;
  if (have_sequence) {
    if (!ParseSequence(sequence_text, steps)) {
      return 2;
    }
  } else {
    steps.push_back(HeightStep{height_m, duration_sec});
  }

  dds::domain::DomainParticipant participant(0);
  leju::dds_common::TopicPublisher<leju::msgs::Float64> pub(
      participant, leju::dds_topics::kPostureHeightCmd);

  for (const HeightStep& step : steps) {
    PublishHeightForDuration(pub, step.height_m, step.duration_sec, rate_hz, standing_height);
  }

  std::printf("[posture_height_cmd_node] done\n");
  return 0;
}
