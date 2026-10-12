// play_audio_cli — 命令行调试工具：向 audio_player_node 发 RPC 播放请求。
//
// 用法: play_audio_cli <file_name> [volume]
//   file_name : 裸文件名(相对节点的 AUDIO_FILE_DIR) 或绝对路径
//   volume    : 音量百分比，默认 100
#include <dds_rpc/requester.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

#include "rpc_idl/AudioPlayerService.hpp"

using leju::srv::PlayAudioReply;
using leju::srv::PlayAudioRequest;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "用法: " << argv[0] << " <file_name> [volume]" << std::endl;
    std::cerr << "  file_name : 裸文件名(相对 AUDIO_FILE_DIR) 或绝对路径" << std::endl;
    std::cerr << "  volume    : 音量百分比，默认 100" << std::endl;
    return 2;
  }
  std::string file_name = argv[1];
  int volume = argc >= 3 ? std::atoi(argv[2]) : 100;

  try {
    dds_entity_t participant = dds_create_participant(0, nullptr, nullptr);
    if (participant < 0) {
      std::cerr << "Failed to create participant" << std::endl;
      return 1;
    }

    dds_rpc::Requester<PlayAudioRequest, PlayAudioReply> requester(participant,
                                                                   "AudioPlayerService");

    std::cout << "Waiting for AudioPlayerService..." << std::endl;
    if (!requester.wait_for_service(std::chrono::seconds(5))) {
      std::cerr << "Service not available (audio_player_node 没在跑?)" << std::endl;
      dds_delete(participant);
      return 1;
    }

    PlayAudioRequest req{};
    req.file_name(file_name);
    req.volume(volume);

    std::cout << "play_file: '" << file_name << "' volume=" << volume << std::endl;
    try {
      auto reply = requester.send_request(req, std::chrono::seconds(30));
      std::cout << (reply.success() ? "[OK] " : "[FAIL] ") << reply.message() << std::endl;
      dds_delete(participant);
      return reply.success() ? 0 : 1;
    } catch (const dds_rpc::TimeoutException&) {
      std::cerr << "TIMEOUT" << std::endl;
      dds_delete(participant);
      return 1;
    } catch (const dds_rpc::RpcException& e) {
      std::cerr << "ERROR: " << e.what() << std::endl;
      dds_delete(participant);
      return 1;
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
