#pragma once

#include <string>

#include "motor_preflight/motor_preflight.h"

namespace motor_preflight {

// 参数帧的 raw SocketCAN 传输实现。
// 预检运行在任何控制线程启动之前, 总线空闲, 因此用最简单的
// 阻塞式请求-响应 (poll 超时) 即可, 不与 canbus_sdk 的收发线程共存。
class SocketCanParamTransport : public ParamTransport {
public:
    SocketCanParamTransport(int timeout_ms, int retries);
    ~SocketCanParamTransport() override;

    SocketCanParamTransport(const SocketCanParamTransport&) = delete;
    SocketCanParamTransport& operator=(const SocketCanParamTransport&) = delete;

    // use_fd 判定与 canbus_sdk 一致: dbitrate > 0 && dbitrate != nbitrate
    // CAN 接口由 init 脚本配置, 这里只打开 socket, 不执行 ip link
    bool Open(const std::string& ifname, bool use_fd);
    void Close();

    // drain 残留帧 → 发送 → poll 等待 0x600+motor_id 的响应帧; 超时重试 retries 次
    bool Transact(uint8_t motor_id, const std::array<uint8_t, 8>& req,
                  std::array<uint8_t, 8>* resp) override;
    bool SendOnly(uint8_t motor_id, const std::array<uint8_t, 8>& req) override;

private:
    bool SendFrame(uint8_t motor_id, const std::array<uint8_t, 8>& req);
    // 读空接收缓冲, 丢弃残留帧 (如写参数帧的 ack)
    void Drain();

    int fd_ = -1;
    bool use_fd_ = false;
    int timeout_ms_;
    int retries_;
};

}  // namespace motor_preflight
