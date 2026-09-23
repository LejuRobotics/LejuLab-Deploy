#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "motor_preflight/preflight_config.h"

namespace motor_preflight {

// 单电机检查项 (来自 canbus yaml 的 motor 设备)
struct MotorCheckItem {
    std::string bus;          // CAN 接口名 (bcan0...)
    uint8_t motor_id = 0;     // 设备 ID
    std::string device_name;  // yaml 设备名 (Lleg_joint_01...)
};

struct MotorCheckResult {
    MotorCheckItem item;
    std::string model;                 // device_map 解析出的型号 (缺条目时为空)
    bool ok = false;                   // = errors 为空; warnings 不影响 ok
    bool fw_valid = false;             // 固件版本读到且与期望一致 (不一致仅告警)
    uint32_t fw_read = 0;              // 读到的固件版本 (fw 读取成功时有效)
    int params_total = 0;              // 实际尝试写入校验的参数数
    int params_ok = 0;
    std::vector<std::string> errors;    // 导致拒绝启动的错误
    std::vector<std::string> warnings;  // 仅告警不拦截 (固件版本不匹配)
};

// 参数帧收发抽象: 生产实现为 SocketCanParamTransport, 单测注入假实现
class ParamTransport {
public:
    virtual ~ParamTransport() = default;
    // 发送请求帧到 0x600+motor_id 并等待响应 (实现内部负责超时与重试), 失败返回 false
    virtual bool Transact(uint8_t motor_id, const std::array<uint8_t, 8>& req,
                          std::array<uint8_t, 8>* resp) = 0;
    // 只发不等响应 (写参数帧, 部分固件不回 ack)
    virtual bool SendOnly(uint8_t motor_id, const std::array<uint8_t, 8>& req) = 0;
};

// 对一条总线上的电机执行: 固件版本比对 → 逐参数写入+回读校验。
// 门控策略: 固件版本不匹配仅告警(warnings), 继续写参数不拦截启动;
// 参数回读不一致 / 通信无响应 / device_map 缺条目 才计入 errors 拦截启动。
// 检完所有电机才返回, 不因单个失败提前退出。
std::vector<MotorCheckResult> CheckMotorsOnBus(ParamTransport& transport,
                                               const std::vector<MotorCheckItem>& items,
                                               const PreflightConfig& config);

struct PreflightOptions {
    std::string canbus_yaml_path;    // 空 = canbus_sdk::ConfigParser::getDefaultConfigFilePath()
    std::string firmware_json_path;  // 必填: config/<robot_type>/motor_firmware.json (随代码走)
    int timeout_ms = 200;
    int retries = 2;                 // 每次请求的额外重试次数 (共 1+retries 次尝试)
};

// 顶层入口: 解析 canbus yaml + motor_firmware.json, 逐 SOCKETCAN 总线检查, 打印汇总报告。
// 返回 true = 全部通过 (或整机无 SOCKETCAN 电机, 预检跳过);
// 返回 false = 任何失败, 调用方应拒绝初始化硬件层。
bool RunMotorParamPreflight(const PreflightOptions& options = {});

}  // namespace motor_preflight
