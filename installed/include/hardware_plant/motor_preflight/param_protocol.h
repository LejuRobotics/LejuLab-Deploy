#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace motor_preflight {

// 电机参数读写协议(与 scripts/can_param_tools/can_param_tool.py 逐字节一致):
//   读帧:   67 <idx> 00 00 00 00 04 76
//   写帧:   67 <idx> <payload 4B LE> 15 76
//   响应帧: <motor_id> <idx> <payload 4B LE> <tail 2B>
//   CAN ID = 0x600 + motor_id, 请求与响应使用同一 ID
constexpr uint32_t kParamCanIdBase = 0x600;

// 固件版本参数索引 (只读, hex32)
constexpr uint8_t kFirmwareVersionIdx = 10;

// float 写入回读校验容差: float32 往返精度损失上限约 1e-7, 取 1e-5
// 足够区分真实写入失败与浮点精度损失 (与 Python 工具一致)
constexpr float kFloatReadbackTol = 1e-5f;

enum class ParamDtype { kFloat, kUint32 };

struct ParamValue {
    ParamDtype dtype = ParamDtype::kUint32;
    float f32 = 0.0f;
    uint32_t u32 = 0;
};

std::array<uint8_t, 8> BuildReadFrame(uint8_t idx);
std::array<uint8_t, 8> BuildWriteFrame(uint8_t idx, const ParamValue& value);

// 响应帧解码: 要求 len == 8、data[0] != 0x67(排除自环请求帧)、data[1] == expect_idx,
// 按 out->dtype 从 data[2..5] 小端解码; 不满足返回 false
bool DecodeResponse(const uint8_t* data, size_t len, uint8_t expect_idx, ParamValue* out);

// 期望值与回读值比较: float 用 kFloatReadbackTol 容差, uint32 按位相等
bool ValueMatches(const ParamValue& expected, const ParamValue& actual);

}  // namespace motor_preflight
