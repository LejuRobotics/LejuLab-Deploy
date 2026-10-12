#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "motor_preflight/param_protocol.h"

namespace motor_preflight {

struct ParamSpec {
    uint8_t idx = 0;
    ParamValue value;
};

struct ModelSpec {
    std::vector<uint32_t> firmware_versions;  // 允许的固件版本白名单, 命中任一即通过
    std::vector<ParamSpec> params;            // 按配置顺序写入
};

// motor_firmware.json:
// {
//   "models": {
//     "<型号>": {
//       "firmware_version": "0x02030001",          // 或数字; 也可写数组表示
//                                                  // 白名单: ["0x02030001", "0x0203000A"]
//       "params": [                                 // 可省略
//         { "idx": 61, "dtype": "uint32", "value": 100 },
//         { "idx": 30, "dtype": "float",  "value": 1.5 },
//         { "idx": 56, "dtype": "hex32",  "value": "0xFF" }
//       ]
//     }
//   },
//   "device_map": { "<canbus yaml 设备名>": "<型号>" }
// }
struct PreflightConfig {
    std::map<std::string, ModelSpec> models;
    std::map<std::string, std::string> device_map;
};

// 解析并校验配置; 文件不存在/格式错误/规则违反时抛 std::runtime_error(含具体原因)
// 校验规则: firmware_version 必填; params.idx ∈ [0,255] 且禁止 10(固件版本只读);
// dtype ∈ float/uint32/hex32; device_map 引用的型号必须在 models 中定义
// 配置随代码走, 位于 config/<robot_type>/motor_firmware.json (与 kuavo.json 同目录)
PreflightConfig LoadPreflightConfig(const std::string& json_path);

}  // namespace motor_preflight
