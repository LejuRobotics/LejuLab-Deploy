#pragma once
#include <string>

enum class CanbusWiringType {
    SINGLE_BUS,  // 单总线
    DUAL_BUS,    // 双总线 左右手臂各接一个CAN模块
    UNKNOWN      // 未知
};

namespace leju{
namespace hw {     
namespace config_utils  {

std::string GetEcmasterType();

std::string GetIMUType();

CanbusWiringType GetCanbusWiringType();

} // namepsace config_utils
} // namespace hw
} // namespace leju
