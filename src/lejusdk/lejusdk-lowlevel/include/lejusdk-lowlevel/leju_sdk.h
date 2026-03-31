#ifndef _LEJU_SDK_DEF_H_
#define _LEJU_SDK_DEF_H_

#include "lejusdk-lowlevel/robot_api/robot_base.h"
#include "lejusdk-utils/robot_version.hpp"
#include "lejusdk-lowlevel/robot_api/roban_humanoid.h"
#include "lejusdk-lowlevel/robot_api/kuavo_humanoid.h"
#include <memory>
#include <string>
#include <mutex>

namespace leju {

/// @name SDK 版本定义
/// @{
#define LEJU_SDK_VERSION_MAJOR 1   ///< 主版本号
#define LEJU_SDK_VERSION_MINOR 0   ///< 次版本号
#define LEJU_SDK_VERSION_PATCH 0   ///< 补丁版本号
#define LEJU_SDK_VERSION_STRING "1.0.0"  ///< 版本字符串
/// 版本号: (major << 16) | (minor << 8) | patch
#define LEJU_SDK_VERSION_NUMBER ((LEJU_SDK_VERSION_MAJOR << 16) | \
                                 (LEJU_SDK_VERSION_MINOR << 8) | \
                                 LEJU_SDK_VERSION_PATCH)
/// @}

/**
 * @class GlobalRobot
 * @brief 全局机器人单例管理器
 *
 * 提供统一的机器人创建和管理接口，确保全局只有一个机器人实例。
 *
 * @code{.cpp}
 * // 从环境变量初始化
 * GlobalRobot::init_env(RobotVersion::from_env());
 * auto& robot = GlobalRobot::getInstance();
 * robot.publishRobotCmd(cmd);
 *
 * // 使用预定义版本常量
 * GlobalRobot::init_env(RobotVersions::KUAVO5_BASE);
 * @endcode
 */
class GlobalRobot {
public:
    /// @brief 初始化机器人运行环境（只能调用一次）
    /// @param version 机器人版本信息，参见 @ref RobotVersion
    /// @return 初始化是否成功
    /// @throws std::runtime_error 重复初始化时抛出
    static bool init_env(const RobotVersion& version);

    /**
     * @brief 获取已初始化的机器人实例
     *
     * 必须在调用 init_env() 之后使用，否则将抛出异常。
     *
     * @code{.cpp}
     * // 从环境变量中获取机器人版本信息
     * auto robot_version = RobotVersion::from_env();
     * if (GlobalRobot::init_env(robot_version)) {
     *     auto& robot = GlobalRobot::getInstance();
     *     robot.publishRobotCmd(cmd);
     * }
     *
     * // 使用预定义的版本常量 - Kuavo5
     * if (GlobalRobot::init_env(RobotVersions::KUAVO5_BASE)) {
     *     auto& robot = GlobalRobot::getInstance();
     * }
     * @endcode
     *
     * @return 机器人实例引用
     * @throws std::runtime_error 未初始化时抛出
     */
    static RobotBaseAPI& getInstance();

    /// @brief 检查机器人环境是否已初始化
    /// @return 已初始化返回 true
    static bool is_initialized();

/** @cond INTERNAL */
private:
    static std::unique_ptr<RobotBaseAPI> createRobot(const RobotVersion& version);
    static std::unique_ptr<RobotBaseAPI> instance_;
    static std::mutex mutex_;
    GlobalRobot() = delete;
    GlobalRobot(const GlobalRobot&) = delete;
    GlobalRobot& operator=(const GlobalRobot&) = delete;
/** @endcond */
};

} // namespace leju

#endif  // _LEJU_SDK_DEF_H_