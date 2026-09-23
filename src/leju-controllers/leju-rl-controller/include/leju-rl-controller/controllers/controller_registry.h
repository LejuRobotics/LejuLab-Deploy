#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "leju-rl-controller/controllers/controller_base.h"

namespace leju {

class RobotVersion;

/**
 * @brief 控制器创建函数类型
 *
 * 参数: RobotVersion, controller_name
 * 返回: 控制器基类智能指针
 */
using ControllerCreator = std::function<std::unique_ptr<ControllerBase>(
    const RobotVersion&, const std::string&)>;

/**
 * @brief 控制器注册表
 *
 * 单例模式，管理所有控制器类型的注册和创建
 *
 * 使用示例:
 *   // 注册（在控制器 cpp 文件末尾）
 *   REGISTER_CONTROLLER("GenericRLController", GenericRLController, generic_rl);
 *
 *   // 创建
 *   auto ctrl = ControllerRegistry::create("GenericRLController", version, "amp");
 */
class ControllerRegistry {
 public:
  /**
   * @brief 注册控制器类型
   * @param type 类型名称字符串 (如 "GenericRLController")
   * @param creator 创建函数
   * @return 是否注册成功（false 表示该类型已存在）
   */
  static bool registerType(const std::string& type, ControllerCreator creator);

  /**
   * @brief 创建控制器实例
   * @param type 控制器类型名称
   * @param version 机器人版本
   * @param name 控制器实例名称
   * @return 控制器智能指针，失败返回 nullptr
   */
  static std::unique_ptr<ControllerBase> create(
      const std::string& type,
      const RobotVersion& version,
      const std::string& name);

  /**
   * @brief 检查类型是否已注册
   * @param type 控制器类型名称
   * @return 是否已注册
   */
  static bool isRegistered(const std::string& type);

  /**
   * @brief 获取所有已注册的类型名称
   * @return 类型名称列表
   */
  static std::vector<std::string> getRegisteredTypes();

  /**
   * @brief 注销指定类型（主要用于测试）
   * @param type 控制器类型名称
   */
  static void unregisterType(const std::string& type);

  /**
   * @brief 清空所有注册（主要用于测试）
   */
  static void clear();

 private:
  // 私有构造函数，禁止实例化
  ControllerRegistry() = delete;
  ~ControllerRegistry() = delete;

  inline static std::unordered_map<std::string, ControllerCreator> creators_;
  inline static std::mutex mutex_;
};

/**
 * @brief 控制器注册辅助类
 *
 * 利用 RAII 在静态初始化时自动注册
 */
class ControllerRegistrar {
 public:
  ControllerRegistrar(const std::string& type, ControllerCreator creator) {
    ControllerRegistry::registerType(type, std::move(creator));
  }
};

/**
 * @brief 注册控制器宏
 *
 * 使用方法：在控制器实现文件（.cpp）末尾添加：
 *   REGISTER_CONTROLLER("TypeName", ClassName, unique_var_name);
 *
 * 示例：
 *   // 在 namespace leju {} 内部使用（推荐）
 *   REGISTER_CONTROLLER("GenericRLController", GenericRLController, generic_rl);
 *
 *   // 在全局命名空间使用（需要完整限定类名）
 *   REGISTER_CONTROLLER("GenericRLController", leju::GenericRLController, generic_rl);
 */
#define REGISTER_CONTROLLER(TYPE, CLASS, VAR_NAME)                        \
  static ::leju::ControllerRegistrar _registrar_##VAR_NAME(               \
      TYPE,                                                               \
      [](const ::leju::RobotVersion& version,                             \
         const std::string& name) -> std::unique_ptr<::leju::ControllerBase> { \
        return std::make_unique<CLASS>(version, name);                    \
      })

}  // namespace leju
