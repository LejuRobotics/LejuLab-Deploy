#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "leju-rl-controller/controllers/controller_base.h"
#include "leju-rl-controller/robot_data.h"

namespace leju {

/**
 * @brief 控制器条目
 */
struct ControllerEntry {
  std::string name;                            ///< 控制器名称
  std::unique_ptr<ControllerBase> controller;  ///< 控制器实例
};

/**
 * @brief 控制器管理器
 *
 * 负责管理多个控制器的注册、切换和调度
 */
class ControllerManager {
 public:
  ControllerManager() = default;
  ~ControllerManager();

  /**
   * @brief 初始化控制器管理器
   * @param config_file 控制器列表配置文件路径
   * @return 是否初始化成功
   */
  bool initialize(const std::string& config_file);

  /**
   * @brief 阻塞等待外部触发启动信号
   *
   * 内部阻塞在 condition_variable 上，直到其他线程调用 triggerStart()。
   * 在等待期间会完成数据就绪检查、硬件就绪检查、移动到默认位置等前置工作。
   */
  void starting();

  /**
   * @brief 触发启动信号（线程安全）
   *
   * 其他线程（手柄线程、ROS service 等）调用此接口来触发 starting() 返回。
   * ControllerManager 不关心是谁触发。
   */
  void triggerStart();

  /**
   * @brief 停止控制器管理器
   */
  void stop();

  /**
   * @brief 执行一次控制更新
   *
   * 内部完成: 读取传感器 -> 控制器 update -> 发布指令
   */
  void update();

  /**
   * @brief 休眠到下一个控制周期
   *
   * 委托给当前活跃控制器的 waitNextCycle()，实现不同控制器的变频控制。
   * @param cycle_start 本次循环开始时间点
   */
  void waitNextCycle(std::chrono::steady_clock::time_point cycle_start);

  /**
   * @brief 是否正在运行
   */
  bool isRunning() const;

  /**
   * @brief 分发手柄输入
   *
   * 系统级按键（start/back）由 manager 自己处理，
   * 其余透传给当前活跃控制器的 onJoyInput()。
   *
   * @param joy 当前帧手柄数据
   * @param prev_buttons 上一帧按钮状态
   */
  void dispatchJoyInput(const JoyData& joy, const JoyData::Buttons& prev_buttons);

  /////////////////////////////// Controller Interface ///////////////////////////////////

    /**
   * @brief 添加控制器
   * @param name 控制器名称
   * @param controller 控制器实例（所有权转移）
   * @return 是否添加成功
   */
  bool addController(const std::string& name, std::unique_ptr<ControllerBase> controller);

  /**
   * @brief 切换到指定控制器
   * @param name 目标控制器名称
   * @return 是否切换成功
   */
  bool switchController(const std::string& name);

  /**
   * @brief 检查控制器是否存在
   * @param name 控制器名称
   * @return 是否存在
   */
  bool hasController(const std::string& name) const;

  /**
   * @brief 根据名称获取控制器
   * @param name 控制器名称
   * @return 控制器指针，不存在返回 nullptr
   */
  ControllerBase* getControllerByName(const std::string& name);

  /**
   * @brief 获取当前控制器
   */
  ControllerBase* getCurrentController() const;

  /**
   * @brief 获取上一个控制器
   */
  ControllerBase* getLastController() const;

  /**
   * @brief 获取控制器数量
   */
  size_t getControllerCount() const;
  
  /**
   * @brief 获取当前控制器名称
   */
  std::string getCurrentControllerName() const;

  /**
   * @brief 获取所有控制器名称列表
   */
  std::vector<std::string> getControllerNames() const;

 private:
  /**
   * @brief 从配置文件加载控制器列表
   * @param config_file 控制器列表配置文件路径
   * @return 是否加载成功
   */
  bool loadControllersFromConfig(const std::string& config_file);

  /**
   * @brief 等待所有传感器数据就绪
   * @return 是否等待成功（未被中断）
   */
  bool waitForDataReady();

  /**
  /**
   * @brief 将机器人移动到默认关节位置
   * @param elapse 过渡时间（秒）
   */
  void moveToDefaultPos(double elapse = 3.0);

private:  
  mutable std::recursive_mutex controllers_mutex_;              ///< 控制器列表锁
  std::vector<ControllerEntry> controllers_;                    ///< 控制器列表（保持添加顺序）
  int active_index_ = -1;                                       ///< 当前控制器索引（-1 表示无）
  int last_index_ = -1;                                         ///< 上一个控制器索引（-1 表示无）
  std::string config_dir_;                                      ///< 配置文件根目录
  std::atomic<bool> running_{false};                            ///< 运行状态标志
  RobotData robot_data_;                                        ///< 机器人数据（传感器订阅）

  // 启动信号
  std::mutex start_mutex_;                                      ///< 启动信号锁
  std::condition_variable start_cv_;                            ///< 启动信号条件变量
  std::atomic<bool> start_triggered_{false};                    ///< 启动信号标志
  std::atomic<bool> ready_for_start_{false};                    ///< 数据就绪，可接受启动信号
};

}  // namespace leju
