#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <optional>

#include <yaml-cpp/yaml.h>

#include "leju-rl-controller/controllers/controller_base.h"
#include "leju-rl-controller/rl/rl_controller_types.h"
#include "leju-rl-controller/runtime/data_types.hpp"
#include "leju-rl-controller/runtime/input/command_buffer.h"
#include "leju-rl-controller/robot_data.h"

namespace leju {

/**
 * @brief 控制器切换状态
 */
enum class SwitchState {
  kIdle,           ///< 正常运行
  kTransitioning,  ///< 正在进行控制器切换过渡
};

/**
 * @brief 控制器切换过渡数据结构
 */
struct SwitchTransition {
  SwitchState state = SwitchState::kIdle;  ///< 当前切换状态

  std::string from_controller;  ///< 源控制器名称
  std::string to_controller;    ///< 目标控制器名称

  int source_index = -1;        ///< 源控制器索引
  int target_index = -1;        ///< 目标控制器索引

  double start_time = 0.0;   ///< 过渡开始时间 [s]
  double duration = 2.0;     ///< 过渡持续时间 [s]

  bool rl_to_rl_dual_inference_active = false;  ///< 是否启用 RL->RL 双推理混合
  bool target_prestarted = false;               ///< 目标控制器是否已提前启动
};

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
   * @param urdf_path URDF 文件路径（用于手臂重力补偿，空则跳过）
   * @return 是否初始化成功
   */
  bool initialize(const std::string& config_file, const std::string& urdf_path = "");

  /**
   * @brief 启动控制器管理器（非阻塞）
   *
   * 设置 running_ 标志为 true，使 Update() 可以正常执行。
   * 适用于 ControlLoop 架构，由 Lifecycle 管理启动时机。
   */
  void Start();

  /**
   * @brief 等待传感器数据就绪
   * @return 是否成功等待（false 表示被停止）
   */
  bool waitForDataReady();

  /**
   * @brief 停止控制器管理器
   */
  void stop();

  /**
   * @brief 设计文档要求的统一更新接口
   *
   * ControlLoop 通过此接口调用，无需知道 active_controller 细节。
   * 封装在内部：active_controller 完全由 ControllerManager 管理。
   *
   * @param state 当前机器人状态
   * @param imu IMU数据
   * @param command 命令快照（已合并的最终命令）
   * @return 控制指令
   *
   * 调用链：ControlLoop -> ControllerManager.update() -> active_controller->update()
   */
  RobotCmd update(const RobotState& state,
                  const ImuData& imu_state,
                  const runtime::CommandBuffer::Snapshot& command);

  /**
   * @brief 处理遥操作命令快照
   *
   * 注意：所有命令处理现在由 ControlLogic 完成
   * 此方法保留供未来扩展使用
   *
   * @param snapshot 命令快照
   */
  void processCommandBuffer(const runtime::CommandBuffer::Snapshot& snapshot);

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

  /////////////////////////////// Controller Interface ///////////////////////////////////

    /**
   * @brief 添加控制器
   * @param name 控制器名称
   * @param controller 控制器实例（所有权转移）
   * @return 是否添加成功
   */
  bool addController(const std::string& name, std::unique_ptr<ControllerBase> controller);

  /**
   * @brief 请求切换到指定控制器（两阶段切换）
   *
   * 启动控制器切换过渡流程，过渡期间 Update() 会插值到目标控制器的默认姿态。
   * 过渡完成后自动调用 commitSwitch() 完成切换。
   *
   * @param name 目标控制器名称
   * @param now 当前时间戳 [s]
   * @return 是否成功启动切换请求
   */
  bool requestSwitch(const std::string& name, double now);

  /**
   * @brief 检查是否正在切换中
   * @return true 表示正在进行过渡
   */
  bool isTransitioning() const;

 private:
  /**
   * @brief 提交控制器切换（过渡完成后调用）
   *
   * 完成实际的控制器切换：
   * - 调用旧控制器 OnExit()
   * - 切换到新控制器
   * - 调用新控制器 OnEnter()
   */
  void commitSwitch();

 public:
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

  /**
   * @brief 开始播放 motion（舞蹈/动作）
   * @param name motion 名称（空字符串表示播放当前/默认 motion）
   * @return 是否成功启动
   *
   * 委托给当前活跃控制器的 startMotion() 方法。
   * 只有支持 motion 播放的控制器（如 GenericRLController）会实际执行。
   */
  bool startMotion(const std::string& name = "");

  // ==================== 外部控制接口（由 ControlLogic 调用） ====================

  /**
   * @brief 设置手臂关节目标
   * @param cmd 关节轨迹点
   */
  void setArmTarget(const vr::JointTrajectoryPoint& cmd);

  /**
   * @brief 设置腰部关节目标
   * @param cmd 关节轨迹点
   */
  void setWaistTarget(const vr::JointTrajectoryPoint& cmd);

  /**
   * @brief 设置头部关节目标
   * @param cmd 关节轨迹点
   */
  void setHeadTarget(const vr::JointTrajectoryPoint& cmd);

  /**
   * @brief 设置速度指令
   * @param cmd 速度指令（内部类型）
   */
  void setVelocityCommand(const VelocityCommand& cmd);

  /**
   * @brief 设置手臂控制模式
   * @param mode 目标模式（内部枚举，非 vr::ControlMode）
   * @param message 返回消息
   * @return 是否成功
   */
  bool setArmMode(ArmControlMode mode, std::string& message);

  /**
   * @brief 切换当前控制器的 cmd_stance（0 ↔ 1），由 B 键触发
   */
  void toggleCmdStance();

  /**
   * @brief 设置腰部控制模式
   * @param mode 目标模式（内部枚举，非 vr::ControlMode）
   * @param message 返回消息
   * @return 是否成功
   */
  bool setWaistMode(WaistControlMode mode, std::string& message);

  /**
   * @brief 获取当前手臂控制模式（内部枚举）
   * @return 手臂控制模式，无控制器时返回 std::nullopt
   */
  std::optional<ArmControlMode> getCurrentArmMode() const;

  /**
   * @brief 获取当前腰部控制模式（内部枚举）
   * @return 腰部控制模式，无控制器时返回 std::nullopt
   */
  std::optional<WaistControlMode> getCurrentWaistMode() const;

  /**
   * @brief 获取底层 RobotData （包含传感器数据和状态信息）
   * @return RobotData 引用
   */
  const RobotData& getRobotData() const { return robot_data_; }

  /**
   * @brief 查询当前控制器是否正在播放 motion
   * @return true 正在播放，false 未播放或当前控制器不支持 motion
   */
  bool isCurrentMotionPlaying() const;

  /**
   * @brief 获取当前正在播放的 motion 名称
   * @return motion 名称；当前控制器不支持时返回空字符串
   */
  std::string getCurrentMotionName() const;

  /**
   * @brief 获取当前控制器可用的 motion 名称列表
   * @return motion 名称列表；当前控制器不支持时返回空列表
   */
  std::vector<std::string> getAvailableMotionNames() const;

 private:
  /**
   * @brief 从配置文件加载控制器列表
   * @param config_file 控制器列表配置文件路径
   * @param urdf_path URDF 文件路径（用于手臂重力补偿，空则跳过）
   * @return 是否加载成功
   */
  bool loadControllersFromConfig(const std::string& config_file, const std::string& urdf_path);

  /**
   * @brief 加载切换插值配置（kp/kd）
   * @param config YAML 配置节点
   */
  void loadSwitchInterpolationConfig(const YAML::Node& config);

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

  // 头部指令缓存（由 ControlLogic 通过 setHeadTarget 设置）
  mutable std::mutex head_cmd_mutex_;
  vr::JointTrajectoryPoint head_cmd_;
  bool head_cmd_received_{false};

  // 控制器切换过渡状态（包含插值配置，从 YAML 直接加载）
  SwitchTransition transition_;  ///< 切换过渡数据结构
};

}  // namespace leju
