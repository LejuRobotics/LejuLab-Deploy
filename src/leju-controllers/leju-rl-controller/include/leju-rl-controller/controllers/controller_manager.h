#pragma once

#include <atomic>
#include <chrono>
#include <functional>
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

/// tact fallback 回调类型：CSV motion 失败时降级到 .tact 播放
using TactFallback = std::function<bool(const std::string&)>;

/// 手臂冻结回调类型：LB+B 触发 keep_pose 时通知 TactPlayer 冻结
using ArmFreezeCallback = std::function<void()>;

/**
 * @brief 根据 IMU 判断是否倒地姿态（对齐 kuavo AmpWalk）
 * @param imu 姿态四元数 [w,x,y,z]
 * @param threshold_deg |roll| 或 |pitch| 超过该角度视为倒地
 */
bool IsFallenPose(const ImuData& imu, double threshold_deg = 60.0);

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
  bool pending_auto_start_motion = false;       ///< 切换完成后是否自动起播默认动作
  int stable_ticks = 0;                         ///< 稳定计数器（gyro 低于阈值连续帧数）
};

/**
 * @brief 控制器条目
 */
struct ControllerEntry {
  std::string name;                            ///< 控制器名称
  std::string config_path;                     ///< 控制器配置文件路径
  std::unique_ptr<ControllerBase> controller;  ///< 控制器实例
  std::string music;                           ///< 配套配乐文件名（可选，来自 controller_manager.yaml）
  double music_delay = 0.0;                    ///< 配乐起播延迟[s]（可选）
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
   * @brief 异步信号安全的停止请求：仅原子置位，不打日志/不加锁，
   *        可安全地在信号处理函数中调用，使 waitForDataReady() 等阻塞循环退出。
   */
  void requestStop() noexcept {
    running_.store(false, std::memory_order_relaxed);
    default_pose_stop_requested_.store(true, std::memory_order_relaxed);
  }

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
  bool addController(const std::string& name, std::unique_ptr<ControllerBase> controller,
                     const std::string& config_path = "");

  /**
   * @brief 获取控制器配置文件路径
   */
  std::string getControllerConfigPath(const std::string& name) const;

  /**
   * @brief 请求切换到指定控制器（两阶段切换）
   *
   * 启动控制器切换过渡流程，过渡期间 Update() 会插值到目标控制器的默认姿态。
   * 过渡完成后自动调用 commitSwitch() 完成切换。
   *
   * @param name 目标控制器名称
   * @param now 当前时间戳 [s]
   * @param auto_start_motion 切换完成后是否自动起播
   * @param instant_commit 为 true 时跳过混合立即 commit（用于倒地起身交接：
   *        切 mimic_fall_stand 进瘫软待机，或 START 时已倒地离开 AMP）
   * @return 是否成功启动切换请求
   */
  bool requestSwitch(const std::string& name, double now, bool auto_start_motion = false,
                     bool instant_commit = false);

  /**
   * @brief 检查是否正在切换中
   * @return true 表示正在进行过渡
   */
  bool isTransitioning() const;

  /// 控制器切换混合时长 [s]（来自 controller_manager.yaml switch.duration）
  double switchDuration() const { return transition_.duration; }

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
   * @brief 覆盖默认/当前控制器（须在 initialize 之后调用）
   *
   * 用于 CLI 等场景覆盖 yaml 中的 default_controller，不改配置文件。
   * @param name 目标控制器名称
   * @return 是否设置成功（控制器不存在时返回 false）
   */
  bool setDefaultController(const std::string& name);

  /**
   * @brief 传感器就绪后按 IMU 姿态选择启动默认控制器
   *
   * 对齐 kuavo：|roll| 或 |pitch| 超过 threshold_deg（IsFallenPose 默认值）视为倒地。
   * - 倒地且存在 mimic_fall_stand → 切到 mimic_fall_stand
   * - 否则若存在 amp → 切到 amp
   * 若已 Start() 过，会 pause 旧控制器并 resume 新控制器。
   * @return 是否完成选择（无 IMU / 无候选控制器时返回 false）
   */
  bool applyStartupPosePolicy();

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
   * @brief 获取当前激活控制器配置的默认配乐（如果有）
   *
   * 用于 MotionCommand 触发器未显式指定 music 时的回退：不同舞蹈控制器可在
   * controller_manager.yaml 里各自配置 music/music_delay，共享同一个"启动"按键
   * 也能按当前激活的控制器播放各自的配乐。
   *
   * @param music[out] 配乐文件名
   * @param music_delay[out] 起播延迟[s]
   * @return 当前控制器是否配置了 music（未配置时 false，out 参数不修改）
   */
  bool getCurrentControllerMusic(std::string& music, double& music_delay) const;

  /**
   * @brief 根据名称获取指定控制器的默认配乐
   *
   * 用于 auto_start_motion 场景：切换前根据目标控制器名称查找其 music/delay。
   *
   * @param name 控制器名称
   * @param music[out] 配乐文件名
   * @param music_delay[out] 起播延迟[s]
   * @return 是否找到该控制器并配置了 music
   */
  bool getControllerMusic(const std::string& name, std::string& music, double& music_delay) const;

  /**
   * @brief 开始播放 motion（舞蹈/动作）
   * @param name motion 名称（空字符串表示播放当前/默认 motion）
   * @return 是否成功启动
   *
   * 委托给当前活跃控制器的 startMotion() 方法。
   * 只有支持 motion 播放的控制器（如 GenericRLController）会实际执行。
   */
  bool startMotion(const std::string& name = "");

  /// @brief 两阶段起身：插值到当前控制器 CSV 首帧并保持
  /// @param max_joint_velocity 最大关节速度 [rad/s]；<=0 时由控制器配置决定
  bool prepareToMotionStart(double max_joint_velocity = -1.0);
  /// @brief START 后待机 hold（锁当前姿态、不跑策略）
  bool enterStandbyHold();
  bool isPreparingMotionStart() const;
  bool isHoldingMotionStart() const;
  bool isStandbyHolding() const;

  /**
   * @brief 注册 tact fallback 回调
   *
   * CSV motion 播放失败后，ControllerManager 将尝试通过此回调
   * 降级到 tact 播放（由 ExternalInterface 提供 TactPlayer）。
   *
   * @param callback 回调函数（参数为动作名，返回 true 表示播放成功）
   */
  void setTactFallback(TactFallback callback) { tact_fallback_ = std::move(callback); }

  /**
   * @brief 设置手臂冻结回调
   *
   * 当手臂模式被切换为 keep_pose（LB+B 锁定）时触发。
   * ExternalInterface 用此回调通知 TactPlayer 冻结。
   *
   * @param callback 回调函数（无参数）
   */
  void setArmFreezeCallback(ArmFreezeCallback callback) { arm_freeze_callback_ = std::move(callback); }

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

  /// @brief Running 入口：零速度并清滤波状态
  void clearVelocityOnRunningEntry();

  /**
   * @brief 设置当前控制器的 cmd_stance
   * @param mode 0=行走, 1=站立/下蹲/弯腰
   */
  bool setCmdStanceMode(int mode, std::string& message);

  /// @brief 当前控制器是否处于深蹲守备（阻止退出 posture）
  bool isDeepSquatGuardActive() const;

  /// @brief 当前控制器的 cmd_stance（0=行走/站立，1=下蹲；无控制器时返回 0）
  int getCurrentCmdStanceMode() const;

  /// @brief 是否仍有残留平滑下蹲高度命令
  bool hasResidualStanceHeightCommand() const;

  /// @brief 是否满足大角速度转身退出（|smoothed cmd_z| >= 阈值）
  bool canTurnExitDeepSquatGuard(double pending_angular_z) const;

  /// @brief 行走模式下用于判定的 smoothed cmd_z（policy command unit）
  double getSmoothedWalkingCmdZ(double pending_angular_z) const;

  /// @brief 转身退出守备的最小 |smoothed cmd_z| 阈值
  double getTurnExitGuardThreshold() const;

  /**
   * @brief 设置手臂控制模式
   * @param mode 目标模式（内部枚举，非 vr::ControlMode）
   * @param message 返回消息
   * @return 是否成功
   */
  bool setArmMode(ArmControlMode mode, std::string& message);

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

  /**
   * @brief 热重载控制器配置（仅添加新控制器）
   *
   * 重新读取 controller_manager.yaml，对新增的控制器条目调用 addController()。
   * 已存在的控制器（按 name 匹配）跳过，不重启、不影响正在运行的控制器。
   *
   * @return 是否重载成功
   */
  bool reloadControllersFromConfig();

 private:
  /**
   * @brief 从配置文件加载控制器列表
   * @param config_file 控制器列表配置文件路径
   * @param urdf_path URDF 文件路径（用于手臂重力补偿，空则跳过）
   * @return 是否加载成功
   */
  bool loadControllersFromConfig(const std::string& config_file, const std::string& urdf_path);

  /**
   * @brief 设置指定控制器的配套配乐（供 loadControllersFromConfig 解析 music 字段后调用）
   * @param name 控制器名称
   * @param music 配乐文件名
   * @param music_delay 起播延迟[s]
   */
  void setControllerMusic(const std::string& name, const std::string& music, double music_delay);

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
  std::string config_file_;                                     ///< 控制器列表配置文件路径 (初始化后缓存, 用于热重载)
  std::string urdf_path_;                                       ///< URDF 路径 (初始化后缓存, 用于热重载)
  std::atomic<bool> running_{false};                            ///< 运行状态标志
  std::atomic<bool> default_pose_stop_requested_{false};        ///< 默认姿态过渡取消标志
  RobotData robot_data_;                                        ///< 机器人数据（传感器订阅）

  // 头部指令缓存（由 ControlLogic 通过 setHeadTarget 设置）
  mutable std::mutex head_cmd_mutex_;
  vr::JointTrajectoryPoint head_cmd_;
  bool head_cmd_received_{false};
  /// 记录最后一次收到 VR 头部指令的时间(用在超时失效)
  std::chrono::steady_clock::time_point head_cmd_rx_time_{};
  int head_state{0};    ///< 头部状态,0->手柄态, 1->中间态， 2->quest态
  const double kHeadCmdTimeoutSec = 0.5;
  /// 头部模式切换五次多项式插值时长 [s]
  const double kHeadCmdInterpDurationSec = 1.0;
  /// 中间态插值完成后的目标状态（0=手柄 / 2=quest）
  int head_state_target_{0};
  /// 中间态插值起点（=进入中间态时实际下发角度）
  float head_interp_start_yaw_{0.0f};
  float head_interp_start_pitch_{0.0f};
  /// 中间态插值终点（=目标源在切换瞬间的角度）
  float head_interp_target_yaw_{0.0f};
  float head_interp_target_pitch_{0.0f};
  /// 中间态插值开始时间（steaady_clock）
  std::chrono::steady_clock::time_point head_interp_start_time_{};
  /// 当前实际下发的头部角度（每帧更新，作模式切换插值起点）
  float head_out_yaw_{0.0f};
  float head_out_pitch_{0.0f};
  // 手柄缓存：RT + 右摇杆控制头部
  mutable std::mutex joy_mutex_;
  float head_joy_yaw_ = 0.0f;       ///< 右摇杆 X → head_yaw 目标位置 (rad)
  float head_joy_pitch_ = 0.0f;     ///< 右摇杆 Y → head_pitch 目标位置 (rad)
  bool head_joy_active_ = false;    ///< RT 按下时启用头部手柄控制
  bool head_return_active_ = false; ///< RT 松开后头部平滑回正
  std::chrono::steady_clock::time_point last_head_joy_update_{};
  bool head_joy_time_initialized_ = false;

  // 控制器切换过渡状态（包含插值配置，从 YAML 直接加载）
  SwitchTransition transition_;  ///< 切换过渡数据结构

  TactFallback tact_fallback_;    ///< tact 降级回调（CSV motion 失败时尝试 .tact 播放）
  ArmFreezeCallback arm_freeze_callback_;  ///< 手臂冻结回调（LB+B keep_pose 时通知 TactPlayer 冻结）
};

}  // namespace leju
