// Copyright 2024 Leju Robotics. All rights reserved.
//
// tact 动作播放器（控制器内部，独立于 CSV motion）。
//
// 与 GenericRLController 的 CSV motion 系统完全解耦：
//   - CSV motion: 作为 RL 推理观测 (motion_command)，由 ControllerService/StartMotion 触发
//   - tact 播放:  External arm mode 下直推 cmd_buffer，由 TactPlayerService/PlayTact 触发
//
// 播放流程：
//   1. 切手臂(及腰部)为 External 模式（push SetArmMode trigger）
//   2. 解析 tact 文件 + 贝塞尔插值
//   3. 后台线程以固定频率写 cmd_buffer（手臂/腰部/手部目标）
//   4. 播完恢复 Auto 模式
//
// 播放中再次请求播放：拒绝（返回 false, message=busy）。

#ifndef LEJU_RL_CONTROLLER_MOTION_TACT_PLAYER_H_
#define LEJU_RL_CONTROLLER_MOTION_TACT_PLAYER_H_

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "leju-rl-controller/motion/bezier_interpolator.h"
#include "leju-rl-controller/runtime/input/command_buffer.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"

namespace leju {
namespace runtime {

// tact 播放状态（int32 上报，避免 IDL enum 坑）。
// 终态 finished/failed/stopped/frozen 保持到下次 Play 成功覆盖。
enum TactPlayState {
  kTactIdle = 0,      // 空闲，无播放
  kTactPlaying = 1,   // 播放中
  kTactFinished = 2,  // 正常播完
  kTactFailed = 3,    // Play 失败（文件不存在/解析/插值失败/controller 未启动）
  kTactStopped = 4,   // 被主动 Stop
  kTactFrozen = 5,    // 被 Freeze 锁臂
};

// tact 播放错误码（仅 failed 态有意义，其余为 0）。
// stopped/frozen 态 error_code=0（主动操作不算错误）。
enum TactErrorCode {
  kTactOk = 0,                 // 无错误
  kTactErrBusy = 1,            // 播放中拒绝新请求
  kTactErrNotInitialized = 2,  // TactPlayer 未初始化
  kTactErrControllerNotRunning = 3,  // controller 未启动（onPlayTactRequest 层设置）
  kTactErrInvalidName = 4,     // 名字为空/含路径分隔符/..
  kTactErrFileNotFound = 5,    // tact 文件不存在
  kTactErrParseFailed = 6,     // tact 解析失败
  kTactErrInterpolationFailed = 7,  // 贝塞尔插值失败
};

// 接入过渡参数：播放前从当前位姿平滑走到轨迹首采样点
struct TactApproachOptions {
  double velocity = 1.0;      // 峰值速度上限 (rad/s)，与 External 接入阶段一致
  double min_duration = 0.2;  // 过渡时长下限 (秒)
  double max_duration = 2.0;  // 过渡时长上限 (秒)
  double threshold = 1e-3;    // 最大关节距离低于此值 (rad) 时不生成过渡
};

// 生成从当前位姿到 first_sample 的最小急动度接入段（按 loop_dt 采样，
// 覆盖 [0, T)，不含 first_sample 本身）。
//
// 背景：tact 文件首帧均在 keyframe=0，parser 的 init 帧插入逻辑永不触发，
// 播放第一拍即输出动作 0 帧位姿；tact 速度前馈又绕过 External 限速器，
// 当前位姿离 0 帧较远时（如 LB+B 冻结后再播放）指令阶跃导致手臂快速甩动。
//
// @param first_sample  预计算轨迹的第一个采样点（弧度）
// @param init_arm_deg  当前手臂关节位置（度）；为空或维度不足则返回空
// @param init_waist_deg 当前腰部关节位置（度）；first_sample 含腰部而此项
//                       为空时，过渡期间腰部锁定在首采样点值
// @param loop_dt       采样周期（秒）
// @return 过渡采样序列；无需过渡时为空
std::vector<vr::tact_player::JointTrajectorySample> BuildTactApproachSamples(
    const vr::tact_player::JointTrajectorySample& first_sample,
    const std::vector<double>& init_arm_deg,
    const std::vector<double>& init_waist_deg,
    double loop_dt,
    const TactApproachOptions& options = {});

class TactPlayer {
 public:
  // @param cmd_buffer     写关节目标的缓冲（与 ExternalInterface 共用同一实例）
  // @param trigger_buffer 推 SetArmMode trigger 的缓冲
  // @param arm_dof        手臂关节数（自动从机器人配置传入）
  // @param waist_index    腰部关节在 tact servos 数组中的索引，-1 表示无腰部
  // @param hand_index     手部在 tact servos 数组中的起始索引，-1 表示无手部
  // @param action_dir     tact 文件所在目录
  // @param loop_dt        播放控制周期（秒），默认 0.001 = 1000 Hz，对齐 ControlLoop
  TactPlayer(CommandBuffer* cmd_buffer,
             TriggerBuffer* trigger_buffer,
             std::size_t arm_dof,
             int waist_index,
             int hand_index,
             const std::string& action_dir,
             double loop_dt = 0.001);
  ~TactPlayer();

  TactPlayer(const TactPlayer&) = delete;
  TactPlayer& operator=(const TactPlayer&) = delete;

  // 播放指定名字的 tact 动作（异步，立即返回；返回前已完成解析/插值等准备）。
  // @param name    动作名（不含 .tact 后缀），文件为 <action_dir>/<name>.tact
  // @param init_arm_pos  当前手臂关节位置（度），用于 t=0 平滑过渡；为空则用第一帧值
  // @param init_waist_pos 当前腰部关节位置（度），用于 t=0 平滑过渡；为空则用第一帧值
  // @param message 输出消息
  // @param target_execution_time_ns 绝对目标起播时刻（Unix epoch 纳秒）。时间守门员：
  //                                 群控多机对齐同一目标起跳；0=立即播（向后兼容）。
  // @return true 已开始播放；false 文件不存在/解析失败/正在播放中
  bool Play(const std::string& name,
            const std::vector<double>& init_arm_pos,
            const std::vector<double>& init_waist_pos,
            std::string* message,
            int64_t target_execution_time_ns = 0);

  // 停止当前播放（恢复 Auto 模式）。
  void Stop();

  // 冻结：停止播放且不恢复 Auto 模式（LB+B 锁定手臂时调用）。
  // 与 Stop() 的区别：播放线程退出后不会推 SetArmMode(kAuto)。
  void Freeze();

  bool IsPlaying() const { return playing_.load(); }

  // 状态模型 getter（线程安全，供阶段 C 的 1Hz Loop 跨线程读取）。
  // state_/error_code_ 为 atomic；last_name_/error_message_ 由 state_mutex_ 保护。
  int getState() const;
  int getErrorCode() const;
  std::string getLastName() const;
  std::string getErrorMessage() const;

  // 标记外部失败（供 ExternalInterface 在 Play 之前的外部失败场景调用，
  // 如 controller_not_running）：设 state=failed + error_code + error_message + name。
  void markFailed(int error_code, const std::string& message, const std::string& name);

 private:
  void PlaybackLoop(int64_t target_execution_time_ns);
  void SetArmMode(const std::string& mode_name);
  void SetWaistMode(const std::string& mode_name);

  CommandBuffer* cmd_buffer_;
  TriggerBuffer* trigger_buffer_;
  std::size_t arm_dof_;
  int waist_index_;
  int hand_index_;
  std::string action_dir_;
  double loop_dt_;  // 播放控制周期（秒），默认 0.001 = 1000 Hz

  // 预计算轨迹：在 Play() 中以 loop_dt_ 步长采样整条贝塞尔曲线，
  // PlaybackLoop 直接读取，消除 100Hz→1000Hz 的台阶效应
  std::vector<vr::tact_player::JointTrajectorySample> precomputed_samples_;
  bool precomputed_has_waist_ = false;

  std::atomic<bool> playing_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> frozen_{false};  // Freeze() 置位，PlaybackLoop 检测后跳过 Auto 恢复
  std::thread worker_;
  std::mutex play_mutex_;

  // 状态模型：state_/error_code_ 为 atomic；last_name_/error_message_ 为 string，
  // 由 state_mutex_ 保护（ExternalInterface 1Hz Loop 读，PlaybackLoop 写，并发访问）。
  std::atomic<int> state_{kTactIdle};
  std::atomic<int> error_code_{kTactOk};
  std::string last_name_;
  std::string error_message_;
  mutable std::mutex state_mutex_;
};

}  // namespace runtime
}  // namespace leju

#endif  // LEJU_RL_CONTROLLER_MOTION_TACT_PLAYER_H_
