/**
 * @file topic_names.h
 * @brief DDS Topic 名称定义
 *
 * 定义系统中所有 DDS Topic 的名称常量。
 * 命名规范：/rt/ 前缀表示实时 Topic
 */

#ifndef _LEJU_DDS_TOPICS_DEF_H_
#define _LEJU_DDS_TOPICS_DEF_H_

namespace leju {
namespace dds_topics {

constexpr char kJointCmd[] = "/rt/joint_cmd";      ///< 全身关节命令 (JointCmd)
constexpr char kHandCmd[] = "/rt/hand_cmd";        ///< 双手控制命令 (Float64Array, 12维)
constexpr char kHandState[] = "/rt/hand_state";    ///< 双手状态反馈 (HandState, 12维)
constexpr char kMotorCmd[] = "/rt/motor_cmd";      ///< 写电机前最终命令（post-ankle-fwd, post-c2t）
constexpr char kJointState[] = "/rt/joint_state";  ///< 关节状态反馈 (JointState)
constexpr char kMotorState[] = "/rt/motor_state";  ///< 电机层原始反馈（pre-c2t, pre-ankle-inv）
constexpr char kImuState[] = "/rt/imu_state";      ///< IMU 数据 (ImuData)
constexpr char kJoy[] = "/rt/joy";                 ///< 手柄输入 (Joy)

// hardware topics
constexpr char kHardwareStop[] = "/rt/hardware/stop";   ///< 硬件急停信号
constexpr char kHardwareState[] = "/rt/hardware/state"; ///< 硬件状态


constexpr char kCmdVel[] = "/rt/cmd_vel";                      ///< 速度指令 (VelocityCmd)
constexpr char kVrCmdVel[] = "/rt/vr/cmd_vel";                 ///< VR 速度指令 (VelocityCmd)
constexpr char kPostureHeightCmd[] = "/rt/posture_height_cmd"; ///< 姿态高度指令 (Float64, 单位 m)
constexpr char kArmTrajectory[] = "/rt/arm_trajectory";        ///< 手臂参考轨迹 (JointTrajectoryPoint)
constexpr char kHeadTrajectory[] = "/rt/head_trajectory";      ///< 头部参考轨迹 (JointTrajectoryPoint)
constexpr char kWaistTrajectory[] = "/rt/waist_trajectory";    ///< 腰部参考轨迹 (JointTrajectoryPoint)

// Quest3 remote control topics
constexpr char kQuestBonePoses[] = "/rt/quest/bone_poses";     ///< Quest3 骨骼位姿 (QuestBonePoses)
constexpr char kQuestJoysticks[] = "/rt/quest/joysticks";      ///< Quest3 手柄数据 (QuestJoysticks)
constexpr char kQuestConnected[] = "/rt/quest/connected";      ///< Quest3 握手成功心跳 (StringData; 握手成功后持续发)

// teleop config hot-reload
constexpr char kReloadTeleopConfig[] = "/rt/teleop/reload_config";  ///< 遥控器绑定配置热重载信号 (StringData; data 可选携带配置路径, 空则重读已加载路径)
constexpr char kReloadControllerConfig[] = "/rt/teleop/reload_controllers";  ///< 控制器列表配置热重载信号 (StringData; data 可选携带配置路径, 空则重读已加载路径)

// audio topics
constexpr char kAudioData[] = "/rt/micphone_data";  ///< 麦克风音频数据 (AudioReceiverData)
constexpr char kAudioPlay[] = "/rt/audio_play";     ///< 待播放 PCM 流 (AudioReceiverData, S16LE/16k/mono)
constexpr char kAudioStop[] = "/rt/audio_stop";     ///< 停止播放信号 (StringData, 空 data 即停)
constexpr char kAudioPlayFile[] = "/rt/audio_play_file";  ///< 按文件名播放 (StringData, data=文件名, 空忽略; 打断当前播放)

// tact command topic (NX bridge → RK3588, 替代跨语言 RPC)
constexpr char kTactCommand[] = "/rt/tact_command";  ///< tact 动作播放命令 (StringData; data = tact 文件名)

// tact playback state report topic (RK3588 -> NX bridge, 1Hz)
constexpr char kTactState[] = "/rt/tact_state";  ///< tact 播放状态回报 (TactState; 1Hz, state/error_code/error_message)

// transport mode topics (RK3588 搬运模式状态机接口，对齐研杨版语义)
constexpr char kTransportModeCommand[] = "/rt/transport_mode_command";  ///< 搬运模式命令 (StringData; data = ENTER/LOCK/EXIT/FALL_DOWN/HAND_OVER)
constexpr char kTransportModeState[] = "/rt/transport_mode_state";      ///< 搬运模式状态 (Float64; data = 0~4)

// fall-stand topics (RK3588 倒地起身细粒度阶段接口，对齐研杨 FallStandCommand/fall_stand_state_)
constexpr char kFallStandCommand[] = "/rt/fall_stand_command";  ///< 倒地起身命令 (StringData; data = PREPARE/STAND_UP；RESET 不实现)
constexpr char kFallStandState[] = "/rt/fall_stand_state";      ///< 倒地起身阶段 (Float64; 0=FALL_DOWN瘫软 1=INTERPOLATING 2=READY_FOR_STAND_UP 3=STAND_UP 4=STANDING)

// system monitor topics (leju-monitor/system_info_node, 5Hz;
// 话题名与 kuavo-ros-control system_info_publisher.py 保持一致, 上位机工具可复用)
constexpr char kMonitorCpuUsage[] = "/monitor/system_info/cpu_usage";              ///< Float64Array 每核使用率 % (data[i]=cpu_i)
constexpr char kMonitorCpuTemperature[] = "/monitor/system_info/cpu_temperature";  ///< Float64Array 各 thermal zone 温度 °C (按 zone 序号排列)
constexpr char kMonitorCpuFrequency[] = "/monitor/system_info/cpu_frequency";      ///< Float64Array 每核当前频率 MHz
constexpr char kMonitorMemory[] = "/monitor/system_info/memory";                   ///< Float64Array [percent, used_GB, total_GB, available_GB]
constexpr char kMonitorRlCpuCore[] = "/monitor/system_info/rl_cpu_core";           ///< Float64Array run_rl_controller 各线程所在核心, [tid0,core0,tid1,core1,...], 主线程(=控制线程)在前; 空数组=进程未运行

}  // namespace dds_topics
}  // namespace leju
#endif  // _LEJU_DDS_TOPICS_DEF_H_
