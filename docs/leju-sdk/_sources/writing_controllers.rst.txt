控制器实践
================

本文档介绍如何编写机器人控制器，包括控制器架构设计、配置文件参数说明、以及控制器的实践。关于 Lowlevel 接口的详细使用方法和数据类型说明，请参考 :doc:`lejusdk-lowlevel 模块文档 <modules/lejusdk-lowlevel/index>`。

概述
----

控制器是机器人控制系统的核心组件，负责：

- **订阅传感器数据**：IMU、关节状态、手柄输入、硬件状态
- **执行控制算法**：根据传感器数据和目标生成控制指令
- **发布控制指令**：将计算得到的关节目标位置/速度/力矩发送给硬件

LejuSDK Lowlevel 提供了统一的 DDS 通信接口，控制器通过 `GlobalRobot` 单例访问这些接口，无需直接处理 DDS 细节。

快速开始
--------

控制器编写的基本流程：

1. **初始化机器人环境**：设置机器人版本，初始化 DDS 通信
2. **订阅传感器数据**：设置回调函数接收 IMU、关节状态等数据
3. **加载配置文件**：读取控制器参数（关节配置、控制模式、观测项等）
4. **运行控制循环**：周期性读取数据、计算控制指令、发布指令

详细的接口使用方法请参考：

- :doc:`lejusdk-lowlevel 示例 <modules/lejusdk-lowlevel/examples>`：订阅/发布的完整代码示例
- :doc:`lejusdk-lowlevel 数据类型 <modules/lejusdk-lowlevel/datatypes>`：RobotState、RobotCmd、ImuData 等数据结构说明
- :doc:`lejusdk-lowlevel API <modules/lejusdk-lowlevel/api>`：GlobalRobot、RobotBaseAPI 等接口文档

控制器架构
----------

典型的控制器类结构：

.. code-block:: cpp

   #include "lejusdk-lowlevel/leju_sdk.h"
   #include <mutex>
   #include <thread>
   
   class MyController {
   public:
     bool initialize(const std::string& config_file) {
       // 1. 初始化机器人环境
       // 方式 1: 从环境变量读取（推荐）
       // robot_version_ = leju::RobotVersion::from_env();
       // 方式 2: 显式指定版本
       robot_version_ = leju::RobotVersions::KUAVO5_BASE;
       if (!leju::GlobalRobot::init_env(robot_version_)) {
         return false;
       }
       
       auto& robot = leju::GlobalRobot::getInstance();
       motor_count_ = robot.getMotorNumber();
       motor_names_ = robot.getMotorNames();
       
       // 2. 设置数据订阅（回调在独立线程执行）
       setupSubscriptions();
       
       // 3. 加载配置文件
       if (!loadConfig(config_file)) {
         return false;
       }
       
       return true;
     }
     
     void run() {
       // 控制循环
       mainLoop();
     }
     
   private:
     void setupSubscriptions();  // 设置订阅回调
     bool loadConfig(const std::string& config_file);  // 加载配置
     void mainLoop();  // 主控制循环
     
     // 数据成员
     leju::RobotVersion robot_version_;
     size_t motor_count_;
     std::vector<std::string> motor_names_;
     
     // 线程安全的数据存储
     std::mutex robot_state_mutex_;
     leju::RobotState robot_state_;
     std::mutex imu_data_mutex_;
     leju::ImuData imu_data_;
   };

关键设计要点
~~~~~~~~~~~~

**线程安全**
  所有 DDS 回调函数在独立线程中执行，访问共享数据时必须使用互斥锁保护。

**硬件状态检查**
  发布控制指令前，确保硬件状态为 ``READY_OK``，否则指令不会生效。

**控制频率**
  - 控制循环通常 1 kHz（``loop_dt = 0.001``），与硬件同步
  - 策略/算法执行频率可降低（如 50 Hz），通过 ``decimation`` 控制

详细实现示例请参考 :doc:`lejusdk-lowlevel 示例 <modules/lejusdk-lowlevel/examples>` 中的综合控制示例。

配置文件参数说明
----------------

本节以 KUAVO5_BASE 版本 AMP 控制器的配置文件为例，说明各参数含义。配置文件为 YAML 格式，位于 ``config/52/config_amp.yaml``。

基础配置
~~~~~~~~

.. code-block:: yaml

   HumanoidRobotCfg:
     loop_dt: 0.001          # 控制循环周期 (s)，通常 1 kHz
     policy_path: "policy/xxx.onnx"  # ONNX 模型文件路径（相对配置文件）

**loop_dt**
  控制循环周期，单位秒。通常设置为 0.001（1 kHz），与硬件控制频率一致。控制循环中应保持此频率，确保指令及时发送。

**policy_path**
  策略模型文件路径，相对于配置文件所在目录。如果控制器不使用模型（如 PID 控制器），可忽略此参数。

机器人参数 (env.robot)
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   env:
     policy_dt: 0.02         # 策略执行周期 (s)，通常 50 Hz
     robot:
       joint_names: 
         ["l_leg_roll", "l_leg_yaw", "l_leg_pitch", "l_knee", "l_foot_pitch", "l_foot_roll", 
          "r_leg_roll", "r_leg_yaw", "r_leg_pitch", "r_knee", "r_foot_pitch", "r_foot_roll", 
          "waist_yaw",
          "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm", "l_hand_yaw", "l_hand_roll", "l_hand_pitch", 
          "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm", "r_hand_yaw", "r_hand_roll", "r_hand_pitch"]
       joint_direction: [1., 1., ..., 1.]
       joint_default_pos: [0., 0., -0.24, 0.5, -0.26, 0., ..., 0.]
       joint_torque_limit: [127., 71., ..., 14.1]
       actuator_kp: [100., 100., ..., 15.]
       actuator_kd: [5., 5., ..., 3.]
       actuator_control_mode: [1, 1, ..., 2]
       action_scale: [0.25, 0.25, ..., 0.25]

**policy_dt**
  策略执行周期，单位秒。通常设置为 0.02（50 Hz），用于降低计算负担。在控制循环中使用 ``decimation = round(policy_dt / loop_dt)`` 控制策略执行频率。

**joint_names**
  关节名称列表，顺序必须与机器人关节顺序一致。KUAVO5_BASE 版本共 27 个关节：12 腿 + 1 腰 + 14 臂。控制器会根据此列表匹配 ``motor_names_`` 获取关节索引。

**joint_direction**
  关节方向系数数组，用于兼容不同旋转方向的关节。通常为 1.0，若关节旋转方向相反则设为 -1.0。在计算目标位置时使用：``target = joint_direction[i] * (default_pos[i] + action[i] * scale)``。

**joint_default_pos**
  默认关节位置数组（rad），机器人启动时的目标姿态。通常腿为站立姿态（如 ``[0, 0, -0.24, 0.5, -0.26, 0]``），臂为下垂姿态（全 0）。

**joint_torque_limit**
  关节力矩限制数组（N·m），用于安全保护。超过此值的力矩指令会被限幅。不同关节的限幅值不同，通常腿关节较大（100-280 N·m），手部关节较小（14.1 N·m）。

**actuator_kp**
  位置增益 Kp 数组（N·m/rad），用于 CSP 模式的位置环控制。值越大，位置跟踪越强，但可能引起振荡。通常腿关节较大（80-100），手部关节较小（15-30）。

**actuator_kd**
  速度增益 Kd 数组（N·m·s/rad），用于 CSP/CSV 模式的阻尼。值越大，阻尼越强，有助于稳定。通常腿关节较大（5-7.5），手部关节较小（3）。

**actuator_control_mode**
  控制模式数组，每个关节对应一个值：

  - ``0``: CST（力矩控制），适用于力控、柔顺控制
  - ``1``: CSV（速度控制），适用于匀速运动
  - ``2``: CSP（位置控制），适用于精确定位


**action_scale**
  动作缩放系数数组，用于将模型输出（通常归一化到 [-1, 1]）缩放到关节角度范围。例如 0.25 表示模型输出 ±1 对应 ±0.25 rad。计算公式：``target = default_pos + action * action_scale``。

观测配置 (env.observations)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   observations:
     history_length: 1       # 历史帧数
     stack_order: "classic"  # 堆叠顺序："classic" 或 "isaaclab"
     terms:
       base_ang_vel:
         scale: 1.
         clip: [-9999., 9999.]
       projected_gravity:
         scale: 1.
         clip: [-9999., 9999.]
       velocity_commands:
         scale: 1.
         clip: [-9999., 9999.]
       cmd_stance:
         scale: 1.
         clip: [0., 1.]
       joint_pos:
         scale: 1.
         clip: [-9999., 9999.]
       joint_vel:
         scale: 1.
         clip: [-9999., 9999.]
       actions:
         scale: 1.
         clip: [-18., 18.]

**history_length**
  历史帧数，用于时序模型。``1`` 表示仅使用当前帧，``2`` 表示使用当前帧 + 上一帧。历史帧会增加观测维度：总维度 = 单帧维度 × history_length。

**stack_order**
  堆叠顺序，决定历史帧的组织方式：

  - ``"classic"``: 按时间顺序堆叠，`[历史帧0的所有项, 历史帧1的所有项, ...]`
  - ``"isaaclab"``: 按观测项堆叠，`[项0的所有历史, 项1的所有历史, ...]`

**terms**
  观测项配置字典，每个项包含：

  - ``scale``: 缩放系数，用于归一化观测值
  - ``clip``: 裁剪范围 `[min, max]`，超出范围的值会被裁剪到边界

**常用观测项**：

- ``base_ang_vel``: 基座角速度（3 维），来自 IMU gyro
- ``projected_gravity``: 投影重力（3 维），IMU quat 旋转后的重力向量
- ``velocity_commands``: 速度指令（3 维：lin_vel_x, lin_vel_y, ang_vel_z），来自手柄摇杆
- ``cmd_stance``: 站立/行走指令（1 维：1=站立，0=行走），由 CmdStanceCalculator 计算
- ``joint_pos``: 关节位置（N 维，N 为关节数），相对默认位置的残差
- ``joint_vel``: 关节速度（N 维）
- ``actions``: 上一帧动作（N 维），用于时序模型

命令范围 (command_range)
~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   command_range:
     lin_vel_x: [-1.0, 1.0]    # 线速度 x 范围 (m/s)
     lin_vel_y: [-0.6, 0.6]    # 线速度 y 范围 (m/s)
     ang_vel_z: [-0.3, 0.3]    # 角速度 z 范围 (rad/s)

定义速度指令的允许范围。手柄摇杆输入会按此范围缩放：

- ``left_y`` → ``lin_vel_x`` （前推为正）
- ``left_x`` → ``lin_vel_y`` （左推为正）
- ``right_x`` → ``ang_vel_z`` （左推为正）

手柄轴值范围是 [-1, 1]，乘以 ``command_range`` 的上下界得到实际速度指令。

手臂控制器 (arm_controller)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   arm_controller:
     enabled: true
     leg_joint_count: 12
     waist_joint_count: 1
     interpolation_velocity: 1.0   # rad/s
     min_duration: 0.2
     max_duration: 2.0

手臂控制器用于在行走时使用 RL 摆手，站立时平滑插值到默认姿态。

**enabled**
  是否启用手臂控制器。启用后，当 ``cmd_stance == 1`` （站立）时，手臂会从当前姿态平滑插值到 ``joint_default_pos`` 的手臂段。

**leg_joint_count**
  腿部关节数，KUAVO5_BASE 版本为 12（左右腿各 6）。用于计算手臂在 ``joint_names`` 中的起始索引：``arm_start_index = leg_joint_count + waist_joint_count``。

**waist_joint_count**
  腰部关节数，KUAVO5_BASE 版本为 1（waist_yaw）。

**interpolation_velocity**
  插值最大速度（rad/s），用于计算从当前手臂姿态到默认姿态的插值时长。时长计算公式：``duration = 1.5 * max_joint_displacement / interpolation_velocity``。

**min_duration / max_duration**
  插值时长范围（秒），避免插值过快或过慢。计算出的时长会被限制在此范围内。

cmd_stance 配置
~~~~~~~~~~~~~~~

.. code-block:: yaml

   cmd_stance:
     smart_stop:
       enabled: true
       torso_velocity_threshold: 0.05  # m/s
       feet_alignment_threshold: 0.08   # m
     velocity_magnitude_threshold: 0.01  # 速度低于此值视为"小"

cmd_stance 用于区分站立（1）和行走（0）状态，影响手臂控制器的行为。

**smart_stop**
  智能停止配置，当速度小时自动切换到站立：

  - ``enabled``: 是否启用智能停止
  - ``torso_velocity_threshold``: 躯干速度阈值（m/s），低于此值视为静止
  - ``feet_alignment_threshold``: 双脚对齐阈值（m），左右脚 x 方向距离小于此值视为对齐

  智能停止需要 torso_state 和 feet_positions 数据，如果使用 ``computeSimple()`` （无这些数据），则速度小时默认返回站立。

**velocity_magnitude_threshold**
  速度幅值阈值，用于判断摇杆是否松开。速度低于此值视为"小"，可能触发站立。设置较大值（如 0.05）可避免摇杆漂移导致的误判。

控制器实践
--------------

线程安全
~~~~~~~~

所有 DDS 回调函数在独立线程中执行，访问共享数据时必须使用互斥锁：

.. code-block:: cpp

   class MyController {
   private:
     std::mutex robot_state_mutex_;
     leju::RobotState robot_state_;
     
     void setupSubscriptions() {
       auto& robot = leju::GlobalRobot::getInstance();
       robot.subscribeRobotState(
         [this](const leju::RobotStateConstPtr& state) {
           std::lock_guard<std::mutex> lock(robot_state_mutex_);  // 必须加锁
           robot_state_ = *state;
         });
     }
     
     leju::RobotState getRobotState() {
       std::lock_guard<std::mutex> lock(robot_state_mutex_);
       return robot_state_;
     }
   };

硬件状态检查
~~~~~~~~~~~~

发布控制指令前，确保硬件状态为 ``READY_OK``：

.. code-block:: cpp

   leju::HardwareState hw_state = getHardwareState();
   if (hw_state != leju::HardwareState::READY_OK) {
     // 硬件未就绪，不发布指令
     return;
   }
   robot.publishRobotCmd(cmd);

控制频率管理
~~~~~~~~~~~~

控制循环通常以 1 kHz 运行，但策略/算法执行频率可降低：

.. code-block:: cpp

   double loop_dt = 0.001;      // 1 kHz 控制循环
   double policy_dt = 0.02;     // 50 Hz 策略执行
   int decimation = std::round(policy_dt / loop_dt);  // 20
   
   int cnt = 0;
   auto t0 = std::chrono::steady_clock::now();
   while (running_) {
     // 策略每 decimation 步执行一次
     if (cnt % decimation == 0) {
       computePolicy();  // 50 Hz
     }
     
     // 控制指令每步都更新
     updateControlCommand();  // 1 kHz
     robot.publishRobotCmd(cmd);
     
     // 控制循环频率
     t0 += std::chrono::milliseconds(1);
     std::this_thread::sleep_until(t0);
     cnt++;
   }

配置加载示例
~~~~~~~~~~~~

使用 YAML-CPP 加载配置文件：

.. code-block:: cpp

   #include <yaml-cpp/yaml.h>
   
   bool MyController::loadConfig(const std::string& config_file) {
     YAML::Node cfg = YAML::LoadFile(config_file);
     YAML::Node robot_cfg = cfg["HumanoidRobotCfg"];
     
     // 基础配置
     loop_dt_ = robot_cfg["loop_dt"].as<double>();
     
     // 机器人参数
     YAML::Node robot = robot_cfg["env"]["robot"];
     joint_names_ = robot["joint_names"].as<std::vector<std::string>>();
     
     auto kp_vec = robot["actuator_kp"].as<std::vector<double>>();
     kp_ = Eigen::Map<const Eigen::ArrayXd>(kp_vec.data(), kp_vec.size());
     
     auto control_mode_vec = robot["actuator_control_mode"].as<std::vector<int>>();
     control_mode_ = Eigen::Map<const Eigen::ArrayXi>(
         control_mode_vec.data(), control_mode_vec.size());
     
     // 观测配置
     YAML::Node obs = robot_cfg["env"]["observations"];
     history_length_ = obs["history_length"].as<int>();
     
     // 命令范围
     YAML::Node cmd_range = robot_cfg["env"]["command_range"];
     auto lin_vel_x_range = cmd_range["lin_vel_x"].as<std::vector<double>>();
     lin_vel_x_min_ = lin_vel_x_range[0];
     lin_vel_x_max_ = lin_vel_x_range[1];
     
     return true;
   }
