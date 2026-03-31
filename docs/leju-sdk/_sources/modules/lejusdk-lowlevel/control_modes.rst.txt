电机控制模式 (CSP/CST)
======================

本文档介绍 lejusdk-lowlevel 支持的两种电机控制模式：CSP（位置控制）、CST（力矩控制）。

相关数据类型请参考：

- :ref:`datatypes-motorcontrolmode` - 控制模式枚举
- :ref:`datatypes-robotcmd` - 机器人控制指令结构体

概述
----

.. list-table:: 控制模式对比
   :header-rows: 1
   :widths: 15 15 70

   * - 模式
     - 枚举值
     - 说明
   * - CSP
     - 2
     - 位置控制模式，发送目标位置、Kp、Kd，内部计算力矩
   * - CST
     - 0
     - 力矩控制模式，直接发送目标力矩

.. warning::

   当前 **手臂和头部** 的关节仅支持 **CSP** 控制模式。

CSP 模式
--------

位置控制模式，使用 PD 控制律计算力矩：

.. code-block:: text

   τ = Kp × (q_target - q_actual) + Kd × (v_target - v_actual) + τ_ff

控制框图
^^^^^^^^

框图左侧为用户通过 ``RobotCmd`` 发送的控制参数，右侧为驱动器内部的 PD 控制计算。
用户发送的参数均为关节空间量，底层控制软件处理流程：

1. **传动转换**：将关节位置/速度转换为电机位置/速度
2. **PD 控制**：使用编码器获取实际位置 ``q_actual`` 和实际速度 ``v_actual``，结合目标值和增益计算力矩
3. **力矩限幅**：根据硬件配置文件中的 ``max_torque`` 对计算出的力矩进行限幅保护
4. **单位转换**：转换为驱动器所需的单位后发送

.. code-block:: text

       lejusdk (User)        |                 Drive (Motor)
                              |
    +-----------+             |
    | q_target  |-------------+--->+-----------------------------------------+
    | (cmd.q)   |             |    |                                         |
    +-----------+             |    |  +--------+             +------+        |
                              |    |  |        |  +------+   |      |        |
    +-----------+             |    +->|   -    |->| x Kp |-->|      |        |
    | v_target  |-------------+--->+  +--------+  +------+   |      |        |
    | (cmd.v)   |             |    |       ^                 |      |        |
    +-----------+             |    |       | q_actual        |  +   |  +-------+
                              |    |                         |      |->| Motor |
    +-----------+             |    |  +--------+             |      |  +-------+
    | tau_ff    |-------------+--->+->|   -    |->| x Kd |-->|      |        |
    | (cmd.tau) |             |    |  +--------+  +------+   |      |        |
    +-----------+             |    |       ^                 +------+        |
                              |    |       | v_actual                        |
    +-----------+             |    |                                         |
    | kp        |-------------+--->+  Kp gain                                |
    | (cmd.kp)  |             |    |                                         |
    +-----------+             |    |                                         |
                              |    |                                         |
    +-----------+             |    |                                         |
    | kd        |-------------+--->+  Kd gain                                |
    | (cmd.kd)  |             |    |                                         |
    +-----------+             |    +-----------------------------------------+
                              |
    +-----------+             |
    | mode = 2  |-------------+--->  CSP mode
    | (cmd.modes)|            |
    +-----------+             |

    tau = Kp * (q_target - q_actual) + Kd * (v_target - v_actual) + tau_ff

    Legend:
      q_target  : Target position (rad)
      v_target  : Target velocity (rad/s)
      tau_ff    : Feedforward torque (N.m)
      Kp        : Position gain (N.m/rad)
      Kd        : Velocity gain (N.m.s/rad)

参数对应关系：

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - 参数
     - RobotCmd 字段
     - 说明
   * - q_target
     - ``cmd.q``
     - 目标位置 (rad)
   * - v_target
     - ``cmd.v``
     - 目标速度 (rad/s)，通常为 0
   * - τ_ff
     - ``cmd.tau``
     - 前馈力矩 (N·m)
   * - Kp
     - ``cmd.kp``
     - 位置增益 (N·m/rad)，CST 模式下必须为 0
   * - Kd
     - ``cmd.kd``
     - 速度增益 (N·m·s/rad)，CST 模式下必须为 0

使用方法
^^^^^^^^

.. code-block:: cpp

   #include <lejusdk-lowlevel/data_types.h>
   #include <lejusdk-lowlevel/leju_sdk.h>

   using namespace leju;

   auto& robot = GlobalRobot::getInstance();
   size_t motor_count = robot.getMotorNumber();

   RobotCmd cmd(motor_count);

   for (size_t i = 0; i < motor_count; ++i) {
       cmd.q[i] = target_position[i];  // 目标位置 (rad)
       cmd.v[i] = 0.0;                 // 目标速度 (rad/s)
       cmd.tau[i] = 0.0;               // 前馈力矩 (N·m)
       cmd.kp[i] = 100.0;              // 位置增益 (N·m/rad)
       cmd.kd[i] = 10.0;               // 速度增益 (N·m·s/rad)
       cmd.modes[i] = static_cast<uint8_t>(MotorControlMode::CSP);
   }

   robot.publishRobotCmd(cmd);

.. warning::

   CSP 模式下，用户层的 Kp/Kd 参数需要设置成国际单位的值 (N·m/rad, N·m·s/rad)

.. warning::
   .. note:: 已实现
   .. raw:: html

      <del>我们会在下一个版本补充用户层 Kp/Kd 到内部转换，届时用户只需要关心用户层的 Kp/Kd 即可。</del>


CST 模式
--------

力矩控制模式，直接发送目标力矩。适用于力控、阻抗控制、RL 策略等场合。

控制框图
^^^^^^^^

框图左侧为用户通过 ``RobotCmd`` 发送的控制参数，右侧为驱动器内部的力矩控制。
用户发送的 ``tau_target`` 为关节力矩，底层控制软件处理流程：

1. **传动转换**：根据机器人传动机构（并联腿、减速器等）将关节力矩转换为电机力矩
2. **力矩限幅**：根据硬件配置文件中的 ``max_torque`` 对电机力矩进行限幅保护
3. **单位转换**：转换为驱动器所需的单位后发送

.. code-block:: text

              lejusdk (User)                   |        Drive (Motor)
                                               |
    +-------------+                            |
    | tau_target  |----------------------------+--->+----------+
    | (cmd.tau)   |                            |    |          |   +-------+
    +-------------+                            |    |  Torque  |   |       |
                                               |    |  Control |-->| Motor |
    +-------------+                            |    |          |   |       |
    | kp = 0      |  Must be 0, otherwise      |    +----------+   +-------+
    | (cmd.kp)    |  PD control will be added  |         ^
    +-------------+                            |         |
    | kd = 0      |                            |         |
    | (cmd.kd)    |                            |         |
    +-------------+                            |         |
                                               |         |
    +-------------+                            |         |
    | mode = 0    |----------------------------+---------+
    | (cmd.modes) |  CST mode                  |
    +-------------+                            |

使用方法
^^^^^^^^

.. code-block:: cpp

   #include <lejusdk-lowlevel/data_types.h>
   #include <lejusdk-lowlevel/leju_sdk.h>

   using namespace leju;

   auto& robot = GlobalRobot::getInstance();
   size_t motor_count = robot.getMotorNumber();

   RobotCmd cmd(motor_count);

   for (size_t i = 0; i < motor_count; ++i) {
       cmd.tau[i] = target_torque[i];  // 目标力矩 (N·m)
       cmd.kp[i] = 0.0;                // 必须为 0
       cmd.kd[i] = 0.0;                // 必须为 0
       cmd.modes[i] = static_cast<uint8_t>(MotorControlMode::CST);
   }

   robot.publishRobotCmd(cmd);

.. warning::

   CST 模式下，``cmd.kp`` 和 ``cmd.kd`` **必须设置为 0**，否则会叠加 PD 控制律的输出。
