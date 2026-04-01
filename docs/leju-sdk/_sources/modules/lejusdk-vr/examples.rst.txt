lejusdk-vr 示例
================

VR SDK 使用示例，展示初始化、关节控制、速度指令、状态查询等功能。文中数据类型可点击跳转至 :doc:`数据类型 <datatypes>` 查看详细说明。

初始化与关闭
------------

演示 VR API 的基本初始化和关闭流程。

**功能说明：**

- 创建 ``KuavoVRAPI`` 实例
- 调用 ``initialize()`` 初始化 API
- 调用 ``shutdown()`` 关闭 API 并释放资源

**源代码：**

.. literalinclude:: ../../../examples/src/lejusdk-vr/01_basic_init.cpp
   :language: cpp
   :caption: 01_basic_init.cpp

Kuavo 手臂关节控制
------------------

演示如何使用 VR API 控制 Kuavo 机器人手臂进行周期性运动。

**功能说明：**

- 使用 ``setArmMode()`` 将手臂切换到外部控制模式
- 使用正弦波生成平滑的周期运动
- 100 Hz 高频率控制循环
- 支持 Ctrl+C 优雅退出

**Kuavo 手臂关节定义 (14 DOF)：**

左臂 [0-6]:

- ``[0]`` zarm_l1: shoulder_pitch（肩部前屈/后伸）
- ``[1]`` zarm_l2: shoulder_roll（肩部外展/内收）
- ``[2]`` zarm_l3: shoulder_yaw（肩部旋转）
- ``[3]`` zarm_l4: elbow（肘部弯曲）
- ``[4]`` zarm_l5: wrist_roll（手腕旋转）
- ``[5]`` zarm_l6: wrist_pitch（手腕俯仰）
- ``[6]`` zarm_l7: wrist_yaw（手腕偏航）

右臂 [7-13]: zarm_r1 ~ zarm_r7（与左臂对应）

**源代码：**

.. literalinclude:: ../../../examples/src/lejusdk-vr/02_kuavo_arm.cpp
   :language: cpp
   :caption: 02_kuavo_arm.cpp

速度指令
--------

演示如何发布 VR 速度指令控制机器人移动。

**功能说明：**

- 构建 :ref:`VrVelocityCmd <vr-datatypes-vrvelocitycmd>` VR 速度指令
- 使用 ``publishVrVelocityCmd()`` 发布速度指令
- 设置零速度停止机器人

**坐标系说明：**

- ``linear_x`` - 正值前进，负值后退
- ``linear_y`` - 正值向左，负值向右
- ``angular_z`` - 正值逆时针，负值顺时针

**源代码：**

.. literalinclude:: ../../../examples/src/lejusdk-vr/03_velocity_cmd.cpp
   :language: cpp
   :caption: 03_velocity_cmd.cpp

查询控制器状态
--------------

演示如何查询当前控制器状态，包括控制模式和可用控制器列表。

**功能说明：**

- 使用 ``getControllerState()`` 获取 :ref:`ControllerState <vr-datatypes-controllerstate>`
- 查看当前控制器名称
- 查看各部位控制模式（手臂、头部、腰部）
- 查看可用控制器列表

**源代码：**

.. literalinclude:: ../../../examples/src/lejusdk-vr/04_get_state.cpp
   :language: cpp
   :caption: 04_get_state.cpp

头部关节控制
------------

演示如何控制机器人头部进行点头和摇头运动。

**功能说明：**

- 头部控制无需切换模式，指令直接透传
- 头部关节为 2 DOF：pitch（俯仰）和 yaw（偏航）
- 使用 ``publishHeadJointCmd()`` 发布头部指令
- 100 Hz 高频率控制循环
- 支持 Ctrl+C 优雅退出

**关节定义：**

- ``q[0]``: pitch（俯仰角，正值低头）
- ``q[1]``: yaw（偏航角，正值左转）

**源代码：**

.. literalinclude:: ../../../examples/src/lejusdk-vr/05_head_control.cpp
   :language: cpp
   :caption: 05_head_control.cpp

腰部关节控制
------------

演示如何控制机器人腰部进行旋转运动。

**功能说明：**

- 使用 ``setWaistMode()`` 将腰部切换到外部控制模式
- 腰部关节为 1 DOF：yaw（偏航/旋转）
- 使用 ``publishWaistJointCmd()`` 发布腰部指令
- 支持 Kuavo5 和 Roban2（Kuavo4 无腰部关节）
- 100 Hz 高频率控制循环
- 支持 Ctrl+C 优雅退出

**源代码：**

.. literalinclude:: ../../../examples/src/lejusdk-vr/06_waist_control.cpp
   :language: cpp
   :caption: 06_waist_control.cpp

Roban 机器人手臂控制
--------------------

演示如何使用 ``RobanVRAPI`` 控制 Roban 机器人手臂。

**功能说明：**

- 使用 ``RobanVRAPI`` 而非 ``KuavoVRAPI``
- Roban 手臂为 8 DOF（左臂 4 + 右臂 4）
- Kuavo 手臂为 14 DOF（左臂 7 + 右臂 7）
- 100 Hz 高频率控制循环
- 支持 Ctrl+C 优雅退出

**Roban 手臂关节定义：**

- ``[0]`` zarm_l1: shoulder_pitch（肩部前屈/后伸）
- ``[1]`` zarm_l2: shoulder_roll（肩部外展/内收）
- ``[2]`` zarm_l3: wrist（手腕旋转）
- ``[3]`` zarm_l4: elbow（肘部弯曲）
- ``[4-7]`` 右臂 zarm_r1 ~ zarm_r4

**源代码：**

.. literalinclude:: ../../../examples/src/lejusdk-vr/07_roban_arm.cpp
   :language: cpp
   :caption: 07_roban_arm.cpp

交互式控制器命令行工具
------------------------

演示如何使用交互式命令行工具管理和切换控制器。

**功能说明：**

- 使用 ``getControllerState()`` 查询当前控制器状态
- 使用 ``switchController()`` 切换控制器
- 使用 ``setArmMode()`` / ``setWaistMode()`` 设置控制模式
- 支持彩色终端输出，提供直观的交互体验
- 支持 ``list``, ``switch``, ``mode``, ``clear``, ``help``, ``quit`` 等命令

**可用命令：**

- ``list`` - 列出当前控制器状态和可用控制器列表
- ``switch <name>`` - 切换到指定控制器
- ``mode <arm|waist> <keep|auto|external>`` - 设置控制模式
- ``clear`` / ``cls`` - 清屏
- ``help`` - 显示帮助信息
- ``quit`` / ``exit`` - 退出程序

**运行示例：**

.. code-block:: text

    [leju-cli] > list

    ╔══════════════════════════════════════╗
    ║        Controller State              ║
    ╠══════════════════════════════════════╣
    ║ Current Controller: amp
    ║ Arm Mode:   Auto
    ║ Waist Mode: Auto
    ╠══════════════════════════════════════╣
    ║ Available Controllers:
    ║   [0] amp  <-- current
    ║   [1] mimic_hpny
    ║   [2] mimic_prone
    ╚══════════════════════════════════════╝

    [leju-cli] > help

    Usage:
      list                    List current controller state and available controllers
      switch <name>           Switch to specified controller
      mode <part> <mode>      Set control mode
                              part: arm | waist
                              mode: keep | auto | external
      clear / cls              Clear the screen
      help                    Show this help message
      quit / exit             Exit the program

    Examples:
      switch amp              Switch to amp controller
      mode arm external       Set arm to external control mode
      mode waist auto         Set waist to auto mode

**源代码：**

.. literalinclude:: ../../../examples/src/lejusdk-vr/08_controller_cli.cpp
   :language: cpp
   :caption: 08_controller_cli.cpp
