lejusdk-lowlevel 示例
========================

底层 SDK 使用示例，展示传感器数据订阅、控制指令发布等功能。文中数据类型可点击跳转至 :doc:`数据类型 <datatypes>` 查看详细说明。

IMU 数据订阅
------------

订阅并打印 IMU 传感器数据，包括角速度、加速度和姿态四元数。

**功能说明：**

- 使用 ``subscribeImuData()`` 订阅 IMU 数据
- 通过回调函数接收 :ref:`ImuDataConstPtr <datatypes-imudata>` 数据
- 支持 Ctrl+C 信号优雅退出

**运行方式：**

.. code-block:: bash

   export ROBOT_VERSION=46  # 设置机器人版本
   rosrun lejusdk_examples imu_data_subscriber

**源代码：**

.. literalinclude:: ../../../examples/src/imu_data_subscriber.cpp
   :language: cpp
   :caption: imu_data_subscriber.cpp

关节状态订阅
------------

订阅并打印关节状态数据，包括位置、速度、加速度和力矩。

**功能说明：**

- 使用 ``subscribeRobotState()`` 订阅关节状态
- 通过回调函数接收 :ref:`RobotStateConstPtr <datatypes-robotstate>` 数据
- 适用于状态监控和数据记录场景

**运行方式：**

.. code-block:: bash

   export ROBOT_VERSION=46
   rosrun lejusdk_examples joint_state_subscriber

**源代码：**

.. literalinclude:: ../../../examples/src/joint_state_subscriber.cpp
   :language: cpp
   :caption: joint_state_subscriber.cpp

硬件状态订阅
------------

订阅硬件状态，监控机器人底层驱动的运行状态。

**功能说明：**

- 使用 ``subscribeHardwareState()`` 订阅硬件状态
- 使用 ``String2HwState()`` 将字符串转换为 :ref:`HardwareState<datatypes-hardwarestate>` 枚举
- 硬件状态包括：``UNKNOWN``, ``INITIALIZING``, ``READY_OK``, ``ERROR``, ``STOPPED``

**重要提示：**

只有当硬件状态为 ``READY_OK`` 时，发布的控制指令才会生效。

**运行方式：**

.. code-block:: bash

   export ROBOT_VERSION=46
   rosrun lejusdk_examples hw_state_subscriber

**源代码：**

.. literalinclude:: ../../../examples/src/hw_state_subscriber.cpp
   :language: cpp
   :caption: hw_state_subscriber.cpp

控制指令发布
------------

等待硬件就绪后发布正弦波控制指令，演示完整的控制流程。

**功能说明：**

- 订阅硬件状态，等待 ``READY_OK`` 后开始发布
- 使用 ``publishRobotCmd()`` 发布 :ref:`RobotCmd<datatypes-robotcmd>` 控制指令
- 生成正弦波轨迹控制机器人头部关节
- 支持位置控制模式 (CSP)，设置 Kp/Kd 增益

**控制参数：**

- 控制频率：50Hz
- 正弦波频率：0.5Hz
- 振幅：0.2 rad

**运行方式：**

.. code-block:: bash

   export ROBOT_VERSION=46
   rosrun lejusdk_examples joint_cmd_publisher

**源代码：**

.. literalinclude:: ../../../examples/src/joint_cmd_publisher.cpp
   :language: cpp
   :caption: joint_cmd_publisher.cpp

综合控制示例
------------

Kuavo 机器人综合控制示例，展示完整的控制流程：初始化、数据订阅、控制指令发布。

**功能说明：**

- 封装为 ``KuavoRobotController`` 类，展示面向对象设计
- 同时订阅 IMU 和关节状态数据
- 生成正弦波控制指令并发布
- 每10条消息打印一次，避免日志刷屏

**代码结构：**

- ``initialize()`` - 初始化机器人和订阅
- ``setupSubscriptions()`` - 设置数据订阅回调
- ``generateSineWaveCommand()`` - 生成正弦波控制指令
- ``runControlLoop()`` - 运行控制循环

**运行方式：**

.. code-block:: bash

   export ROBOT_VERSION=46
   rosrun lejusdk_examples kuavo_control_example

**源代码：**

.. literalinclude:: ../../../examples/src/kuavo_control_example.cpp
   :language: cpp
   :caption: kuavo_control_example.cpp
