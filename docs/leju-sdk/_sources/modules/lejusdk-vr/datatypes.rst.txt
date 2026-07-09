lejusdk-vr 数据类型
====================

VR SDK 中使用的核心数据结构定义。示例代码中涉及的类型可点击跳转至本页对应说明。

.. _vr-datatypes-pose:

Pose
----

.. doxygenstruct:: leju::vr::Pose
   :project: LejuSDK
   :members:

.. _vr-datatypes-questboneposes:

QuestBonePosesData
------------------

.. doxygenstruct:: leju::vr::QuestBonePosesData
   :project: LejuSDK
   :members:

.. _vr-datatypes-questjoystick:

QuestJoystickData
-----------------

.. doxygenstruct:: leju::vr::QuestJoystickData
   :project: LejuSDK
   :members:

.. _vr-datatypes-jointtrajectorypoint:

JointTrajectoryPoint
--------------------

.. doxygenstruct:: leju::vr::JointTrajectoryPoint
   :project: LejuSDK
   :members:


.. _vr-datatypes-velocitycmd:

VelocityCmd
-----------

.. doxygenstruct:: leju::vr::VelocityCmd
   :project: LejuSDK
   :members:


.. _vr-datatypes-vrvelocitycmd:

VrVelocityCmd
-------------

.. doxygenstruct:: leju::vr::VrVelocityCmd
   :project: LejuSDK
   :members:


.. _vr-datatypes-controlmode:

ControlMode
-----------

.. doxygenenum:: leju::vr::ControlMode
   :project: LejuSDK

部位控制模式枚举，定义各部位（手臂、头部、腰部）的控制方式。

**枚举值：**

- ``kKeepPose`` (0) - 保持当前姿态，不接受外部指令
- ``kAuto`` (1) - 自动模式，由 RL 控制器控制
- ``kExternal`` (2) - 外部控制模式，接受 VR 指令

**使用场景：**

在发布关节指令前，需要先将对应部位切换到 ``kExternal`` 模式：

.. code-block:: cpp

   // 切换手臂到外部控制模式
   vr_api.setArmMode(ControlMode::kExternal);

   // 发布手臂关节指令
   vr_api.publishArmJointCmd(arm_cmd);

.. _vr-datatypes-controllerstate:

ControllerState
---------------

.. doxygenstruct:: leju::vr::ControllerState
   :project: LejuSDK
   :members:

