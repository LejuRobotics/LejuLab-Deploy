lejusdk-vr API
===============

VR 控制 API 参考，提供关节指令发布、速度指令发布、控制模式切换等接口。

VRBaseAPI
---------

.. doxygenclass:: leju::vr::VRBaseAPI
   :project: LejuSDK
   :members:

VR 控制 API 基类，提供通用接口实现。主要功能包括：

- **生命周期管理** - ``initialize()``, ``shutdown()``, ``isInitialized()``
- **关节指令发布** - ``publishArmJointCmd()``, ``publishHeadJointCmd()``, ``publishWaistJointCmd()``
- **速度指令发布** - ``publishVelocityCmd()``, ``publishVrVelocityCmd()``
- **指令订阅** - ``subscribeArmJointCmd()``, ``subscribeVelocityCmd()``, ``subscribeVrVelocityCmd()`` 等
- **控制器服务** - ``switchController()``, ``setArmMode()``, ``getControllerState()``
- **RPC 服务端** - ``registerSwitchControllerHandler()`` 等（控制器端使用）

KuavoVRAPI
----------

.. doxygenclass:: leju::vr::KuavoVRAPI
   :project: LejuSDK
   :members:

Kuavo 机器人 VR 控制 API，继承自 VRBaseAPI。

**使用示例：**

.. code-block:: cpp

   #include <lejusdk-vr/lejusdk_vr.h>

   using namespace leju::vr;

   KuavoVRAPI vr;
   if (!vr.initialize()) {
     return 1;
   }

   // 设置手臂为外部控制模式
   vr.setArmMode(ControlMode::kExternal);

   // 发布关节指令
   JointTrajectoryPoint cmd;
   // 关节数量取决于机器人版本
   cmd.q = {0.0, 0.5, -0.3, 0.0, 0.0, 0.0, 0.0,   // 左臂
            0.0, -0.5, 0.3, 0.0, 0.0, 0.0, 0.0};  // 右臂
   vr.publishArmJointCmd(cmd);

   vr.shutdown();

RobanVRAPI
----------

.. doxygenclass:: leju::vr::RobanVRAPI
   :project: LejuSDK
   :members:

Roban 机器人 VR 控制 API，继承自 VRBaseAPI。

**使用示例：**

.. code-block:: cpp

   #include <lejusdk-vr/lejusdk_vr.h>

   using namespace leju::vr;

   RobanVRAPI vr;
   if (!vr.initialize()) {
     return 1;
   }

   // 使用方式与 KuavoVRAPI 相同
   vr.setArmMode(ControlMode::kExternal);

   vr.shutdown();
