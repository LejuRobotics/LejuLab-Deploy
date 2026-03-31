lejusdk-utils API
=================

RobotVersion
^^^^^^^^^^^^

.. doxygenclass:: leju::RobotVersion
   :project: LejuSDK
   :members:

.. _api-robot-versions:

RobotVersions
^^^^^^^^^^^^^

预定义的机器人版本常量。

.. _api-kuavo-versions:

Kuavo 系列
^^^^^^^^^^

.. doxygengroup:: KuavoVersions
   :project: LejuSDK
   :content-only:

.. _api-roban-versions:

Roban 系列
^^^^^^^^^^

.. doxygengroup:: RobanVersions
   :project: LejuSDK
   :content-only:

Kuavo 足式机器人检测宏
^^^^^^^^^^^^^^^^^^^^^^

.. c:macro:: IS_KUAVO_LEGGED(rb)

   判断是否为 Kuavo 足式机器人（4代或5代）

.. c:macro:: IS_KUAVO4_LEGGED(rb)

   判断是否为 Kuavo 4代足式机器人

.. c:macro:: IS_KUAVO5_LEGGED(rb)

   判断是否为 Kuavo 5代足式机器人

.. c:macro:: IS_KUAVO4PRO_LEGGED(rb)

   判断是否为 Kuavo 4代 PRO 足式机器人（4.5及以上版本）

Roban 足式机器人检测宏
^^^^^^^^^^^^^^^^^^^^^^

.. c:macro:: IS_ROBAN_LEGGED(rb)

   判断是否为 Roban 足式机器人

.. c:macro:: IS_ROBAN2_LEGGED(rb)

   判断是否为 Roban 2代足式机器人

Kuavo 轮式机器人检测宏
^^^^^^^^^^^^^^^^^^^^^^

.. c:macro:: IS_KUAVO_WHEELED(rb)

   判断是否为 Kuavo 轮式机器人

产品系列检测宏
^^^^^^^^^^^^^^

.. c:macro:: IS_KUAVO(rb)

   判断是否为 Kuavo 系列机器人（足式或轮式）

.. c:macro:: IS_ROBAN(rb)

   判断是否为 Roban 系列机器人

时间工具函数
^^^^^^^^^^^^

.. doxygenfunction:: leju::common::GetUnixTimestampS
   :project: LejuSDK

.. doxygenfunction:: leju::common::GetUnixTimestampNs
   :project: LejuSDK

.. doxygenfunction:: leju::common::GetUnixTimestampSecNsec
   :project: LejuSDK

.. doxygenfunction:: leju::common::GetSteadyTimestampNs
   :project: LejuSDK

Loop
^^^^

.. doxygenclass:: leju::Loop
   :project: LejuSDK
   :members:

日志宏
^^^^^^

基础日志宏

.. c:macro:: LOG_D(tag, ...)

   输出 DEBUG 级别日志

.. c:macro:: LOG_T(tag, ...)

   输出 TRACE 级别日志

.. c:macro:: LOG_I(tag, ...)

   输出 INFO 级别日志

.. c:macro:: LOG_W(tag, ...)

   输出 WARN 级别日志

.. c:macro:: LOG_E(tag, ...)

   输出 ERROR 级别日志

彩色日志宏

.. c:macro:: LOG_SUCCESS(tag, format, ...)

   输出绿色的成功信息

.. c:macro:: LOG_WARNING(tag, format, ...)

   输出黄色的警告信息

.. c:macro:: LOG_FAILURE(tag, format, ...)

   输出红色的错误信息
