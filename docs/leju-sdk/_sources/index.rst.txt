LejuSDK 文档
============

|License| |Platform| |C++| |Roban2.1| |Kuavo4pro| |Kuavo5|

.. |License| image:: https://img.shields.io/badge/License-MIT-yellow.svg
   :target: https://opensource.org/licenses/MIT
   :alt: License: MIT

.. |Platform| image:: https://img.shields.io/badge/Platform-Ubuntu%2020.04-orange.svg
   :target: https://ubuntu.com/
   :alt: Platform: Ubuntu 20.04

.. |C++| image:: https://img.shields.io/badge/C%2B%2B-17-blue.svg
   :target: https://isocpp.org/
   :alt: C++ 17

.. |Roban2.1| image:: https://img.shields.io/badge/Robot-Roban2.1-green.svg
   :alt: Robot: Roban2.1

.. |Kuavo4pro| image:: https://img.shields.io/badge/Robot-Kuavo4pro-green.svg
   :alt: Robot: Kuavo4pro

.. |Kuavo5| image:: https://img.shields.io/badge/Robot-Kuavo5-green.svg
   :alt: Robot: Kuavo5

LejuSDK 是 Leju 机器人软件开发工具包，提供多层次的机器人控制接口。

.. toctree::
   :maxdepth: 2
   :caption: 目录

   getting_started
   modules/index
   logging-and-debugging/index
   writing_controllers

模块概览
--------

.. list-table::
   :widths: 25 75
   :header-rows: 1

   * - 模块
     - 说明
   * - :doc:`lejusdk-utils <modules/lejusdk-utils/index>`
     - 工具库：版本管理、时间工具、定时循环、日志等，SDK 基础依赖
   * - :doc:`lejusdk-lowlevel <modules/lejusdk-lowlevel/index>`
     - 底层控制 SDK，提供电机控制、传感器数据读取等接口（含示例）
   * - :doc:`lejusdk-highlevel <modules/lejusdk-highlevel/index>`
     - 高层控制 SDK，提供应用层控制接口（含示例） **暂未上线，敬请期待**

.. TODO: 新增模块时在此添加
.. 示例:
.. * - :doc:`new-module <modules/new-module/index>`
..   - 新模块说明

索引
----

* :ref:`genindex`
