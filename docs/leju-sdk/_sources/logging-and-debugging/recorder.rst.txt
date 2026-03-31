录制工具使用
============

命令行用法
----------

基本录制（默认域 0）
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   lejusdk_recorder -o recording.mcap

指定域 ID 与录制时长（秒）
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   lejusdk_recorder -d 0 -o recording.mcap -t 60

使用配置文件（可配置话题白名单/黑名单等）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   lejusdk_recorder -c config/recorder_example.yaml -o recording.mcap

详细日志输出
~~~~~~~~~~~~

.. code-block:: bash

   lejusdk_recorder -o recording.mcap -v

命令行参数
----------

.. list-table::
   :widths: 20 60
   :header-rows: 1

   * - 参数
     - 说明
   * - ``-h, --help``
     - 显示帮助信息
   * - ``-o, --output``
     - 输出 MCAP 文件路径（必填）
   * - ``-c, --config``
     - 配置文件路径 (YAML)
   * - ``-d, --domain``
     - DDS 域 ID，默认 0
   * - ``-t, --duration``
     - 录制时长（秒），0 表示无限制
   * - ``-v, --verbose``
     - 详细日志输出

配置文件
--------

recorder 支持 YAML 配置文件，可通过 ``-c`` 参数指定。配置示例位于功能包内 ``config/recorder_example.yaml``。

配置文件结构
~~~~~~~~~~~~

.. code-block:: yaml

   # DDS Configuration
   dds:
     domain_id: 0                    # DDS 域 ID
     discovery_timeout: 10.0         # 话题发现超时（秒）
     topics:                         # 话题过滤列表（正则表达式，空则录制全部）
       - "/rt/imu_state"
       - "/rt/joint_state"
       - "/rt/joint_cmd"
       - "/rt/joy"
       - "/rt/hardware/state"
       - "/rt/hardware/stop"

   # Output Configuration
   output:
     prefix: "recording"             # 文件名前缀
     append_date: true               # 是否在文件名后追加日期时间
     max_file_size: 500              # 单文件最大大小（MB），0 表示无限制
     max_duration: 0                 # 单文件最大时长（秒），0 表示无限制
     split: true                     # 是否启用文件分割（达到大小/时长限制时）
     max_splits: 8                   # 最大分割文件数（0 表示无限制）
     min_space: 1024                 # 最小剩余磁盘空间（MB），低于此值停止录制
     buffer_size: 1                  # 写入缓冲区大小（MB）
     compression:
       enabled: true                 # 是否启用压缩
       type: "zstd"                  # 压缩类型：zstd, lz4, 或 none
       level: 3                      # 压缩级别：zstd 1-19，lz4 1-12

   # Timing Configuration
   timing:
     use_sim_time: false             # 是否使用仿真时间
     start_immediately: true         # 是否立即开始录制
     # start_trigger_topic: "/start_recording"  # 触发话题（可选）

   # Logging Configuration
   logging:
     level: "info"                   # 日志级别：trace, debug, info, warning, error, critical
     console: true                   # 是否输出到控制台
     file_enabled: false             # 是否输出到文件
     file_path: "recorder.log"       # 日志文件路径

配置项说明
~~~~~~~~~~

**DDS 配置** （``dds``）

- ``domain_id``：DDS 域 ID，需与机器人/仿真程序一致（默认 0）
- ``discovery_timeout``：话题发现超时时间，超过此时间未发现的话题将被跳过
- ``topics``：话题过滤列表，支持正则表达式；空列表表示录制所有话题

**输出配置** （``output``）

- ``prefix``：输出文件名前缀
- ``append_date``：若为 ``true``，文件名格式为 ``prefix_YYYY-MM-DD-HH-MM-SS.mcap``
- ``max_file_size``：单文件最大大小（MB），达到后自动分割（需 ``split: true``）
- ``max_duration``：单文件最大录制时长（秒），达到后自动分割
- ``split``：是否启用文件分割
- ``max_splits``：最大分割文件数，超过后循环覆盖最旧的文件
- ``min_space``：最小剩余磁盘空间（MB），低于此值停止录制并警告
- ``buffer_size``：写入缓冲区大小，影响写入性能
- ``compression``：压缩配置
  - ``enabled``：是否启用压缩
  - ``type``：``zstd``（压缩率高）、``lz4``（速度快）、``none``（不压缩）
  - ``level``：压缩级别，数值越大压缩率越高但速度越慢

**时间配置** （``timing``）

- ``use_sim_time``：是否使用仿真时间（需 DDS 提供仿真时间支持）
- ``start_immediately``：是否立即开始录制
- ``start_trigger_topic`` （可选）：指定话题作为录制触发，收到该话题消息后才开始录制

**日志配置** （``logging``）

- ``level``：日志级别，从低到高：trace < debug < info < warning < error < critical
- ``console``：是否输出到控制台
- ``file_enabled``：是否输出到日志文件
- ``file_path``：日志文件路径（相对或绝对路径）

使用示例
~~~~~~~~

**录制 LejuSDK 常用话题**：

.. code-block:: yaml

   dds:
     domain_id: 0
     topics:
       - "/rt/imu_state"
       - "/rt/joint_state"
       - "/rt/joint_cmd"
       - "/rt/joy"
       - "/rt/hardware/state"
       - "/rt/hardware/stop"

**限制文件大小并启用压缩**：

.. code-block:: yaml

   output:
     prefix: "robot_recording"
     append_date: true
     max_file_size: 500      # 500 MB
     split: true
     max_splits: 10
     compression:
       enabled: true
       type: "zstd"
       level: 3

**录制所有话题（无过滤）**：

.. code-block:: yaml

   dds:
     domain_id: 0
     topics: []              # 空列表表示录制所有话题

配置文件路径
~~~~~~~~~~~~

配置文件路径可以是：

- 相对路径：相对于当前工作目录
- 绝对路径：完整文件路径
- 功能包内路径：如 ``config/recorder_example.yaml`` （需在功能包目录下运行）

完整配置示例见功能包内 ``config/recorder_example.yaml``。
