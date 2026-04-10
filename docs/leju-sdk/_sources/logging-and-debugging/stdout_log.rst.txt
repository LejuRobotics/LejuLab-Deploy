日志数据
========

stdout 日志
-----------

运行时的 stdout 日志存放在 ``~/.ros/lejulab/stdout/`` 目录下。

每次启动时，``start_node.sh`` 脚本会自动创建以时间戳命名的目录，格式为 ``YYYY-MM-DD_HH-MM-SS``，所有节点的日志输出到该目录下的 ``stdout.log`` 文件中。

.. code-block:: bash

   ls -lh ~/.ros/lejulab/stdout/

示例输出：

.. code-block:: text

   总用量 688K
   drwxrwxr-x 2 lab lab 4.0K 1月  29 19:28 2026-01-29_19-28-00
   drwxrwxr-x 2 lab lab 4.0K 1月  29 19:29 2026-01-29_19-29-42
   drwxrwxr-x 2 lab lab 4.0K 1月  29 19:33 2026-01-29_19-33-11
   ...

目录结构：

.. code-block:: text

   ~/.ros/lejulab/
   ├── stdout/                          # stdout 日志根目录
   │   ├── 2026-01-29_19-28-00/
   │   │   └── stdout.log               # 该次运行的所有节点日志
   │   ├── 2026-01-29_19-29-42/
   │   │   └── stdout.log
   │   └── ...
   └── coredumps/                        # coredump 文件目录（启用时）
       └── <PPID>/
           ├── README.txt
           └── core.*

MCAP 录制数据
-------------

录制工具生成的 MCAP 文件存放在 ``~/.ros/lejulab/mcap/`` 目录下，命名格式为 ``recording_YYYY-MM-DD-HH-MM-SS_n.mcap``。

.. code-block:: bash

   ls -lh ~/.ros/lejulab/mcap/

示例输出：

.. code-block:: text

   -rw-rw-r-- 1 lab lab 1.2M 1月  29 19:37 recording_2026-01-29-19-37-35_0.mcap
   -rw-rw-r-- 1 lab lab 2.3M 1月  30 14:48 recording_2026-01-30-14-48-21_0.mcap
   ...
