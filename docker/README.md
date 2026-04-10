# lejulab_platform Docker 使用说明

## 进入容器

普通模式：

```bash
./docker/run.sh
```

GPU 模式：

```bash
./docker/run_with_gpu.sh
```

## 首次使用时怎么操作

第一次运行 `./docker/run.sh` 或 `./docker/run_with_gpu.sh` 时，如果本地没有镜像，脚本会提示你是否继续下载和准备环境。

这时直接输入：

```bash
yes
```

然后等待脚本完成即可。

## 普通模式切换到 GPU 模式

直接执行：

```bash
./docker/run_with_gpu.sh
```

如果当前已经有普通模式容器，脚本会提示你是否删除旧容器再继续。

这时输入：

```bash
yes
```

## GPU 模式切回普通模式

直接执行：

```bash
./docker/run.sh
```

如果当前已经有 GPU 模式容器，脚本会提示你是否删除旧容器再继续。

这时输入：

```bash
yes
```

## 第一次使用 GPU 模式

执行：

```bash
./docker/run_with_gpu.sh
```

如果宿主机还没有配置好 GPU 容器环境，脚本会：

1. 提示当前缺少 GPU 相关配置
2. 打印需要安装的命令
3. 询问你是否现在就帮你安装

如果你希望脚本自动完成安装，输入：

```bash
yes
```

## 日常开发

进入容器后直接编译：

```bash
catkin build
```

## Mujoco 仿真

终端 1：

```bash
./docker/run.sh
./src/leju_launch/scripts/start_roudi.sh
```

终端 2：

```bash
./docker/run.sh
roslaunch leju_launch load_mujoco_sim.launch
```

如果不想使用共享内存：

```bash
roslaunch leju_launch load_mujoco_sim.launch enable_iceoryx_shm:=false
```

## VR 遥操作

先保证 Mujoco 仿真已经在运行。

终端 1：

```bash
./docker/run.sh
./src/leju_launch/scripts/start_roudi.sh
```

终端 2：

```bash
./docker/run.sh
roslaunch leju_launch load_mujoco_sim.launch
```

终端 3：

```bash
./docker/run.sh
roslaunch leju_launch vr_teleop.launch quest_ip:=<Quest IP>
```
