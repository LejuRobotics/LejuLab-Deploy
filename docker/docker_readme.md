## 前提
请正确安装docker；正确安装nvidia显卡驱动，nvidia-container-toolkit并配置（如需使用GPU）

## 下载镜像并进入docker container

- 从[这里](https://kuavo.lejurobot.com/kuavo_research_editiion/docker_images/kuavo_opensource_mpc_wbc_img_v1.3.0.tar.gz)下载容器镜像

```bash
docker load -i kuavo_opensource_mpc_wbc_img_v1.3.0.tar.gz # 导入镜像

cd LejuLab-Deploy   #先clone项目到本地
./docker/run_with_gpu.sh    #或者 ./docker/run.sh
```

## 依赖安装

```bash
sudo apt-get update && sudo apt-get install -y \
    build-essential cmake \
    libacl1-dev libncurses5-dev
```

## 编译

```bash
source installed/setup.zsh  # !!! IMPORTANT !!! 非常重要,不可省略 docker中默认是zsh
catkin build
```

## 部署 iceoryx 共享内存

项目支持通过 iceoryx 共享内存加速进程间通信，建议部署以获得更优的实时性能：

```bash
./src/leju_launch/scripts/setup_cyclonedds_config_for_docker.sh
```

### Mujoco 仿真

```bash
source devel/setup.zsh

# Roban2
export ROBOT_VERSION=14
roslaunch leju_launch load_mujoco_sim.launch

# Kuavo4pro
export ROBOT_VERSION=46
roslaunch leju_launch load_mujoco_sim.launch

# Kuavo5
export ROBOT_VERSION=52
roslaunch leju_launch load_mujoco_sim.launch

###如果不想使用共享内存可以不部署 iceoryx 共享内存，启动命令如下：roslaunch leju_launch load_mujoco_sim.launch enable_iceoryx_shm:=false
```

- 通过如上命令启动控制器、Mujoco 仿真器和手柄控制等功能包
- 根据终端提示，按下`start`按键
- 点击 Mujoco 仿真中的 `Run` 运行按钮
- tips: 如果开始时机器人倒地，可以先`Pause`和`Reset`仿真，然后再`Run`