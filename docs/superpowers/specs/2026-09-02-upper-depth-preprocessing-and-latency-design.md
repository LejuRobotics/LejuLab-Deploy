# 上位机深度图预处理与端到端延迟设计

## 目标

把 Gemini 330 原始深度图的白点剔除、空洞修复、缩放、模糊和归一化全部迁移到上位机。上位机只向下位机发送 `64×36` 的策略单帧；下位机不再修改像素，只负责校验、历史帧缓存和策略输入组装。同时保留相机采集时间戳，在下位机测量“相机采集到 DDS 接收”的端到端延迟。

## 已确认的模型与参考实现

`kuavo-ros-control/src/humanoid-control/humanoid_controllers/config/kuavo_v54/rl/depth_loco_param.info` 定义单帧深度历史输入为：

- `8 × 36 × 64 = 18432` 个元素；
- 策略直接使用 `[0, 1]` 的归一化深度。

像素处理的参考实现不是该 `.info` 文件，而是 `kuavo-ros-control/src/image_processing/launch/depth_inpainter.launch` 和 `scripts/depth_inpainter.py`。新链路必须保持以下顺序和参数：

1. `16UC1` 毫米深度转为 `float32` 米；
2. 原图按 `0.5` 倍、`INTER_NEAREST` 缩放；
3. 两轮白色噪点剔除：
   - `(depth_diff=0.25, white_threshold=2.5, max_spot_size=1024, surrounding_black_ratio=0.5)`；
   - `(depth_diff=0.5, white_threshold=1.0, max_spot_size=1024, surrounding_black_ratio=0.5)`；
4. 为零深度生成 mask，并用 `2×2` kernel 膨胀一次；
5. 深度图和 mask 均用 `INTER_NEAREST` 缩放到 `64×36`；
6. 使用 OpenCV Navier–Stokes inpaint，半径为 `2`；
7. 使用 `3×3` Gaussian blur，`sigmaX=1`、`sigmaY=1`；
8. clip 到 `[0, 2.5 m]`，再除以 `2.5`，得到 `[0, 1]` 的 `float32` 图像。

旧脚本的 `resize_scale=0.151` 只会把 `424×240` 输入精确变成 `64×36`。Gemini 330 当前原生输出为 `480×270`，所以新处理器保留前置 0.5 倍缩放，但最终明确输出 `64×36`。这是满足模型 18432 维契约所必需的分辨率适配。

## 方案选择

采用方案 A：复用 `kuavo_ros_application/depth_image_recorder` 中已经存在并经过测试的 `depth_processor_node.py`，再由 `leju_head_lab/ros_dds_bridge` 转发处理后的 ROS Image。

不把算法复制到 DDS bridge，避免两个实现日后漂移；暂不新增 Float32 DDS IDL，避免重新生成和同步上下位机消息代码。DDS 使用已有 `leju::msgs::Float64Array`，单帧数组为 2304 个 double。

## 架构与数据流

```text
Gemini 330
  /camera/depth/image_raw
  480×270（兼容 424×240），16UC1，毫米，携带采集时间戳
      │
      ▼
kuavo_ros_application/depth_processor_node.py
  参考 depth_inpainter 的完整像素处理
      │
      ▼
  /camera/depth/image_processed
  64×36，32FC1，[0,1]，原样保留采集时间戳
      │
      ▼
leju_head_lab/depth_camera_bridge.py
  只做 ROS Image → DDS Float64Array 序列化
      │
      ▼
  rt/depth_camera/frame_normalized_36x64
  Float64Array.header = 相机采集时间戳
  Float64Array.data = 2304 个归一化像素
      │
      ▼
lejulab_platform/DepthObservationProvider
  校验消息和记录端到端延迟，不做像素变换
      │
      ▼
  真机 43 帧缓存 → 选择 [0,6,12,18,24,30,36,42]
      │
      ▼
  8×36×64 策略深度观测
```

原始 `480×270 UInt16Array` 的像素载荷约为 259200 字节；新 `64×36 Float64Array` 的像素载荷约为 18432 字节，减少约 92.9%。DDS/CDR 元数据不改变这个量级结论。

## 话题和兼容性

- 真机新话题：`rt/depth_camera/frame_normalized_36x64`，类型为 `Float64Array`，值域 `[0,1]`。
- MuJoCo 旧话题：`rt/depth_camera/frame_meters_36x64`，类型仍为 `Float64Array`，单位仍为米。
- 原始真机话题 `rt/depth_camera/frame_u16_mm_270x480` 保留为旧配置兼容路径，但新的 S17 depth-walk 配置不再订阅它。
- 新真机话题和 MuJoCo 话题名称不同，下位机可明确选择“已处理真机”或“仿真”路径，不能仅凭数组长度猜测来源。

## 上位机改动

### `kuavo_ros_application`

- 保留 `depth_image_recorder.processor.process_depth_mm` 作为唯一像素算法实现。
- 新增不启动 rosbag 的独立 launch，只运行 raw → processed 节点，默认输出 `64×36 / 32FC1 / [0,1]`。
- 补充 480×270 输入的参考对齐测试、输出形状/类型/时间戳测试，以及非法 encoding/分辨率测试。

### `leju_head_lab`

- 扩展 `depth_camera_bridge.py`，在保留 `16UC1 → UInt16Array/Float64Array` 兼容能力的同时支持 `32FC1 → Float64Array`。
- 解析时严格遵守 `width`、`height`、`step` 和 `is_bigendian`；拒绝截断数据、非法 encoding 和形状不匹配。
- `Float64Array.header_sec/header_nanosec` 原样使用 ROS Image 的相机采集时间戳。
- 新增 processed-depth launch，默认订阅 `/camera/depth/image_processed` 并发布新 DDS 话题。
- 上位机每秒记录一次 `当前系统时间 - 相机采集时间`，作为“采集到上位机处理并进入 DDS bridge”的耗时。

## 下位机改动

### `lejulab_platform`

- `DepthDdsInputConfig` 增加明确的 `preprocessed` 配置，不用尺寸隐式判断。
- 新真机回调把 DDS header 传入，而不是丢弃 header 后用接收时刻冒充采集时刻。
- 接收时保存两个时间概念：
  - `steady_clock` 接收时间：只用于本机 freshness/timeout；
  - DDS header 的 epoch 时间：只用于跨机端到端延迟统计。
- 已处理帧只做契约校验：长度必须为 2304，所有值必须有限且位于 `[0,1]`；校验通过后将 double 转为内部 float 并直接写真机历史缓存。
- 已处理帧不调用 `DepthImageProcessor`，不进行二次 resize、inpaint、blur、clip 或 normalize。
- 真机继续使用 43 帧缓存和 `[0,6,12,18,24,30,36,42]`；MuJoCo 继续使用 22 帧缓存和 `[0,3,6,9,12,15,18,21]`。
- 更新 S17 `config_depth_walk.yaml`，将真机输入切换到新话题和 `preprocessed: true`。

## 时间戳与延迟

端到端延迟定义为：

```text
下位机 DDS 回调的 system_clock 时间 - ROS Image.header.stamp
```

该值包含相机驱动排队、上位机像素处理、ROS 内部传递、DDS 序列化、网络传输和下位机 DDS 反序列化。上下位机必须先用 chrony 同步。

控制器每秒输出一次当前延迟，并维护可读的接收帧数、拒绝帧数和异常时间戳计数。时间戳为零、纳秒字段非法、延迟显著为负或超出合理上限时发出节流警告；这类异常只影响延迟统计，不改变基于 `steady_clock` 的安全超时判断。

## 错误处理

- 上位机原图 encoding 或尺寸错误：拒绝该帧并节流记录错误。
- 上位机处理结果不是 `64×36 / 32FC1`：DDS bridge 拒绝该帧。
- DDS data 长度不是 2304：下位机拒绝该帧。
- DDS data 含 NaN、Inf 或超出 `[0,1]`：下位机拒绝整帧，不对坏值静默修补。
- 没有有效帧或帧超时：沿用控制器现有 depth-not-ready 行为，不切换或继续运行深度策略。
- chrony 未同步：延迟日志明确提示可能的时钟偏差；控制器 freshness 仍然只看本机单调时钟。

## 测试与验收

### 自动测试

1. `kuavo_ros_application`：424×240 和 480×270 合成深度图经过新上位机节点后，与参考 `depth_inpainter` 流程在容差内一致。
2. `kuavo_ros_application`：输出固定为 `64×36 / float32 / [0,1]`，并保留输入 header。
3. `leju_head_lab`：测试 32FC1 的紧凑行、带 padding 行、大小端、截断数据和错误 encoding。
4. `leju_head_lab`：测试 DDS 消息长度 2304，header 与输入相同。
5. `lejulab_platform`：测试 preprocessed 帧进入历史缓存前后像素不变。
6. `lejulab_platform`：测试 preprocessed 真机路径使用 43 帧历史选择，MuJoCo 路径保持旧行为。
7. `lejulab_platform`：测试 header 转秒、端到端延迟、零时间戳、未来时间戳和本地 timeout 相互独立。

### 实机验收

- 上位机 `/camera/depth/image_raw` 接近 60 Hz。
- 上位机 `/camera/depth/image_processed` 接近 60 Hz，格式为 `64×36 / 32FC1`。
- DDS 新话题接近 60 Hz，payload 长度恒为 2304。
- 下位机 depth history ready，策略输入为 18432 个深度元素。
- 对同一录制帧，上位机输出与原 `kuavo-ros-control/depth_inpainter.py` 输出在既定浮点容差内一致。
- 下位机日志可稳定显示端到端延迟；chrony 正常时不出现持续负值或异常跳变。
- 旧 MuJoCo 深度话题及历史行为通过回归测试。

## 部署文档

更新 `/home/lejurl/Downloads/Roban上楼梯部署教程（深度相机版-3588版）.md`：

- 启动顺序改为相机 → 上位机处理器 → processed DDS bridge → 下位机控制器；
- 删除“下位机启动原始深度处理”的旧步骤；
- 增加 ROS 与 DDS 的尺寸、encoding、频率和 payload 检查；
- 增加 chrony 前置条件、上下位机延迟日志说明和异常判断；
- 标明新旧话题、单位和值域，防止把米制仿真帧与归一化真机帧混用。

## 非目标

- 不修改模型结构或 8 帧历史排列。
- 不新增 Float32 DDS IDL。
- 不压缩深度图或引入有损图像编码。
- 不移除原始真机 DDS 兼容代码。
- 不用跨机系统时间控制安全 timeout。
