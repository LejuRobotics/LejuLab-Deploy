# 深度相机 DDS 调试工具

本目录提供一组 Python 脚本，用于在 **launch 运行期间** 订阅深度相机 DDS 话题，检查发布频率、原始深度画面，以及 `depth_walk` 控制器实际送入网络的 **64×36** 归一化深度。

所有 `.sh` 包装脚本会自动配置 CycloneDDS（`CYCLONEDDS_URI`、`LD_LIBRARY_PATH`），用法与 `dds_topic_hz.sh` 一致。

## 前置条件

在**上位机或下位机**（与 `run_rl_controller` 相同 DDS 域）执行：

```bash
pip3 install cyclonedds==0.10.2 opencv-python numpy
```

**必须先启动** `launch_real.sh`（或等价 launch），且深度相机正在发布，再运行下列工具。

默认话题（Roban / Gemini 330 真机，上位机处理后的控制器输入）：

```text
rt/depth_camera/frame_normalized_36x64
```

消息类型：`Float64Array`，长度 `64 × 36 = 2304`（单位：米，已由上位机归一化）。

MuJoCo 仿真 legacy 深度话题（可选，与真机默认不同）：

```text
rt/depth_camera/frame_meters_36x64
```

长度 `64 × 36 = 2304`（单位：米，已处理）。

---

## 工具一览

| 脚本 | 作用 | 典型场景 |
|------|------|----------|
| `dds_topic_hz.sh` | 统计话题发布频率与 payload 长度 | 确认深度在发、Hz 是否正常 |
| `dds_depth_to_video.sh` | 原始 **480×270 mm** 深度录屏/预览 | 看相机原始画面是否正常 |
| `dds_depth_policy_input_video.sh` | 经控制器同款处理后 **64×36** 网络输入录屏/预览 | 看策略实际“看到”的深度 |
| `dds_depth_jet_compare.sh` | **Jet 彩虹云图**左右对比 raw vs 64×36 | 直观对比原始与网络输入 |
| `dds_depth_pointcloud_compare.sh` | **点云**左右对比 raw vs 64×36 | 3D 几何分布对比 |
| `depth_processor.py` | 处理逻辑库 | 一般无需直接调用 |
| `depth_viz.py` | 伪彩色 / 色条可视化库 | 被 jet compare 脚本 import |
| `depth_pointcloud.py` | 深度反投影 / 点云渲染库 | 被 pointcloud compare import |

---

## 1. 话题频率：`dds_topic_hz.sh`

等价于 ROS 的 `rostopic hz`。

```bash
# 真机默认话题（上位机已处理，Float64Array，2304 个元素）
bash src/leju_launch/scripts/dds_topic_hz.sh

# 指定话题
bash src/leju_launch/scripts/dds_topic_hz.sh rt/depth_camera/frame_normalized_36x64

# MuJoCo legacy 64×36
bash src/leju_launch/scripts/dds_topic_hz.sh rt/depth_camera/frame_meters_36x64 --expected-len 2304

# 只跑 10 秒后退出
bash src/leju_launch/scripts/dds_topic_hz.sh --duration 10
```

**输出示例：**

```text
average rate: 59.8 Hz  (total=299, elapsed=5.0s, last_len=2304)
```

- `last_len=2304`：每帧长度正确
- 频率约 **48–60 Hz** 为正常（与相机/launch 配置有关）

Ctrl+C 结束并打印 summary。

| 参数 | 说明 |
|------|------|
| `topic` | DDS 话题名（可选，有默认值） |
| `--expected-len N` | 期望 `data` 长度，`0` 表示不检查 |
| `--type auto\|float64\|uint16` | DDS payload 类型；`auto` 按话题名选择（归一化/仿真 64×36 用 `Float64Array`，原始真机话题用 `UInt16Array`） |
| `--duration SEC` | 运行秒数，`0` 表示一直跑 |
| `--report-interval SEC` | 打印间隔（默认 1s） |

---

## 2. 原始深度视频：`dds_depth_to_video.sh`

订阅 **480×270 毫米** 原始深度，伪彩色显示或录制 MP4。

```bash
# 本地弹窗预览（按 q 退出）
bash src/leju_launch/scripts/dds_depth_to_video.sh

# SSH 无界面：录 10 秒
bash src/leju_launch/scripts/dds_depth_to_video.sh \
  --no-show -o /tmp/depth_raw.mp4 --duration 10

# 右下角叠加简易 64×36 缩略图（仅 nearest，不含 inpaint/blur）
bash src/leju_launch/scripts/dds_depth_to_video.sh --match-controller -o /tmp/depth_raw.mp4
```

**画面说明：**

- 伪彩色（默认 `turbo`）：近处偏蓝/绿，远处偏红  
- 无效/零深度为黑色  
- 深度 clip 至 **2500 mm**（与控制器 `max_depth_m: 2.5` 一致）  
- 顶部文字：`valid=` 有效像素比例、`depth=[min,max]mm`、帧率  

| 参数 | 说明 |
|------|------|
| `-o`, `--output PATH` | 输出 MP4 路径 |
| `--duration SEC` | 录制时长 |
| `--no-show` | 不弹窗（SSH 录屏用） |
| `--fps 30` | 输出视频帧率 |
| `--colormap gray\|turbo\|inferno` | 配色 |
| `--max-depth-mm 2500` | 显示 clip 上限 |
| `--match-controller` | 右下角叠加 64×36 简易预览 |
| `--scale N` | 缩略图放大倍数（配合 `--match-controller`） |
| `--width` / `--height` | 输入分辨率（默认 480×270） |

---

## 3. 网络输入 64×36 视频：`dds_depth_policy_input_video.sh`

该工具订阅原始真机诊断话题 `rt/depth_camera/frame_u16_mm_270x480`，在本地按
**`depth_walk` 控制器旧版 `DepthImageProcessor` 真机路径** 处理；它不是当前下位机的
控制器输入（控制器输入是上位机发布的 `frame_normalized_36x64`）：

```text
480×270 mm
  → 转为 float32 米制，nearest 缩放到 240×135
  → 两轮连通域白点过滤（与 depth_inpainter.launch 参数一致）
  → 零值 mask 做 2×2 膨胀
  → depth 与 mask 分别 nearest 缩放到 64×36
  → 8-bit 量化后执行 OpenCV Navier-Stokes inpaint（radius=2）
  → 恢复原有效深度，再做 3×3、sigma=1 Gaussian 模糊
  → clip [0, 2.5] m、除以 2.5
  → 归一化 float [0, 1]（即网络单帧深度输入）
```

```bash
# 预览 64×36 网络输入（默认放大 10 倍显示为 640×360）
bash src/leju_launch/scripts/dds_depth_policy_input_video.sh

# 录 15 秒
bash src/leju_launch/scripts/dds_depth_policy_input_video.sh \
  --no-show -o /tmp/policy_depth_64x36.mp4 --duration 15

# 左右对比：原始 480×270 | 处理后 64×36
bash src/leju_launch/scripts/dds_depth_policy_input_video.sh \
  --side-by-side --no-show -o /home/lab/zqb/lejulab_platform/depth_compare0826_11.mp4 --duration 15
```

**说明：**

- 显示的是 **单帧** 64×36 归一化深度（网络观测 18432 维中的 1 帧），**不含** 8 帧 history stack 与 5 帧 frame stack  
- 该图由工具订阅原始 DDS 帧后在本地执行控制器同款算法得到；控制器没有额外发布处理后图像话题
- 底部文字：`valid=`、`min/max/mean`（归一化值）、帧率  
- 若 `valid` 长期 **< 5%**，控制器也会判定 depth not ready（见 `config_depth_walk.yaml` 中 `min_valid_ratio: 0.05`）

| 参数 | 说明 |
|------|------|
| `-o`, `--output PATH` | 输出 MP4（内容为放大后的预览帧） |
| `--duration SEC` | 录制时长 |
| `--no-show` | 不弹窗 |
| `--display-scale 10` | 64×36 放大倍数（默认 10） |
| `--side-by-side` | 左 raw、右 policy 64×36 |
| `--no-blur` | 关闭 Gaussian（对比调试用） |
| `--max-depth-m 2.5` | 与控制器一致 |
| `--colormap gray\|turbo\|inferno` | 配色 |
| `--fps 30` | 输出帧率 |
| `--input-width` / `--input-height` | 原始 DDS 分辨率 |

处理逻辑实现在同目录 `depth_processor.py`，与 C++ `depth_image_processor.cpp` 的 480×270 mm 真机分支及 `gaussian_blur=true` 对齐，同时保留 424×240 兼容输入。`dds_depth_policy_input_video.sh`、`dds_depth_jet_compare.sh` 和 `dds_depth_pointcloud_compare.sh` 共用这一实现。

---

## 4. Jet 彩虹云图对比：`dds_depth_jet_compare.sh`

左：**原始 480×270 mm**；右：**本地执行控制器同款流程后的 64×36**；最右侧可选 **深度色条**（0–2.5 m）。
默认使用 OpenCV **JET** 配色（经典彩虹“云图”），便于肉眼对比两路深度分布。

```bash
# 本地预览
bash src/leju_launch/scripts/dds_depth_jet_compare.sh

# 录对比视频（推荐）
bash src/leju_launch/scripts/dds_depth_jet_compare.sh \
  --no-show -o docs/depth_compare.mp4 --duration 15

# 灰度显示
bash src/leju_launch/scripts/dds_depth_jet_compare.sh \
  --colormap gray

# 换 turbo / inferno 配色
bash src/leju_launch/scripts/dds_depth_jet_compare.sh \
  --colormap turbo -o /tmp/depth_compare_turbo.mp4 --duration 10 --no-show
```

| 参数 | 说明 |
|------|------|
| `-o`, `--output PATH` | 输出 MP4 |
| `--colormap gray\|jet\|turbo\|inferno` | 配色（默认 **jet**，`gray` 为灰度） |
| `--panel-height 360` | 每路面板高度 |
| `--policy-scale 10` | 64×36 放大倍数 |
| `--no-colorbar` | 隐藏右侧色条 |
| `--no-blur` | policy 路径关闭 Gaussian |
| `--duration` / `--no-show` / `--fps` | 同其他脚本 |

---

## 5. 点云对比：`dds_depth_pointcloud_compare.sh`

将 **原始深度** 与 **policy 64×36** 反投影为相机坐标系点云（针孔模型），左右渲染对比并录屏。

- **左**：480×270 原始深度 subsample 点云
- **右**：本地执行控制器同款流程后的 64×36 点云
- 点颜色：默认 Jet 伪彩色；可用 `--colormap gray` 显示深度灰度
- 坐标系：相机光学系 **X 右、Y 下、Z 前**

```bash
# 默认鸟瞰 BEV（X-Z，前方向上）
bash src/leju_launch/scripts/dds_depth_pointcloud_compare.sh

# 灰度点云
bash src/leju_launch/scripts/dds_depth_pointcloud_compare.sh --colormap gray

# 录 15 秒对比视频
bash src/leju_launch/scripts/dds_depth_pointcloud_compare.sh \
  --no-show -o docs/depth_pcd_compare.mp4 --duration 15

# 每面板上下叠 BEV + 侧视 Y-Z
bash src/leju_launch/scripts/dds_depth_pointcloud_compare.sh --view both --no-show -o pcd_both.mp4

# 侧视 Y-Z
bash src/leju_launch/scripts/dds_depth_pointcloud_compare.sh --view side
```

| 参数 | 说明 |
|------|------|
| `--view bev\|side\|both` | 渲染视角（默认 **bev**） |
| `--colormap gray\|jet\|turbo\|inferno` | 点深度配色（默认 **jet**） |
| `--hfov-deg 87` | 水平 FOV，用于估算 fx/fy（无标定时近似） |
| `--fx` / `--fy` | 手动指定内参（像素），优先于 FOV |
| `--raw-step 3` | 原始图 subsample 步长（越小点越密、越慢） |
| `--point-radius 2` | 点绘制半径 |
| `--panel-width/height` | 单面板尺寸 |
| `-o` / `--duration` / `--no-show` | 同其他脚本 |

**说明：** 控制器本身不做点云反投影；本工具仅用于调试可视化。内参默认由 FOV 估算，若与 Gemini 330 标定差较大，可用 `--fx --fy` 调整。

---

## 推荐排查流程（depth_walk 真机）

按顺序执行，便于定位问题在「没发 / 原始图不对 / 处理后不对 / 控制器没订阅」：

```bash
# ① 控制器输入话题是否在发（上位机处理后）
bash src/leju_launch/scripts/dds_topic_hz.sh

# 原始真机诊断话题（仅供本地处理工具）
bash src/leju_launch/scripts/dds_topic_hz.sh rt/depth_camera/frame_u16_mm_270x480

# ② 原始深度是否正常（地面、障碍、valid 比例）
bash src/leju_launch/scripts/dds_depth_to_video.sh \
  --no-show -o /tmp/depth_raw.mp4 --duration 10

# ③ 网络看到的 64×36 是否正常
bash src/leju_launch/scripts/dds_depth_policy_input_video.sh \
  --side-by-side --no-show -o /tmp/depth_policy.mp4 --duration 10

# ④ Jet 云图左右对比（最直观）
bash src/leju_launch/scripts/dds_depth_jet_compare.sh \
  --no-show -o docs/depth_compare.mp4 --duration 15

# ⑤ 点云几何对比
bash src/leju_launch/scripts/dds_depth_pointcloud_compare.sh \
  --no-show -o docs/depth_pcd_compare.mp4 --duration 15
```

将 MP4 拷到开发机查看，或在本机直接 `--no-show` 省略弹窗。

---

## 文件结构

```text
src/leju_launch/scripts/
├── dds_leju_types.py              # DDS UInt16Array / Float64Array 类型定义
├── dds_topic_hz.sh / .py          # 话题频率
├── dds_depth_to_video.sh / .py    # 原始 480×270 视频
├── depth_processor.py             # 64×36 处理（与控制器一致）
├── dds_depth_policy_input_video.sh / .py  # 网络输入 64×36 视频
├── depth_viz.py                   # Jet/色条可视化
├── dds_depth_jet_compare.sh / .py # Jet 云图 raw vs policy 对比
├── depth_pointcloud.py            # 点云反投影与渲染
├── dds_depth_pointcloud_compare.sh / .py  # 点云对比视频
└── README_depth_dds_tools.md      # 本文档
```

---

## 常见问题

**Q: 脚本能收到 Hz，但控制器 log 没有 `Depth diagnostic`？**  
A: 确认 `run_rl_controller` 与脚本使用相同 `CYCLONEDDS_URI`；并确认已修复 depth DDS 订阅初始化顺序（须先 `loadConfig` 再订阅真机话题）。可对比本目录 `dds_depth_policy_input_video` 的 `valid` 与控制器 `min_valid_ratio`。

**Q: SSH 无显示器，`imshow` 报错？**  
A: 加 `--no-show`，仅用 `-o` 录 MP4。

**Q: 仿真深度话题？**  
A: `dds_topic_hz.sh rt/depth_camera/frame_meters_36x64 --expected-len 2304`；原始 64×36 可直接用 `dds_depth_to_video.sh` 并改 `--width 64 --height 36`（单位为米时需自行理解 colormap，仿真通常已归一化）。

**Q: 与训练时 8 帧历史一致吗？**  
A: 本工具只可视化 **当前帧** 经处理后的 64×36。历史栈（43 帧缓存取 8 帧）在控制器 `DepthHistoryBuffer` 内完成，不在这些脚本中复现。

### 工具链路选择

四个工具默认订阅 rt/depth_camera/frame_normalized_36x64 的 Float64Array（64x36、[0,1]），下位机不重复填洞或 resize。旧原始话题可用 --type uint16 并指定 --input-width 480 --input-height 270。所有工具支持 --type auto|float64|uint16。
