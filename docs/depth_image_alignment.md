# 深度图像对齐基准

## 源端最终策略输入

源端 `DepthWalkController.cpp` 的 `singleInputData` 顺序（`depth_loco_param.info:501`）为：

```text
depth_image    18432  (8 × 36 × 64)
command            3
commandPhase       5  (sin, cos, leg_bias, stance_ratio, gait_frequency)
bodyAngVel         3
gravity_body       3
jointPos          21  (waist-first)
jointVel          21  (waist-first)
action            21  (waist-first)
frePhase           1  (模型输出的原始 gait 维度)
总计            18510
```

`frameStack=5`，网络输入为 `92550`，模型输出为 25：21 action、1 gait frequency、3 velocity estimate。

## 源端深度处理

1. 原始 MuJoCo 射线帧为 64×36；源端 `mujoco_node.cc:117-125` 使用 `focal=2.12`、水平 aperture=4.24、垂直 aperture=2.448、范围 0.17–2.5 m。
2. `mujoco_node.cc:1943-1959` 将 ray distance 转为 z-depth；无命中保持 0，随后 clip 到 `[0,2.5]` 并除以 2.5。
3. `depth_inpainter.py:142-149` 的真实相机输入先从毫米转米，并用 nearest resize 到 64×36；裁剪默认全图。
4. `depth_inpainter.py:114-124,232` 对零值建立 mask，执行 Telea inpaint，半径默认 3。
5. `depth_inpainter.py:266-270` 进行 3×3 Gaussian（sigma x/y=1），再 clip `[0,2.5]` 和 `/2.5`。
6. `depth_inpainter.py:27-30,281-292` 维护 43 帧时间正序缓冲，输出 `[0,6,12,18,24,30,36,42]`；首次帧复制填满全部历史。

## 目标端当前对照

- DDS topic 使用 `rt/depth_camera/frame_meters_36x64`，消息单位为米。
- MuJoCo 射线尺寸和 FOV 等价；核对源端 `RayCaster::compute_ray()` 后确认无命中会被设为最大量程，目标端当前 `2.5m` 语义一致；`0` 主要用于噪声/空洞，不能简单把所有无命中改成 0。
- 目标端历史选择顺序已用编号帧测试为 `0,6,12,18,24,30,36,42`。
- 目标端 phase/frePhase 已按源端修正；仍需补充原始帧、处理帧、历史帧和 18510 维观测快照。

## 验收数据

使用 `leju-mujoco-sim --headless --dump-depth` 记录原始射线帧和处理帧的 min/max/mean/zero ratio；只有当这些统计和源端定义一致，且首个策略动作不产生异常 q_target，才继续进行上楼梯/下楼梯验收。
