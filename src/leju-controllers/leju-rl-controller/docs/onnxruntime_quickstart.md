# ONNX Runtime 推理引擎使用说明

## 编译项目

```bash
cd ~/lejulab_platform
catkin build 
```

## 切换推理引擎

只需在配置文件中添加或修改 `inference_engine` 字段：

```yaml
HumanoidRobotCfg:
  inference_engine: "onnxruntime"  # 或 "openvino"
  loop_dt: 0.001
  policy_path: "policy/xxx.onnx"
  # ...
```

## 支持的推理引擎

| 引擎 | 配置值 | 说明 |
|------|---------|------|
| ONNX Runtime | `onnxruntime` | 跨平台，支持 GPU |
| OpenVINO | `openvino` | Intel CPU 优化（默认） |

## 配置文件列表

所有配置文件都已添加 `inference_engine` 字段：

| 配置文件 | 推理引擎 |
|----------|----------|
| config/14/config_amp.yaml | onnxruntime |
| config/14/config_mimic_prone.yaml | onnxruntime |
| config/14/config_mimic_charleston_dance.yaml | onnxruntime |
| config/14/config_mimic_HPNY_dance.yaml | onnxruntime |
| config/46/config_amp.yaml | openvino |
| config/46/config.yaml | openvino |
| config/52/config_amp.yaml | openvino |

如需切换引擎，直接修改对应配置文件的 `inference_engine` 值即可。
