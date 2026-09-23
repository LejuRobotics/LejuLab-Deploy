# leju-audio

机器人音频包,含两条对称链路:
- **采集侧**(`micphone_to_dds_node`):麦克风 → DDS 发布。从 NX Python/ROS `micphone_receiver_node.py` 迁移到 RK3588 C++/CycloneDDS。
- **播放侧**(`audio_player_node`):RPC/DDS 触发 → 扬声器播放。复刻 dev `kuavo_audio_player`(ROS1)到 RK3588 C++/CycloneDDS。详见 [§ 播放侧](#播放侧audio_player_node)。

全链路约定 **16k / 单声道 / S16LE**。

## 架构

```
USB 麦克风 → ALSA plughw → MicCapture → micphone_to_dds_node → DDS /rt/micphone_data
                                                              (AudioReceiverData{sequence<octet>})
```

- **采集**: `MicCapture`（ALSA plughw,按设备名打开,非卡号）。自动重采样到目标采样率(48k→16k 由 plughw 完成)。
- **发布**: `micphone_to_dds_node` 把 PCM 字节流(S16LE)封进 `AudioReceiverData.data` 发到 `/rt/micphone_data`。
- **下游**: NX 侧 `leju_head_lab` 仓的 `micphone_bridge.py` 订阅后桥到 ROS `/micphone_data`,供 `voice_control_node`(FunASR VAD+ASR)消费。完整跨机联调见 [§ 跨机联调](#跨机联调rk3588-麦克风--nx-语音识别)。

## 组件

| 目标 | 说明 |
|---|---|
| `leju_audio_capture` (static lib) | `MicCapture` 封装,不依赖 DDS |
| `micphone_to_dds_node` (可执行) | 采集 + DDS 发布节点 |
| `mic_capture_test` (单测) | 不依赖真机,验证默认参数 + 设备发现逻辑 |

## 配置(环境变量)

| 变量 | 默认 | 说明 |
|---|---|---|
| `MIC_PCM_DEVICE` | `plughw:CARD=Device,DEV=0` | ALSA 设备名(禁止卡号) |
| `MIC_KEYWORD` | `USB Composite Device` | 设备名匹配关键词 |
| `MIC_SAMPLE_RATE` | `16000` | 目标采样率 |
| `MIC_CHANNELS` | `1` | 声道数 |
| `MIC_PERIOD_FRAMES` | `512` | ALSA period 帧数 |

## 运行

### 随系统启动(实物)

`launch_real.sh` 已编排(可选节点 `launch_node`,非 `launch_required_node`):
- 无 USB 麦克风时节点自行退出(return 1),不影响主控制链路。
- 仿真脚本未编排(仿真用于控制测试,无音频需求)。

### 手动运行

```bash
# 编译
cmake --build build_cmake --target micphone_to_dds_node -j$(nproc)

# 默认参数
./build_cmake/src/leju-audio/micphone_to_dds_node

# 指定设备
MIC_PCM_DEVICE="plughw:CARD=Device,DEV=0" \
MIC_KEYWORD="USB Composite Device" \
./build_cmake/src/leju-audio/micphone_to_dds_node
```

### 录制

`config/recorder.yaml` 已加入 `/rt/micphone_data`,MCAP 录制会包含音频流。

## 播放侧（audio_player_node）

单节点双输入:RPC 按文件名播放 + 订阅 DDS PCM 流播放,汇入同一有界队列由播放线程写 ALSA 输出。

```
  RPC play_file(file,volume) ─► ffmpeg 解码(→16k/mono/S16LE) ─┐
                                                              ├─► [PCM 有界队列] ─► 播放线程 ─► ALSA plughw 输出
  DDS /rt/audio_play (AudioReceiverData PCM) ──────────────────┘                          (声卡做最终重采样)
  DDS /rt/audio_stop (StringData, 空 data 即停) ─► 清空队列
```

> 双节点拆分是 dev 参考(ROS)的产物;本仓合并为单节点,本地文件播放免去 topic 往返。

### 组件

| 目标 | 说明 |
|---|---|
| `leju_audio_playback` (static lib) | `SpeakerOutput`(ALSA 播放封装)+ `AudioDecoder`(ffmpeg 解码),不依赖 DDS |
| `audio_player_node` (可执行) | RPC replier(play_file) + DDS 订阅(PCM 流 + stop) + 播放线程 |
| `play_audio_cli` (可执行) | 命令行调试工具,发 RPC 播放请求 |
| `audio_stream_probe` (诊断) | 合成正弦 PCM 推 `/rt/audio_play` + 可选发 stop,验证 DDS 流输入路径(无需音频文件) |
| `speaker_output_test` (单测) | 不依赖真机,验证默认参数 + 播放设备发现逻辑 |

### 控制接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `AudioPlayerService` / `play_file` | DDS-RPC | 请求 `{file_name, volume}` → 应答 `{success, message}`。`file_name` 裸名(相对 `AUDIO_FILE_DIR`)或绝对路径;`volume` 100=原音量 |
| `/rt/audio_play` | DDS topic (`AudioReceiverData`) | 外部推送 PCM 流(S16LE/16k/mono),如上位机 TTS 下发 |
| `/rt/audio_stop` | DDS topic (`StringData`) | 任意消息即停,清空队列 |

### 配置（环境变量）

| 变量 | 默认 | 说明 |
|---|---|---|
| `SPK_PCM_DEVICE` | `plughw:CARD=Device_1,DEV=0` | ALSA 播放设备名(禁止卡号) |
| `SPK_KEYWORD` | `USB Audio` | 播放设备匹配关键词;找不到则节点退出 |
| `AUDIO_FILE_DIR` | `~/.config/lejuconfig/music` | 裸文件名解析目录 |
| `SPK_SAMPLE_RATE` | `16000` | 目标采样率 |
| `SPK_CHANNELS` | `1` | 声道数 |
| `SPK_PERIOD_FRAMES` | `512` | ALSA period 帧数 |
| `AUDIO_QUEUE_MAX` | `500` | PCM 队列最大块数(满丢最旧) |

> **播放设备 ≠ 麦克风卡**。USB 麦(`CARD=Device`)只采集;扬声器是另一张卡(实测板上 card2 C-Media `CARD=Device_1`,或板载 `rockchipes8316c`)。`aplay -l` 看可用播放卡。
> **root 下无 PulseAudio**:节点随 `launch_real.sh` 以 sudo 运行,必须直开 plughw 硬件设备,不能用 `default`。

### 运行

```bash
# 编译
cmake --build build_cmake --target audio_player_node play_audio_cli -j$(nproc)

# 准备音频目录与文件
mkdir -p ~/.config/lejuconfig/music
cp some.wav ~/.config/lejuconfig/music/

# 启动播放节点(默认参数)
./build_cmake/src/leju-audio/audio_player_node

# 另开终端,命令行触发播放
./build_cmake/src/leju-audio/play_audio_cli some.wav        # 原音量
./build_cmake/src/leju-audio/play_audio_cli some.wav 50     # 半音量
./build_cmake/src/leju-audio/play_audio_cli /abs/path.mp3   # 绝对路径,任意格式

# 停止(发空 StringData 到 /rt/audio_stop)即可;或上位机推 PCM 到 /rt/audio_play

# 验证 DDS 流输入 + stop 路径(无需音频文件)
./build_cmake/src/leju-audio/audio_stream_probe 3                 # 推 3s 正弦
./build_cmake/src/leju-audio/audio_stream_probe 5 --stop-after 1500  # 推 5s 但 1.5s 后打断
```

随系统启动已编排进 `launch_real.sh`(可选节点 `launch_node`,无播放设备时自行退出 `return 1`,不影响主控制链路)。

## 依赖

- ALSA: `libasound2-dev`(aarch64 需 `sudo apt-get install -y libasound2-dev`)
- CycloneDDS-CXX(IDL + pubsub + dds-rpc)
- ffmpeg(**播放侧运行时**依赖,文件解码用;`sudo apt-get install -y ffmpeg`)
- 不依赖 Drake,不依赖 ROS

## aarch64(3588)说明

- CMake `find_package(ALSA REQUIRED)`,3588 上需装 `libasound2-dev`。
- CLAUDE.md 的 aarch64 自动禁用清单(EC-Master/Xsens/BMAPI/DexHand)不含 audio——audio 在 3588 上正常构建。
- 实测:RK3588 USB 麦克风(card1)采集 → DDS → NX voice 识别,全链路通(2026-06-23)。详见下节 [§ 跨机联调](#跨机联调rk3588-麦克风--nx-语音识别)。

## 下游接入

`/rt/micphone_data` 的下游消费者已在 NX 侧实现:`leju_head_lab` 仓的 `ros_dds_bridge/scripts/micphone_bridge.py` 把 DDS `AudioReceiverData` 桥到 ROS `/micphone_data`,供 NX 上的 `voice_control_node`(FunASR VAD+ASR)消费。完整跨机联调步骤见下节 [§ 跨机联调](#跨机联调rk3588-麦克风--nx-语音识别)。

如需在 RK3588 本机直接订阅(无 ROS):

```cpp
TopicSubscriber<AudioReceiverData> sub(participant, kAudioData);
// AudioReceiverData.data() → std::vector<uint8_t>，S16LE PCM
```

## 跨机联调(RK3588 麦克风 → NX 语音识别)

实测通过链路: **RK3588 USB 麦克风 → DDS /rt/micphone_data → NX micphone_bridge → ROS /micphone_data → voice_control_node (VAD+ASR)**。

完整复现指南在 NX 侧仓库: `leju_head_lab` 的 `docs/ros_dds_bridge_howto.md` §15。下面只列 RK3588 侧(本仓)的关键启动点。

### 1. 网络(关键陷阱)

DDS 必须走 **192.168.26.0/24 有线网段**。wifi (.50.x) 上 CycloneDDS multicast 发现不通,发布端与 NX 桥互相收不到。

- NX 桥固定绑 `nuccon` (.26.12)。
- RK3588 默认 `cyclonedds_shm.xml` 绑定 `192.168.26.1`；`setup_cyclonedds_config.sh` 部署时若检测不到则自动回退 `lo`。

**手动运行时**,需确保 `CYCLONEDDS_URI` 指向正确配置:

```bash
export CYCLONEDDS_URI=file://${LEJULAB_PROJECT}/src/leju_launch/config/cyclonedds_shm.xml
```

### 2. 声卡选择

`MIC_PCM_DEVICE` 决定**真正打开**哪个设备;`MIC_KEYWORD` 只是"现场有没有这种麦"的开工门禁(匹配卡名,找不到就退出)。两者默认对应同一张卡。查看可用卡:

```bash
cat /proc/asound/cards
#   0 [rockchipes8316c]: ... rockchip,es8316-codec       # 板载 codec (能采到噪声底, 不是真麦)
#   1 [Device        ]: USB-Audio - USB Composite Device  # USB 外置麦 (要用的)
```

USB 麦(默认,推荐):
```bash
export MIC_PCM_DEVICE=plughw:CARD=Device,DEV=0     # card1
export MIC_KEYWORD="USB Composite Device"
```
板载 codec(USB 麦坏时的临时替代,只采噪声底):
```bash
export MIC_PCM_DEVICE=plughw:CARD=rockchipes8316c,DEV=0   # card0
export MIC_KEYWORD=es8316
```

### 3. 启动 + 验证

```bash
cd /path/to/lejulab_platform_zx
./build_cmake/src/leju-audio/micphone_to_dds_node
# 预期: negotiated rate=16000 channels=1 ... (31.25 Hz, 1024 bytes/frame)
#       DDS topic: /rt/micphone_data, publisher ready
```

**话题有帧 ≠ 麦克风在收声**。ALSA 驱动无脑产帧,即使没插麦也照发(全 0 或 codec 噪声底)。验证有没有真声音:

```bash
# 物理层: 直接录一段看峰值
arecord -D plughw:CARD=Device,DEV=0 -f S16_LE -r 16000 -c 1 -d 5 /tmp/m.wav
python3 -c "import wave,struct; w=wave.open('/tmp/m.wav'); d=struct.unpack('<'+str(w.getnframes())+'h', w.readframes(w.getnframes())); print('peak=',max(abs(x) for x in d),'of 32768')"
# 说话时 peak 应冲到几千~上万; 全程 0 = 麦坏了, 换一只
```

NX 侧(在 NX 上跑,见 howto §15):
```bash
rostopic hz /micphone_data          # ≈ 31.25 Hz = 链路通
# voice 节点说话后打印: [ASR] 识别结果: '往前走'
```

### 4. 排障速查(本仓侧)

| 症状 | 解法 |
|---|---|
| 节点报 `no mic matching` 重试后退出 | 现场 ALSA 卡名不含 `MIC_KEYWORD`; 确认 `cat /proc/asound/cards` 有目标卡, 且 keyword 拼写对 |
| 话题有帧(31.25Hz)但 peak=0 | USB 麦硬件坏; `arecord` 直录验证, 换麦 |
| 帧率正常但 NX 收不到 | 发布端走了 wifi 网卡; 设 `CYCLONEDDS_URI` 绑 `192.168.26.1` 有线 |
| 话题有数据但 peak 只 ~300 | 读到了 card0 板载 codec 噪声底, 没读到 USB 真麦; `MIC_PCM_DEVICE` 指回 card1 |
