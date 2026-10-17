# Upper-host Depth Preprocessing and Latency Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Process Gemini 330 depth frames into normalized `64×36` policy images on the upper host, send only those images over DDS with the camera timestamp, consume them without pixel processing on the lower host, and report capture-to-receive latency.

**Architecture:** `kuavo_ros_application` remains the single owner of the depth pixel algorithm and publishes a timestamp-preserving ROS `32FC1` image. `leju_head_lab` serializes that image into the existing DDS `Float64Array` on a distinct normalized-frame topic. `lejulab_platform` recognizes the configured preprocessed source explicitly, validates without transforming pixels, records latency from the DDS header, and writes the frame into the real-camera 43-frame history.

**Tech Stack:** ROS Noetic (`rospy`, `sensor_msgs/Image`, `cv_bridge`), Python 3, NumPy/OpenCV, CycloneDDS Python/C++, C++17, GoogleTest, CMake/catkin, YAML.

---

## File map

### `kuavo_ros_application`

- Create `src/depth_image_recorder/launch/process_depth_images.launch`: standalone raw-to-policy-image launch without rosbag.
- Modify `src/depth_image_recorder/test/test_processor.py`: lock native 480×270 output to the reference algorithm.
- Modify `src/depth_image_recorder/test/test_recording_assets.py`: validate the standalone launch contract.

### `leju_head_lab`

- Create `src/ros_dds_bridge/src/ros_dds_bridge/depth_image_codec.py`: pure, unit-testable ROS Image byte decoder.
- Modify `src/ros_dds_bridge/scripts/depth_camera_bridge.py`: support `32FC1`, preserve camera timestamps, and log upper-host elapsed time.
- Create `src/ros_dds_bridge/launch/processed_depth_camera_bridge.launch`: processed ROS image to normalized DDS topic.
- Create `src/ros_dds_bridge/tests/test_depth_image_codec.py`: byte layout, endian, validation, and timestamp tests.
- Modify `src/ros_dds_bridge/CMakeLists.txt`: register Python tests.

### `lejulab_platform`

- Modify `src/leju-controllers/leju-rl-controller/include/leju-rl-controller/depth/depth_observation_provider.h`: add explicit preprocessed input and direct-submit/latency interfaces.
- Modify `src/leju-controllers/leju-rl-controller/src/depth/depth_observation_provider.cpp`: pass DDS headers, validate direct frames, keep local monotonic freshness, and report latency.
- Modify `src/leju-controllers/leju-rl-controller/src/controllers/depth_walk_controller.cpp`: parse/log `input_preprocessed`.
- Modify `src/leju-controllers/leju-rl-controller/config/17/controllers/config_depth_walk.yaml`: select the new normalized true-camera topic.
- Modify `src/leju-controllers/leju-rl-controller/tests/depth/test_depth_pipeline.cpp`: direct-pixel, real-history, invalid-data, latency, and timeout regression tests.
- Modify depth DDS diagnostic scripts/readme only where topic defaults or examples refer to the old raw true-camera topic.

### Deployment document

- Modify `/home/lejurl/Downloads/Roban上楼梯部署教程（深度相机版-3588版）.md`: commands, topics, payload, timestamps, and troubleshooting.

## Task 1: Lock the upper-host processor contract

**Files:**

- Modify: `kuavo_ros_application/src/depth_image_recorder/test/test_processor.py`
- Modify: `kuavo_ros_application/src/depth_image_recorder/test/test_ros_adapter.py`

- [ ] **Step 1: Add failing native-resolution and timestamp tests**

Add a `_kuavo_ros_native_reference()` helper that performs the same sequence as `_kuavo_ros_reference()` but starts with a `270×480` image and first resizes to `(240, 135)`. Add:

```python
def test_native_gemini_frame_matches_fixed_64_by_36_reference(self):
    rows, columns = np.indices((270, 480))
    source = (650 + rows * 3 + columns * 2).astype(np.uint16)
    source[80:125, 150:205] = 0
    source[170:178, 330:338] = 4300
    actual = process_depth_mm(source)
    expected = _kuavo_ros_native_reference(source)
    self.assertEqual(actual.shape, (36, 64))
    self.assertEqual(actual.dtype, np.float32)
    np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-6)
```

Extend `FakeBridge` to accept a configured array, then add to `test_ros_adapter.py`:

```python
def test_native_frame_is_fixed_64_by_36_and_preserves_capture_stamp(self):
    source = Image(encoding="16UC1", height=270, width=480)
    source.header.stamp.secs = 1234
    source.header.stamp.nsecs = 5678
    bridge = FakeBridge(np.full((270, 480), 1000, dtype=np.uint16))
    output = build_processed_message(source, bridge)
    self.assertEqual((output.height, output.width), (36, 64))
    self.assertEqual(output.header.stamp, source.header.stamp)
```

- [ ] **Step 2: Run the focused tests and confirm the fixture support fails**

```bash
cd /root/low/kuavo_ros_application
python3 -m unittest src/depth_image_recorder/test/test_processor.py src/depth_image_recorder/test/test_ros_adapter.py -v
```

Expected: FAIL because `FakeBridge` does not yet accept a native input array and `_kuavo_ros_native_reference` is absent.

- [ ] **Step 3: Add the minimal fixture/reference support**

Make `FakeBridge.__init__(self, input_array=None)` retain the supplied array and return it from `imgmsg_to_cv2`. Implement `_kuavo_ros_native_reference` with the exact fixed `(240,135) → (64,36)` pipeline and existing launch parameters; do not alter production processing code.

- [ ] **Step 4: Re-run the tests**

Run the command from Step 2. Expected: all unittest cases PASS.

- [ ] **Step 5: Commit**

```bash
git -C /root/low/kuavo_ros_application add src/depth_image_recorder/test/test_processor.py src/depth_image_recorder/test/test_ros_adapter.py
git -C /root/low/kuavo_ros_application commit -m "test: lock native Gemini depth preprocessing"
```

## Task 2: Add the standalone upper-host processor launch

**Files:**

- Create: `kuavo_ros_application/src/depth_image_recorder/launch/process_depth_images.launch`
- Modify: `kuavo_ros_application/src/depth_image_recorder/test/test_recording_assets.py`

- [ ] **Step 1: Write a failing launch-contract test**

Add:

```python
PROCESS_LAUNCH = os.path.join(PACKAGE_ROOT, "launch", "process_depth_images.launch")

def test_standalone_processor_launch_has_policy_contract(self):
    root = ET.parse(PROCESS_LAUNCH).getroot()
    node = root.find("node")
    self.assertEqual(node.attrib["pkg"], "depth_image_recorder")
    self.assertEqual(node.attrib["type"], "depth_processor_node.py")
    params = {item.attrib["name"]: item.attrib["value"] for item in node.findall("param")}
    self.assertEqual(params["input_topic"], "$(arg raw_topic)")
    self.assertEqual(params["output_topic"], "$(arg processed_topic)")
    self.assertEqual(params["output_width"], "64")
    self.assertEqual(params["output_height"], "36")
    self.assertEqual(params["max_depth_m"], "2.5")
```

- [ ] **Step 2: Verify RED**

```bash
cd /root/low/kuavo_ros_application
python3 -m unittest src/depth_image_recorder/test/test_recording_assets.py -v
```

Expected: ERROR because `process_depth_images.launch` does not exist.

- [ ] **Step 3: Create the launch**

```xml
<launch>
  <arg name="raw_topic" default="/camera/depth/image_raw" />
  <arg name="processed_topic" default="/camera/depth/image_processed" />
  <node pkg="depth_image_recorder" type="depth_processor_node.py"
        name="depth_image_processor" output="screen" required="true">
    <param name="input_topic" value="$(arg raw_topic)" />
    <param name="output_topic" value="$(arg processed_topic)" />
    <param name="output_width" value="64" />
    <param name="output_height" value="36" />
    <param name="max_depth_m" value="2.5" />
  </node>
</launch>
```

- [ ] **Step 4: Verify GREEN and commit**

```bash
cd /root/low/kuavo_ros_application
python3 -m unittest discover -s src/depth_image_recorder/test -p 'test_*.py' -v
git add src/depth_image_recorder/launch/process_depth_images.launch src/depth_image_recorder/test/test_recording_assets.py
git commit -m "feat: add standalone depth preprocessing launch"
```

Expected: all depth-image-recorder tests PASS.

## Task 3: Add a testable ROS Image decoder to `leju_head_lab`

**Files:**

- Create: `leju_head_lab/src/ros_dds_bridge/src/ros_dds_bridge/depth_image_codec.py`
- Create: `leju_head_lab/src/ros_dds_bridge/tests/test_depth_image_codec.py`
- Modify: `leju_head_lab/src/ros_dds_bridge/CMakeLists.txt`

- [ ] **Step 1: Write failing decoder tests**

Use a small fake message class and cover compact/padded little-endian 32FC1, big-endian 32FC1, legacy 16UC1, invalid encoding, invalid step, and truncated data. Core assertions:

```python
self.assertEqual(decode_depth_values(msg32), [0.0, 0.25, 0.5, 1.0])
self.assertEqual(decode_depth_values(padded_msg32), [0.0, 0.25, 0.5, 1.0])
self.assertEqual(decode_depth_values(big_endian_msg32), [0.0, 0.25, 0.5, 1.0])
self.assertEqual(decode_depth_values(msg16), [0, 1000, 2000, 2500])
with self.assertRaisesRegex(ValueError, "expected 16UC1 or 32FC1"):
    decode_depth_values(bad_encoding)
with self.assertRaisesRegex(ValueError, "step"):
    decode_depth_values(bad_step)
with self.assertRaisesRegex(ValueError, "truncated"):
    decode_depth_values(truncated)
```

- [ ] **Step 2: Verify RED**

```bash
cd /root/low/leju_head_lab
PYTHONPATH=src/ros_dds_bridge/src python3 -m unittest src/ros_dds_bridge/tests/test_depth_image_codec.py -v
```

Expected: FAIL because `ros_dds_bridge.depth_image_codec` does not exist.

- [ ] **Step 3: Implement the decoder**

Implement this API:

```python
def decode_depth_values(message):
    layouts = {"16UC1": ("H", 2), "32FC1": ("f", 4)}
    encoding = message.encoding.upper()
    if encoding not in layouts:
        raise ValueError("expected 16UC1 or 32FC1, got %s" % message.encoding)
    if message.width <= 0 or message.height <= 0:
        raise ValueError("invalid image dimensions")
    fmt, bytes_per_pixel = layouts[encoding]
    row_bytes = message.width * bytes_per_pixel
    if message.step < row_bytes:
        raise ValueError("invalid image step")
    if len(message.data) < message.step * message.height:
        raise ValueError("truncated image data")
    endian = ">" if message.is_bigendian else "<"
    values = []
    for row in range(message.height):
        start = row * message.step
        values.extend(struct.unpack(
            endian + "%d%s" % (message.width, fmt),
            bytes(message.data[start:start + row_bytes]),
        ))
    return values
```

Register the test with `catkin_add_nosetests(tests/test_depth_image_codec.py)` under `CATKIN_ENABLE_TESTING`.

- [ ] **Step 4: Verify GREEN and commit**

```bash
cd /root/low/leju_head_lab
PYTHONPATH=src/ros_dds_bridge/src python3 -m unittest src/ros_dds_bridge/tests/test_depth_image_codec.py -v
git add src/ros_dds_bridge/src/ros_dds_bridge/depth_image_codec.py src/ros_dds_bridge/tests/test_depth_image_codec.py src/ros_dds_bridge/CMakeLists.txt
git commit -m "test: add depth image DDS codec coverage"
```

## Task 4: Publish the processed ROS image over DDS

**Files:**

- Modify: `leju_head_lab/src/ros_dds_bridge/scripts/depth_camera_bridge.py`
- Create: `leju_head_lab/src/ros_dds_bridge/launch/processed_depth_camera_bridge.launch`
- Modify: `leju_head_lab/src/ros_dds_bridge/tests/test_depth_image_codec.py`

- [ ] **Step 1: Add failing configuration and timestamp tests**

Extract and test these pure helpers:

```python
def validate_bridge_contract(encoding, dds_type, width, height):
    normalized = encoding.upper()
    if normalized not in ("16UC1", "32FC1"):
        raise ValueError("unsupported depth encoding: %s" % encoding)
    if dds_type not in ("uint16", "float64"):
        raise ValueError("dds_type must be uint16 or float64")
    if normalized == "32FC1" and dds_type != "float64":
        raise ValueError("32FC1 requires float64 DDS output")
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")

def stamp_seconds(stamp):
    if stamp.secs == 0 and stamp.nsecs == 0:
        return None
    if stamp.nsecs < 0 or stamp.nsecs >= 1000000000:
        return None
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9
```

Assert `32FC1` requires `float64`, processed dimensions are `64×36`, zero stamps return `None`, and `123s + 456ns` returns `123.000000456`.

- [ ] **Step 2: Verify RED with the Task 3 unittest command**

Expected: FAIL because the helpers do not exist.

- [ ] **Step 3: Modify the bridge and add the launch**

Replace the 16-bit-only decoder call with `decode_depth_values(msg)`. Add `~encoding` and validate it against the incoming message. Keep:

```python
stamp = msg.header.stamp
self.writer.write(self.message_class(int(stamp.secs), int(stamp.nsecs), values))
```

Do not replace a zero capture stamp with `rospy.Time.now()`; reject it so latency always has a defined meaning. For nonzero stamps log once per second:

```python
upper_ms = (rospy.Time.now().to_sec() - stamp.to_sec()) * 1000.0
rospy.loginfo_throttle(1.0, "depth DDS published=%d rejected=%d upper_elapsed=%.2f ms", self.count, self.rejected, upper_ms)
```

Create:

```xml
<launch>
  <arg name="dds_domain" default="0" />
  <arg name="ros_topic" default="/camera/depth/image_processed" />
  <arg name="dds_topic" default="rt/depth_camera/frame_normalized_36x64" />
  <node pkg="ros_dds_bridge" type="depth_camera_bridge.py"
        name="processed_depth_camera_bridge" output="screen" respawn="true">
    <param name="dds_domain" value="$(arg dds_domain)" />
    <param name="ros_topic" value="$(arg ros_topic)" />
    <param name="dds_topic" value="$(arg dds_topic)" />
    <param name="dds_type" value="float64" />
    <param name="encoding" value="32FC1" />
    <param name="width" value="64" />
    <param name="height" value="36" />
  </node>
</launch>
```

- [ ] **Step 4: Run all bridge tests and commit**

```bash
cd /root/low/leju_head_lab
PYTHONPATH=src/ros_dds_bridge/src python3 -m unittest discover -s src/ros_dds_bridge/tests -p 'test_*.py' -v
git add src/ros_dds_bridge/scripts/depth_camera_bridge.py src/ros_dds_bridge/launch/processed_depth_camera_bridge.launch src/ros_dds_bridge/tests/test_depth_image_codec.py
git commit -m "feat: bridge processed depth frames over DDS"
```

## Task 5: Specify direct lower-host consumption and latency

**Files:**

- Modify: `lejulab_platform/src/leju-controllers/leju-rl-controller/tests/depth/test_depth_pipeline.cpp`
- Modify: `lejulab_platform/src/leju-controllers/leju-rl-controller/include/leju-rl-controller/depth/depth_observation_provider.h`

- [ ] **Step 1: Add failing unit tests**

Declare the target APIs in the header, then test:

```cpp
TEST(PreprocessedDepth, PreservesEveryPolicyPixel) {
  std::vector<double> input(DepthHistoryBuffer::kImageSize);
  for (size_t i = 0; i < input.size(); ++i)
    input[i] = static_cast<double>(i) / (input.size() - 1);
  const auto result = BuildPreprocessedDepth(input, 10.0, 7);
  ASSERT_TRUE(result.has_value());
  for (size_t i = 0; i < input.size(); ++i)
    EXPECT_FLOAT_EQ(result->pixels[i], static_cast<float>(input[i]));
  EXPECT_DOUBLE_EQ(result->timestamp_sec, 10.0);
}

TEST(PreprocessedDepth, RejectsInvalidContractValues) {
  std::vector<double> input(DepthHistoryBuffer::kImageSize, 0.5);
  input[5] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(BuildPreprocessedDepth(input, 1.0, 1));
  input[5] = 1.01;
  EXPECT_FALSE(BuildPreprocessedDepth(input, 1.0, 1));
}

TEST(DepthLatency, UsesCaptureEpochAndRejectsInvalidStamp) {
  EXPECT_NEAR(*ComputeDepthLatencySec(100, 250000000, 100.5), 0.25, 1e-12);
  EXPECT_FALSE(ComputeDepthLatencySec(0, 0, 100.5));
  EXPECT_FALSE(ComputeDepthLatencySec(100, 1000000000u, 100.5));
}
```

Add a provider test that calls `submitPreprocessed` 43 times with distinguishable pixels, verifies the eight selected frames, and verifies `snapshot(receive_steady + timeout)` uses receive steady time.

- [ ] **Step 2: Verify RED**

```bash
cd /root/low/lejulab_platform/build_catkin
cmake --build . --target test_depth_pipeline -j2
./src/leju-controllers/leju-rl-controller/test_depth_pipeline
```

Expected: link/compile failure because the target APIs are not implemented.

- [ ] **Step 3: Keep the tests staged without production implementation**

Do not commit a permanently broken branch. Continue directly to Task 6 after recording the expected failure output.

## Task 6: Implement direct consumption, source separation, and latency logs

**Files:**

- Modify: `lejulab_platform/src/leju-controllers/leju-rl-controller/include/leju-rl-controller/depth/depth_observation_provider.h`
- Modify: `lejulab_platform/src/leju-controllers/leju-rl-controller/src/depth/depth_observation_provider.cpp`
- Modify: `lejulab_platform/src/leju-controllers/leju-rl-controller/src/controllers/depth_walk_controller.cpp`

- [ ] **Step 1: Add explicit configuration and direct-submit API**

Add `bool preprocessed = false` to `DepthDdsInputConfig`, plus:

```cpp
std::optional<ProcessedDepth> BuildPreprocessedDepth(
    const std::vector<double>& values, double receive_steady_sec,
    uint64_t sequence);
std::optional<double> ComputeDepthLatencySec(
    int32_t header_sec, uint32_t header_nanosec, double receive_system_sec);
bool submitPreprocessed(const ProcessedDepth& frame);
```

- [ ] **Step 2: Implement strict validation without pixel transformation**

`BuildPreprocessedDepth` must require exactly 2304 finite values in `[0,1]`, convert each once from double to float, count positive pixels for `valid_ratio`, set `64×36`, and retain `receive_steady_sec` as the freshness timestamp. It must not call `DepthImageProcessor`.

`submitPreprocessed` must reset/switch to `ActiveSource::kRealCamera` and push directly into `real_camera_history_`.

- [ ] **Step 3: Pass the DDS header and separate clocks**

Change callbacks from `Handle(message.data())` to passing `message.header_sec()` and `message.header_nanosec()`. In the configured preprocessed callback:

```cpp
const double receive_steady = SteadySeconds();
const double receive_system = SystemSeconds();
auto processed = BuildPreprocessedDepth(values, receive_steady, ++sequence_);
if (!processed || !GlobalDepthObservationProvider()->submitPreprocessed(*processed)) {
  ++rejected_frames_;
  return;
}
const auto latency = ComputeDepthLatencySec(sec, nanosec, receive_system);
```

Throttle normal latency output to one line per second. Treat `< -0.05 s` or `> 5 s` as a clock/queue warning. Always retain the legacy MuJoCo subscriber on `rt/depth_camera/frame_meters_36x64`; route it through the existing processed-simulation behavior, not the true-camera history.

- [ ] **Step 4: Parse `input_preprocessed` and validate combinations**

In `DepthWalkController::loadConfig`, read:

```cpp
if (depth_cfg["input_preprocessed"])
  input.preprocessed = depth_cfg["input_preprocessed"].as<bool>();
```

Reject `preprocessed=true` unless type is `float64`, dimensions are `64×36`, and unit is meters (the unit field is ignored for pixels but fixed for a non-ambiguous config contract). Include `preprocessed=true/false` in the startup log.

- [ ] **Step 5: Verify GREEN and commit tests plus implementation**

```bash
cd /root/low/lejulab_platform/build_catkin
cmake --build . --target test_depth_pipeline -j2
./src/leju-controllers/leju-rl-controller/test_depth_pipeline
git -C /root/low/lejulab_platform add \
  src/leju-controllers/leju-rl-controller/include/leju-rl-controller/depth/depth_observation_provider.h \
  src/leju-controllers/leju-rl-controller/src/depth/depth_observation_provider.cpp \
  src/leju-controllers/leju-rl-controller/src/controllers/depth_walk_controller.cpp \
  src/leju-controllers/leju-rl-controller/tests/depth/test_depth_pipeline.cpp
git -C /root/low/lejulab_platform commit -m "feat: consume preprocessed depth with latency metrics"
```

Expected: all `test_depth_pipeline` cases PASS.

## Task 7: Switch S17 configuration and DDS diagnostics

**Files:**

- Modify: `lejulab_platform/src/leju-controllers/leju-rl-controller/config/17/controllers/config_depth_walk.yaml`
- Modify: `lejulab_platform/src/leju_launch/scripts/dds_topic_hz.sh`
- Modify: `lejulab_platform/src/leju_launch/scripts/README_depth_dds_tools.md`

- [ ] **Step 1: Write a failing configuration assertion**

Run this read-only assertion before changing the YAML:

```bash
python3 -c 'from pathlib import Path; p=Path("/root/low/lejulab_platform/src/leju-controllers/leju-rl-controller/config/17/controllers/config_depth_walk.yaml").read_text(); required=["input_topic: \"rt/depth_camera/frame_normalized_36x64\"", "input_data_type: \"float64\"", "input_width: 64", "input_height: 36", "input_unit: \"meters\"", "input_preprocessed: true"]; assert all(x in p for x in required)'
```

The required strings are:

```text
input_topic: "rt/depth_camera/frame_normalized_36x64"
input_data_type: "float64"
input_width: 64
input_height: 36
input_unit: "meters"
input_preprocessed: true
```

- [ ] **Step 2: Verify RED with the focused test target**

Expected: the assertion exits nonzero because the current configuration selects raw UInt16 480×270 input.

- [ ] **Step 3: Update configuration and diagnostic defaults**

Set the six values above and retain `input_frame_decimation: 1`. Change true-camera examples/defaults to `rt/depth_camera/frame_normalized_36x64 --expected-len 2304`; clearly label the MuJoCo meters topic separately.

- [ ] **Step 4: Run and commit**

```bash
cd /root/low/lejulab_platform/build_catkin
cmake --build . --target test_depth_pipeline -j2
./src/leju-controllers/leju-rl-controller/test_depth_pipeline
git -C /root/low/lejulab_platform add \
  src/leju-controllers/leju-rl-controller/config/17/controllers/config_depth_walk.yaml \
  src/leju_launch/scripts/dds_topic_hz.sh \
  src/leju_launch/scripts/README_depth_dds_tools.md
git -C /root/low/lejulab_platform commit -m "config: select normalized upper-host depth frames"
```

## Task 8: Update the 3588 deployment guide

**Files:**

- Modify: `/home/lejurl/Downloads/Roban上楼梯部署教程（深度相机版-3588版）.md`

- [ ] **Step 1: Replace the startup sequence**

Document these upper-host terminals after sourcing both workspaces:

```bash
roslaunch orbbec_camera gemini_330_series_60hz_depth.launch usb_port:=<实际端口>
roslaunch depth_image_recorder process_depth_images.launch
roslaunch ros_dds_bridge processed_depth_camera_bridge.launch
```

Remove the instruction to run raw depth processing on the lower host.

- [ ] **Step 2: Add validation commands and expected contracts**

```bash
rostopic hz /camera/depth/image_raw
rostopic hz /camera/depth/image_processed
rostopic echo -n 1 /camera/depth/image_processed | head
bash src/leju_launch/scripts/dds_topic_hz.sh \
  rt/depth_camera/frame_normalized_36x64 --expected-len 2304
```

State expected ROS encoding/dimensions (`32FC1`, `64×36`), DDS value range (`[0,1]`), expected ~60 Hz, 18 KiB pixel payload, and the `depth DDS received=<count> latency=<value> ms` lower-host log.

- [ ] **Step 3: Add timestamp troubleshooting**

Explain that negative or multi-second latency indicates chrony/clock or queue problems, while local depth timeout uses a monotonic clock. Preserve the existing chrony verification commands.

- [ ] **Step 4: Check Markdown consistency**

```bash
rg -n "frame_u16_mm_270x480|frame_mm_240x424|下位机.*深度处理|frame_normalized_36x64|latency" \
  '/home/lejurl/Downloads/Roban上楼梯部署教程（深度相机版-3588版）.md'
```

Expected: old topics appear only in a labeled compatibility/troubleshooting section; normal deployment uses the normalized topic.

## Task 9: Full verification

**Files:** No new files.

- [ ] **Step 1: Run all upper-host package tests**

```bash
cd /root/low/kuavo_ros_application
python3 -m unittest discover -s src/depth_image_recorder/test -p 'test_*.py' -v
```

Expected: PASS.

- [ ] **Step 2: Run all bridge tests**

```bash
cd /root/low/leju_head_lab
PYTHONPATH=src/ros_dds_bridge/src python3 -m unittest discover -s src/ros_dds_bridge/tests -p 'test_*.py' -v
```

Expected: PASS.

- [ ] **Step 3: Build and run lower-host depth tests**

```bash
cd /root/low/lejulab_platform/build_catkin
cmake --build . --target test_depth_pipeline run_rl_controller -j2
./src/leju-controllers/leju-rl-controller/test_depth_pipeline
```

Expected: build and tests PASS.

- [ ] **Step 4: Inspect only task-owned diffs**

```bash
git -C /root/low/kuavo_ros_application diff --check HEAD~2..HEAD
git -C /root/low/leju_head_lab diff --check HEAD~2..HEAD
git -C /root/low/lejulab_platform diff --check HEAD~2..HEAD
```

Expected: no whitespace errors. Confirm unrelated pre-existing untracked models/build files remain untouched.

- [ ] **Step 5: Perform a no-hardware ROS launch syntax check where available**

Run `roslaunch --files` for both new launch files after sourcing the relevant workspaces. Expected: each resolves to the intended Python node without starting hardware.

- [ ] **Step 6: Report manual real-hardware checks separately**

Do not claim camera frequency or real network latency without a connected Gemini 330 and two synchronized hosts. Provide the exact commands from Task 8 for the user to run during deployment.
