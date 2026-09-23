/**
 * motor_keyboard_ctrl 单元测试
 *
 * 测试纯工具函数/类，不依赖 DDS 或硬件
 */

#include "../keyboard_ctrl_utils.h"
#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <thread>

// ============================================================
// keyToIndex / indexToKey 映射测试
// ============================================================

TEST(KeyMapping, DigitKeys) {
    for (int i = 0; i <= 9; ++i) {
        EXPECT_EQ(keyToIndex('0' + i), i) << "key='0'+" << i;
    }
}

TEST(KeyMapping, LetterKeys) {
    // a=10, b=11, ..., m=22
    for (int i = 0; i <= 12; ++i) {
        EXPECT_EQ(keyToIndex('a' + i), 10 + i) << "key='a'+" << i;
    }
}

TEST(KeyMapping, InvalidKeys) {
    EXPECT_EQ(keyToIndex('n'), -1);  // 超过 m
    EXPECT_EQ(keyToIndex('z'), -1);
    EXPECT_EQ(keyToIndex(' '), -1);
    EXPECT_EQ(keyToIndex('W'), -1);
    EXPECT_EQ(keyToIndex(-1), -1);
}

TEST(KeyMapping, RoundTrip) {
    // keyToIndex → indexToKey 往返一致
    for (int idx = 0; idx < 23; ++idx) {
        char key = indexToKey(idx);
        EXPECT_EQ(keyToIndex(key), idx) << "idx=" << idx;
    }
}

// ============================================================
// shortName 测试
// ============================================================

TEST(ShortName, AlreadyShort) {
    EXPECT_EQ(shortName("leg_l1"), "leg_l1");
    EXPECT_EQ(shortName("waist_yaw"), "waist_yaw");
}

TEST(ShortName, RemovesJointSuffix) {
    EXPECT_EQ(shortName("leg_l1_joint"), "leg_l1");
    EXPECT_EQ(shortName("zarm_l1_joint"), "zarm_l1");
}

TEST(ShortName, RemovesLinkSuffix) {
    EXPECT_EQ(shortName("some_long_link"), "some_long");
}

TEST(ShortName, TruncatesLongName) {
    EXPECT_EQ(shortName("very_long_motor_name_here", 10), "very_long_");
}

TEST(ShortName, CustomMaxLen) {
    EXPECT_EQ(shortName("abcde", 5), "abcde");
    EXPECT_EQ(shortName("abcdef", 5), "abcde");
}

// ============================================================
// buildGroups 测试 (Roban v14 = 23 电机)
// ============================================================

static std::vector<std::string> makeRobanV14Names() {
    return {
        "leg_l1", "leg_l2", "leg_l3", "leg_l4", "leg_l5", "leg_l6",   // 0-5
        "leg_r1", "leg_r2", "leg_r3", "leg_r4", "leg_r5", "leg_r6",   // 6-11
        "waist_yaw",                                                     // 12
        "zarm_l1", "zarm_l2", "zarm_l3", "zarm_l4",                    // 13-16
        "zarm_r1", "zarm_r2", "zarm_r3", "zarm_r4",                    // 17-20
        "zhead_1", "zhead_2"                                             // 21-22
    };
}

TEST(BuildGroups, RobanV14Has6Groups) {
    auto names = makeRobanV14Names();
    ASSERT_EQ(names.size(), 23u);
    auto groups = buildGroups(names);
    ASSERT_EQ(groups.size(), 6u);
}

TEST(BuildGroups, RobanV14GroupSizes) {
    auto groups = buildGroups(makeRobanV14Names());
    EXPECT_EQ(groups[0].indices.size(), 6u);  // 左腿
    EXPECT_EQ(groups[1].indices.size(), 6u);  // 右腿
    EXPECT_EQ(groups[2].indices.size(), 1u);  // 腰
    EXPECT_EQ(groups[3].indices.size(), 4u);  // 左臂
    EXPECT_EQ(groups[4].indices.size(), 4u);  // 右臂
    EXPECT_EQ(groups[5].indices.size(), 2u);  // 头
}

TEST(BuildGroups, RobanV14IndicesCoverAll) {
    auto groups = buildGroups(makeRobanV14Names());
    std::vector<bool> covered(23, false);
    for (auto& g : groups) {
        for (int idx : g.indices) {
            ASSERT_GE(idx, 0);
            ASSERT_LT(idx, 23);
            EXPECT_FALSE(covered[idx]) << "index " << idx << " covered twice";
            covered[idx] = true;
        }
    }
    for (int i = 0; i < 23; ++i) {
        EXPECT_TRUE(covered[i]) << "index " << i << " not covered";
    }
}

TEST(BuildGroups, NonStandardCountFallsBack) {
    std::vector<std::string> names = {"m1", "m2", "m3", "m4", "m5"};
    auto groups = buildGroups(names);
    ASSERT_EQ(groups.size(), 1u);
    EXPECT_EQ(groups[0].indices.size(), 5u);
}

// ============================================================
// SineTest 测试
// ============================================================

TEST(SineTest, InactiveReturnsCenter) {
    SineTest st;
    st.center_rad = 1.5;
    st.active = false;
    auto now = std::chrono::steady_clock::now();
    EXPECT_DOUBLE_EQ(st.compute(now), 1.5);
}

TEST(SineTest, ActiveAtT0ReturnsCenter) {
    SineTest st;
    st.active = true;
    st.center_rad = 0.5;
    st.amplitude_rad = 0.1;
    st.frequency_hz = 1.0;
    st.start_time = std::chrono::steady_clock::now();
    double val = st.compute(st.start_time);
    EXPECT_NEAR(val, 0.5, 1e-10);
}

TEST(SineTest, ActiveAtQuarterPeriodReturnsMax) {
    SineTest st;
    st.active = true;
    st.center_rad = 0.0;
    st.amplitude_rad = 0.1;
    st.frequency_hz = 1.0;
    auto t0 = std::chrono::steady_clock::now();
    st.start_time = t0;
    auto t_quarter = t0 + std::chrono::microseconds(250000);
    double val = st.compute(t_quarter);
    EXPECT_NEAR(val, st.amplitude_rad, 1e-6);
}

TEST(SineTest, ActiveAtHalfPeriodReturnsCenter) {
    SineTest st;
    st.active = true;
    st.center_rad = 1.0;
    st.amplitude_rad = 0.2;
    st.frequency_hz = 2.0;
    auto t0 = std::chrono::steady_clock::now();
    st.start_time = t0;
    auto t_half = t0 + std::chrono::microseconds(250000);
    double val = st.compute(t_half);
    EXPECT_NEAR(val, 1.0, 1e-6);
}

TEST(SineTest, AmplitudeClamping) {
    SineTest st;
    // 验证幅度调节逻辑 (rad, clamp: 0.01 ~ 1.0, step 0.05)
    st.amplitude_rad = 0.06;
    st.amplitude_rad = std::max(0.01, st.amplitude_rad - 0.05);
    EXPECT_NEAR(st.amplitude_rad, 0.01, 1e-10);
    st.amplitude_rad = std::max(0.01, st.amplitude_rad - 0.05);
    EXPECT_NEAR(st.amplitude_rad, 0.01, 1e-10);

    st.amplitude_rad = 0.95;
    st.amplitude_rad = std::min(1.0, st.amplitude_rad + 0.05);
    EXPECT_DOUBLE_EQ(st.amplitude_rad, 1.0);
    st.amplitude_rad = std::min(1.0, st.amplitude_rad + 0.05);
    EXPECT_DOUBLE_EQ(st.amplitude_rad, 1.0);
}

// ============================================================
// StepTest 测试
// ============================================================

TEST(StepTest, InactiveReturnsZero) {
    StepTest stp;
    EXPECT_DOUBLE_EQ(stp.compute(std::chrono::steady_clock::now()), 0.0);
}

TEST(StepTest, SequenceLevels) {
    // 序列: 0, x/2, 0, x, 0, -x/2, 0, -x  (x=4.0)
    StepTest stp;
    stp.buildSequence(4.0);
    stp.hold_duration_s = 10.0;
    stp.active = true;
    auto t0 = std::chrono::steady_clock::now();
    stp.start_time = t0;

    EXPECT_DOUBLE_EQ(stp.compute(t0), 0.0);                                        // 级0: 0
    EXPECT_DOUBLE_EQ(stp.compute(t0 + std::chrono::seconds(10)), 2.0);             // 级1: x/2
    EXPECT_DOUBLE_EQ(stp.compute(t0 + std::chrono::seconds(20)), 0.0);             // 级2: 0
    EXPECT_DOUBLE_EQ(stp.compute(t0 + std::chrono::seconds(30)), 4.0);             // 级3: x
    EXPECT_DOUBLE_EQ(stp.compute(t0 + std::chrono::seconds(40)), 0.0);             // 级4: 0
    EXPECT_DOUBLE_EQ(stp.compute(t0 + std::chrono::seconds(50)), -2.0);            // 级5: -x/2
    EXPECT_DOUBLE_EQ(stp.compute(t0 + std::chrono::seconds(60)), 0.0);             // 级6: 0
    EXPECT_DOUBLE_EQ(stp.compute(t0 + std::chrono::seconds(70)), -4.0);            // 级7: -x
}

TEST(StepTest, ExpiresAfterAllLevels) {
    StepTest stp;
    stp.buildSequence(1.0);
    stp.hold_duration_s = 5.0;
    stp.active = true;
    auto t0 = std::chrono::steady_clock::now();
    stp.start_time = t0;

    // 8 级 × 5 秒 = 40 秒后过期
    EXPECT_FALSE(stp.isExpired(t0 + std::chrono::seconds(39)));
    EXPECT_TRUE(stp.isExpired(t0 + std::chrono::seconds(41)));
    EXPECT_DOUBLE_EQ(stp.compute(t0 + std::chrono::seconds(41)), 0.0);
}

TEST(StepTest, CurrentIndex) {
    StepTest stp;
    stp.buildSequence(3.0);
    stp.hold_duration_s = 2.0;
    stp.active = true;
    auto t0 = std::chrono::steady_clock::now();
    stp.start_time = t0;

    EXPECT_EQ(stp.currentIndex(t0), 0);
    EXPECT_EQ(stp.currentIndex(t0 + std::chrono::seconds(3)), 1);
    EXPECT_EQ(stp.currentIndex(t0 + std::chrono::seconds(13)), -1);  // 过期
}

// ============================================================
// DanceKpKd / DanceControlModes 测试
// ============================================================

TEST(DanceKpKd, ReturnsCorrectSize) {
    auto gains = getDanceKpKd(23);
    EXPECT_EQ(gains.size(), 23u);
}

TEST(DanceKpKd, LegJointsHaveNonZeroKp) {
    auto gains = getDanceKpKd(23);
    for (int i = 0; i < 12; ++i) {
        EXPECT_GT(gains[i].kp, 0.0) << "joint " << i;
    }
}

TEST(DanceKpKd, HeadJointsAreZero) {
    auto gains = getDanceKpKd(23);
    EXPECT_DOUBLE_EQ(gains[21].kp, 0.0);
    EXPECT_DOUBLE_EQ(gains[22].kp, 0.0);
}

TEST(DanceControlModes, LegsCSTArmsCSP) {
    auto modes = getDanceControlModes(23);
    EXPECT_EQ(modes.size(), 23u);
    for (int i = 0; i <= 12; ++i) EXPECT_EQ(modes[i], 0) << "joint " << i;
    for (int i = 13; i <= 20; ++i) EXPECT_EQ(modes[i], 2) << "joint " << i;
    EXPECT_EQ(modes[21], 0);
    EXPECT_EQ(modes[22], 0);
}

TEST(SineTest, FrequencyClamping) {
    SineTest st;
    st.frequency_hz = 0.2;
    st.frequency_hz = std::max(0.1, st.frequency_hz - 0.1);
    EXPECT_NEAR(st.frequency_hz, 0.1, 1e-10);
    st.frequency_hz = std::max(0.1, st.frequency_hz - 0.1);
    EXPECT_NEAR(st.frequency_hz, 0.1, 1e-10);  // 不低于 0.1

    st.frequency_hz = 4.9;
    st.frequency_hz = std::min(5.0, st.frequency_hz + 0.1);
    EXPECT_NEAR(st.frequency_hz, 5.0, 1e-10);
    st.frequency_hz = std::min(5.0, st.frequency_hz + 0.1);
    EXPECT_NEAR(st.frequency_hz, 5.0, 1e-10);  // 不超过 5.0
}

// ============================================================
// ChirpSweepTest 测试 (CST + 上层 PD 扫频)
// ============================================================

TEST(ChirpSweepTest, InactiveReturnsCenter) {
    ChirpSweepTest sw;
    sw.center_rad = 1.234;
    sw.active = false;
    auto ref = sw.compute(std::chrono::steady_clock::now());
    EXPECT_NEAR(ref.q_ref, 1.234, 1e-12);
    EXPECT_NEAR(ref.v_ref, 0.0, 1e-12);
}

TEST(ChirpSweepTest, AtTimeZeroRefIsCenter) {
    // ramp_s>0 时 t=0 幅度系数 = 0，q_ref == center，v_ref == 0
    ChirpSweepTest sw;
    sw.amplitude_rad = 0.1;
    sw.center_rad    = 0.5;
    sw.freq_start_hz = 1.0;
    sw.freq_end_hz   = 10.0;
    sw.duration_s    = 10.0;
    sw.ramp_s        = 0.5;
    sw.active        = true;
    sw.start_time    = std::chrono::steady_clock::now();
    auto ref = sw.compute(sw.start_time);
    EXPECT_NEAR(ref.q_ref, sw.center_rad, 1e-12);
    EXPECT_NEAR(ref.v_ref, 0.0, 1e-12);
    EXPECT_NEAR(ref.f_hz, sw.freq_start_hz, 1e-12);
}

TEST(ChirpSweepTest, RampDisabledGivesFullVelocity) {
    // ramp_s = 0 时 t=0 速度参考 = A*2π*f0
    ChirpSweepTest sw;
    sw.amplitude_rad = 0.1;
    sw.freq_start_hz = 2.0;
    sw.freq_end_hz   = 8.0;
    sw.duration_s    = 5.0;
    sw.ramp_s        = 0.0;
    sw.active        = true;
    sw.start_time    = std::chrono::steady_clock::now();
    auto ref = sw.compute(sw.start_time);
    EXPECT_NEAR(ref.v_ref, sw.amplitude_rad * 2.0 * M_PI * sw.freq_start_hz, 1e-9);
}

TEST(ChirpSweepTest, FrequencyChirpsLinearly) {
    // t = duration/2 时 f = (f0 + f1) / 2
    ChirpSweepTest sw;
    sw.freq_start_hz = 1.0;
    sw.freq_end_hz   = 11.0;
    sw.duration_s    = 10.0;
    sw.active        = true;
    sw.start_time    = std::chrono::steady_clock::now();
    auto mid = sw.start_time + std::chrono::milliseconds(5000);
    auto ref = sw.compute(mid);
    EXPECT_NEAR(ref.f_hz, 6.0, 1e-6);
}

TEST(ChirpSweepTest, ExpiresAfterDuration) {
    ChirpSweepTest sw;
    sw.duration_s = 2.0;
    sw.active     = true;
    sw.start_time = std::chrono::steady_clock::now();
    auto before = sw.start_time + std::chrono::milliseconds(1900);
    auto after  = sw.start_time + std::chrono::milliseconds(2100);
    EXPECT_FALSE(sw.isExpired(before));
    EXPECT_TRUE(sw.isExpired(after));
}

// ============================================================
// CsvLogger 测试
// ============================================================

class CsvLoggerTest : public ::testing::Test {
protected:
    std::string test_file;

    void SetUp() override {
        test_file = "/tmp/test_csv_logger_" + std::to_string(getpid()) + ".csv";
    }

    void TearDown() override {
        std::remove(test_file.c_str());
    }

    std::string readFile(const std::string& path) {
        std::ifstream f(path);
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }

    std::vector<std::string> splitLines(const std::string& s) {
        std::vector<std::string> lines;
        std::istringstream iss(s);
        std::string line;
        while (std::getline(iss, line)) {
            if (!line.empty()) lines.push_back(line);
        }
        return lines;
    }
};

TEST_F(CsvLoggerTest, OpenCreatesFile) {
    CsvLogger logger;
    EXPECT_TRUE(logger.open(test_file, {"m1", "m2"}));
    EXPECT_TRUE(logger.is_open());
    logger.close();
    EXPECT_FALSE(logger.is_open());
}

TEST_F(CsvLoggerTest, HeaderFormat) {
    CsvLogger logger;
    logger.open(test_file, {"motor_a", "motor_b"});
    logger.close();

    auto content = readFile(test_file);
    auto lines = splitLines(content);
    ASSERT_GE(lines.size(), 1u);

    // 检查 header 列 (PlotJuggler 格式: prefix/motor_name)
    auto& header = lines[0];
    EXPECT_NE(header.find("time_ms"), std::string::npos);
    EXPECT_NE(header.find("state_seq"), std::string::npos);
    EXPECT_NE(header.find("dt_ms"), std::string::npos);
    EXPECT_NE(header.find("cmd_p/motor_a"), std::string::npos);
    EXPECT_NE(header.find("cmd_p/motor_b"), std::string::npos);
    EXPECT_NE(header.find("cmd_kp/motor_a"), std::string::npos);
    EXPECT_NE(header.find("cmd_kd/motor_a"), std::string::npos);
    EXPECT_NE(header.find("cmd_tau/motor_a"), std::string::npos);
    EXPECT_NE(header.find("fb_p/motor_a"), std::string::npos);
    EXPECT_NE(header.find("fb_tau/motor_a"), std::string::npos);
}

TEST_F(CsvLoggerTest, HeaderColumnCount) {
    CsvLogger logger;
    int n_motors = 3;
    logger.open(test_file, {"m1", "m2", "m3"});
    logger.close();

    auto content = readFile(test_file);
    auto lines = splitLines(content);
    ASSERT_GE(lines.size(), 1u);

    // 列数 = 3 (time_ms, state_seq, dt_ms) + n_motors * 8 (cmd_p + cmd_v + cmd_kp + cmd_kd + cmd_tau + fb_p + fb_v + fb_tau)
    int expected_cols = 3 + n_motors * 8;
    int actual_cols = 1;
    for (char c : lines[0]) {
        if (c == ',') actual_cols++;
    }
    EXPECT_EQ(actual_cols, expected_cols);
}

TEST_F(CsvLoggerTest, LogWritesDataRows) {
    CsvLogger logger;
    logger.open(test_file, {"m1", "m2"});

    std::vector<double> cmd = {1.0, 2.0};
    std::vector<double> cmdv = {0.1, 0.2};
    std::vector<double> kp = {100.0, 100.0};
    std::vector<double> kd = {10.0, 10.0};
    std::vector<double> fb  = {1.1, 2.1};
    logger.log(42, cmd, cmdv, kp, kd, fb);
    logger.log(43, cmd, cmdv, kp, kd, fb);
    logger.close();

    auto content = readFile(test_file);
    auto lines = splitLines(content);
    ASSERT_EQ(lines.size(), 3u);  // header + 2 data rows
}

TEST_F(CsvLoggerTest, DataRowContainsValues) {
    CsvLogger logger;
    logger.open(test_file, {"m1"});

    std::vector<double> cmd = {3.14};
    std::vector<double> cmdv = {0.0};
    std::vector<double> kp = {100.0};
    std::vector<double> kd = {10.0};
    std::vector<double> fb  = {3.15};
    logger.log(100, cmd, cmdv, kp, kd, fb);
    logger.close();

    auto content = readFile(test_file);
    auto lines = splitLines(content);
    ASSERT_EQ(lines.size(), 2u);

    // 数据行包含 state_seq=100
    EXPECT_NE(lines[1].find(",100,"), std::string::npos);
    // 包含命令和反馈值
    EXPECT_NE(lines[1].find("3.14"), std::string::npos);
    EXPECT_NE(lines[1].find("3.15"), std::string::npos);
}

TEST_F(CsvLoggerTest, FirstRowDtIsZero) {
    CsvLogger logger;
    logger.open(test_file, {"m1"});
    logger.log(1, {0.0}, {0.0}, {0.0}, {0.0}, {0.0});
    logger.close();

    auto content = readFile(test_file);
    auto lines = splitLines(content);
    ASSERT_EQ(lines.size(), 2u);

    // 第一行 dt_ms 应该是 0
    // 格式: time_ms,state_seq,dt_ms,...
    // dt_ms 是第 3 个字段
    std::istringstream iss(lines[1]);
    std::string field;
    std::getline(iss, field, ',');  // time_ms
    std::getline(iss, field, ',');  // state_seq
    std::getline(iss, field, ',');  // dt_ms
    EXPECT_DOUBLE_EQ(std::stod(field), 0.0);
}

TEST_F(CsvLoggerTest, LogWithoutOpenIsNoop) {
    CsvLogger logger;
    // 不 open 直接 log，不应崩溃
    logger.log(1, {1.0}, {0.0}, {0.0}, {0.0}, {1.0});
    logger.close();
}

TEST_F(CsvLoggerTest, InvalidPathReturnsFalse) {
    CsvLogger logger;
    EXPECT_FALSE(logger.open("/nonexistent/path/test.csv", {"m1"}));
    EXPECT_FALSE(logger.is_open());
}

// ============================================================
// 步长调节逻辑测试
// ============================================================

TEST(StepAdjust, HalvingClamps) {
    double step_deg = 1.0;
    // 连续减半
    for (int i = 0; i < 10; ++i) {
        step_deg = std::max(0.1, step_deg / 2.0);
    }
    EXPECT_GE(step_deg, 0.1);
}

TEST(StepAdjust, DoublingClamps) {
    double step_deg = 1.0;
    // 连续翻倍
    for (int i = 0; i < 10; ++i) {
        step_deg = std::min(20.0, step_deg * 2.0);
    }
    EXPECT_LE(step_deg, 20.0);
}
