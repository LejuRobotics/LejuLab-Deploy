#pragma once
/**
 * 键盘控制器可测试的纯工具函数/类
 * 从 motor_keyboard_ctrl.cpp 提取，供单元测试使用
 */

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <pwd.h>
#include <unistd.h>
#include <lejusdk-utils/robot_version.hpp>

// ---- 缩短电机名用于显示 ----

inline std::string shortName(const std::string& full, int max_len = 10) {
    if ((int)full.size() <= max_len) return full;
    std::string s = full;
    auto pos = s.find("_joint");
    if (pos != std::string::npos) {
        s = s.substr(0, pos);
    }
    pos = s.find("_link");
    if (pos != std::string::npos) {
        s = s.substr(0, pos);
    }
    if ((int)s.size() <= max_len) return s;
    return s.substr(0, max_len);
}

// ---- 电机分组 ----

struct MotorGroup {
    std::string label;
    std::vector<int> indices;
};

inline std::vector<MotorGroup> buildGroups(const std::vector<std::string>& names) {
    std::vector<MotorGroup> groups;
    int n = (int)names.size();

    auto addGroup = [&](const std::string& label, int start, int end) {
        MotorGroup g;
        g.label = label;
        for (int i = start; i <= end && i < n; ++i)
            g.indices.push_back(i);
        if (!g.indices.empty())
            groups.push_back(g);
    };

    if (n == 23) {
        addGroup("左腿 (leg_l)", 0, 5);
        addGroup("右腿 (leg_r)", 6, 11);
        addGroup("腰 (waist)", 12, 12);
        addGroup("左臂 (zarm_l)", 13, 16);
        addGroup("右臂 (zarm_r)", 17, 20);
        addGroup("头 (zhead)", 21, 22);
    } else {
        MotorGroup g;
        g.label = "全部电机";
        for (int i = 0; i < n; ++i) g.indices.push_back(i);
        groups.push_back(g);
    }
    return groups;
}

// ---- 配置文件解析 (dance config 同格式) ----

struct KeyboardCtrlConfig {
    std::vector<double> joint_kp;
    std::vector<double> joint_kd;
    std::vector<double> action_scale;
};

// 解析 (idx,0) value 格式的 section
inline std::map<int, double> parseIndexedSection(std::ifstream& ifs) {
    std::map<int, double> result;
    std::string line;
    while (std::getline(ifs, line)) {
        // 去掉注释
        auto semi = line.find(';');
        if (semi != std::string::npos) line = line.substr(0, semi);
        // 去掉前后空格
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        line = line.substr(start);
        // section 结束
        if (line[0] == '}') break;
        // 解析 (idx,0)  value
        if (line[0] != '(') continue;
        auto comma = line.find(',');
        auto paren_close = line.find(')');
        if (comma == std::string::npos || paren_close == std::string::npos) continue;
        int idx = std::stoi(line.substr(1, comma - 1));
        std::string rest = line.substr(paren_close + 1);
        double val = std::stod(rest);
        result[idx] = val;
    }
    return result;
}

inline std::vector<double> indexedMapToVector(const std::map<int, double>& m, int n, double default_val) {
    std::vector<double> v(n, default_val);
    for (auto& [idx, val] : m) {
        if (idx >= 0 && idx < n) v[idx] = val;
    }
    return v;
}

// ---- 从 kuavo.json 中解析 JSON 数组 (简易解析，不引入 JSON 库) ----

// 解析形如 "key": [1, 2, 3] 的 JSON 数组
inline std::vector<double> parseJsonArray(const std::string& json_content, const std::string& key) {
    std::vector<double> result;
    auto key_pattern = "\"" + key + "\"";
    auto pos = json_content.find(key_pattern);
    if (pos == std::string::npos) return result;

    auto bracket_start = json_content.find('[', pos);
    auto bracket_end = json_content.find(']', bracket_start);
    if (bracket_start == std::string::npos || bracket_end == std::string::npos) return result;

    std::string arr = json_content.substr(bracket_start + 1, bracket_end - bracket_start - 1);
    std::istringstream ss(arr);
    std::string token;
    while (std::getline(ss, token, ',')) {
        size_t start = token.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        token = token.substr(start);
        if (token.empty()) continue;
        try { result.push_back(std::stod(token)); } catch (...) {}
    }
    return result;
}

// 构造 kuavo.json 路径并读取 ruiwo_kp/ruiwo_kd
inline bool loadKpKdFromKuavoJson(const leju::RobotVersion& version, int motor_count,
                                   std::vector<double>& kp_out, std::vector<double>& kd_out) {
    // 构造 robot_type 名称
    std::string robot_type;
    if (version.major() == 1) {
        robot_type = "roban_v" + version.to_string();
    } else {
        robot_type = "kuavo_v" + version.to_string();
    }

    // 按优先级尝试多个路径
    std::vector<std::string> search_paths;

    // 1. ~/.config/lejuconfig/config/<robot_type>/kuavo.json (部署环境)
    const char* sudo_user = getenv("SUDO_USER");
    passwd* pw = sudo_user ? getpwnam(sudo_user) : getpwuid(getuid());
    if (pw) {
        search_paths.push_back(std::string(pw->pw_dir) + "/.config/lejuconfig/config/" + robot_type + "/kuavo.json");
    }

    // 2. 通过 /proc/self/exe 推导源码 config 目录 (开发环境)
    {
        char exe_path[1024];
        ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
        if (len > 0) {
            exe_path[len] = '\0';
            std::string exe_dir(exe_path);
            // 从可执行文件路径向上找项目根目录
            // 典型: .../build_cmake/.../motor_keyboard_ctrl 或 .../devel/lib/.../motor_keyboard_ctrl
            for (auto anchor : {"build_cmake", "build_docker", "build", "devel", "installed"}) {
                auto pos = exe_dir.find(anchor);
                if (pos != std::string::npos) {
                    std::string project_root = exe_dir.substr(0, pos);
                    search_paths.push_back(project_root + "src/leju-hardware/config/" + robot_type + "/kuavo.json");
                    break;
                }
            }
        }
    }

    // 3. 相对当前工作目录 (直接在项目根目录运行)
    search_paths.push_back("src/leju-hardware/config/" + robot_type + "/kuavo.json");

    std::ifstream ifs;
    std::string json_path;
    for (auto& path : search_paths) {
        ifs.open(path);
        if (ifs.is_open()) {
            json_path = path;
            break;
        }
    }
    if (!ifs.is_open()) {
        std::cerr << "kuavo.json 未找到，已搜索:\n";
        for (auto& p : search_paths) std::cerr << "  " << p << "\n";
        return false;
    }

    std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

    auto ruiwo_kp = parseJsonArray(content, "ruiwo_kp");
    auto ruiwo_kd = parseJsonArray(content, "ruiwo_kd");

    if (ruiwo_kp.size() >= static_cast<size_t>(motor_count)) {
        kp_out.assign(ruiwo_kp.begin(), ruiwo_kp.begin() + motor_count);
    } else if (!ruiwo_kp.empty()) {
        kp_out = ruiwo_kp;
        kp_out.resize(motor_count, ruiwo_kp.back());
    } else {
        return false;
    }

    if (ruiwo_kd.size() >= static_cast<size_t>(motor_count)) {
        kd_out.assign(ruiwo_kd.begin(), ruiwo_kd.begin() + motor_count);
    } else if (!ruiwo_kd.empty()) {
        kd_out = ruiwo_kd;
        kd_out.resize(motor_count, ruiwo_kd.back());
    } else {
        return false;
    }

    std::cout << "已从 kuavo.json 加载 kp/kd: " << json_path << "\n";
    return true;
}

inline KeyboardCtrlConfig loadKeyboardCtrlConfig(const std::string& path, int motor_count,
                                                   double default_kp = 100.0, double default_kd = 10.0) {
    KeyboardCtrlConfig cfg;
    cfg.joint_kp.assign(motor_count, default_kp);
    cfg.joint_kd.assign(motor_count, default_kd);
    cfg.action_scale.assign(motor_count, 1.0);

    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "配置文件未找到: " << path << ", 使用默认 kp=" << default_kp << " kd=" << default_kd << "\n";
        return cfg;
    }

    std::string line;
    while (std::getline(ifs, line)) {
        auto semi = line.find(';');
        if (semi != std::string::npos) line = line.substr(0, semi);
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        line = line.substr(start);

        if (line.find("jointKp") == 0) {
            // 跳到 {
            while (std::getline(ifs, line)) { if (line.find('{') != std::string::npos) break; }
            auto m = parseIndexedSection(ifs);
            cfg.joint_kp = indexedMapToVector(m, motor_count, default_kp);
        } else if (line.find("jointKd") == 0) {
            while (std::getline(ifs, line)) { if (line.find('{') != std::string::npos) break; }
            auto m = parseIndexedSection(ifs);
            cfg.joint_kd = indexedMapToVector(m, motor_count, default_kd);
        } else if (line.find("actionScaleTest") == 0) {
            while (std::getline(ifs, line)) { if (line.find('{') != std::string::npos) break; }
            auto m = parseIndexedSection(ifs);
            cfg.action_scale = indexedMapToVector(m, motor_count, 1.0);
        }
    }

    std::cout << "已加载配置: " << path << " (" << motor_count << " 关节)\n";
    return cfg;
}

// ---- 键/索引映射 ----

inline int keyToIndex(int key) {
    if (key >= '0' && key <= '9') return key - '0';
    if (key >= 'a' && key <= 'm') return 10 + (key - 'a');
    return -1;
}

inline char indexToKey(int idx) {
    if (idx < 10) return '0' + idx;
    return 'a' + (idx - 10);
}

// ---- 正弦测试状态 ----

// 每个关节的正弦默认参数 (参考 DanceController)
// { amplitude_rad, phase_rad, offset_rad }
struct SinePreset {
    double amplitude_rad;
    double phase_rad;
    double offset_rad;
};

// DanceController 原始控制模式: 0=CST(主机PD), 2=CSP(电机PD)
// 注意: RUIWO 电机实际全部走 CSP (mode=2)，腿部通过 kp=kd=0 模拟 CST
// 顺序: 0-5 左腿, 6-11 右腿, 12 腰, 13-16 左臂, 17-20 右臂, 21-22 头
inline std::vector<uint8_t> getDanceControlModes(int motor_count) {
    static const uint8_t default_modes[] = {
        0, 0, 0, 0, 0, 0,   // 左腿 CST(主机PD)
        0, 0, 0, 0, 0, 0,   // 右腿 CST(主机PD)
        0,                   // 腰   CST(主机PD)
        0, 0, 0, 0,          // 左臂 CST(主机PD)
        0, 0, 0, 0,          // 右臂 CST(主机PD)
        0, 0,                // 头   CST(零力矩)
    };
    int n = sizeof(default_modes) / sizeof(default_modes[0]);
    std::vector<uint8_t> modes(motor_count, 0);
    for (int i = 0; i < std::min(motor_count, n); ++i)
        modes[i] = default_modes[i];
    return modes;
}

// DanceController 的 Kp/Kd (来自 dance_param.info, Roban 重排后顺序)
// dance_param.info 顺序: waist(0), leg_l(1-6), leg_r(7-12), arm_l(13-16), arm_r(17-20)
// Roban 重排后: leg_l(0-5), leg_r(6-11), waist(12), arm_l(13-16), arm_r(17-20), head(21-22)
struct DanceKpKd {
    double kp;
    double kd;
};

inline std::vector<DanceKpKd> getDanceKpKd(int motor_count) {
    // 已按 Roban 重排后的顺序 (keyboard_ctrl 关节顺序)
    static const DanceKpKd presets[] = {
        // 左腿 (0-5): dance_param [1-6]
        { 90.179,  3.557},   // 0  leg_l1
        {150.098,  8.308},   // 1  leg_l2
        { 40.179,  3.557},   // 2  leg_l3
        {150.098,  8.308},   // 3  leg_l4
        { 34.25,   1.907},   // 4  leg_l5
        { 34.25,   1.907},   // 5  leg_l6
        // 右腿 (6-11): dance_param [7-12]
        { 90.179,  3.557},   // 6  leg_r1
        {150.098,  8.308},   // 7  leg_r2
        { 40.179,  3.557},   // 8  leg_r3
        {150.098,  8.308},   // 9  leg_r4
        { 34.25,   1.907},   // 10 leg_r5
        { 34.25,   1.907},   // 11 leg_r6
        // 腰 (12): dance_param [0]
        { 40.179,  2.557},   // 12 waist
        // 左臂 (13-16): dance_param [13-16]
        { 14.25,   0.907},   // 13 arm_l1
        { 14.25,   0.907},   // 14 arm_l2
        { 14.25,   0.907},   // 15 arm_l3
        { 14.25,   0.907},   // 16 arm_l4
        // 右臂 (17-20): dance_param [17-20]
        { 14.25,   0.907},   // 17 arm_r1
        { 14.25,   0.907},   // 18 arm_r2
        { 14.25,   0.907},   // 19 arm_r3
        { 14.25,   0.907},   // 20 arm_r4
        // 头部 (21-22): DanceController 设为 0
        {  0.0,    0.0  },   // 21 head_yaw
        {  0.0,    0.0  },   // 22 head_pitch
    };
    int n = sizeof(presets) / sizeof(presets[0]);
    std::vector<DanceKpKd> result(motor_count, {0.0, 0.0});
    for (int i = 0; i < std::min(motor_count, n); ++i)
        result[i] = presets[i];
    return result;
}

// DanceController actionScaleTest = 0.25 (全关节统一)
constexpr double kDanceActionScaleTest = 0.25;

// 23 关节默认正弦参数 (单位: 弧度)
// sine 测试默认参数, 适配 AnkleSolver 的关节限位
// 索引: 0-5 左腿, 6-11 右腿, 12 腰, 13-16 左臂, 17-20 右臂, 21-22 头
//
// AnkleSolver::joint_to_motor_position 中的限位:
//   膝关节 [3],[9]: >= 0  → offset=amplitude 使 sine 在 [0, 2*amp] 范围
//   踝 pitch [4],[10]: [-0.88, 0.52] (最严格的 solver)
//   踝 roll  [5],[11]: [-0.26, 0.26] (S2GEN)
//   其他关节: 无软件限位
inline const SinePreset& getDefaultSinePreset(int joint_idx) {
    static constexpr double AMP = 0.125;  // 统一振幅 0.125 rad (≈7.2°)
    static const SinePreset presets[] = {
        // 左腿 (0-5)
        { AMP, 0.0, 0.0 },          // 0  leg_l1 — 无限位
        { AMP, 0.0, 0.0 },          // 1  leg_l2 — 无限位
        { AMP, 0.0, 0.0 },          // 2  leg_l3 — 无限位
        { AMP, 0.0, AMP },          // 3  leg_l4 — 膝关节 >= 0, offset=AMP → [0, 2*AMP]
        { AMP, 0.0, 0.0 },          // 4  leg_l5 — 踝 pitch, 限位宽, 0 在范围内
        { AMP, 0.0, 0.0 },          // 5  leg_l6 — 踝 roll
        // 右腿 (6-11)
        { AMP, 0.0, 0.0 },          // 6  leg_r1
        { AMP, 0.0, 0.0 },          // 7  leg_r2
        { AMP, 0.0, 0.0 },          // 8  leg_r3
        { AMP, 0.0, AMP },          // 9  leg_r4 — 膝关节 >= 0
        { AMP, 0.0, 0.0 },          // 10 leg_r5
        { AMP, 0.0, 0.0 },          // 11 leg_r6
        // 腰 + 手臂 + 头 (12-22) — 无限位
        { AMP, 0.0, 0.0 },          // 12 waist
        { AMP, 0.0, 0.0 },          // 13 arm_l1
        { AMP, 0.0, 0.0 },          // 14 arm_l2
        { AMP, 0.0, 0.0 },          // 15 arm_l3
        { AMP, 0.0, 0.0 },          // 16 arm_l4
        { AMP, 0.0, 0.0 },          // 17 arm_r1
        { AMP, 0.0, 0.0 },          // 18 arm_r2
        { AMP, 0.0, 0.0 },          // 19 arm_r3
        { AMP, 0.0, 0.0 },          // 20 arm_r4
        { 0.0, 0.0, 0.0 },          // 21 head_yaw — 不动
        { 0.0, 0.0, 0.0 },          // 22 head_pitch — 不动
    };
    static const SinePreset fallback = { AMP, 0.0, 0.0 };
    if (joint_idx >= 0 && joint_idx < (int)(sizeof(presets)/sizeof(presets[0])))
        return presets[joint_idx];
    return fallback;
}

struct SineTest {
    bool active = false;
    double amplitude_rad = 0.3;    // 振幅 (rad)
    double frequency_hz = 0.833;   // 频率 (Hz) — T=1.2s, 对应文档要求
    double phase_rad = 0.0;        // 相位偏移 (rad)
    double offset_rad = 0.0;       // 直流偏移 (rad), 叠加在 center_rad 上
    double center_rad = 0.0;       // 激活时的初始位置
    std::chrono::steady_clock::time_point start_time;

    // 用关节默认参数初始化
    void loadPreset(int joint_idx) {
        auto& p = getDefaultSinePreset(joint_idx);
        amplitude_rad = p.amplitude_rad;
        phase_rad = p.phase_rad;
        offset_rad = p.offset_rad;
    }

    double compute(std::chrono::steady_clock::time_point now) const {
        if (!active) return center_rad;
        double t = std::chrono::duration<double>(now - start_time).count();
        return center_rad + offset_rad
             + amplitude_rad * std::sin(2.0 * M_PI * frequency_hz * t + phase_rad);
    }
};

// ---- 阶跃力矩测试 ----

struct StepTest {
    bool active = false;
    double hold_duration_s = 10.0;  // 每级保持时间 (s)
    std::chrono::steady_clock::time_point start_time;

    // 多级阶跃序列: 0 → +A → +2A → 0 → -A → -2A → 0
    std::vector<double> levels;     // 各级力矩值
    int current_level = 0;          // 当前级别索引

    // 用基准力矩 x 构建序列: 0, x/2, 0, x, 0, -x/2, 0, -x
    void buildSequence(double x) {
        levels = {0.0, x/2.0, 0.0, x, 0.0, -x/2.0, 0.0, -x};
        current_level = 0;
    }

    double compute(std::chrono::steady_clock::time_point now) const {
        if (!active || levels.empty()) return 0.0;
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        int idx = static_cast<int>(elapsed / hold_duration_s);
        if (idx >= static_cast<int>(levels.size())) return 0.0;
        return levels[idx];
    }

    bool isExpired(std::chrono::steady_clock::time_point now) const {
        if (!active || levels.empty()) return false;
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        return elapsed >= hold_duration_s * levels.size();
    }

    double remaining(std::chrono::steady_clock::time_point now) const {
        if (!active || levels.empty()) return 0.0;
        double total = hold_duration_s * levels.size();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        return std::max(0.0, total - elapsed);
    }

    // 当前级别索引 (用于显示)
    int currentIndex(std::chrono::steady_clock::time_point now) const {
        if (!active || levels.empty()) return -1;
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        int idx = static_cast<int>(elapsed / hold_duration_s);
        return (idx < static_cast<int>(levels.size())) ? idx : -1;
    }

    // 当前级别的剩余时间
    double levelRemaining(std::chrono::steady_clock::time_point now) const {
        if (!active || levels.empty()) return 0.0;
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        double in_level = std::fmod(elapsed, hold_duration_s);
        return hold_duration_s - in_level;
    }
};

// ---- Chirp 扫频测试 (CST 模式 + 上层 PD) ----
//
// 用法约定:
//   1) compute() 返回 (q_ref, v_ref, f_hz)，调用方用 ctrl_cfg.kp/kd 算 tau:
//        tau = kp_user * (q_ref - fb_q) - kd_user * fb_v   (v_ref 目标取 0, 仅做阻尼)
//      v_ref 仍随结构体返回, 仅供记录/分析, 不参与 tau
//   2) 电机 cmd.modes 须设为 0 (CST)，cmd.kp/kd 须设为 0，避免电机端 PD 重复施加
//   3) 启动后前 ramp_s 秒幅度从 0 线性渐增到 amplitude_rad，避免 q_ref 突变冲击
//
// 频率轨迹: 线性 chirp f(t) = f0 + k*t, k = (f1-f0)/duration
// 相位:     φ(t) = 2π * (f0*t + 0.5*k*t²)
// 输出:     q_ref = center + A(t) * sin(φ),  v_ref = A(t) * 2π * f(t) * cos(φ)
//           A(t) = amplitude_rad * min(1, t/ramp_s)  (ramp_s>0 时；否则恒为 amp)
struct ChirpSweepTest {
    bool   active = false;
    double amplitude_rad = 10.0 * M_PI / 180.0;  // ±10° 默认
    double center_rad    = 0.0;                  // 激活时锁定当前 q
    double freq_start_hz = 0.1;
    double freq_end_hz   = 2.0;
    double duration_s    = 15.0;
    double ramp_s        = 0.5;                  // 幅度软启动时间
    std::chrono::steady_clock::time_point start_time;

    struct Ref { double q_ref; double v_ref; double f_hz; };

    Ref compute(std::chrono::steady_clock::time_point now) const {
        if (!active) return {center_rad, 0.0, freq_start_hz};
        double t = std::chrono::duration<double>(now - start_time).count();
        if (duration_s > 1e-6 && t > duration_s) t = duration_s;

        double k = (duration_s > 1e-6) ? (freq_end_hz - freq_start_hz) / duration_s : 0.0;
        double f = freq_start_hz + k * t;
        double phi = 2.0 * M_PI * (freq_start_hz * t + 0.5 * k * t * t);

        double amp = amplitude_rad;
        if (ramp_s > 1e-6 && t < ramp_s) amp *= (t / ramp_s);

        double q = center_rad + amp * std::sin(phi);
        double v = amp * 2.0 * M_PI * f * std::cos(phi);
        return {q, v, f};
    }

    bool isExpired(std::chrono::steady_clock::time_point now) const {
        if (!active) return false;
        double t = std::chrono::duration<double>(now - start_time).count();
        return duration_s > 1e-6 && t >= duration_s;
    }
};

// ---- CSV 数据记录 ----

class CsvLogger {
public:
    CsvLogger() = default;

    bool open(const std::string& filename, const std::vector<std::string>& motor_names) {
        ofs_.open(filename);
        if (!ofs_.is_open()) return false;
        motor_count_ = motor_names.size();
        motor_names_ = motor_names;
        start_time_ = std::chrono::steady_clock::now();

        // 宽格式表头: 一行一个时间步
        // 列名格式: cmd_p/<电机名>, fb_p/<电机名>, ...
        // '/' 分隔让 PlotJuggler 自动建树
        ofs_ << "time_ms,state_seq,dt_ms";
        const char* prefixes[] = {"cmd_p", "cmd_v", "cmd_kp", "cmd_kd", "cmd_tau",
                                  "fb_p", "fb_v", "fb_tau", "tau_theory"};
        for (const char* prefix : prefixes) {
            for (size_t i = 0; i < motor_count_; ++i) {
                ofs_ << "," << prefix << "/" << motor_names[i];
            }
        }
        ofs_ << "\n";

        return true;
    }

    void log(int state_seq,
             const std::vector<double>& cmd_pos,
             const std::vector<double>& cmd_vel,
             const std::vector<double>& cmd_kp,
             const std::vector<double>& cmd_kd,
             const std::vector<double>& fb_pos,
             const std::vector<double>& fb_vel = {},
             const std::vector<double>& fb_tau = {},
             const std::vector<double>& cmd_tau = {}) {
        if (!ofs_.is_open()) return;
        auto now = std::chrono::steady_clock::now();
        double t_ms = std::chrono::duration<double, std::milli>(now - start_time_).count();

        double dt_ms = 0;
        if (last_time_valid_) {
            dt_ms = std::chrono::duration<double, std::milli>(now - last_time_).count();
        }
        last_time_ = now;
        last_time_valid_ = true;

        auto val = [](const std::vector<double>& v, size_t i) -> double {
            return i < v.size() ? v[i] : 0.0;
        };

        // 一行写完所有关节
        ofs_ << std::fixed << std::setprecision(3) << t_ms
             << "," << state_seq
             << "," << std::setprecision(3) << dt_ms;

        const std::vector<double>* arrays[] = {
            &cmd_pos, &cmd_vel, &cmd_kp, &cmd_kd, &cmd_tau,
            &fb_pos, &fb_vel, &fb_tau
        };
        for (const auto* arr : arrays) {
            for (size_t i = 0; i < motor_count_; ++i) {
                ofs_ << "," << std::setprecision(6) << val(*arr, i);
            }
        }
        // tau_theory = kp * (cmd_p - fb_p) + kd * (cmd_v - fb_v) + cmd_tau
        for (size_t i = 0; i < motor_count_; ++i) {
            double kp = val(cmd_kp, i);
            double kd = val(cmd_kd, i);
            double pos_err = val(cmd_pos, i) - val(fb_pos, i);
            double vel_err = val(cmd_vel, i) - val(fb_vel, i);
            double tau_ff = val(cmd_tau, i);
            double tau_theory = kp * pos_err + kd * vel_err + tau_ff;
            ofs_ << "," << std::setprecision(6) << tau_theory;
        }
        ofs_ << "\n";
    }

    void close() {
        if (ofs_.is_open()) {
            ofs_.flush();
            ofs_.close();
        }
    }

    bool is_open() const { return ofs_.is_open(); }

private:
    std::ofstream ofs_;
    size_t motor_count_ = 0;
    std::vector<std::string> motor_names_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_time_;
    bool last_time_valid_ = false;
};
