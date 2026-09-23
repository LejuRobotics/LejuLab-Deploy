/**
 * 电机键盘遥控工具 (DDS 版)
 *
 * 通过 lejusdk-lowlevel DDS 接口与 hardware_node 通信，
 * 终端按键选择单个电机并增减位置，用于全身电机手动调试。
 *
 * 用法:
 *   export ROBOT_VERSION=14
 *   sudo ./motor_keyboard_ctrl [--step <度>] [--freq <Hz>] [--mode <csp|cst|csv>]
 *
 * 按键:
 *   0-9/a-m: 选择电机    W/S: +/- 位置
 *   [/]: 步长减半/翻倍    Space: 回零    P: 打印详情    Q: 退出
 *   T: 正弦测试开关       E/R: 幅度-/+    F/G: 频率-/+
 *   Y: chirp 扫频开关 (CST, 上层 PD 用 ctrl_cfg.kp/kd, 默认 ±10° / 0.1→2Hz / 15s)
 */

#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-lowlevel/data_types.h"
#include "keyboard_ctrl_utils.h"

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <csignal>
#include <atomic>
#include <mutex>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sched.h>
#include <pthread.h>

using namespace leju;

static std::atomic<bool> g_running(true);
static struct termios g_orig_termios;
static bool g_termios_saved = false;

// ---- termios raw 模式 ----

static void restoreTermios() {
    if (g_termios_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
        g_termios_saved = false;
    }
}

static void enableRawMode() {
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    g_termios_saved = true;

    struct termios raw = g_orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

static int readKey() {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    struct timeval tv = {0, 0};
    if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
        char c;
        if (read(STDIN_FILENO, &c, 1) == 1) return c;
    }
    return -1;
}

// ---- 信号处理 ----

static void signalHandler(int) {
    g_running = false;
}

// shortName, MotorGroup, buildGroups, keyToIndex, indexToKey
// → 已提取到 keyboard_ctrl_utils.h

// ---- 插值回初始位置 ----

static void interpolateBack(RobotBaseAPI& robot, size_t motor_count,
                            const std::vector<double>& current_pos,
                            const std::vector<double>& init_pos,
                            const KeyboardCtrlConfig& cfg, double duration_sec,
                            int freq, uint8_t mode) {
    const double dt = 1.0 / freq;
    const int sleep_ms = 1000 / freq;
    std::vector<double> prev_pos = current_pos;
    for (double t = 0.0; t < duration_sec; t += dt) {
        double phase = t / duration_sec;
        RobotCmd cmd(motor_count);
        for (size_t i = 0; i < motor_count; ++i) {
            cmd.q[i] = current_pos[i] + phase * (init_pos[i] - current_pos[i]);
            cmd.v[i] = (cmd.q[i] - prev_pos[i]) * freq;
            cmd.kp[i] = cfg.joint_kp[i];
            cmd.kd[i] = cfg.joint_kd[i];
            cmd.modes[i] = mode;
            prev_pos[i] = cmd.q[i];
        }
        robot.publishRobotCmd(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
}

// CsvLogger → 已提取到 keyboard_ctrl_utils.h

// SineTest → 已提取到 keyboard_ctrl_utils.h

// ---- 主程序 ----

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 解析参数
    double step_deg = 1.0;
    int ctrl_freq = 250;
    uint8_t ctrl_mode = 2;  // 0=CST, 1=CSV, 2=CSP
    bool dance_mode = false; // dance: 按关节分 CST/CSP (同 DanceController)
    bool hold_mode_cst = false; // --hold-mode cst: 非扫频关节统一 CST + 主机 PD 跟踪 target_pos
    bool bypass_ankle_solver = false; // --bypass-ankle-solver: 绕过踝关节解算，直接下发
    std::string mode_name = "CSP";
    std::string csv_file;
    std::string config_file;
    double cli_kp = -1.0;           // --kp 覆盖所有关节 kp (-1=不覆盖)
    double cli_kd = -1.0;           // --kd 覆盖所有关节 kd
    double step_tau_default = 1.0;  // --step-tau 阶跃默认力矩 (Nm)
    double step_hold_default = 10.0;// --step-hold 阶跃保持时间 (s)
    double sweep_amp_deg     = 10.0;// --sweep-amp chirp 位置幅度 (度)
    double sweep_f0_hz       = 0.1; // --sweep-f0
    double sweep_f1_hz       = 2.0; // --sweep-f1
    double sweep_dur_s       = 15.0;// --sweep-dur

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--step" && i + 1 < argc) {
            step_deg = std::stod(argv[++i]);
        } else if (arg == "--freq" && i + 1 < argc) {
            ctrl_freq = std::stoi(argv[++i]);
            if (ctrl_freq < 10 || ctrl_freq > 1000) {
                std::cerr << "频率范围: 10-1000 Hz\n";
                return 1;
            }
        } else if (arg == "--mode" && i + 1 < argc) {
            std::string m = argv[++i];
            if (m == "cst" || m == "CST") { ctrl_mode = 0; mode_name = "CST"; }
            else if (m == "csv" || m == "CSV") { ctrl_mode = 1; mode_name = "CSV"; }
            else if (m == "csp" || m == "CSP") { ctrl_mode = 2; mode_name = "CSP"; }
            else if (m == "dance" || m == "DANCE") { dance_mode = true; mode_name = "DANCE"; }
            else {
                std::cerr << "无效模式: " << m << " (可选: csp, cst, csv, dance)\n";
                return 1;
            }
        } else if (arg == "--csv" && i + 1 < argc) {
            csv_file = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        } else if (arg == "--kp" && i + 1 < argc) {
            cli_kp = std::stod(argv[++i]);
        } else if (arg == "--kd" && i + 1 < argc) {
            cli_kd = std::stod(argv[++i]);
        } else if (arg == "--step-tau" && i + 1 < argc) {
            step_tau_default = std::stod(argv[++i]);
        } else if (arg == "--step-hold" && i + 1 < argc) {
            step_hold_default = std::stod(argv[++i]);
        } else if (arg == "--sweep-amp" && i + 1 < argc) {
            sweep_amp_deg = std::stod(argv[++i]);
        } else if (arg == "--sweep-f0" && i + 1 < argc) {
            sweep_f0_hz = std::stod(argv[++i]);
        } else if (arg == "--sweep-f1" && i + 1 < argc) {
            sweep_f1_hz = std::stod(argv[++i]);
        } else if (arg == "--sweep-dur" && i + 1 < argc) {
            sweep_dur_s = std::stod(argv[++i]);
        } else if (arg == "--hold-mode" && i + 1 < argc) {
            std::string h = argv[++i];
            if (h == "cst" || h == "CST") hold_mode_cst = true;
            else if (h == "motor" || h == "MOTOR") hold_mode_cst = false;
            else { std::cerr << "无效 --hold-mode: " << h << " (可选: motor, cst)\n"; return 1; }
        } else if (arg == "--bypass-ankle-solver") {
            bypass_ankle_solver = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cerr << "用法: sudo " << argv[0]
                      << " [--step <度>] [--freq <Hz>] [--mode <csp|cst|csv|dance>]\n"
                      << "  --step:      步长 (度/Nm, 默认 1.0)\n"
                      << "  --freq:      控制频率 (Hz, 默认 250)\n"
                      << "  --mode:      控制模式 (csp/cst/csv/dance, 默认 csp)\n"
                      << "  --kp <val>:  覆盖所有关节 Kp\n"
                      << "  --kd <val>:  覆盖所有关节 Kd\n"
                      << "  --step-tau:  阶跃力矩目标 (Nm, 默认 1.0)\n"
                      << "  --step-hold: 阶跃保持时间 (s, 默认 10.0)\n"
                      << "  --sweep-amp: chirp 位置幅度 (度, 默认 10)\n"
                      << "  --sweep-f0:  chirp 起始频率 (Hz, 默认 0.1)\n"
                      << "  --sweep-f1:  chirp 终止频率 (Hz, 默认 2)\n"
                      << "  --sweep-dur: chirp 扫描时长 (s, 默认 15)\n"
                      << "  --hold-mode: 非扫频关节保持策略 (motor|cst, 默认 motor)\n"
                      << "               cst: 全身 CST + 主机侧 PD 跟踪 target_pos,\n"
                      << "                    电机端 kp/kd=0; 覆盖 dance 分支; X(step) 被禁用\n"
                      << "  --bypass-ankle-solver: 绕过踝关节运动学解算, 直接下发关节命令\n"
                      << "  --config:    配置文件路径\n"
                      << "  --csv:       CSV 记录文件\n"
                      << "\n  按键: W/S=位置/力矩  [/]=步长  T=正弦  X=阶跃  Y=chirp扫频  h/H=Kp±10  j/J=Kd±1\n";
            return 0;
        }
    }

    // 默认 CSV 文件名: keyboard_ctrl_<时间戳>.csv
    if (csv_file.empty()) {
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        char buf[64];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
        csv_file = std::string("keyboard_ctrl_") + buf + ".csv";
    }

    // ---- 1. 初始化 SDK ----
    std::cout << "\033[32m=== 电机键盘遥控 (DDS) ===\033[0m\n";
    std::cout << "步长=" << step_deg << "°  频率=" << ctrl_freq << "Hz  模式=" << mode_name;
    if (hold_mode_cst) {
        std::cout << "  \033[1;33mHOLD MODE: CST (host PD to target_pos, motor kp/kd=0)\033[0m";
    }
    if (bypass_ankle_solver) {
        std::cout << "  \033[1;33m[绕过踝关节解算]\033[0m";
    }
    std::cout << "\n" << std::endl;

    if (!GlobalRobot::init_env(RobotVersion::from_env())) {
        std::cerr << "\033[31m初始化 SDK 失败，请检查 ROBOT_VERSION 环境变量\033[0m\n";
        return 1;
    }

    auto& robot = GlobalRobot::getInstance();
    size_t motor_count = robot.getMotorNumber();
    auto motor_names = robot.getMotorNames();

    std::cout << "电机数量: " << motor_count << "\n";

    // ---- 2. 订阅状态 ----
    std::mutex state_mutex;
    RobotState latest_state(motor_count);
    std::atomic<int> state_msg_cnt(0);

    robot.subscribeRobotState([&](const RobotStateConstPtr& rs) {
        std::lock_guard<std::mutex> lock(state_mutex);
        latest_state = *rs;
        state_msg_cnt++;
    });

    std::mutex hw_mutex;
    HardwareState hw_state = HardwareState::UNKNOWN;
    std::atomic<int> hw_msg_cnt(0);

    robot.subscribeHardwareState([&](const StringDataConstPtr& hs) {
        std::lock_guard<std::mutex> lock(hw_mutex);
        hw_state = String2HwState(hs->data);
        hw_msg_cnt++;
    });

    // ---- 3. 等待 hardware_node 就绪 ----
    std::cout << "等待 hardware_node 就绪..." << std::flush;
    while (g_running) {
        {
            std::lock_guard<std::mutex> lock(hw_mutex);
            if (hw_state == HardwareState::READY_OK) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "." << std::flush;
    }
    if (!g_running) return 0;
    std::cout << " OK\n";

    // 等待第一帧状态数据
    std::cout << "等待状态数据..." << std::flush;
    while (g_running && state_msg_cnt.load() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "." << std::flush;
    }
    if (!g_running) return 0;
    std::cout << " OK\n";

    // 记录初始位置
    std::vector<double> init_positions(motor_count);
    std::vector<double> target_pos(motor_count);
    std::vector<double> prev_target_pos(motor_count);
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        init_positions = latest_state.q;
        target_pos = latest_state.q;
        prev_target_pos = target_pos;
    }

    std::cout << "初始化成功，发现 \033[33m" << motor_count << "\033[0m 个电机\n\n";

    // ---- 初始化 CSV 记录 ----
    CsvLogger csv_logger;
    if (csv_logger.open(csv_file, motor_names)) {
        std::cout << "\033[32mCSV 记录: " << csv_file << "\033[0m\n";
    } else {
        std::cerr << "\033[31mCSV 文件打开失败: " << csv_file << "\033[0m\n";
    }

    // ---- 构建分组 ----
    auto groups = buildGroups(motor_names);

    // ---- 设置终端 ----
    enableRawMode();
    std::atexit(restoreTermios);

    // ---- 加载配置文件 (per-joint kp/kd/actionScale) ----
    auto ctrl_cfg = loadKeyboardCtrlConfig(config_file, motor_count);
    auto& actionScaleTestRL = ctrl_cfg.action_scale;

    // 从 kuavo.json 读取 ruiwo_kp/ruiwo_kd 覆盖默认值
    {
        std::vector<double> json_kp, json_kd;
        if (loadKpKdFromKuavoJson(robot.getRobotVersion(), motor_count, json_kp, json_kd)) {
            ctrl_cfg.joint_kp = std::move(json_kp);
            ctrl_cfg.joint_kd = std::move(json_kd);
        }
    }

    // CLI --kp/--kd 覆盖
    if (cli_kp >= 0.0) {
        std::fill(ctrl_cfg.joint_kp.begin(), ctrl_cfg.joint_kp.end(), cli_kp);
        std::cout << "\033[33mCLI override: 所有关节 kp = " << cli_kp << "\033[0m\n";
    }
    if (cli_kd >= 0.0) {
        std::fill(ctrl_cfg.joint_kd.begin(), ctrl_cfg.joint_kd.end(), cli_kd);
        std::cout << "\033[33mCLI override: 所有关节 kd = " << cli_kd << "\033[0m\n";
    }

    std::cout << "关节参数:\n";
    for (size_t i = 0; i < motor_count; ++i) {
        std::cout << "  [" << i << "] " << std::setw(12) << shortName(motor_names[i])
                  << "  kp=" << std::setw(7) << std::fixed << std::setprecision(1) << ctrl_cfg.joint_kp[i]
                  << "  kd=" << std::setw(5) << std::setprecision(1) << ctrl_cfg.joint_kd[i]
                  << "  scale=" << std::setprecision(2) << ctrl_cfg.action_scale[i] << "\n";
    }
    std::cout << "\n";

    int selected = 0;
    double step_rad = step_deg * M_PI / 180.0;
    std::vector<SineTest> sine_tests(motor_count);
    for (size_t i = 0; i < motor_count; ++i) {
        sine_tests[i].loadPreset(i);
    }
    std::vector<StepTest> step_tests(motor_count);
    std::vector<ChirpSweepTest> sweep_tests(motor_count);
    for (size_t i = 0; i < motor_count; ++i) {
        sweep_tests[i].amplitude_rad = sweep_amp_deg * M_PI / 180.0;
        sweep_tests[i].freq_start_hz = sweep_f0_hz;
        sweep_tests[i].freq_end_hz   = sweep_f1_hz;
        sweep_tests[i].duration_s    = sweep_dur_s;
    }

    // dance 模式: 按关节分配控制模式 + 使用 DanceController 的 Kp/Kd
    std::vector<uint8_t> per_joint_modes;
    std::vector<double> manual_tau;  // dance 模式下 W/S 手动力矩偏移 (Nm)
    if (dance_mode) {
        per_joint_modes = getDanceControlModes(motor_count);
        manual_tau.assign(motor_count, 0.0);
        // 覆盖 Kp/Kd 为 DanceController dance_param.info 中的值
        auto dance_gains = getDanceKpKd(motor_count);
        for (size_t i = 0; i < motor_count; ++i) {
            ctrl_cfg.joint_kp[i] = dance_gains[i].kp;
            ctrl_cfg.joint_kd[i] = dance_gains[i].kd;
        }
        std::cout << "Dance 模式: 全身=CST(主机PD), 头=零力矩\n";
        std::cout << "  Kp/Kd: dance_param.info | actionScaleTest: " << kDanceActionScaleTest << "\n";
        std::cout << "  W/S=力矩  X=阶跃  h/H=Kp±10  j/J=Kd±1\n";
    }
    int display_interval = ctrl_freq / 10;  // 10Hz 刷新
    int counter = 0;

    // ---- 设置实时调度 ----
    {
        struct sched_param param;
        param.sched_priority = 20;  // 低于 hardware_node sensor loop (30)
        int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        if (ret != 0) {
            std::cerr << "\033[33m[warn] SCHED_FIFO 设置失败 (需要 sudo 运行)\033[0m\n";
        } else {
            std::cout << "\033[32mSCHED_FIFO priority=20 已设置\033[0m\n";
        }
    }

    // ---- 丢帧检测 ----
    int prev_seq = state_msg_cnt.load();
    int stale_cycles = 0;       // 控制周期内无新状态 (DDS 层)
    int total_cycles = 0;
    auto rate_start = std::chrono::steady_clock::now();
    int rate_start_seq = prev_seq;

    // ---- CANFD 反馈冻结检测 ----
    // 如果电机反馈位置连续多个周期不变，说明 CAN 反馈帧丢了
    std::vector<double> prev_fb_pos(motor_count, 0.0);
    std::vector<int> fb_frozen_cnt(motor_count, 0);      // 连续冻结计数
    std::vector<int> fb_frozen_total(motor_count, 0);     // 累计冻结周期数
    int fb_check_cycles = 0;
    bool fb_pos_inited = false;

    // ---- 4. 控制循环 (250Hz) ----
    while (g_running) {
        int key = readKey();
        if (key != -1) {
            int sel = keyToIndex(key);
            if (sel >= 0 && sel < (int)motor_count) {
                selected = sel;
            } else if (key == 'w' || key == 'W') {
                if (!hold_mode_cst && dance_mode && per_joint_modes[selected] == 0) {
                    manual_tau[selected] += step_deg;  // dance CST: 步长当力矩增量 (Nm)
                } else {
                    target_pos[selected] += step_rad;
                }
            } else if (key == 's' || key == 'S') {
                if (!hold_mode_cst && dance_mode && per_joint_modes[selected] == 0) {
                    manual_tau[selected] -= step_deg;
                } else {
                    target_pos[selected] -= step_rad;
                }
            } else if (key == '[') {
                step_deg = std::max(0.1, step_deg / 2.0);
                step_rad = step_deg * M_PI / 180.0;
            } else if (key == ']') {
                step_deg = std::min(20.0, step_deg * 2.0);
                step_rad = step_deg * M_PI / 180.0;
            } else if (key == ' ') {
                for (size_t i = 0; i < motor_count; ++i) {
                    target_pos[i] = init_positions[i];
                    if (dance_mode) manual_tau[i] = 0.0;
                }
            } else if (key == 'p' || key == 'P') {
                // 打印详情
                std::vector<double> act_pos;
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    act_pos = latest_state.q;
                }
                std::cout << "\033[H\033[J";
                std::cout << "=== 电机详细位置 ===\n";
                for (size_t i = 0; i < motor_count; ++i) {
                    double tgt_deg = target_pos[i] * 180.0 / M_PI;
                    double act_deg = (i < act_pos.size()) ? act_pos[i] * 180.0 / M_PI : 0.0;
                    double err_deg = tgt_deg - act_deg;
                    std::cout << std::fixed << std::setprecision(2)
                              << "  [" << indexToKey(i) << "] "
                              << std::left << std::setw(18) << motor_names[i]
                              << " 目标: " << std::setw(8) << tgt_deg << "°"
                              << " 实际: " << std::setw(8) << act_deg << "°"
                              << " 误差: " << std::setw(8) << err_deg << "°"
                              << "\n";
                }
                std::cout << "\n按任意键继续..." << std::flush;
                // 等按键，持续发送保持位置 (dance CST 发零力矩, 其余保持位置)
                while (g_running && readKey() == -1) {
                    RobotCmd cmd(motor_count);
                    std::vector<double> fb_q_p, fb_v_p;
                    if (hold_mode_cst) {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        fb_q_p = latest_state.q;
                        fb_v_p = latest_state.v;
                    }
                    for (size_t i = 0; i < motor_count; ++i) {
                        if (sweep_tests[i].active) {
                            // 查看详情期间扫频关节挂起为零力矩 CST，避免位置 PD 突然拉
                            cmd.modes[i] = 0;
                            cmd.tau[i] = 0.0;
                            continue;
                        }
                        if (hold_mode_cst) {
                            double fb_pos = (i < fb_q_p.size()) ? fb_q_p[i] : 0.0;
                            double fb_vel = (i < fb_v_p.size()) ? fb_v_p[i] : 0.0;
                            cmd.modes[i] = 0;
                            cmd.q[i]  = target_pos[i];
                            cmd.kp[i] = 0.0;
                            cmd.kd[i] = 0.0;
                            cmd.tau[i] = ctrl_cfg.joint_kp[i] * (target_pos[i] - fb_pos)
                                       - ctrl_cfg.joint_kd[i] * fb_vel;
                            continue;
                        }
                        cmd.modes[i] = dance_mode ? per_joint_modes[i] : ctrl_mode;
                        if (dance_mode && per_joint_modes[i] == 0) {
                            cmd.tau[i] = 0.0;  // CST: 零力矩保持
                        } else {
                            cmd.q[i] = target_pos[i];
                            cmd.kp[i] = ctrl_cfg.joint_kp[i];
                            cmd.kd[i] = ctrl_cfg.joint_kd[i];
                        }
                    }
                    robot.publishRobotCmd(cmd);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / ctrl_freq));
                }
            } else if (key == 't' || key == 'T') {
                auto& st = sine_tests[selected];
                st.active = !st.active;
                if (st.active) {
                    st.center_rad = target_pos[selected];
                    // 重新加载 preset 的 offset (可能被上次激活清零)
                    auto& preset = getDefaultSinePreset(selected);
                    st.offset_rad = preset.offset_rad;
                    // 设初始相位使 t=0 时 sine 输出 = center_rad (当前位置)
                    // center + offset + amp*sin(phase) = center → sin(phase) = -offset/amp
                    if (st.amplitude_rad > 1e-6 && std::abs(st.offset_rad) > 1e-6) {
                        double ratio = -st.offset_rad / st.amplitude_rad;
                        ratio = std::max(-1.0, std::min(1.0, ratio));
                        st.phase_rad = std::asin(ratio);
                    }
                    st.start_time = std::chrono::steady_clock::now();
                } else {
                    target_pos[selected] = st.center_rad;
                }
            } else if (key == 'e' || key == 'E') {
                auto& st = sine_tests[selected];
                st.amplitude_rad = std::max(0.01, st.amplitude_rad - 0.05);
            } else if (key == 'r' || key == 'R') {
                auto& st = sine_tests[selected];
                st.amplitude_rad = std::min(1.0, st.amplitude_rad + 0.05);
            } else if (key == 'f' || key == 'F') {
                auto& st = sine_tests[selected];
                st.frequency_hz = std::max(0.1, st.frequency_hz - 0.1);
            } else if (key == 'g' || key == 'G') {
                auto& st = sine_tests[selected];
                st.frequency_hz = std::min(5.0, st.frequency_hz + 0.1);
            } else if (key == 'x' || key == 'X') {
                if (hold_mode_cst) {
                    // CST hold 闭环 PD 与力矩阶跃互斥, 避免一边推一边拉
                    std::cout << "\r\033[1;33m[X disabled under --hold-mode cst]\033[0m" << std::flush;
                } else {
                    // 多级阶跃测试: 0→+A→+2A→0→-A→-2A，每级保持 step_hold 秒
                    auto& stp = step_tests[selected];
                    if (!stp.active) {
                        double base = (dance_mode && manual_tau[selected] != 0.0)
                                    ? manual_tau[selected] : step_tau_default;
                        stp.buildSequence(base);
                        stp.hold_duration_s = step_hold_default;
                        stp.start_time = std::chrono::steady_clock::now();
                        stp.active = true;
                    } else {
                        stp.active = false;
                    }
                }
            } else if (key == 'y' || key == 'Y') {
                auto& sw = sweep_tests[selected];
                sw.active = !sw.active;
                if (sw.active) {
                    sw.center_rad = target_pos[selected];
                    sw.start_time = std::chrono::steady_clock::now();
                } else {
                    target_pos[selected] = sw.center_rad;
                }
            } else if (key == 'h') {
                ctrl_cfg.joint_kp[selected] = std::max(0.0, ctrl_cfg.joint_kp[selected] - 10.0);
            } else if (key == 'H') {
                ctrl_cfg.joint_kp[selected] += 10.0;
            } else if (key == 'j') {
                ctrl_cfg.joint_kd[selected] = std::max(0.0, ctrl_cfg.joint_kd[selected] - 1.0);
            } else if (key == 'J') {
                ctrl_cfg.joint_kd[selected] += 1.0;
            } else if (key == 'q' || key == 'Q') {
                g_running = false;
                break;
            }
        }

        // 更新正弦测试位置
        auto now_tp = std::chrono::steady_clock::now();
        for (size_t i = 0; i < motor_count; ++i) {
            if (sine_tests[i].active) {
                target_pos[i] = sine_tests[i].compute(now_tp);
            }
        }

        // 发送控制命令
        RobotCmd cmd(motor_count);
        {
            std::vector<double> fb_q, fb_v;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                fb_q = latest_state.q;
                fb_v = latest_state.v;
            }

            for (size_t i = 0; i < motor_count; ++i) {
                // chirp sweep 优先：强制 CST + 上层 PD，无视 dance/ctrl_mode
                if (sweep_tests[i].active) {
                    if (sweep_tests[i].isExpired(now_tp)) {
                        sweep_tests[i].active = false;
                        target_pos[i] = sweep_tests[i].center_rad;
                        // 回退到非 sweep 路径处理本周期
                    } else {
                        double fb_pos = (i < fb_q.size()) ? fb_q[i] : 0.0;
                        double fb_vel = (i < fb_v.size()) ? fb_v[i] : 0.0;
                        auto ref = sweep_tests[i].compute(now_tp);
                        cmd.modes[i] = 0;          // 强制 CST
                        cmd.q[i] = ref.q_ref;      // 仅供 CSV / tau_theory
                        cmd.v[i] = ref.v_ref;
                        cmd.kp[i] = 0.0;           // 电机端 PD 关闭
                        cmd.kd[i] = 0.0;
                        cmd.tau[i] = ctrl_cfg.joint_kp[i] * (ref.q_ref - fb_pos)
                                   - ctrl_cfg.joint_kd[i] * fb_vel;
                        continue;
                    }
                }
                if (hold_mode_cst) {
                    // --hold-mode cst: 全身统一 CST + 主机侧 PD 跟踪 target_pos
                    // 形态与 sweep 路径一致 (电机端 kp/kd=0), 便于扫频调试时口径统一
                    double fb_pos = (i < fb_q.size()) ? fb_q[i] : 0.0;
                    double fb_vel = (i < fb_v.size()) ? fb_v[i] : 0.0;
                    cmd.modes[i] = 0;                                       // CST
                    cmd.q[i]  = target_pos[i];                              // 仅记录用
                    cmd.v[i]  = 0.0;
                    cmd.kp[i] = 0.0;
                    cmd.kd[i] = 0.0;
                    cmd.tau[i] = ctrl_cfg.joint_kp[i] * (target_pos[i] - fb_pos)
                               - ctrl_cfg.joint_kd[i] * fb_vel;             // v_ref = 0
                } else if (dance_mode) {
                    // dance 模式: 按 DanceController 逻辑, 直接发对应 control_mode
                    // 底层 writeCommand 对 RUIWO CST 自动走 CSP 路径模拟
                    double fb_pos = (i < fb_q.size()) ? fb_q[i] : 0.0;
                    double fb_vel = (i < fb_v.size()) ? fb_v[i] : 0.0;

                    cmd.modes[i] = per_joint_modes[i];  // 0=CST, 2=CSP

                    // 自动过期阶跃测试
                    if (step_tests[i].active && step_tests[i].isExpired(now_tp)) {
                        step_tests[i].active = false;
                    }

                    if (per_joint_modes[i] == 0) {
                        // CST (腿+腰): 纯力矩下发，不做主机 PD (避免反馈噪声放大)
                        cmd.q[i] = 0.0;
                        cmd.v[i] = 0.0;
                        cmd.kp[i] = 0.0;
                        cmd.kd[i] = 0.0;
                        if (step_tests[i].active) {
                            cmd.tau[i] = step_tests[i].compute(now_tp);
                        } else {
                            cmd.tau[i] = manual_tau[i];
                        }
                    } else {
                        // CSP (手臂): q=current_pos, tau=kp*(desired-actual), 同 DanceController
                        cmd.q[i] = fb_pos;
                        cmd.v[i] = 0.0;
                        cmd.kp[i] = ctrl_cfg.joint_kp[i];
                        cmd.kd[i] = ctrl_cfg.joint_kd[i];
                        cmd.tau[i] = ctrl_cfg.joint_kp[i] * (target_pos[i] - fb_pos);
                    }
                } else {
                    // 非 dance 模式: 原有 CSP 逻辑
                    cmd.modes[i] = ctrl_mode;
                    cmd.q[i] = target_pos[i];
                    cmd.v[i] = (target_pos[i] - prev_target_pos[i]) * ctrl_freq;
                    cmd.kp[i] = ctrl_cfg.joint_kp[i];
                    cmd.kd[i] = ctrl_cfg.joint_kd[i];
                    cmd.tau[i] = 0.0;
                }
            }
        }
        prev_target_pos = target_pos;
        robot.publishRobotCmd(cmd);

        // CSV 记录
        {
            std::vector<double> fb_pos, fb_vel, fb_tau;
            int seq;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                fb_pos = latest_state.q;
                fb_vel = latest_state.v;
                fb_tau = latest_state.tau;
                seq = state_msg_cnt.load();
            }
            csv_logger.log(seq, cmd.q, cmd.v, cmd.kp, cmd.kd, fb_pos, fb_vel, fb_tau, cmd.tau);
        }

        // 丢帧检测
        {
            int cur_seq = state_msg_cnt.load();
            int delta = cur_seq - prev_seq;
            if (delta == 0) stale_cycles++;
            total_cycles++;
            prev_seq = cur_seq;
        }

        // 显示刷新 (10Hz)
        if (++counter >= display_interval) {
            counter = 0;

            std::vector<double> act_pos;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                act_pos = latest_state.q;
            }

            std::cout << "\033[H\033[J";
            std::cout << "\033[1m=== 电机键盘遥控 (DDS) ===\033[0m  步长: "
                      << std::fixed << std::setprecision(1) << step_deg << "°  "
                      << mode_name << "@" << ctrl_freq << "Hz";
            if (hold_mode_cst) {
                std::cout << "  \033[1;33m[HOLD: CST host-PD]\033[0m";
            }
            std::cout << "  选中: ["
                      << indexToKey(selected) << "] " << motor_names[selected] << "\n";
            std::cout << "kp=" << std::setprecision(1) << ctrl_cfg.joint_kp[selected]
                      << "  kd=" << ctrl_cfg.joint_kd[selected];
            if (dance_mode && per_joint_modes[selected] == 0) {
                std::cout << "  \033[1;36m[TAU] " << std::setprecision(2) << manual_tau[selected]
                          << " Nm (W/S±" << step_deg << ")\033[0m";
            }
            if (step_tests[selected].active) {
                auto& stp = step_tests[selected];
                int idx = stp.currentIndex(now_tp);
                double cur_tau = stp.compute(now_tp);
                std::cout << "  \033[1;31m[STEP " << (idx+1) << "/" << stp.levels.size()
                          << "] " << std::setprecision(2) << cur_tau
                          << " Nm (" << std::setprecision(1) << stp.levelRemaining(now_tp)
                          << "s)\033[0m";
            }
            {
                auto& st = sine_tests[selected];
                if (st.active) {
                    std::cout << "  \033[1;35m[SINE] A="
                              << std::setprecision(3) << st.amplitude_rad
                              << "rad f=" << std::setprecision(1) << st.frequency_hz
                              << "Hz ph=" << std::setprecision(2) << st.phase_rad
                              << " off=" << std::setprecision(2) << st.offset_rad
                              << "\033[0m";
                }
            }
            {
                auto& sw = sweep_tests[selected];
                if (sw.active) {
                    auto ref = sw.compute(now_tp);
                    double t = std::chrono::duration<double>(now_tp - sw.start_time).count();
                    std::cout << "  \033[1;32m[SWEEP] A="
                              << std::fixed << std::setprecision(2) << (sw.amplitude_rad * 180.0 / M_PI)
                              << "° f=" << std::setprecision(2) << ref.f_hz
                              << "Hz t=" << std::setprecision(1) << t
                              << "/" << sw.duration_s << "s"
                              << " q_ref=" << std::setprecision(2) << (ref.q_ref * 180.0 / M_PI) << "°"
                              << "\033[0m";
                }
            }
            std::cout << "\n";
            std::cout << std::string(60, '-') << "\n";

            for (auto& grp : groups) {
                std::cout << " \033[36m[" << grp.label << "]\033[0m\n";
                int col = 0;
                for (int idx : grp.indices) {
                    double pos_deg = (idx < (int)act_pos.size())
                                         ? act_pos[idx] * 180.0 / M_PI
                                         : 0.0;
                    bool is_sel = (idx == selected);

                    bool is_sine = sine_tests[idx].active;
                    std::cout << "   " << indexToKey(idx) << ": "
                              << std::left << std::setw(10) << shortName(motor_names[idx]);
                    if (is_sel) std::cout << "\033[1;33m";
                    else if (is_sine) std::cout << "\033[35m";
                    std::cout << (is_sel ? "►" : (is_sine ? "~" : " "))
                              << std::right << std::fixed << std::setprecision(2)
                              << std::setw(7) << pos_deg << "°";
                    if (is_sel || is_sine) std::cout << "\033[0m";

                    if (++col % 3 == 0) std::cout << "\n";
                }
                if (col % 3 != 0) std::cout << "\n";
            }

            // 通信统计
            {
                auto now_tp2 = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now_tp2 - rate_start).count();
                int cur_seq = state_msg_cnt.load();
                double recv_rate = (elapsed > 0.1) ? (cur_seq - rate_start_seq) / elapsed : 0;
                double stale_pct = (total_cycles > 0) ? 100.0 * stale_cycles / total_cycles : 0;
                std::cout << " \033[36m状态接收: " << std::fixed << std::setprecision(1) << recv_rate << " Hz"
                          << "  无更新周期: " << stale_cycles << "/" << total_cycles
                          << " (" << std::setprecision(1) << stale_pct << "%)\033[0m\n";
            }
            std::cout << std::string(60, '-') << "\n";
            std::cout << " W/S: +/- | [/]: 步长 | Space: 回零 | P: 详情 | Q: 退出\n";
            std::cout << " T: 正弦开关 | E/R: 幅度-/+ | F/G: 频率-/+ | Y: chirp扫频(CST)\n";
            std::cout << std::flush;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / ctrl_freq));
    }

    // ---- 5. 退出: 停止正弦/扫频 → 插值回初始位置 → 停止电机 ----
    for (auto& st : sine_tests) st.active = false;
    for (auto& sw : sweep_tests) {
        if (sw.active) { sw.active = false; }
    }
    restoreTermios();
    std::cout << "\n\033[33m正在回到初始位置...\033[0m\n";
    interpolateBack(robot, motor_count, target_pos, init_positions, ctrl_cfg, 2.0, ctrl_freq, ctrl_mode);

    // 发送停止命令，确保电机 DISABLE
    std::cout << "\033[33m正在停止电机...\033[0m\n";
    robot.publishStopRobot();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    robot.publishStopRobot();

    // 等待 hardware_node 完成电机失能序列
    // disableAll() 需要等每个电机反馈确认 (200ms/motor, 最多23个)
    // 主循环 1s 轮询 + 失能序列 ≈ 6s
    std::cout << "等待电机失能..." << std::flush;
    for (int i = 0; i < 12; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "." << std::flush;
        // 检查 hardware_state 是否已变为 STOPPED
        {
            std::lock_guard<std::mutex> lock(hw_mutex);
            if (hw_state == HardwareState::STOPPED) {
                std::cout << " OK\n";
                break;
            }
        }
    }
    std::cout << "\n";

    csv_logger.close();
    robot.shutdown();
    std::cout << "\033[32m已退出\033[0m\n";
    std::cout << "CSV 数据已保存: " << csv_file << "\n";
    return 0;
}
