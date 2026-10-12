/**
 * 降本版电机自动化测试工具
 *
 * 基于 lejusdk-lowlevel DDS 接口与 hardware_node 通信，
 * 自动执行 Kp/Kd 验证、电流阶跃、Sin Action 等测试用例。
 *
 * 用法:
 *   export ROBOT_VERSION=14
 *   sudo -E ./motor_test_runner --test <kp|kd|current-step|sin-action> --joint <idx> [选项]
 *
 * 测试用例:
 *   kp            TC-KPKD-001: 单独 Kp 验证 (机械固定关节, kd=0)
 *   kd            TC-KPKD-002: 单独 Kd 验证 (机械固定关节, kp=0)
 *   current-step  TC-CURR-001: 静态力矩阶跃电流跟踪误差
 *   sin-action    TC-SIM2REAL-001: 应用层 PD sin action
 *
 * 前提: hardware_node 已启动且状态为 READY_OK
 */

#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-lowlevel/data_types.h"
#include "keyboard_ctrl_utils.h"
#include "motor_test_utils.h"

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
#include <sched.h>

using namespace leju;

static std::atomic<bool> g_running(true);
static void signalHandler(int) { g_running = false; }

// ============================================================
// 测试参数
// ============================================================
struct TestParams {
    std::string test_type;         // kp | kd | current-step | sin-action
    int joint_idx = -1;            // 被测关节索引 (0-based)
    double test_kp = -1;           // Kp 值 (kp/sin-action 测试)
    double test_kd = -1;           // Kd 值 (kd/sin-action 测试)
    std::vector<double> positions = {0.05, 0.10, 0.15, 0.20};  // q_des 扫描 (rad)
    std::vector<double> velocities = {0.5, 1.0, 1.5, 2.0};     // dq_des 扫描 (rad/s)
    std::vector<double> kp_sweep;  // Kp 扫描列表 (kp 测试变体)
    double hold_s = 5.0;           // 每个设定点保持时长
    double settle_s = 2.0;         // 稳态提取前等待
    int freq = 250;                // 控制频率
    std::string csv_file;
    double rated_torque = -1;      // 额定力矩 (current-step)
    double c2t = -1;               // c2t 系数覆盖
    double sine_amp = 0.15;        // sin 振幅 (rad)
    double sine_period = 1.2;      // sin 周期 (s)
    double sine_duration = 30.0;   // sin 持续时长 (s)
    double pos_limit = 0.5;        // 位置安全限制 (rad)
    double ramp_s = 0.5;           // 斜坡过渡时长 (s)
};

// ============================================================
// 共享状态
// ============================================================
struct SharedState {
    std::mutex mtx;
    RobotState latest;
    std::atomic<int> msg_cnt{0};

    std::mutex hw_mtx;
    HardwareState hw_state{HardwareState::UNKNOWN};

    RobotState read() {
        std::lock_guard<std::mutex> lock(mtx);
        return latest;
    }
};

// ============================================================
// 插值回初始位置 (复用 motor_keyboard_ctrl 模式)
// ============================================================
static void interpolateBack(RobotBaseAPI& robot, size_t motor_count,
                            const std::vector<double>& current_pos,
                            const std::vector<double>& init_pos,
                            const std::vector<double>& kp,
                            const std::vector<double>& kd,
                            double duration_sec, int freq) {
    const int sleep_ms = 1000 / freq;
    const double dt = 1.0 / freq;
    std::vector<double> prev = current_pos;
    for (double t = 0.0; t < duration_sec && g_running; t += dt) {
        double phase = t / duration_sec;
        RobotCmd cmd(motor_count);
        for (size_t i = 0; i < motor_count; ++i) {
            cmd.q[i] = current_pos[i] + phase * (init_pos[i] - current_pos[i]);
            cmd.v[i] = (cmd.q[i] - prev[i]) * freq;
            cmd.kp[i] = kp[i];
            cmd.kd[i] = kd[i];
            cmd.modes[i] = 2;  // CSP
            prev[i] = cmd.q[i];
        }
        robot.publishRobotCmd(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
}

// ============================================================
// 构建一帧命令: 非测试关节锁定在初始位置
// ============================================================
static RobotCmd buildBaseCmd(size_t motor_count,
                             const std::vector<double>& init_pos,
                             const std::vector<double>& kp,
                             const std::vector<double>& kd) {
    RobotCmd cmd(motor_count);
    for (size_t i = 0; i < motor_count; ++i) {
        cmd.q[i] = init_pos[i];
        cmd.v[i] = 0;
        cmd.tau[i] = 0;
        cmd.kp[i] = kp[i];
        cmd.kd[i] = kd[i];
        cmd.modes[i] = 2;  // CSP hold
    }
    return cmd;
}

// ============================================================
// TC-KPKD-001: Kp 正确性验证
// ============================================================
static int run_kp_test(RobotBaseAPI& robot, SharedState& ss, CsvLogger& csv,
                       const TestParams& p, size_t motor_count,
                       const std::vector<double>& init_pos,
                       const std::vector<double>& default_kp,
                       const std::vector<double>& default_kd,
                       const std::vector<std::string>& names) {
    int j = p.joint_idx;
    double kp = p.test_kp;
    double c2t = p.c2t;

    std::cout << "\n\033[32m=== TC-KPKD-001: Kp 正确性验证 ===\033[0m\n"
              << "关节: [" << j << "] " << names[j] << "\n"
              << "Kp=" << kp << "  Kd=0  c2t=" << c2t << "\n"
              << "位置序列 (rad):";
    for (auto v : p.positions) std::cout << " " << v;
    std::cout << "\n保持=" << p.hold_s << "s  稳态提取=" << p.settle_s << "s\n\n";

    std::vector<double> x_vals, y_vals;  // (q_des, I_real)
    int seq = 0;
    const int sleep_us = 1000000 / p.freq;

    for (size_t pi = 0; pi < p.positions.size() && g_running; ++pi) {
        double offset = p.positions[pi];
        std::cout << "\033[36m[" << pi+1 << "/" << p.positions.size()
                  << "] q_des_offset=" << std::fixed << std::setprecision(3) << offset << " rad\033[0m\n";

        std::vector<double> tau_samples;
        double target_q = init_pos[j] + offset;

        int total_iters = static_cast<int>((p.ramp_s + p.hold_s) * p.freq);
        int ramp_iters = static_cast<int>(p.ramp_s * p.freq);

        for (int it = 0; it < total_iters && g_running; ++it) {
            auto state = ss.read();
            auto cmd = buildBaseCmd(motor_count, init_pos, default_kp, default_kd);

            // 被测关节: CSP, kd=0, kp=test_kp
            cmd.modes[j] = 2;
            cmd.kp[j] = kp;
            cmd.kd[j] = 0;
            // 斜坡过渡
            double ramp_phase = (it < ramp_iters) ? (double)it / ramp_iters : 1.0;
            cmd.q[j] = init_pos[j] + offset * ramp_phase;
            cmd.v[j] = 0;
            cmd.tau[j] = 0;

            robot.publishRobotCmd(cmd);

            // 记录 (斜坡结束后开始)
            if (it >= ramp_iters) {
                tau_samples.push_back(state.tau[j]);
            }

            // CSV
            csv.log(seq++, cmd.q, cmd.v, cmd.kp, cmd.kd,
                    state.q, state.v, state.tau, cmd.tau);

            // 安全检查
            if (std::fabs(state.q[j] - init_pos[j]) > p.pos_limit) {
                std::cerr << "\033[31m安全中止: 位置偏移 "
                          << std::fabs(state.q[j] - init_pos[j]) << " rad > " << p.pos_limit << "\033[0m\n";
                return 1;
            }

            std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }

        // 稳态提取
        auto steady = motor_test::extractSteadyState(tau_samples, p.freq, p.settle_s);
        double I_real = (std::fabs(c2t) > 1e-6) ? steady.mean / c2t : steady.mean;

        std::cout << "  tau_fb_mean=" << std::fixed << std::setprecision(4) << steady.mean
                  << " Nm  I_real=" << I_real << " A  (±" << std::setprecision(4) << steady.std_dev << ")\n";

        x_vals.push_back(offset);
        y_vals.push_back(I_real);

        // 短暂回零
        if (pi + 1 < p.positions.size()) {
            auto cmd = buildBaseCmd(motor_count, init_pos, default_kp, default_kd);
            cmd.modes[j] = 2; cmd.kp[j] = kp; cmd.kd[j] = 0;
            cmd.q[j] = init_pos[j];
            for (int it = 0; it < static_cast<int>(p.ramp_s * p.freq) && g_running; ++it) {
                robot.publishRobotCmd(cmd);
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
            }
        }
    }

    if (!g_running) return 1;

    // 线性回归
    auto reg = motor_test::linearRegression(x_vals, y_vals);
    double expected_slope = (std::fabs(c2t) > 1e-6) ? kp / c2t : kp;
    auto verdict = motor_test::judgeKpKd(reg, expected_slope);

    std::cout << "\n\033[33m--- Kp 线性回归结果 ---\033[0m\n"
              << "  数据点: " << reg.n << "\n"
              << "  " << verdict.summary << "\n\n";

    // Kp 扫描 (可选)
    if (!p.kp_sweep.empty()) {
        std::cout << "\033[36m--- Kp 扫描 (固定 q_des=0.10 rad) ---\033[0m\n";
        std::vector<double> kp_x, kp_y;
        double fixed_offset = 0.10;

        for (double sweep_kp : p.kp_sweep) {
            if (!g_running) break;
            std::cout << "  Kp=" << sweep_kp << ": ";

            std::vector<double> tau_samples;
            int total_iters = static_cast<int>((p.ramp_s + p.hold_s) * p.freq);
            int ramp_iters = static_cast<int>(p.ramp_s * p.freq);

            for (int it = 0; it < total_iters && g_running; ++it) {
                auto state = ss.read();
                auto cmd = buildBaseCmd(motor_count, init_pos, default_kp, default_kd);
                cmd.modes[j] = 2; cmd.kp[j] = sweep_kp; cmd.kd[j] = 0;
                double ramp = (it < ramp_iters) ? (double)it / ramp_iters : 1.0;
                cmd.q[j] = init_pos[j] + fixed_offset * ramp;
                robot.publishRobotCmd(cmd);
                if (it >= ramp_iters) tau_samples.push_back(state.tau[j]);
                csv.log(seq++, cmd.q, cmd.v, cmd.kp, cmd.kd, state.q, state.v, state.tau, cmd.tau);
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
            }

            auto ss_val = motor_test::extractSteadyState(tau_samples, p.freq, p.settle_s);
            double I = (std::fabs(c2t) > 1e-6) ? ss_val.mean / c2t : ss_val.mean;
            std::cout << "tau=" << std::fixed << std::setprecision(3) << ss_val.mean
                      << " Nm  I=" << I << " A\n";
            kp_x.push_back(sweep_kp);
            kp_y.push_back(I);

            // 回零
            auto cmd0 = buildBaseCmd(motor_count, init_pos, default_kp, default_kd);
            cmd0.modes[j] = 2; cmd0.kp[j] = sweep_kp; cmd0.kd[j] = 0; cmd0.q[j] = init_pos[j];
            for (int it = 0; it < static_cast<int>(p.ramp_s * p.freq) && g_running; ++it) {
                robot.publishRobotCmd(cmd0);
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
            }
        }

        if (kp_x.size() >= 2) {
            auto kp_reg = motor_test::linearRegression(kp_x, kp_y);
            double kp_expected_slope = (std::fabs(c2t) > 1e-6) ? fixed_offset / c2t : fixed_offset;
            auto kp_verdict = motor_test::judgeKpKd(kp_reg, kp_expected_slope);
            std::cout << "  Kp 扫描: " << kp_verdict.summary << "\n";
        }
    }

    return verdict.pass ? 0 : 1;
}

// ============================================================
// TC-KPKD-002: Kd 正确性验证
// ============================================================
static int run_kd_test(RobotBaseAPI& robot, SharedState& ss, CsvLogger& csv,
                       const TestParams& p, size_t motor_count,
                       const std::vector<double>& init_pos,
                       const std::vector<double>& default_kp,
                       const std::vector<double>& default_kd,
                       const std::vector<std::string>& names) {
    int j = p.joint_idx;
    double kd = p.test_kd;
    double c2t = p.c2t;

    std::cout << "\n\033[32m=== TC-KPKD-002: Kd 正确性验证 ===\033[0m\n"
              << "关节: [" << j << "] " << names[j] << "\n"
              << "Kp=0  Kd=" << kd << "  c2t=" << c2t << "\n"
              << "速度序列 (rad/s):";
    for (auto v : p.velocities) std::cout << " " << v;
    std::cout << "\n\n";

    std::vector<double> x_vals, y_vals;
    int seq = 0;
    const int sleep_us = 1000000 / p.freq;

    for (size_t vi = 0; vi < p.velocities.size() && g_running; ++vi) {
        double dq_des = p.velocities[vi];
        std::cout << "\033[36m[" << vi+1 << "/" << p.velocities.size()
                  << "] dq_des=" << std::fixed << std::setprecision(2) << dq_des << " rad/s\033[0m\n";

        std::vector<double> tau_samples;
        int total_iters = static_cast<int>((p.ramp_s + p.hold_s) * p.freq);
        int ramp_iters = static_cast<int>(p.ramp_s * p.freq);

        for (int it = 0; it < total_iters && g_running; ++it) {
            auto state = ss.read();
            auto cmd = buildBaseCmd(motor_count, init_pos, default_kp, default_kd);

            cmd.modes[j] = 2;  // CSP
            cmd.kp[j] = 0;
            cmd.kd[j] = kd;
            cmd.q[j] = init_pos[j];  // 位置不变
            double ramp = (it < ramp_iters) ? (double)it / ramp_iters : 1.0;
            cmd.v[j] = dq_des * ramp;  // 速度设定值
            cmd.tau[j] = 0;

            robot.publishRobotCmd(cmd);
            if (it >= ramp_iters) tau_samples.push_back(state.tau[j]);
            csv.log(seq++, cmd.q, cmd.v, cmd.kp, cmd.kd, state.q, state.v, state.tau, cmd.tau);

            if (std::fabs(state.q[j] - init_pos[j]) > p.pos_limit) {
                std::cerr << "\033[31m安全中止\033[0m\n";
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }

        auto steady = motor_test::extractSteadyState(tau_samples, p.freq, p.settle_s);
        double I_real = (std::fabs(c2t) > 1e-6) ? steady.mean / c2t : steady.mean;
        std::cout << "  tau_fb_mean=" << std::fixed << std::setprecision(4) << steady.mean
                  << " Nm  I_real=" << I_real << " A\n";
        x_vals.push_back(dq_des);
        y_vals.push_back(I_real);

        // 回零速度
        auto cmd0 = buildBaseCmd(motor_count, init_pos, default_kp, default_kd);
        cmd0.modes[j] = 2; cmd0.kp[j] = 0; cmd0.kd[j] = kd; cmd0.v[j] = 0;
        for (int it = 0; it < static_cast<int>(p.ramp_s * p.freq) && g_running; ++it) {
            robot.publishRobotCmd(cmd0);
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }
    }

    if (!g_running) return 1;

    auto reg = motor_test::linearRegression(x_vals, y_vals);
    double expected_slope = (std::fabs(c2t) > 1e-6) ? kd / c2t : kd;
    auto verdict = motor_test::judgeKpKd(reg, expected_slope);
    std::cout << "\n\033[33m--- Kd 线性回归结果 ---\033[0m\n"
              << "  " << verdict.summary << "\n\n";
    return verdict.pass ? 0 : 1;
}

// ============================================================
// TC-CURR-001: 静态力矩阶跃电流跟踪误差
// ============================================================
static int run_current_step_test(RobotBaseAPI& robot, SharedState& ss, CsvLogger& csv,
                                  const TestParams& p, size_t motor_count,
                                  const std::vector<double>& init_pos,
                                  const std::vector<double>& default_kp,
                                  const std::vector<double>& default_kd,
                                  const std::vector<std::string>& names) {
    int j = p.joint_idx;
    double rated = p.rated_torque;

    std::cout << "\n\033[32m=== TC-CURR-001: 静态力矩阶跃电流跟踪误差 ===\033[0m\n"
              << "关节: [" << j << "] " << names[j] << "\n"
              << "额定力矩=" << rated << " Nm  c2t=" << p.c2t << "\n\n";

    const double load_ratios[] = {0.20, 0.50, 0.80, 0.90};
    std::vector<motor_test::CurrentStepVerdict> verdicts;
    int seq = 0;
    const int sleep_us = 1000000 / p.freq;

    for (double ratio : load_ratios) {
        if (!g_running) break;
        double tau_target = ratio * rated;
        std::cout << "\033[36m负载 " << std::fixed << std::setprecision(0) << (ratio * 100)
                  << "% → tau=" << std::setprecision(2) << tau_target << " Nm\033[0m\n";

        std::vector<double> tau_samples;
        int total_iters = static_cast<int>((p.ramp_s + p.hold_s) * p.freq);
        int ramp_iters = static_cast<int>(p.ramp_s * p.freq);

        for (int it = 0; it < total_iters && g_running; ++it) {
            auto state = ss.read();
            auto cmd = buildBaseCmd(motor_count, init_pos, default_kp, default_kd);

            // CST 模式: kp=0, kd=0, 纯力矩
            cmd.modes[j] = 0;
            cmd.kp[j] = 0;
            cmd.kd[j] = 0;
            double ramp = (it < ramp_iters) ? (double)it / ramp_iters : 1.0;
            cmd.tau[j] = tau_target * ramp;
            cmd.q[j] = init_pos[j];

            robot.publishRobotCmd(cmd);
            if (it >= ramp_iters) tau_samples.push_back(state.tau[j]);
            csv.log(seq++, cmd.q, cmd.v, cmd.kp, cmd.kd, state.q, state.v, state.tau, cmd.tau);

            // 位置安全
            if (std::fabs(state.q[j] - init_pos[j]) > p.pos_limit) {
                std::cerr << "\033[31m安全中止: 位置偏移 "
                          << std::fabs(state.q[j] - init_pos[j]) << " rad\033[0m\n";
                // 清零力矩
                cmd.tau[j] = 0;
                robot.publishRobotCmd(cmd);
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }

        auto v = motor_test::judgeCurrentStep(ratio, tau_target, tau_samples, p.freq, p.settle_s);
        std::cout << "  " << v.summary << "\n";
        verdicts.push_back(v);

        // 回零力矩
        for (int it = 0; it < static_cast<int>(p.ramp_s * p.freq) && g_running; ++it) {
            auto cmd = buildBaseCmd(motor_count, init_pos, default_kp, default_kd);
            cmd.modes[j] = 0; cmd.kp[j] = 0; cmd.kd[j] = 0; cmd.tau[j] = 0;
            robot.publishRobotCmd(cmd);
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }
    }

    // 汇总
    bool all_pass = true;
    std::cout << "\n\033[33m--- 电流阶跃汇总 ---\033[0m\n";
    for (auto& v : verdicts) {
        std::cout << "  " << v.summary << "\n";
        if (!v.pass) all_pass = false;
    }
    std::cout << "  总判定: " << (all_pass ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m") << "\n\n";
    return all_pass ? 0 : 1;
}

// ============================================================
// TC-SIM2REAL-001: 应用层 PD sin action
// ============================================================
static int run_sin_action_test(RobotBaseAPI& robot, SharedState& ss, CsvLogger& csv,
                                const TestParams& p, size_t motor_count,
                                const std::vector<double>& init_pos,
                                const std::vector<double>& default_kp,
                                const std::vector<double>& default_kd,
                                const std::vector<std::string>& names) {
    int j = p.joint_idx;
    double app_kp = p.test_kp;
    double app_kd = p.test_kd;
    double A = p.sine_amp;
    double T = p.sine_period;
    double q0 = init_pos[j];

    std::cout << "\n\033[32m=== TC-SIM2REAL-001: 应用层 PD Sin Action ===\033[0m\n"
              << "关节: [" << j << "] " << names[j] << "\n"
              << "app_Kp=" << app_kp << "  app_Kd=" << app_kd << "\n"
              << "振幅=" << A << " rad  周期=" << T << " s  时长=" << p.sine_duration << " s\n\n";

    int seq = 0;
    const int sleep_us = 1000000 / p.freq;
    auto t_start = std::chrono::steady_clock::now();

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - t_start).count();
        if (t >= p.sine_duration) break;

        auto state = ss.read();
        auto cmd = buildBaseCmd(motor_count, init_pos, default_kp, default_kd);

        // 正弦参考
        double omega = 2.0 * M_PI / T;
        double q_des = q0 + A * std::sin(omega * t);
        double dq_des = A * omega * std::cos(omega * t);

        // 应用层 PD 计算力矩
        double q_real = state.q[j];
        double dq_real = state.v[j];
        double tau_cmd = app_kp * (q_des - q_real) + app_kd * (dq_des - dq_real);

        // CST 模式下发应用层计算的力矩
        cmd.modes[j] = 0;  // CST
        cmd.kp[j] = 0;
        cmd.kd[j] = 0;
        cmd.tau[j] = tau_cmd;
        cmd.q[j] = q_des;  // 记录用，不影响 CST 输出
        cmd.v[j] = dq_des;

        robot.publishRobotCmd(cmd);
        csv.log(seq++, cmd.q, cmd.v, cmd.kp, cmd.kd, state.q, state.v, state.tau, cmd.tau);

        // 安全检查
        if (std::fabs(state.q[j] - q0) > p.pos_limit) {
            std::cerr << "\033[31m安全中止: 位置偏移 "
                      << std::fabs(state.q[j] - q0) << " rad\033[0m\n";
            cmd.tau[j] = 0;
            robot.publishRobotCmd(cmd);
            return 1;
        }

        // 10Hz 进度打印
        if (seq % (p.freq / 10) == 0) {
            std::cout << "\r  t=" << std::fixed << std::setprecision(1) << t << "s"
                      << "  q_des=" << std::setprecision(3) << q_des
                      << "  q_real=" << q_real
                      << "  tau_cmd=" << std::setprecision(2) << tau_cmd
                      << "  tau_fb=" << state.tau[j] << "    " << std::flush;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }

    std::cout << "\n\n\033[32mSin action 测试完成\033[0m\n";
    std::cout << "请用 analyze_sim2real_gap.py 分析 CSV 数据\n\n";
    return 0;
}

// ============================================================
// 主程序
// ============================================================
int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    TestParams p;
    std::string config_file;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--test" && i+1 < argc) p.test_type = argv[++i];
        else if (a == "--joint" && i+1 < argc) p.joint_idx = std::stoi(argv[++i]);
        else if (a == "--kp" && i+1 < argc) p.test_kp = std::stod(argv[++i]);
        else if (a == "--kd" && i+1 < argc) p.test_kd = std::stod(argv[++i]);
        else if (a == "--positions" && i+1 < argc) p.positions = motor_test::parseDoubleList(argv[++i]);
        else if (a == "--velocities" && i+1 < argc) p.velocities = motor_test::parseDoubleList(argv[++i]);
        else if (a == "--kp-sweep" && i+1 < argc) p.kp_sweep = motor_test::parseDoubleList(argv[++i]);
        else if (a == "--hold" && i+1 < argc) p.hold_s = std::stod(argv[++i]);
        else if (a == "--settle" && i+1 < argc) p.settle_s = std::stod(argv[++i]);
        else if (a == "--freq" && i+1 < argc) p.freq = std::stoi(argv[++i]);
        else if (a == "--csv" && i+1 < argc) p.csv_file = argv[++i];
        else if (a == "--rated-torque" && i+1 < argc) p.rated_torque = std::stod(argv[++i]);
        else if (a == "--c2t" && i+1 < argc) p.c2t = std::stod(argv[++i]);
        else if (a == "--sine-amp" && i+1 < argc) p.sine_amp = std::stod(argv[++i]);
        else if (a == "--sine-period" && i+1 < argc) p.sine_period = std::stod(argv[++i]);
        else if (a == "--sine-duration" && i+1 < argc) p.sine_duration = std::stod(argv[++i]);
        else if (a == "--pos-limit" && i+1 < argc) p.pos_limit = std::stod(argv[++i]);
        else if (a == "--config" && i+1 < argc) config_file = argv[++i];
        else if (a == "-h" || a == "--help") {
            std::cout << "用法: sudo -E ./motor_test_runner --test <kp|kd|current-step|sin-action> --joint <idx>\n"
                      << "\n测试用例:\n"
                      << "  kp            Kp 正确性验证 (机械固定关节)\n"
                      << "  kd            Kd 正确性验证 (机械固定关节)\n"
                      << "  current-step  静态力矩阶跃电流跟踪误差\n"
                      << "  sin-action    应用层 PD sin action\n"
                      << "\n选项:\n"
                      << "  --joint <idx>          关节索引 (0-22)\n"
                      << "  --kp <val>             Kp 值\n"
                      << "  --kd <val>             Kd 值\n"
                      << "  --positions <list>     q_des 序列 (逗号分隔, rad)\n"
                      << "  --velocities <list>    dq_des 序列 (逗号分隔, rad/s)\n"
                      << "  --kp-sweep <list>      Kp 扫描列表\n"
                      << "  --hold <秒>            保持时长 (默认 5)\n"
                      << "  --settle <秒>          稳态等待 (默认 2)\n"
                      << "  --freq <Hz>            控制频率 (默认 250)\n"
                      << "  --csv <file>           CSV 路径\n"
                      << "  --rated-torque <Nm>    额定力矩\n"
                      << "  --c2t <val>            c2t 系数\n"
                      << "  --sine-amp <rad>       sin 振幅 (默认 0.15)\n"
                      << "  --sine-period <s>      sin 周期 (默认 1.2)\n"
                      << "  --sine-duration <s>    sin 时长 (默认 30)\n"
                      << "  --pos-limit <rad>      位置限制 (默认 0.5)\n";
            return 0;
        }
    }

    // 参数校验
    if (p.test_type.empty()) {
        std::cerr << "\033[31m错误: 请指定 --test <kp|kd|current-step|sin-action>\033[0m\n";
        return 1;
    }
    if (p.joint_idx < 0) {
        std::cerr << "\033[31m错误: 请指定 --joint <index>\033[0m\n";
        return 1;
    }

    // ---- 1. 初始化 SDK ----
    std::cout << "\033[32m=== 电机自动化测试工具 ===\033[0m\n"
              << "测试: " << p.test_type << "  关节: " << p.joint_idx
              << "  频率: " << p.freq << " Hz\n\n";

    if (!GlobalRobot::init_env(RobotVersion::from_env())) {
        std::cerr << "\033[31m初始化 SDK 失败，请检查 ROBOT_VERSION\033[0m\n";
        return 1;
    }

    auto& robot = GlobalRobot::getInstance();
    size_t motor_count = robot.getMotorNumber();
    auto motor_names = robot.getMotorNames();

    if (p.joint_idx >= static_cast<int>(motor_count)) {
        std::cerr << "\033[31m错误: joint " << p.joint_idx << " 超出范围 (0-" << motor_count-1 << ")\033[0m\n";
        return 1;
    }

    // ---- 2. 订阅状态 ----
    SharedState shared;
    shared.latest.resize(motor_count);

    robot.subscribeRobotState([&](const RobotStateConstPtr& rs) {
        std::lock_guard<std::mutex> lock(shared.mtx);
        shared.latest = *rs;
        shared.msg_cnt++;
    });
    robot.subscribeHardwareState([&](const StringDataConstPtr& hs) {
        std::lock_guard<std::mutex> lock(shared.hw_mtx);
        shared.hw_state = String2HwState(hs->data);
    });

    // ---- 3. 等待 READY_OK ----
    std::cout << "等待 hardware_node 就绪..." << std::flush;
    while (g_running) {
        { std::lock_guard<std::mutex> lock(shared.hw_mtx);
          if (shared.hw_state == HardwareState::READY_OK) break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "." << std::flush;
    }
    if (!g_running) return 0;
    std::cout << " OK\n";

    std::cout << "等待状态数据..." << std::flush;
    while (g_running && shared.msg_cnt.load() == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!g_running) return 0;
    std::cout << " OK\n";

    // ---- 4. 记录初始位置 ----
    std::vector<double> init_pos;
    { std::lock_guard<std::mutex> lock(shared.mtx);
      init_pos = shared.latest.q; }

    std::cout << "电机数量: " << motor_count << "\n";
    std::cout << "被测关节: [" << p.joint_idx << "] " << motor_names[p.joint_idx]
              << "  初始位置: " << std::fixed << std::setprecision(3)
              << init_pos[p.joint_idx] << " rad ("
              << std::setprecision(1) << init_pos[p.joint_idx] * 180.0 / M_PI << "°)\n";

    // ---- 5. 加载 kp/kd 配置 ----
    auto ctrl_cfg = loadKeyboardCtrlConfig(config_file, motor_count);
    {
        std::vector<double> json_kp, json_kd;
        if (loadKpKdFromKuavoJson(robot.getRobotVersion(), motor_count, json_kp, json_kd)) {
            ctrl_cfg.joint_kp = std::move(json_kp);
            ctrl_cfg.joint_kd = std::move(json_kd);
        }
    }

    // 测试参数默认值 (从配置读取)
    if (p.test_kp < 0) p.test_kp = ctrl_cfg.joint_kp[p.joint_idx];
    if (p.test_kd < 0) p.test_kd = ctrl_cfg.joint_kd[p.joint_idx];
    if (p.c2t < 0) p.c2t = 4.36;  // 默认 c2t (PA76), 可通过 --c2t 覆盖
    if (p.rated_torque < 0) p.rated_torque = 10.0;  // 默认额定力矩

    // ---- 6. CSV ----
    if (p.csv_file.empty()) {
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        char buf[64]; std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
        p.csv_file = p.test_type + "_j" + std::to_string(p.joint_idx) + "_" + buf + ".csv";
    }

    CsvLogger csv;
    if (!csv.open(p.csv_file, motor_names)) {
        std::cerr << "\033[31mCSV 打开失败: " << p.csv_file << "\033[0m\n";
        return 1;
    }
    std::cout << "CSV: " << p.csv_file << "\n\n";

    // ---- 7. 执行测试 ----
    int result = 1;
    if (p.test_type == "kp") {
        result = run_kp_test(robot, shared, csv, p, motor_count, init_pos,
                             ctrl_cfg.joint_kp, ctrl_cfg.joint_kd, motor_names);
    } else if (p.test_type == "kd") {
        result = run_kd_test(robot, shared, csv, p, motor_count, init_pos,
                             ctrl_cfg.joint_kp, ctrl_cfg.joint_kd, motor_names);
    } else if (p.test_type == "current-step") {
        result = run_current_step_test(robot, shared, csv, p, motor_count, init_pos,
                                       ctrl_cfg.joint_kp, ctrl_cfg.joint_kd, motor_names);
    } else if (p.test_type == "sin-action") {
        result = run_sin_action_test(robot, shared, csv, p, motor_count, init_pos,
                                     ctrl_cfg.joint_kp, ctrl_cfg.joint_kd, motor_names);
    } else {
        std::cerr << "\033[31m未知测试类型: " << p.test_type << "\033[0m\n";
        result = 1;
    }

    // ---- 8. 安全关停 ----
    std::cout << "\033[33m正在回到初始位置...\033[0m\n";
    auto cur_state = shared.read();
    interpolateBack(robot, motor_count, cur_state.q, init_pos,
                    ctrl_cfg.joint_kp, ctrl_cfg.joint_kd, 2.0, p.freq);

    std::cout << "\033[33m停止电机...\033[0m\n";
    robot.publishStopRobot();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    robot.publishStopRobot();

    std::cout << "等待失能..." << std::flush;
    for (int i = 0; i < 12; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::lock_guard<std::mutex> lock(shared.hw_mtx);
        if (shared.hw_state == HardwareState::STOPPED) { std::cout << " OK\n"; break; }
        std::cout << "." << std::flush;
    }
    std::cout << "\n";

    csv.close();
    robot.shutdown();
    std::cout << "CSV: " << p.csv_file << "\n";
    std::cout << (result == 0 ? "\033[32m测试通过\033[0m" : "\033[31m测试未通过\033[0m") << "\n";
    return result;
}
