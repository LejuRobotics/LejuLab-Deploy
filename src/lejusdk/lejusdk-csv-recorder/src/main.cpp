// lejusdk-csv-recorder
//
// 实时 CSV 记录器: 订阅 /rt/joint_state 和 /rt/joint_cmd,
// 分别写入 <prefix>_state.csv 和 <prefix>_cmd.csv
//
// 典型用法:
//   lejusdk-csv-recorder                       # 写到 ./joint_rec_<ts>_{state,cmd}.csv
//   lejusdk-csv-recorder --dir /tmp            # 放到 /tmp/
//   lejusdk-csv-recorder --prefix run_20260410 # 自定义前缀
//
// CSV 格式 (列顺序按消息结构展开, 每列一个关节 0..N-1):
//   state: t_rec_s, t_hdr_s, q_0..q_N-1, v_0..v_N-1, vd_0..vd_N-1, tau_0..tau_N-1
//   cmd  : t_rec_s, t_hdr_s, q_0..q_N-1, v_0..v_N-1, tau_0..tau_N-1,
//                            kp_0..kp_N-1, kd_0..kd_N-1, mode_0..mode_N-1
//
// t_rec_s 是回调进入时的 wall clock (unix seconds, double), 是对齐 cmd 和 state
// 的主时间轴; t_hdr_s 是消息自带的 header 时间戳 (如果发布端填了的话, 比如
// moveToDefaultPos 当前没填, 会是 0).

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <dds/dds.hpp>

#include "lejusdk-dds-idl/JointCmd.hpp"
#include "lejusdk-dds-idl/JointState.hpp"
#include "lejusdk-topic-pubsub/topic_names.h"
#include "lejusdk-topic-pubsub/topic_subscriber.hpp"

namespace {

std::atomic<bool> g_stop{false};

void signal_handler(int /*sig*/) {
    g_stop.store(true);
}

std::string make_default_prefix() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << "joint_rec_" << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
    return oss.str();
}

double wall_now_seconds() {
    const auto now = std::chrono::system_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}

double header_seconds(int32_t sec, uint32_t nsec) {
    return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

// ---------------------------------------------------------------------------
// StateCsvWriter: 把 leju::msgs::JointState 写成 CSV
// ---------------------------------------------------------------------------
class StateCsvWriter {
public:
    explicit StateCsvWriter(const std::string& path) : path_(path), ofs_(path) {
        if (!ofs_) {
            throw std::runtime_error("Cannot open " + path + " for writing");
        }
        // 提高内部缓冲提升写入吞吐 (1MB)
        ofs_.rdbuf()->pubsetbuf(nullptr, 0);
    }

    void on_message(const leju::msgs::JointState& data) {
        const double t_rec = wall_now_seconds();
        const double t_hdr = header_seconds(data.header_sec(), data.header_nanosec());

        std::lock_guard<std::mutex> lk(mu_);
        if (!header_written_) {
            n_joints_ = data.q().size();
            write_header();
            header_written_ = true;
        }

        ofs_ << std::fixed << std::setprecision(9) << t_rec
             << ',' << std::setprecision(9) << t_hdr;
        write_doubles(data.q());
        write_doubles(data.v());
        write_doubles(data.vd());
        write_doubles(data.tau());
        ofs_ << '\n';
        ++row_count_;

        // 每 500 行 flush 一次, 大约 1 秒一次 (500 Hz 状态)
        if ((row_count_ % 500) == 0) {
            ofs_.flush();
        }
    }

    uint64_t row_count() const { return row_count_.load(); }
    const std::string& path() const { return path_; }
    size_t n_joints() const { return n_joints_; }

private:
    void write_header() {
        ofs_ << "t_rec_s,t_hdr_s";
        write_indexed_columns("q");
        write_indexed_columns("v");
        write_indexed_columns("vd");
        write_indexed_columns("tau");
        ofs_ << '\n';
    }

    void write_indexed_columns(const char* name) {
        for (size_t i = 0; i < n_joints_; ++i) {
            ofs_ << ',' << name << '_' << i;
        }
    }

    void write_doubles(const std::vector<double>& v) {
        for (size_t i = 0; i < n_joints_; ++i) {
            ofs_ << ',';
            if (i < v.size()) {
                ofs_ << std::fixed << std::setprecision(7) << v[i];
            }
        }
    }

    std::string path_;
    std::ofstream ofs_;
    std::mutex mu_;
    bool header_written_ = false;
    size_t n_joints_ = 0;
    std::atomic<uint64_t> row_count_{0};
};

// ---------------------------------------------------------------------------
// CmdCsvWriter: 把 leju::msgs::JointCmd 写成 CSV
// ---------------------------------------------------------------------------
class CmdCsvWriter {
public:
    explicit CmdCsvWriter(const std::string& path) : path_(path), ofs_(path) {
        if (!ofs_) {
            throw std::runtime_error("Cannot open " + path + " for writing");
        }
        ofs_.rdbuf()->pubsetbuf(nullptr, 0);
    }

    void on_message(const leju::msgs::JointCmd& data) {
        const double t_rec = wall_now_seconds();
        const double t_hdr = header_seconds(data.header_sec(), data.header_nanosec());

        std::lock_guard<std::mutex> lk(mu_);
        if (!header_written_) {
            n_joints_ = data.q().size();
            write_header();
            header_written_ = true;
        }

        ofs_ << std::fixed << std::setprecision(9) << t_rec
             << ',' << std::setprecision(9) << t_hdr;
        write_doubles(data.q());
        write_doubles(data.v());
        write_doubles(data.tau());
        write_doubles(data.kp());
        write_doubles(data.kd());
        write_modes(data.modes());
        ofs_ << '\n';
        ++row_count_;

        // cmd 可能跑到 1 kHz, 每 1000 行 (约 1 秒) flush 一次
        if ((row_count_ % 1000) == 0) {
            ofs_.flush();
        }
    }

    uint64_t row_count() const { return row_count_.load(); }
    const std::string& path() const { return path_; }
    size_t n_joints() const { return n_joints_; }

private:
    void write_header() {
        ofs_ << "t_rec_s,t_hdr_s";
        write_indexed_columns("q");
        write_indexed_columns("v");
        write_indexed_columns("tau");
        write_indexed_columns("kp");
        write_indexed_columns("kd");
        write_indexed_columns("mode");
        ofs_ << '\n';
    }

    void write_indexed_columns(const char* name) {
        for (size_t i = 0; i < n_joints_; ++i) {
            ofs_ << ',' << name << '_' << i;
        }
    }

    void write_doubles(const std::vector<double>& v) {
        for (size_t i = 0; i < n_joints_; ++i) {
            ofs_ << ',';
            if (i < v.size()) {
                ofs_ << std::fixed << std::setprecision(7) << v[i];
            }
        }
    }

    void write_modes(const std::vector<uint8_t>& v) {
        for (size_t i = 0; i < n_joints_; ++i) {
            ofs_ << ',';
            if (i < v.size()) {
                ofs_ << static_cast<int>(v[i]);
            }
        }
    }

    std::string path_;
    std::ofstream ofs_;
    std::mutex mu_;
    bool header_written_ = false;
    size_t n_joints_ = 0;
    std::atomic<uint64_t> row_count_{0};
};

// ---------------------------------------------------------------------------
// 参数解析 / 使用说明
// ---------------------------------------------------------------------------
struct Options {
    std::string dir = ".";
    std::string prefix;  // 空表示用默认带时间戳的前缀
    uint32_t domain_id = 0;
};

void print_usage(const char* prog) {
    std::cerr
        << "Usage: " << prog << " [OPTIONS]\n"
        << "\n"
        << "Real-time CSV recorder for /rt/joint_state and /rt/joint_cmd.\n"
        << "Writes two files: <prefix>_state.csv and <prefix>_cmd.csv\n"
        << "\n"
        << "Options:\n"
        << "  --dir DIR       Output directory (default: .)\n"
        << "  --prefix NAME   File prefix (default: joint_rec_<timestamp>)\n"
        << "  --domain ID     DDS domain id (default: 0)\n"
        << "  -h, --help      Show this help\n";
}

bool parse_args(int argc, char** argv, Options& opts) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };
        if (a == "--dir") {
            const char* v = next("--dir");
            if (!v) return false;
            opts.dir = v;
        } else if (a == "--prefix") {
            const char* v = next("--prefix");
            if (!v) return false;
            opts.prefix = v;
        } else if (a == "--domain") {
            const char* v = next("--domain");
            if (!v) return false;
            opts.domain_id = static_cast<uint32_t>(std::stoul(v));
        } else if (a == "-h" || a == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            std::cerr << "Unknown argument: " << a << "\n";
            print_usage(argv[0]);
            return false;
        }
    }
    if (opts.prefix.empty()) {
        opts.prefix = make_default_prefix();
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Options opts;
    if (!parse_args(argc, argv, opts)) {
        return 1;
    }

    const std::string state_path = opts.dir + "/" + opts.prefix + "_state.csv";
    const std::string cmd_path   = opts.dir + "/" + opts.prefix + "_cmd.csv";

    std::cerr << "[csv-recorder] state -> " << state_path << "\n"
              << "[csv-recorder] cmd   -> " << cmd_path << "\n"
              << "[csv-recorder] domain=" << opts.domain_id << "\n";

    // 捕获 Ctrl+C / SIGTERM 优雅退出
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        StateCsvWriter state_writer(state_path);
        CmdCsvWriter   cmd_writer(cmd_path);

        dds::domain::DomainParticipant participant(opts.domain_id);

        // 订阅 /rt/joint_state
        auto state_sub =
            std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::JointState>>(
                participant,
                leju::dds_topics::kJointState,
                [&state_writer](const leju::msgs::JointState& data) {
                    try {
                        state_writer.on_message(data);
                    } catch (const std::exception& e) {
                        std::cerr << "[csv-recorder] state write error: " << e.what() << "\n";
                    }
                });

        // 订阅 /rt/joint_cmd
        auto cmd_sub =
            std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::JointCmd>>(
                participant,
                leju::dds_topics::kJointCmd,
                [&cmd_writer](const leju::msgs::JointCmd& data) {
                    try {
                        cmd_writer.on_message(data);
                    } catch (const std::exception& e) {
                        std::cerr << "[csv-recorder] cmd write error: " << e.what() << "\n";
                    }
                });

        std::cerr << "[csv-recorder] recording. Press Ctrl+C to stop.\n";

        uint64_t last_state = 0;
        uint64_t last_cmd = 0;
        auto last_print = std::chrono::steady_clock::now();
        while (!g_stop.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 每 5 秒打一次简要进度, 方便确认还在跑
            auto now = std::chrono::steady_clock::now();
            if (now - last_print >= std::chrono::seconds(5)) {
                const uint64_t s = state_writer.row_count();
                const uint64_t c = cmd_writer.row_count();
                std::cerr << "[csv-recorder] state +" << (s - last_state)
                          << " (total " << s << ")  cmd +" << (c - last_cmd)
                          << " (total " << c << ")\n";
                last_state = s;
                last_cmd = c;
                last_print = now;
            }
        }

        std::cerr << "[csv-recorder] stopping... final: state="
                  << state_writer.row_count() << " rows, cmd="
                  << cmd_writer.row_count() << " rows\n";

        // writers 析构时会关闭文件 (flush)
    } catch (const std::exception& e) {
        std::cerr << "[csv-recorder] fatal: " << e.what() << "\n";
        return 1;
    }

    std::cerr << "[csv-recorder] done.\n";
    return 0;
}
