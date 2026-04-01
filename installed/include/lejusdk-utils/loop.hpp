#pragma once

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <chrono>
#include <sched.h>
#include <pthread.h>
#include <iostream>

namespace leju {

/// @brief 定时循环执行器类
/// @details 创建一个独立的线程，按照指定周期循环执行用户提供的函数
///          支持绑定到指定CPU核心以提高性能和确定性
class Loop {
public:
    /// @brief 循环函数类型定义
    using LoopFunction = std::function<void()>;

    /// @brief 构造函数
    /// @param name 循环器名称，用于标识和调试
    /// @param function 要执行的函数对象
    /// @param period_ms 执行周期，单位毫秒
    /// @param cpu_core 要绑定的CPU核心号，-1表示不绑定
    /// @param sched_priority SCHED_FIFO优先级，-1表示不设置实时调度
    Loop(const std::string& name, LoopFunction function, int period_ms, int cpu_core = -1, int sched_priority = 80);

    /// @brief 析构函数，自动停止循环
    ~Loop();

    /// @brief 禁止拷贝构造
    Loop(const Loop&) = delete;

    /// @brief 禁止拷贝赋值
    Loop& operator=(const Loop&) = delete;

    /// @brief 禁止移动构造
    Loop(Loop&&) = delete;

    /// @brief 禁止移动赋值
    Loop& operator=(Loop&&) = delete;

    /// @brief 启动循环执行器
    /// @details 创建新线程并开始按周期执行函数，如果已经在运行则忽略
    void start();

    /// @brief 停止循环执行器
    /// @details 安全停止循环并等待线程结束，如果已经停止则忽略
    void shutdown();

    /// @brief 获取循环器名称
    /// @return 循环器名称
    std::string getName() const;

    /// @brief 获取执行周期
    /// @return 执行周期，单位毫秒
    int getPeriod() const;

    /// @brief 获取运行状态
    /// @return true表示正在运行，false表示已停止
    bool isRunning() const;

/* @cond INTERNAL */
private:
    std::string name_;
    LoopFunction func_;
    std::chrono::milliseconds period_;
    std::atomic<bool> is_running_{false};
    std::thread worker_thread_;
    int cpu_core_;
    int sched_priority_;

    void setThreadAffinity(int core_id) {
        if (core_id < 0) {
            return;
        }

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core_id, &cpuset);

        pthread_t current_thread = pthread_self();
        int result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);

        if (result != 0) {
            std::cerr << "Failed to bind thread to CPU core " << core_id
                      << ", error code: " << result << std::endl;
        }
    }

    void setRealtimeScheduling(int priority = 80) {
        struct sched_param param;
        param.sched_priority = priority;
        int result = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        if (result != 0) {
            std::cout << "[" << name_ << "] Failed to set SCHED_FIFO (priority="
                      << priority << "), error: " << result << std::endl;
        } else {
            std::cout << "[" << name_ << "] SCHED_FIFO priority=" << priority << " set OK" << std::endl;
        }
    }

    void loopFunction() {
        setThreadAffinity(cpu_core_);
        if (sched_priority_ >= 0) {
            setRealtimeScheduling(sched_priority_);
        }

        auto next_time = std::chrono::steady_clock::now();
        const auto period_dur = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period_);

        while (is_running_.load()) {
            next_time += period_dur;

            func_();

            auto now = std::chrono::steady_clock::now();
            auto behind = now - next_time;

            if (behind.count() > 0) {
                auto func_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(behind + period_dur).count();
                auto overrun_us = std::chrono::duration_cast<std::chrono::microseconds>(behind).count();
                long skipped = behind / period_dur;

                // std::cerr << "[" << name_ << "] overrun: func_elapsed="
                //           << func_elapsed_us << "us, overrun="
                //           << overrun_us << "us, skipping "
                //           << skipped << " cycles" << std::endl;

                // 跳过已经错过的周期，重新对齐到下一个周期边界
                next_time += (skipped + 1) * period_dur;
            }

            std::this_thread::sleep_until(next_time);
        }
    }
/* @endcond */
};

inline Loop::Loop(const std::string& name, LoopFunction function, int period_ms, int cpu_core, int sched_priority)
    : name_(name)
    , func_(std::move(function))
    , period_(std::chrono::milliseconds(period_ms))
    , cpu_core_(cpu_core)
    , sched_priority_(sched_priority) {
}

inline Loop::~Loop() {
    if (is_running_.load()) {
        shutdown();
    }
}

inline void Loop::start() {
    if (is_running_.load()) {
        return;
    }

    is_running_.store(true);
    worker_thread_ = std::thread(&Loop::loopFunction, this);
}

inline void Loop::shutdown() {
    if (!is_running_.load()) {
        return;
    }

    is_running_.store(false);

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

inline std::string Loop::getName() const {
    return name_;
}

inline int Loop::getPeriod() const {
    return period_.count();
}

inline bool Loop::isRunning() const {
    return is_running_.load();
}

} // namespace leju