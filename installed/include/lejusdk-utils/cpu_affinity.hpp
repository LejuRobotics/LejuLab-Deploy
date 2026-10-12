#pragma once

#include <pthread.h>
#include <sched.h>
#include <iostream>

namespace leju {
namespace cpu {

// RK3588: CPU 0-3 小核(A55), CPU 4-7 大核(A76)
inline constexpr int kRk3588BigCoreFirst = 4;
inline constexpr int kRk3588BigCoreLast = 7;

inline bool isBigCore(int cpu_id) {
    return cpu_id >= kRk3588BigCoreFirst && cpu_id <= kRk3588BigCoreLast;
}

inline int getCurrentCpu() {
    return sched_getcpu();
}

inline bool bindCurrentThreadToCpuRange(int first_core, int last_core) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (int i = first_core; i <= last_core; ++i) {
        CPU_SET(i, &cpuset);
    }
    const int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        std::cerr << "Failed to bind thread to CPU " << first_core << "-"
                  << last_core << ", error: " << rc << std::endl;
        return false;
    }
    return true;
}

inline bool bindCurrentThreadToBigCores() {
    return bindCurrentThreadToCpuRange(kRk3588BigCoreFirst, kRk3588BigCoreLast);
}

}  // namespace cpu
}  // namespace leju
