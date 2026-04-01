#ifndef _COMMON_TIME_UTILS_H_
#define _COMMON_TIME_UTILS_H_

#include <chrono>
#include <cstdint>

namespace leju { 
namespace common {

/**
 * @brief 获取当前Unix时间戳（秒）
 *
 * @return double 当前Unix时间戳，单位为秒（自1970-01-01以来）
 */
inline double GetUnixTimestampS() {
    const auto now = std::chrono::system_clock::now();
    return std::chrono::duration<double>(
        now.time_since_epoch()).count();
}

/**
 * @brief 获取当前Unix时间戳（纳秒）
 *
 * @return uint64_t 当前Unix时间戳，单位为纳秒（自1970-01-01以来）
 */
inline uint64_t GetUnixTimestampNs() {
    const auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
}

/**
 * @brief 获取当前Unix时间戳（秒和纳秒分离）
 *
 * @param sec 输出参数，秒部分（自1970-01-01以来）
 * @param nsec 输出参数，纳秒部分（0-999,999,999）
 */
inline void GetUnixTimestampSecNsec(int32_t& sec, uint32_t& nsec) {
    const auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto sec_duration = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nsec_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - sec_duration);

    sec = static_cast<int32_t>(sec_duration.count());
    nsec = static_cast<uint32_t>(nsec_duration.count());
}

/**
 * @brief 获取当前高精度时间戳（纳秒，基于steady_clock）
 *
 * @return uint64_t 当前高精度时间戳，单位为纳秒
 */
inline uint64_t GetSteadyTimestampNs() {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
}

} // namespace common
} // namespace leju
#endif // _COMMON_TIME_UTILS_H_