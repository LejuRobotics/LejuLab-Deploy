/**
 * @file leju_log.h
 * @brief Leju SDK 轻量级日志工具
 * @details 提供简单易用的日志输出宏，支持多级别日志和彩色输出
 */

#ifndef _LEJU_LOG_H_
#define _LEJU_LOG_H_

/* @cond INTERNAL */
#define LEJU_PRINT_LOG 1

#ifdef LEJU_PRINT_LOG

#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>

inline void _leju_log(const char*tag, const char*level, const char* format, ...)
{
    va_list args_size;
    va_start(args_size, format);
    int size = vsnprintf(nullptr, 0, format, args_size);
    va_end(args_size);

    va_list args;
    va_start(args, format);
    char* buffer = new char[size + 1]();
    vsprintf(buffer, format, args);
    va_end(args);

    printf("%s [%s] %s\n", tag, level, buffer);

    if (nullptr != buffer) {
        delete[] buffer;
        buffer = nullptr;
    }
}
/* @endcond */


/**
 * @defgroup BasicLogMacros 基础日志宏
 * @brief 标准日志级别输出宏
 * @{
 */

/// @brief 输出 DEBUG 级别日志
/// @param tag 日志标签，标识模块名
/// @param ... printf 风格的格式化参数
#define LOG_D(tag, ...) _leju_log(tag, "DEBUG", __VA_ARGS__)

/// @brief 输出 TRACE 级别日志
/// @param tag 日志标签，标识模块名
/// @param ... printf 风格的格式化参数
#define LOG_T(tag, ...) _leju_log(tag, "TRACE", __VA_ARGS__)

/// @brief 输出 INFO 级别日志
/// @param tag 日志标签，标识模块名
/// @param ... printf 风格的格式化参数
#define LOG_I(tag, ...) _leju_log(tag, "INFO", __VA_ARGS__)

/// @brief 输出 ERROR 级别日志
/// @param tag 日志标签，标识模块名
/// @param ... printf 风格的格式化参数
#define LOG_E(tag, ...) _leju_log(tag, "ERROR", __VA_ARGS__)

/// @brief 输出 WARN 级别日志
/// @param tag 日志标签，标识模块名
/// @param ... printf 风格的格式化参数
#define LOG_W(tag, ...) _leju_log(tag, "WARN",__VA_ARGS__)
/** @} */

/**
 * @defgroup ColoredLogMacros 彩色日志宏
 * @brief 带终端颜色高亮的日志输出宏
 * @details 使用 ANSI 转义序列实现彩色输出
 * @{
 */

/// @brief 输出成功消息（绿色）
/// @param tag 日志标签，标识模块名
/// @param format printf 风格的格式化字符串
/// @param ... 可变参数列表
#define LOG_SUCCESS(tag, format, ...) LOG_I(tag, "\033[32m" format "\033[0m", ##__VA_ARGS__)

/// @brief 输出警告消息（黄色）
/// @param tag 日志标签，标识模块名
/// @param format printf 风格的格式化字符串
/// @param ... 可变参数列表
#define LOG_WARNING(tag, format, ...) LOG_W(tag, "\033[33m" format "\033[0m", ##__VA_ARGS__)

/// @brief 输出失败消息（红色）
/// @param tag 日志标签，标识模块名
/// @param format printf 风格的格式化字符串
/// @param ... 可变参数列表
#define LOG_FAILURE(tag, format, ...) LOG_E(tag, "\033[31m" format "\033[0m", ##__VA_ARGS__)
/** @} */

/* @cond INTERNAL */

/**
 * @defgroup BannerLogMacros 横幅日志宏
 * @{
 */

/// @brief 输出带装饰分隔线的横幅标题（绿色）
/// @details 输出格式如下：
/// @code
/// ---- Title ----
/// @endcode
/// @param title 横幅标题文本
#define LOG_BANNER(title) do { \
    int title_len = strlen(title); \
    int total_len = title_len + 4; \
    char* banner = new char[total_len * 3 + 15](); \
    strcpy(banner, "\033[32m"); \
    memset(banner + 5, '-', total_len); \
    banner[5 + total_len] = '\n'; \
    banner[5 + total_len + 1] = ' '; \
    strcpy(banner + 5 + total_len + 2, title); \
    banner[5 + total_len + 2 + title_len] = ' '; \
    banner[5 + total_len + 3 + title_len] = '\n'; \
    memset(banner + 5 + total_len + 4 + title_len, '-', total_len); \
    strcpy(banner + 5 + total_len * 2 + 4 + title_len, "\033[0m"); \
    printf("%s\n", banner); \
    delete[] banner; \
} while(0)
/** @} */

/* @endcond */

#else
/* @cond INTERNAL */
#define LOG_I(tag, ...)
#define LOG_D(tag, ...)
#define LOG_E(tag, ...)
#define LOG_T(tag, ...)
#define LOG_W(tag, ...)
#define LOG_SUCCESS(tag, format, ...)
#define LOG_WARNING(tag, format, ...)
#define LOG_FAILURE(tag, format, ...)
#define LOG_BANNER(title)
/* @endcond */

#endif // LEJU_PRINT_LOG

#endif // _LEJU_LOG_H_