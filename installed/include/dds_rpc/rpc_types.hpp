/**
 * @file rpc_types.hpp
 * @brief DDS-RPC 框架工具类型定义
 *
 * 本文件定义了 RPC over DDS 框架所需的工具类型，包括：
 * - 类型约束检查 traits (支持 IDL 生成的 C++ 类型)
 * - QoS 配置
 * - 异常定义
 * - 工具函数
 *
 * 注意：基础类型 (SampleIdentity, RequestHeader, ReplyHeader, RemoteExceptionCode)
 * 由 IDL 生成，位于 rpc_idl/rpc_types.hpp
 *
 * @author lejurobot ohh
 * @version 1.0.0
 */
#pragma once

#include <dds/dds.h>
#include <cstdint>
#include <cstring>
#include <string>
#include <random>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <type_traits>
#include <array>

// 包含 IDL 生成的基础类型
#include "rpc_idl/rpc_types.hpp"

namespace dds_rpc {

//=============================================================================
// 工具函数
//=============================================================================

/**
 * @brief 生成随机 GUID
 * @param[out] guid 输出 GUID 数组 (16 字节)
 */
inline void generate_guid(std::array<uint8_t, 16>& guid) {
    static thread_local std::random_device rd;
    static thread_local std::mt19937_64 gen(rd());

    uint64_t* ptr = reinterpret_cast<uint64_t*>(guid.data());
    ptr[0] = gen();
    ptr[1] = gen();
}

/**
 * @brief 生成请求 Topic 名称
 * @param service_name 服务名称
 * @return 请求 Topic 名称 (格式: rq/<service>/Request)
 */
inline std::string request_topic_name(const std::string& service_name) {
    return "rq/" + service_name + "/Request";
}

/**
 * @brief 生成响应 Topic 名称
 * @param service_name 服务名称
 * @return 响应 Topic 名称 (格式: rr/<service>/Reply)
 */
inline std::string reply_topic_name(const std::string& service_name) {
    return "rr/" + service_name + "/Reply";
}

//=============================================================================
// SampleIdentity 哈希支持 (用于 std::unordered_map)
//=============================================================================

/**
 * @brief SampleIdentity 哈希函数对象
 */
struct SampleIdentityHash {
    std::size_t operator()(const SampleIdentity& id) const {
        std::size_t h = 0;
        const auto& guid = id.writer_guid();
        for (int i = 0; i < 16; ++i) {
            h ^= std::hash<uint8_t>{}(guid[i]) << (i % 8);
        }
        h ^= std::hash<int64_t>{}(id.sequence_number()) << 1;
        return h;
    }
};

//=============================================================================
// 类型约束检查 (编译时静态检测)
// 支持 IDL 生成的 C++ 类型 (getter/setter 模式)
//=============================================================================

namespace traits {

/**
 * @brief 检测类型是否有 header() 方法
 */
template<typename T, typename = void>
struct has_header : std::false_type {};

template<typename T>
struct has_header<T, std::void_t<decltype(std::declval<T>().header())>>
    : std::true_type {};

template<typename T>
inline constexpr bool has_header_v = has_header<T>::value;

/**
 * @brief 检测 Request 类型的 header 是否有 request_id() 方法
 */
template<typename T, typename = void>
struct has_request_id : std::false_type {};

template<typename T>
struct has_request_id<T, std::void_t<decltype(std::declval<T>().header().request_id())>>
    : std::true_type {};

template<typename T>
inline constexpr bool has_request_id_v = has_request_id<T>::value;

/**
 * @brief 检测 Reply 类型的 header 是否有 related_request_id() 方法
 */
template<typename T, typename = void>
struct has_related_request_id : std::false_type {};

template<typename T>
struct has_related_request_id<T,
    std::void_t<decltype(std::declval<T>().header().related_request_id())>>
    : std::true_type {};

template<typename T>
inline constexpr bool has_related_request_id_v = has_related_request_id<T>::value;

/**
 * @brief 检测 Reply 类型的 header 是否有 remote_ex() 方法
 */
template<typename T, typename = void>
struct has_remote_ex : std::false_type {};

template<typename T>
struct has_remote_ex<T, std::void_t<decltype(std::declval<T>().header().remote_ex())>>
    : std::true_type {};

template<typename T>
inline constexpr bool has_remote_ex_v = has_remote_ex<T>::value;

/**
 * @brief 验证 Request 类型是否符合 RPC 规范
 *
 * Request 类型必须满足:
 * - 有 header() 方法
 * - header() 有 request_id() 方法
 */
template<typename TRequest>
struct is_valid_request {
    static constexpr bool value =
        has_header_v<TRequest> &&
        has_request_id_v<TRequest>;
};

template<typename TRequest>
inline constexpr bool is_valid_request_v = is_valid_request<TRequest>::value;

/**
 * @brief 验证 Reply 类型是否符合 RPC 规范
 *
 * Reply 类型必须满足:
 * - 有 header() 方法
 * - header() 有 related_request_id() 方法
 * - header() 有 remote_ex() 方法
 */
template<typename TReply>
struct is_valid_reply {
    static constexpr bool value =
        has_header_v<TReply> &&
        has_related_request_id_v<TReply> &&
        has_remote_ex_v<TReply>;
};

template<typename TReply>
inline constexpr bool is_valid_reply_v = is_valid_reply<TReply>::value;

} // namespace traits

//=============================================================================
// 静态断言宏
//=============================================================================

/**
 * @brief 静态断言宏 - 检查 Request 类型
 */
#define DDS_RPC_STATIC_ASSERT_REQUEST(TRequest) \
    static_assert(::dds_rpc::traits::has_header_v<TRequest>, \
        "TRequest must have a 'header()' method"); \
    static_assert(::dds_rpc::traits::has_request_id_v<TRequest>, \
        "TRequest::header() must have a 'request_id()' method")

/**
 * @brief 静态断言宏 - 检查 Reply 类型
 */
#define DDS_RPC_STATIC_ASSERT_REPLY(TReply) \
    static_assert(::dds_rpc::traits::has_header_v<TReply>, \
        "TReply must have a 'header()' method"); \
    static_assert(::dds_rpc::traits::has_related_request_id_v<TReply>, \
        "TReply::header() must have a 'related_request_id()' method"); \
    static_assert(::dds_rpc::traits::has_remote_ex_v<TReply>, \
        "TReply::header() must have a 'remote_ex()' method")

/**
 * @brief 组合检查宏 - 同时检查 Request 和 Reply
 */
#define DDS_RPC_STATIC_ASSERT_TYPES(TRequest, TReply) \
    DDS_RPC_STATIC_ASSERT_REQUEST(TRequest); \
    DDS_RPC_STATIC_ASSERT_REPLY(TReply)

//=============================================================================
// QoS 配置
//=============================================================================

/**
 * @brief RPC QoS 配置
 *
 * 用于配置 RPC 通信的服务质量参数。
 */
struct RpcQosConfig {
    bool reliable = true;                       ///< 是否使用可靠传输
    int32_t history_depth = 100;                ///< 历史深度
    dds_duration_t deadline = DDS_INFINITY;     ///< 截止时间
    dds_duration_t lifespan = DDS_SECS(60);     ///< 生命周期

    // === 服务重复检测配置 ===
    bool allow_duplicate_service = false;       ///< 是否允许同名服务重复注册（默认不允许）
    int32_t discovery_wait_ms = 300;            ///< 服务发现等待时间（毫秒）

    /**
     * @brief 创建 Writer QoS
     * @return QoS 指针 (调用者负责释放)
     */
    dds_qos_t* create_writer_qos() const {
        dds_qos_t* qos = dds_create_qos();
        if (reliable) {
            dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
        }
        dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, history_depth);
        dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
        return qos;
    }

    /**
     * @brief 创建 Reader QoS (用于请求 Reader - 服务端)
     * @return QoS 指针 (调用者负责释放)
     */
    dds_qos_t* create_reader_qos() const {
        dds_qos_t* qos = dds_create_qos();
        if (reliable) {
            dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
        }
        dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, history_depth);
        dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
        return qos;
    }

    /**
     * @brief 创建 Reply Reader QoS (用于响应 Reader - 客户端)
     *
     * 使用 VOLATILE 持久性，避免客户端重启时收到旧的历史响应数据，
     * 这些旧数据可能导致反序列化失败或匹配错误的请求 ID。
     *
     * @return QoS 指针 (调用者负责释放)
     */
    dds_qos_t* create_reply_reader_qos() const {
        dds_qos_t* qos = dds_create_qos();
        if (reliable) {
            dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
        }
        dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, history_depth);
        // 使用 VOLATILE 避免收到历史数据导致段错误
        dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
        return qos;
    }
};

//=============================================================================
// 异常定义
//=============================================================================

/**
 * @brief RPC 异常基类
 */
class RpcException : public std::exception {
public:
    explicit RpcException(const std::string& message,
                          RemoteExceptionCode code = RemoteExceptionCode::REMOTE_EX_UNKNOWN_EXCEPTION)
        : message_(message), code_(code) {}

    const char* what() const noexcept override { return message_.c_str(); }
    RemoteExceptionCode code() const noexcept { return code_; }

private:
    std::string message_;
    RemoteExceptionCode code_;
};

/**
 * @brief 超时异常
 */
class TimeoutException : public RpcException {
public:
    TimeoutException() : RpcException("Request timeout", RemoteExceptionCode::REMOTE_EX_UNKNOWN_EXCEPTION) {}
};

/**
 * @brief 服务未找到异常
 */
class ServiceNotFoundException : public RpcException {
public:
    explicit ServiceNotFoundException(const std::string& service)
        : RpcException("Service not found: " + service, RemoteExceptionCode::REMOTE_EX_UNKNOWN_OPERATION) {}
};

/**
 * @brief 服务已存在异常
 *
 * 当尝试注册一个已存在的服务时抛出。
 * 同一服务名只能有一个 Replier，避免请求被多个服务端同时处理。
 */
class DuplicateServiceException : public RpcException {
public:
    explicit DuplicateServiceException(const std::string& service)
        : RpcException(
            "Service '" + service + "' already exists! "
            "Another Replier is already running. "
            "Only one Replier per service is allowed. "
            "Set RpcQosConfig::allow_duplicate_service=true to disable this check.",
            RemoteExceptionCode::REMOTE_EX_UNSUPPORTED) {}
};

} // namespace dds_rpc
