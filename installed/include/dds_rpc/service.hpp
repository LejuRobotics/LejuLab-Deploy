/**
 * @file service.hpp
 * @brief DDS-RPC 便捷服务包装类
 *
 * 提供更简洁的 API，自动管理 DDS Participant 生命周期。
 *
 * @author lejurobot ohh
 * @version 1.0.0
 */
#pragma once

#include "requester.hpp"
#include "replier.hpp"
#include <memory>

namespace dds_rpc {

/**
 * @brief 服务客户端包装类
 *
 * 封装 Requester，自动管理 DDS Participant。
 * 适用于简单场景，一个客户端对应一个服务。
 *
 * @tparam TRequest 请求类型
 * @tparam TReply 响应类型
 *
 * @example
 * @code
 * ServiceClient<MyRequest, MyReply> client("MyService",
 *     &MyRequest_desc, &MyReply_desc);
 *
 * client.wait_for_service();
 *
 * MyRequest req{};
 * auto reply = client.call(req);
 * @endcode
 */
template<typename TRequest, typename TReply>
class ServiceClient {
public:
    /**
     * @brief 构造函数
     *
     * @param service_name 服务名称
     * @param request_desc 请求类型 DDS 描述符
     * @param reply_desc 响应类型 DDS 描述符
     * @param domain_id DDS 域 ID (默认 0)
     */
    ServiceClient(const std::string& service_name,
                  const dds_topic_descriptor_t* request_desc,
                  const dds_topic_descriptor_t* reply_desc,
                  int domain_id = 0)
        : domain_id_(domain_id)
    {
        participant_ = dds_create_participant(domain_id, nullptr, nullptr);
        if (participant_ < 0) {
            throw RpcException("Failed to create participant");
        }

        requester_ = std::make_unique<Requester<TRequest, TReply>>(
            participant_, service_name, request_desc, reply_desc);
    }

    /**
     * @brief 析构函数
     */
    ~ServiceClient() {
        requester_.reset();
        dds_delete(participant_);
    }

    // 禁用拷贝
    ServiceClient(const ServiceClient&) = delete;
    ServiceClient& operator=(const ServiceClient&) = delete;

    /**
     * @brief 调用服务 (同步)
     *
     * @param request 请求对象
     * @param timeout 超时时间
     * @return 响应对象
     */
    TReply call(TRequest request,
                std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
        return requester_->send_request(std::move(request), timeout);
    }

    /**
     * @brief 等待服务可用
     *
     * @param timeout 超时时间
     * @return 服务是否可用
     */
    bool wait_for_service(std::chrono::milliseconds timeout = std::chrono::seconds(10)) {
        return requester_->wait_for_service(timeout);
    }

    /**
     * @brief 获取底层 Requester
     * @return Requester 引用
     */
    Requester<TRequest, TReply>& requester() { return *requester_; }

    /**
     * @brief 获取 DDS Participant
     * @return Participant 句柄
     */
    dds_entity_t participant() const { return participant_; }

private:
    int domain_id_;
    dds_entity_t participant_;
    std::unique_ptr<Requester<TRequest, TReply>> requester_;
};

/**
 * @brief 服务端包装类
 *
 * 封装 Replier，自动管理 DDS Participant。
 * 适用于简单场景，一个服务端提供一个服务。
 *
 * @tparam TRequest 请求类型
 * @tparam TReply 响应类型
 *
 * @example
 * @code
 * ServiceServer<MyRequest, MyReply> server("MyService",
 *     &MyRequest_desc, &MyReply_desc,
 *     [](const MyRequest& req) {
 *         MyReply reply{};
 *         reply.result = req.data * 2;
 *         return reply;
 *     });
 *
 * server.run();  // 阻塞运行
 * @endcode
 */
template<typename TRequest, typename TReply>
class ServiceServer {
public:
    using Handler = typename Replier<TRequest, TReply>::RequestHandler;

    /**
     * @brief 构造函数
     *
     * @param service_name 服务名称
     * @param request_desc 请求类型 DDS 描述符
     * @param reply_desc 响应类型 DDS 描述符
     * @param handler 请求处理函数
     * @param domain_id DDS 域 ID (默认 0)
     * @param qos_config QoS 配置 (可选，默认不允许重复服务)
     *
     * @throw DuplicateServiceException 服务已存在（当 allow_duplicate_service=false 时）
     */
    ServiceServer(const std::string& service_name,
                  const dds_topic_descriptor_t* request_desc,
                  const dds_topic_descriptor_t* reply_desc,
                  Handler handler,
                  int domain_id = 0,
                  const RpcQosConfig& qos_config = RpcQosConfig())
        : domain_id_(domain_id)
    {
        participant_ = dds_create_participant(domain_id, nullptr, nullptr);
        if (participant_ < 0) {
            throw RpcException("Failed to create participant");
        }

        replier_ = std::make_unique<Replier<TRequest, TReply>>(
            participant_, service_name, request_desc, reply_desc, qos_config);
        replier_->set_handler(std::move(handler));
    }

    /**
     * @brief 析构函数
     */
    ~ServiceServer() {
        replier_.reset();
        dds_delete(participant_);
    }

    // 禁用拷贝
    ServiceServer(const ServiceServer&) = delete;
    ServiceServer& operator=(const ServiceServer&) = delete;

    /**
     * @brief 启动服务 (后台线程)
     */
    void start() { replier_->start(); }

    /**
     * @brief 停止服务
     */
    void stop() { replier_->stop(); }

    /**
     * @brief 在当前线程运行服务 (阻塞)
     */
    void run() { replier_->run(); }

    /**
     * @brief 检查服务是否运行中
     */
    bool is_running() const { return replier_->is_running(); }

    /**
     * @brief 获取已处理请求数
     */
    uint64_t processed_count() const { return replier_->processed_count(); }

    /**
     * @brief 获取底层 Replier
     * @return Replier 引用
     */
    Replier<TRequest, TReply>& replier() { return *replier_; }

    /**
     * @brief 获取 DDS Participant
     * @return Participant 句柄
     */
    dds_entity_t participant() const { return participant_; }

private:
    int domain_id_;
    dds_entity_t participant_;
    std::unique_ptr<Replier<TRequest, TReply>> replier_;
};

} // namespace dds_rpc
