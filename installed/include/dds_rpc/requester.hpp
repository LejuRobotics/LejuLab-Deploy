/**
 * @file requester.hpp
 * @brief DDS-RPC Requester (客户端) 实现
 *
 * 纯同步实现，没有后台线程，没有竞态条件。
 * 如果需要并发，用户可以：
 * 1. 每个线程创建独立的 Requester 实例（推荐）
 * 2. 使用互斥锁保护单个 Requester 的访问
 * 3. 使用连接池模式
 *
 * @author lejurobot ohh
 * @version 2.0.0
 */
#pragma once

#include "rpc_types.hpp"
#include <optional>
#include <thread>
#include <cstring>
#include <iostream>

// CycloneDDS C++ TopicTraits support
#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace dds_rpc {

/**
 * @brief 请求者模板类（纯同步版本）
 *
 * 用于发送请求并接收响应的客户端组件。
 * 没有后台线程，所有操作在调用线程中完成。
 *
 * @tparam TRequest 请求类型 (必须包含符合规范的 header 成员)
 * @tparam TReply 响应类型 (必须包含符合规范的 header 成员)
 *
 * @example
 * @code
 * Requester<MyRequest, MyReply> requester(participant, "MyService");
 *
 * MyRequest req{};
 * req.data = 42;
 * auto reply = requester.send_request(req);  // 阻塞等待
 * @endcode
 */
template<typename TRequest, typename TReply>
class Requester {
    DDS_RPC_STATIC_ASSERT_TYPES(TRequest, TReply);

public:
    using RequestType = TRequest;
    using ReplyType = TReply;

    /**
     * @brief 构造函数 (C++ 类型，推荐使用)
     *
     * @param participant DDS Participant
     * @param service_name 服务名称
     * @param qos_config QoS 配置 (可选)
     *
     * @throw RpcException 创建 DDS 实体失败
     */
    Requester(dds_entity_t participant,
              const std::string& service_name,
              const RpcQosConfig& qos_config = RpcQosConfig())
        : participant_(participant)
        , service_name_(service_name)
        , sequence_number_(0)
    {
        using org::eclipse::cyclonedds::topic::TopicTraits;

        generate_guid(client_guid_);

        // 创建请求 Topic
        std::string req_topic_name = request_topic_name(service_name);
        ddsi_sertype* req_sertype = TopicTraits<TRequest>::getSerType();
        request_topic_ = dds_create_topic_sertype(
            participant, req_topic_name.c_str(), &req_sertype, nullptr, nullptr, nullptr);
        if (request_topic_ < 0) {
            throw RpcException("Failed to create request topic: " + req_topic_name);
        }

        // 创建请求 Writer
        dds_qos_t* writer_qos = qos_config.create_writer_qos();
        request_writer_ = dds_create_writer(participant, request_topic_, writer_qos, nullptr);
        dds_delete_qos(writer_qos);
        if (request_writer_ < 0) {
            throw RpcException("Failed to create request writer");
        }

        // 创建响应 Topic
        std::string rep_topic_name = reply_topic_name(service_name);
        ddsi_sertype* rep_sertype = TopicTraits<TReply>::getSerType();
        reply_topic_ = dds_create_topic_sertype(
            participant, rep_topic_name.c_str(), &rep_sertype, nullptr, nullptr, nullptr);
        if (reply_topic_ < 0) {
            throw RpcException("Failed to create reply topic: " + rep_topic_name);
        }

        // 创建响应 Reader (使用 VOLATILE QoS 避免收到历史数据)
        dds_qos_t* reader_qos = qos_config.create_reply_reader_qos();
        reply_reader_ = dds_create_reader(participant, reply_topic_, reader_qos, nullptr);
        dds_delete_qos(reader_qos);
        if (reply_reader_ < 0) {
            throw RpcException("Failed to create reply reader");
        }

        // 创建 WaitSet 用于等待响应
        waitset_ = dds_create_waitset(participant);
        read_condition_ = dds_create_readcondition(reply_reader_, DDS_ANY_STATE);
        dds_waitset_attach(waitset_, read_condition_, 0);

        // 设置 status mask 并附加 writer 到 waitset（用于服务发现）
        // 注意：reply_reader 的数据通过 read_condition_ 监听，这里只监听 match 状态
        dds_set_status_mask(request_writer_, DDS_PUBLICATION_MATCHED_STATUS);
        dds_waitset_attach(waitset_, request_writer_, 1);
    }

    /**
     * @brief 构造函数 (C 类型描述符，向后兼容)
     *
     * @param participant DDS Participant
     * @param service_name 服务名称
     * @param request_desc 请求类型 DDS 描述符
     * @param reply_desc 响应类型 DDS 描述符
     * @param qos_config QoS 配置 (可选)
     */
    Requester(dds_entity_t participant,
              const std::string& service_name,
              const dds_topic_descriptor_t* request_desc,
              const dds_topic_descriptor_t* reply_desc,
              const RpcQosConfig& qos_config = RpcQosConfig())
        : participant_(participant)
        , service_name_(service_name)
        , sequence_number_(0)
    {
        generate_guid(client_guid_);

        // 创建请求 Topic
        std::string req_topic_name = request_topic_name(service_name);
        request_topic_ = dds_create_topic(
            participant, request_desc, req_topic_name.c_str(), nullptr, nullptr);
        if (request_topic_ < 0) {
            throw RpcException("Failed to create request topic: " + req_topic_name);
        }

        // 创建请求 Writer
        dds_qos_t* writer_qos = qos_config.create_writer_qos();
        request_writer_ = dds_create_writer(participant, request_topic_, writer_qos, nullptr);
        dds_delete_qos(writer_qos);
        if (request_writer_ < 0) {
            throw RpcException("Failed to create request writer");
        }

        // 创建响应 Topic
        std::string rep_topic_name = reply_topic_name(service_name);
        reply_topic_ = dds_create_topic(
            participant, reply_desc, rep_topic_name.c_str(), nullptr, nullptr);
        if (reply_topic_ < 0) {
            throw RpcException("Failed to create reply topic: " + rep_topic_name);
        }

        // 创建响应 Reader (使用 VOLATILE QoS 避免收到历史数据)
        dds_qos_t* reader_qos = qos_config.create_reply_reader_qos();
        reply_reader_ = dds_create_reader(participant, reply_topic_, reader_qos, nullptr);
        dds_delete_qos(reader_qos);
        if (reply_reader_ < 0) {
            throw RpcException("Failed to create reply reader");
        }

        // 创建 WaitSet 用于等待响应
        waitset_ = dds_create_waitset(participant);
        read_condition_ = dds_create_readcondition(reply_reader_, DDS_ANY_STATE);
        dds_waitset_attach(waitset_, read_condition_, 0);

        // 设置 status mask 并附加 writer 到 waitset（用于服务发现）
        // 注意：reply_reader 的数据通过 read_condition_ 监听，这里只监听 match 状态
        dds_set_status_mask(request_writer_, DDS_PUBLICATION_MATCHED_STATUS);
        dds_waitset_attach(waitset_, request_writer_, 1);
    }

    /**
     * @brief 析构函数
     *
     * 简单直接，没有线程需要等待。
     */
    ~Requester() {
        // 先 detach 所有 conditions 和实体
        dds_waitset_detach(waitset_, read_condition_);
        dds_waitset_detach(waitset_, request_writer_);
        // 删除 conditions
        dds_delete(read_condition_);
        // 删除 waitset 和 DDS 实体
        dds_delete(waitset_);
        dds_delete(reply_reader_);
        dds_delete(request_writer_);
        dds_delete(reply_topic_);
        dds_delete(request_topic_);
    }

    // 禁用拷贝
    Requester(const Requester&) = delete;
    Requester& operator=(const Requester&) = delete;

    /**
     * @brief 获取服务名称
     * @return 服务名称
     */
    const std::string& service_name() const { return service_name_; }

    /**
     * @brief 同步发送请求
     *
     * 发送请求并阻塞等待响应。
     *
     * @param request 请求对象 (header 会被自动填充)
     * @param timeout 超时时间 (默认 5 秒)
     * @return 响应对象
     *
     * @throw TimeoutException 超时
     * @throw RpcException 发送失败或远程异常
     */
    TReply send_request(TRequest request,
                        std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
        // 填充请求头
        SampleIdentity req_id = fill_request_header(request);

        // 发送请求
        dds_return_t rc = dds_write(request_writer_, &request);
        if (rc != DDS_RETCODE_OK) {
            throw RpcException("Failed to send request: " + std::to_string(rc));
        }

        // 等待并接收响应
        return wait_for_reply(req_id, timeout);
    }

    /**
     * @brief 等待服务可用（事件驱动）
     *
     * 检查双向匹配：
     * 1. request_writer 有订阅者（Replier 的 request_reader）
     * 2. reply_reader 有发布者（Replier 的 reply_writer）
     *
     * 使用 DDS StatusCondition 事件驱动，而非轮询。
     *
     * @param timeout 超时时间
     * @return 服务是否可用
     */
    bool wait_for_service(std::chrono::milliseconds timeout = std::chrono::seconds(10)) {
        auto deadline = std::chrono::steady_clock::now() + timeout;

        while (std::chrono::steady_clock::now() < deadline) {
            // 检查请求方向：request_writer 是否有订阅者
            bool request_matched = false;
            dds_publication_matched_status_t pub_status;
            if (dds_get_publication_matched_status(request_writer_, &pub_status) == DDS_RETCODE_OK) {
                request_matched = pub_status.current_count > 0;
            }

            // 检查响应方向：reply_reader 是否有发布者
            bool reply_matched = false;
            dds_subscription_matched_status_t sub_status;
            if (dds_get_subscription_matched_status(reply_reader_, &sub_status) == DDS_RETCODE_OK) {
                reply_matched = sub_status.current_count > 0;
            }

            // 双向都匹配才算服务可用
            if (request_matched && reply_matched) {
                // 首次 match 需要等待让 DDS 连接完全建立
                if (!service_matched_) {
                    service_matched_ = true;
                    std::this_thread::sleep_for(std::chrono::microseconds(500));
                }
                return true;
            }

            // 计算剩余等待时间
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            if (remaining.count() <= 0) {
                break;
            }

            // 事件驱动等待：阻塞直到 status 变化或超时
            dds_attach_t triggered[3];
            dds_waitset_wait(waitset_, triggered, 3, DDS_MSECS(remaining.count()));
            // 被唤醒后循环检查状态
        }
        return false;
    }

private:
    /**
     * @brief 填充请求头
     */
    SampleIdentity fill_request_header(TRequest& request) {
        request.header().request_id().writer_guid(client_guid_);
        request.header().request_id().sequence_number(++sequence_number_);
        request.header().instance_name(service_name_);
        return request.header().request_id();
    }

    /**
     * @brief 等待指定请求的响应
     */
    TReply wait_for_reply(const SampleIdentity& req_id,
                          std::chrono::milliseconds timeout) {
        auto start_time = std::chrono::steady_clock::now();
        auto deadline = start_time + timeout;
        int poll_count = 0;        // waitset 轮询次数
        int data_events = 0;       // 收到数据事件次数
        int samples_read = 0;      // 读取的样本总数
        int mismatched_samples = 0; // 不匹配的样本数

        while (std::chrono::steady_clock::now() < deadline) {
            // 计算剩余等待时间
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            if (remaining.count() <= 0) {
                break;
            }

            // 等待数据可用
            dds_attach_t triggered;
            dds_return_t rc = dds_waitset_wait(waitset_, &triggered, 1,
                DDS_MSECS(std::min(remaining.count(), int64_t(100))));
            ++poll_count;

            if (rc > 0) {
                ++data_events;
                // 有数据，尝试读取匹配的响应
                auto result = try_read_reply(req_id, samples_read, mismatched_samples);
                if (result) {
                    return *result;
                }
            }
        }

        // 超时，输出诊断信息
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        std::cerr << "[Requester] " << service_name_ << " timeout: "
                  << "elapsed=" << elapsed_ms << "ms, "
                  << "polls=" << poll_count << ", "
                  << "data_events=" << data_events << ", "
                  << "samples_read=" << samples_read << ", "
                  << "mismatched=" << mismatched_samples << ", "
                  << "seq=" << req_id.sequence_number()
                  << std::endl;
        throw TimeoutException();
    }

    /**
     * @brief 尝试读取匹配的响应
     */
    std::optional<TReply> try_read_reply(const SampleIdentity& req_id,
                                          int& samples_read,
                                          int& mismatched_samples) {
        void* samples[16];
        dds_sample_info_t infos[16];
        std::memset(samples, 0, sizeof(samples));

        int32_t n = dds_take(reply_reader_, samples, infos, 16, 16);

        std::optional<TReply> result;

        for (int32_t i = 0; i < n; ++i) {
            if (infos[i].valid_data) {
                ++samples_read;
                TReply* reply = static_cast<TReply*>(samples[i]);

                // 检查是否是我们等待的响应
                if (reply->header().related_request_id() == req_id) {
                    // 检查远程异常
                    if (reply->header().remote_ex() != RemoteExceptionCode::REMOTE_EX_OK) {
                        RemoteExceptionCode ex_code = reply->header().remote_ex();
                        // 先归还内存，再抛异常
                        if (n > 0) {
                            dds_return_loan(reply_reader_, samples, n);
                        }
                        throw RpcException("Remote exception", ex_code);
                    }
                    result = *reply;  // 拷贝响应
                } else {
                    ++mismatched_samples;
                }
            }
        }

        if (n > 0) {
            dds_return_loan(reply_reader_, samples, n);
        }

        return result;
    }

    // 成员变量
    dds_entity_t participant_;
    std::string service_name_;
    std::array<uint8_t, 16> client_guid_;
    int64_t sequence_number_;
    bool service_matched_ = false;  // 是否已首次 match

    dds_entity_t request_topic_;
    dds_entity_t request_writer_;
    dds_entity_t reply_topic_;
    dds_entity_t reply_reader_;
    dds_entity_t waitset_;
    dds_entity_t read_condition_;
};

} // namespace dds_rpc
