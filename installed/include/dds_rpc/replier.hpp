/**
 * @file replier.hpp
 * @brief DDS-RPC Replier (服务端) 实现
 *
 * Replier 用于接收 RPC 请求并发送响应。
 * 支持同步处理和异步处理模式。
 *
 * @author lejurobot ohh
 * @version 1.0.0
 */
#pragma once

#include "rpc_types.hpp"
#include <thread>
#include <atomic>
#include <functional>

// CycloneDDS C++ TopicTraits support
#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace dds_rpc {

/**
 * @brief 响应者模板类
 *
 * 用于接收请求并发送响应的服务端组件。
 *
 * @tparam TRequest 请求类型 (必须包含符合规范的 header 成员)
 * @tparam TReply 响应类型 (必须包含符合规范的 header 成员)
 *
 * 类型约束:
 * - TRequest 必须有 header.request_id (SampleIdentity)
 * - TReply 必须有 header.related_request_id (SampleIdentity)
 * - TReply 必须有 header.remote_ex (RemoteExceptionCode)
 *
 * @example
 * @code
 * Replier<MyRequest, MyReply> replier(participant, "MyService",
 *     &MyRequest_desc, &MyReply_desc);
 *
 * replier.set_handler([](const MyRequest& req) {
 *     MyReply reply{};
 *     reply.result = req.data * 2;
 *     return reply;
 * });
 *
 * replier.start();  // 在新线程中运行
 * // 或
 * replier.run();    // 在当前线程运行 (阻塞)
 * @endcode
 */
template<typename TRequest, typename TReply>
class Replier {
    // =========== 编译时类型检查 ===========
    DDS_RPC_STATIC_ASSERT_TYPES(TRequest, TReply);

public:
    using RequestType = TRequest;
    using ReplyType = TReply;

    /**
     * @brief 请求处理器类型
     *
     * 返回响应对象，header 会被自动填充。
     * 抛出异常会被捕获并转换为错误响应。
     */
    using RequestHandler = std::function<TReply(const TRequest&)>;

    /**
     * @brief 高级请求处理器类型
     *
     * 可以直接设置响应头的异常码。
     */
    using AdvancedRequestHandler = std::function<void(const TRequest&, TReply&)>;

    /**
     * @brief 构造函数 (C++ 类型，推荐使用)
     *
     * 使用 CycloneDDS C++ TopicTraits 自动获取类型信息。
     *
     * @param participant DDS Participant
     * @param service_name 服务名称
     * @param qos_config QoS 配置 (可选)
     *
     * @throw RpcException 创建 DDS 实体失败
     * @throw DuplicateServiceException 服务已存在（当 allow_duplicate_service=false 时）
     */
    Replier(dds_entity_t participant,
            const std::string& service_name,
            const RpcQosConfig& qos_config = RpcQosConfig())
        : participant_(participant)
        , service_name_(service_name)
        , running_(false)
    {
        using org::eclipse::cyclonedds::topic::TopicTraits;

        // 创建请求 Topic (使用 C++ sertype)
        std::string req_topic_name = request_topic_name(service_name);
        ddsi_sertype* req_sertype = TopicTraits<TRequest>::getSerType();
        request_topic_ = dds_create_topic_sertype(
            participant, req_topic_name.c_str(), &req_sertype, nullptr, nullptr, nullptr);
        if (request_topic_ < 0) {
            throw RpcException("Failed to create request topic: " + req_topic_name);
        }

        // 创建请求 Reader
        dds_qos_t* reader_qos = qos_config.create_reader_qos();
        request_reader_ = dds_create_reader(participant, request_topic_, reader_qos, nullptr);
        dds_delete_qos(reader_qos);
        if (request_reader_ < 0) {
            throw RpcException("Failed to create request reader");
        }

        // 创建响应 Topic (使用 C++ sertype)
        std::string rep_topic_name = reply_topic_name(service_name);
        ddsi_sertype* rep_sertype = TopicTraits<TReply>::getSerType();
        reply_topic_ = dds_create_topic_sertype(
            participant, rep_topic_name.c_str(), &rep_sertype, nullptr, nullptr, nullptr);
        if (reply_topic_ < 0) {
            throw RpcException("Failed to create reply topic: " + rep_topic_name);
        }

        // 创建响应 Writer
        dds_qos_t* writer_qos = qos_config.create_writer_qos();
        reply_writer_ = dds_create_writer(participant, reply_topic_, writer_qos, nullptr);
        dds_delete_qos(writer_qos);
        if (reply_writer_ < 0) {
            throw RpcException("Failed to create reply writer");
        }

        // === 检测是否已有其他服务端 ===
        if (!qos_config.allow_duplicate_service) {
            check_duplicate_service(qos_config.discovery_wait_ms);
        }

        // 创建 WaitSet
        waitset_ = dds_create_waitset(participant);
        read_condition_ = dds_create_readcondition(request_reader_, DDS_ANY_STATE);
        dds_waitset_attach(waitset_, read_condition_, 0);
    }

    /**
     * @brief 构造函数 (C 类型描述符，向后兼容)
     *
     * @param participant DDS Participant
     * @param service_name 服务名称
     * @param request_desc 请求类型 DDS 描述符
     * @param reply_desc 响应类型 DDS 描述符
     * @param qos_config QoS 配置 (可选)
     *
     * @throw RpcException 创建 DDS 实体失败
     * @throw DuplicateServiceException 服务已存在（当 allow_duplicate_service=false 时）
     */
    Replier(dds_entity_t participant,
            const std::string& service_name,
            const dds_topic_descriptor_t* request_desc,
            const dds_topic_descriptor_t* reply_desc,
            const RpcQosConfig& qos_config = RpcQosConfig())
        : participant_(participant)
        , service_name_(service_name)
        , running_(false)
    {
        // 创建请求 Topic
        std::string req_topic_name = request_topic_name(service_name);
        request_topic_ = dds_create_topic(
            participant, request_desc, req_topic_name.c_str(), nullptr, nullptr);
        if (request_topic_ < 0) {
            throw RpcException("Failed to create request topic: " + req_topic_name);
        }

        // 创建请求 Reader
        dds_qos_t* reader_qos = qos_config.create_reader_qos();
        request_reader_ = dds_create_reader(participant, request_topic_, reader_qos, nullptr);
        dds_delete_qos(reader_qos);
        if (request_reader_ < 0) {
            throw RpcException("Failed to create request reader");
        }

        // 创建响应 Topic
        std::string rep_topic_name = reply_topic_name(service_name);
        reply_topic_ = dds_create_topic(
            participant, reply_desc, rep_topic_name.c_str(), nullptr, nullptr);
        if (reply_topic_ < 0) {
            throw RpcException("Failed to create reply topic: " + rep_topic_name);
        }

        // 创建响应 Writer
        dds_qos_t* writer_qos = qos_config.create_writer_qos();
        reply_writer_ = dds_create_writer(participant, reply_topic_, writer_qos, nullptr);
        dds_delete_qos(writer_qos);
        if (reply_writer_ < 0) {
            throw RpcException("Failed to create reply writer");
        }

        // === 检测是否已有其他服务端 ===
        if (!qos_config.allow_duplicate_service) {
            check_duplicate_service(qos_config.discovery_wait_ms);
        }

        // 创建 WaitSet
        waitset_ = dds_create_waitset(participant);
        read_condition_ = dds_create_readcondition(request_reader_, DDS_ANY_STATE);
        dds_waitset_attach(waitset_, read_condition_, 0);
    }

    /**
     * @brief 析构函数
     */
    ~Replier() {
        stop();

        dds_waitset_detach(waitset_, read_condition_);
        dds_delete(read_condition_);
        dds_delete(waitset_);
        dds_delete(reply_writer_);
        dds_delete(request_reader_);
        dds_delete(reply_topic_);
        dds_delete(request_topic_);
    }

    // 禁用拷贝
    Replier(const Replier&) = delete;
    Replier& operator=(const Replier&) = delete;

    /**
     * @brief 获取服务名称
     * @return 服务名称
     */
    const std::string& service_name() const { return service_name_; }

    /**
     * @brief 设置请求处理器
     *
     * @param handler 处理函数，接收请求并返回响应
     */
    void set_handler(RequestHandler handler) {
        handler_ = std::move(handler);
    }

    /**
     * @brief 设置高级请求处理器
     *
     * @param handler 处理函数，可以直接修改响应对象
     */
    void set_advanced_handler(AdvancedRequestHandler handler) {
        advanced_handler_ = std::move(handler);
    }

    /**
     * @brief 启动服务 (在新线程中运行)
     *
     * 非阻塞，服务在后台线程中运行。
     */
    void start() {
        if (running_) return;

        running_ = true;
        worker_thread_ = std::thread(&Replier::run_loop, this);
    }

    /**
     * @brief 停止服务
     */
    void stop() {
        running_ = false;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    /**
     * @brief 检查服务是否运行中
     * @return 是否运行中
     */
    bool is_running() const { return running_; }

    /**
     * @brief 处理单个请求 (用于手动控制)
     *
     * @param timeout 等待超时
     * @return 是否处理了请求
     */
    bool process_one(std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
        dds_attach_t triggered;
        dds_return_t rc = dds_waitset_wait(
            waitset_, &triggered, 1,
            static_cast<dds_duration_t>(timeout.count()) * DDS_NSECS_IN_MSEC);

        if (rc > 0) {
            return take_and_process();
        }
        return false;
    }

    /**
     * @brief 在当前线程中运行服务
     *
     * 阻塞直到 stop() 被调用。
     */
    void run() {
        running_ = true;
        run_loop();
    }

    /**
     * @brief 获取已处理的请求数
     * @return 请求处理计数
     */
    uint64_t processed_count() const { return processed_count_; }

private:
    /**
     * @brief 检测是否已有其他服务端
     *
     * 通过检查 Reply Topic 是否已有其他 Writer 来判断是否存在重复服务。
     * 如果检测到其他服务端，抛出 DuplicateServiceException。
     *
     * @param discovery_wait_ms 等待 DDS 发现的时间（毫秒）
     * @throw DuplicateServiceException 已存在其他服务端
     */
    void check_duplicate_service(int32_t discovery_wait_ms) {
        // 给 DDS 一些时间完成发现过程
        std::this_thread::sleep_for(std::chrono::milliseconds(discovery_wait_ms));

        // 方法：检查 Reply Topic 是否已有其他 Writer
        // 我们刚创建了 reply_writer_，如果有其他 Replier，
        // 那么 Reply Topic 上会有其他 Writer

        // 创建一个临时 Reader 来检测其他 Writer
        dds_qos_t* qos = dds_create_qos();
        dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
        dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);

        dds_entity_t temp_reader = dds_create_reader(
            participant_, reply_topic_, qos, nullptr);
        dds_delete_qos(qos);

        if (temp_reader < 0) {
            // 无法创建临时 reader，跳过检测
            return;
        }

        // 等待一小段时间让匹配完成
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 检查有多少 Writer 匹配了这个 Reader
        dds_subscription_matched_status_t status;
        dds_return_t rc = dds_get_subscription_matched_status(temp_reader, &status);

        dds_delete(temp_reader);

        if (rc == DDS_RETCODE_OK) {
            // current_count 包括我们自己的 reply_writer_
            // 如果 > 1，说明有其他 Replier
            if (status.current_count > 1) {
                // 清理已创建的资源
                cleanup_on_error();
                throw DuplicateServiceException(service_name_);
            }
        }
    }

    /**
     * @brief 错误时清理资源
     */
    void cleanup_on_error() {
        if (reply_writer_ > 0) dds_delete(reply_writer_);
        if (request_reader_ > 0) dds_delete(request_reader_);
        if (reply_topic_ > 0) dds_delete(reply_topic_);
        if (request_topic_ > 0) dds_delete(request_topic_);
        reply_writer_ = 0;
        request_reader_ = 0;
        reply_topic_ = 0;
        request_topic_ = 0;
    }

    /**
     * @brief 主循环
     */
    void run_loop() {
        while (running_) {
            process_one(std::chrono::milliseconds(100));
        }
    }

    /**
     * @brief 取出并处理请求
     */
    bool take_and_process() {
        void* samples[1];
        dds_sample_info_t infos[1];
        samples[0] = nullptr;

        int32_t n = dds_take(request_reader_, samples, infos, 1, 1);

        if (n > 0 && infos[0].valid_data) {
            TRequest* request = static_cast<TRequest*>(samples[0]);

            TReply reply{};
            reply.header().related_request_id(request->header().request_id());
            reply.header().remote_ex(RemoteExceptionCode::REMOTE_EX_OK);

            try {
                if (advanced_handler_) {
                    advanced_handler_(*request, reply);
                } else if (handler_) {
                    reply = handler_(*request);
                    // 确保响应头正确设置
                    reply.header().related_request_id(request->header().request_id());
                    reply.header().remote_ex(RemoteExceptionCode::REMOTE_EX_OK);
                } else {
                    reply.header().remote_ex(RemoteExceptionCode::REMOTE_EX_UNKNOWN_OPERATION);
                }
            } catch (const RpcException& e) {
                reply.header().remote_ex(e.code());
            } catch (const std::invalid_argument&) {
                reply.header().remote_ex(RemoteExceptionCode::REMOTE_EX_INVALID_ARGUMENT);
            } catch (...) {
                reply.header().remote_ex(RemoteExceptionCode::REMOTE_EX_UNKNOWN_EXCEPTION);
            }

            // 发送响应
            dds_write(reply_writer_, &reply);
            ++processed_count_;

            dds_return_loan(request_reader_, samples, n);
            return true;
        }

        return false;
    }

    // 成员变量
    dds_entity_t participant_;
    std::string service_name_;
    std::atomic<bool> running_;
    std::atomic<uint64_t> processed_count_{0};

    dds_entity_t request_topic_;
    dds_entity_t request_reader_;
    dds_entity_t reply_topic_;
    dds_entity_t reply_writer_;
    dds_entity_t waitset_;
    dds_entity_t read_condition_;

    std::thread worker_thread_;
    RequestHandler handler_;
    AdvancedRequestHandler advanced_handler_;
};

} // namespace dds_rpc
