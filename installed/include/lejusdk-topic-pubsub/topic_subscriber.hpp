#ifndef LEJU_DDS_TOPIC_SUBSCRIBER_H_
#define LEJU_DDS_TOPIC_SUBSCRIBER_H_

#include <string>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>

#include <dds/dds.hpp>
#include <dds/sub/ddssub.hpp>

namespace leju {
namespace dds_common {

/**
 * @brief DDS 话题订阅器模板类
 * 
 * @tparam T 订阅的数据类型（必须是DDS数据类型）
 */
template<typename T>
class TopicSubscriber {
    static_assert(!std::is_void_v<T>, "T must be a valid DDS data type");
    static_assert(std::is_class_v<T> || std::is_arithmetic_v<T>, 
                  "T must be a class or arithmetic type");
public:
    /// @brief 数据回调函数类型
    using DataCallback = std::function<void(const T& data)>;
    
    /// @brief 错误回调函数类型  
    using ErrorCallback = std::function<void(const std::string& error)>;

private:
    /**
     * @brief 内部Listener类，处理DDS事件
     */
    class InternalListener : public dds::sub::DataReaderListener<T> {
    public:
        InternalListener(DataCallback data_callback, ErrorCallback error_callback, 
                        const std::string& topic_name)
            : data_callback_(std::move(data_callback))
            , error_callback_(std::move(error_callback))
            , topic_name_(topic_name) {}
        
        /**
         * @brief 数据可用时的回调
         */
        virtual void on_data_available(dds::sub::DataReader<T>& reader) override {
            if (!data_callback_) return;
            
            try {
                dds::sub::LoanedSamples<T> samples = reader.take();
                
                for (const auto& sample : samples) {
                    if (sample.info().valid()) {
                        try {
                            data_callback_(sample.data());
                        } catch (const std::exception& e) {
                            std::cerr << "Exception in data callback for topic " << topic_name_ 
                                      << ": " << e.what() << std::endl;
                            if (error_callback_) {
                                error_callback_(std::string("Data callback error: ") + e.what());
                            }
                        }
                    }
                }
            } catch (const dds::core::Exception& e) {
                std::cerr << "DDS exception while processing data for topic " << topic_name_ 
                          << ": " << e.what() << std::endl;
                if (error_callback_) {
                    error_callback_(std::string("DDS data processing error: ") + e.what());
                }
            }
        }
        
        /**
         * @brief 订阅匹配时的回调（有发布者连接）
         */
        virtual void on_subscription_matched(dds::sub::DataReader<T>& reader,
                                           const dds::core::status::SubscriptionMatchedStatus& status) override {
            // NOTE: Disabled to reduce log verbosity
            // std::cout << "Subscription matched for topic " << topic_name_
            //           << " - current count: " << status.current_count()
            //           << ", total count: " << status.total_count() << std::endl;
            (void)reader;
            (void)status;
        }
        
        /**
         * @brief 请求不兼容QoS时的回调
         */
        virtual void on_requested_incompatible_qos(dds::sub::DataReader<T>& reader,
                                                 const dds::core::status::RequestedIncompatibleQosStatus& status) override {
            std::cerr << "Requested incompatible QoS for topic " << topic_name_ 
                      << " - policy: " << status.last_policy_id() << std::endl;
            if (error_callback_) {
                error_callback_("Requested incompatible QoS for policy: " + 
                              std::to_string(status.last_policy_id()));
            }
        }
        
        /**
         * @brief 活跃度变化时的回调
         */
        virtual void on_liveliness_changed(dds::sub::DataReader<T>& reader,
                                         const dds::core::status::LivelinessChangedStatus& status) override {
            // NOTE: Disabled to reduce log verbosity
            // std::cout << "Liveliness changed for topic " << topic_name_
            //           << " - alive: " << status.alive_count()
            //           << ", not alive: " << status.not_alive_count() << std::endl;
            (void)reader;
            (void)status;
        }
        
        /**
         * @brief 请求的截止期限错过时的回调
         */
        virtual void on_requested_deadline_missed(dds::sub::DataReader<T>& reader,
                                                const dds::core::status::RequestedDeadlineMissedStatus& status) override {
            std::cerr << "Requested deadline missed for topic " << topic_name_ 
                      << " - total count: " << status.total_count() << std::endl;
            if (error_callback_) {
                error_callback_("Requested deadline missed, total count: " + 
                              std::to_string(status.total_count()));
            }
        }
        
        /**
         * @brief 样本被拒绝时的回调
         */
        virtual void on_sample_rejected(dds::sub::DataReader<T>& reader,
                                      const dds::core::status::SampleRejectedStatus& status) override {
            std::cerr << "Sample rejected for topic " << topic_name_ 
                      << " - total count: " << status.total_count() << std::endl;
            if (error_callback_) {
                error_callback_("Sample rejected, total count: " + 
                              std::to_string(status.total_count()));
            }
        }
        
        /**
         * @brief 样本丢失时的回调
         */
        virtual void on_sample_lost(dds::sub::DataReader<T>& reader,
                                  const dds::core::status::SampleLostStatus& status) override {
            std::cerr << "Sample lost for topic " << topic_name_ 
                      << " - total count: " << status.total_count() << std::endl;
            if (error_callback_) {
                error_callback_("Sample lost, total count: " + 
                              std::to_string(status.total_count()));
            }
        }
        
    private:
        DataCallback data_callback_;
        ErrorCallback error_callback_;
        std::string topic_name_;
    };

public:
    /**
     * @brief 构造函数
     * @param participant 域参与者
     * @param topic_name 主题名称
     * @param data_callback 数据到达时的回调函数
     * @param error_callback 错误发生时的回调函数（可选）
     * @param qos 数据读取者QoS（可选）
     */
    TopicSubscriber(const dds::domain::DomainParticipant& participant, 
                   const std::string& topic_name,
                   DataCallback data_callback,
                   ErrorCallback error_callback = nullptr,
                   const dds::sub::qos::DataReaderQos& qos = dds::sub::qos::DataReaderQos())
        : participant_(participant)
        , subscriber_(dds::core::null)  // 初始化为null
        , topic_(dds::core::null)      // 初始化为null
        , reader_(dds::core::null)     // 初始化为null
        , topic_name_(topic_name)
        , qos_(qos)
        , data_callback_(std::move(data_callback))
        , error_callback_(std::move(error_callback))
        , listener_(nullptr)
        , is_initialized_(false)
    {
        initialize();
    }
    
    /**
     * @brief 析构函数
     */
    ~TopicSubscriber() {
        cleanup();
    }
    
    // 禁用拷贝
    TopicSubscriber(const TopicSubscriber&) = delete;
    TopicSubscriber& operator=(const TopicSubscriber&) = delete;
    
    // 允许移动
    TopicSubscriber(TopicSubscriber&&) = default;
    TopicSubscriber& operator=(TopicSubscriber&&) = default;
    
    /**
     * @brief 获取主题名称
     */
    const std::string& topic_name() const { return topic_name_; }
    
    /**
     * @brief 检查订阅器是否有效
     */
    bool is_valid() const { 
        return is_initialized_ && !reader_.is_nil(); 
    }
    
    /**
     * @brief 获取关联的DataReader
     */
    dds::sub::DataReader<T>& reader() {
        if (!is_initialized_ || reader_.is_nil()) {
            throw std::runtime_error("Subscriber not initialized or DataReader invalid");
        }
        return reader_;
    }
    
    /**
     * @brief 获取关联的DataReader（const版本）
     */
    const dds::sub::DataReader<T>& reader() const {
        if (!is_initialized_ || reader_.is_nil()) {
            throw std::runtime_error("Subscriber not initialized or DataReader invalid");
        }
        return reader_;
    }
    
    /**
     * @brief 获取发布者数量
     */
    uint32_t publisher_count() {
        if (!is_initialized_ || reader_.is_nil()) {
            return 0;
        }
        
        try {
            auto status = reader_.subscription_matched_status();
            return status.current_count();
        } catch (const dds::core::Exception& e) {
            std::cerr << "DDS exception while getting publisher count: " << e.what() << std::endl;
            if (error_callback_) {
                error_callback_(std::string("Publisher count error: ") + e.what());
            }
            return 0;
        }
    }
    
    /**
     * @brief 等待指定数量的发布者
     */
    bool wait_for_publishers(uint32_t count, uint32_t timeout_ms = 5000) {
        if (!is_initialized_ || reader_.is_nil()) {
            return false;
        }
        
        try {
            auto start = std::chrono::steady_clock::now();
            auto timeout = std::chrono::milliseconds(timeout_ms);
            
            while (std::chrono::steady_clock::now() - start < timeout) {
                auto status = reader_.subscription_matched_status();
                if (status.current_count() >= count) {
                    return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return false;
        } catch (const dds::core::Exception& e) {
            std::cerr << "DDS exception while waiting for publishers: " << e.what() << std::endl;
            if (error_callback_) {
                error_callback_(std::string("Wait for publishers error: ") + e.what());
            }
            return false;
        }
    }

private:
    /**
     * @brief 初始化DDS实体
     */
    void initialize() {
        try {
            // 创建订阅者
            subscriber_ = dds::sub::Subscriber(participant_);
            if (subscriber_.is_nil()) {
                throw std::runtime_error("Failed to create subscriber");
            }
            
            // 创建主题
            topic_ = dds::topic::Topic<T>(participant_, topic_name_);
            if (topic_.is_nil()) {
                throw std::runtime_error("Failed to create topic");
            }
            
            // 创建监听器
            listener_ = std::make_shared<InternalListener>(data_callback_, error_callback_, topic_name_);
            
            // 创建DataReader
            reader_ = dds::sub::DataReader<T>(subscriber_, topic_, qos_, listener_.get(), 
                                           dds::core::status::StatusMask::all());
            if (reader_.is_nil()) {
                throw std::runtime_error("Failed to create DataReader");
            }
            
            is_initialized_ = true;
            // NOTE: Disabled to reduce log verbosity
            // std::cout << "TopicSubscriber initialized successfully for topic: " << topic_name_
            //           << " (Listener mode)" << std::endl;
            
        } catch (const dds::core::Exception& e) {
            std::cerr << "DDS exception during initialization: " << e.what() << std::endl;
            if (error_callback_) {
                error_callback_(std::string("DDS initialization error: ") + e.what());
            }
            cleanup();
        } catch (const std::exception& e) {
            std::cerr << "Standard exception during initialization: " << e.what() << std::endl;
            if (error_callback_) {
                error_callback_(std::string("Initialization error: ") + e.what());
            }
            cleanup();
        }
    }
    
    /**
     * @brief 清理资源
     */
    void cleanup() {
        is_initialized_ = false;
    }

private:
    dds::domain::DomainParticipant participant_;
    dds::sub::Subscriber subscriber_;
    dds::topic::Topic<T> topic_;
    dds::sub::DataReader<T> reader_;
    std::string topic_name_;
    dds::sub::qos::DataReaderQos qos_;
    
    DataCallback data_callback_;
    ErrorCallback error_callback_;
    std::shared_ptr<InternalListener> listener_;
    bool is_initialized_;
};

} // namesapce dds_common
} // namespace leju

#endif // LEJU_DDS_TOPIC_SUBSCRIBER_H_