#ifndef LEJU_DDS_TOPIC_PUBLISHER_H_
#define LEJU_DDS_TOPIC_PUBLISHER_H_

#include <string>
#include <atomic>
#include <iostream>
#include <thread>
#include <chrono>

#include <dds/dds.hpp>
#include <dds/pub/ddspub.hpp>

namespace leju {
namespace dds_common {

/**
 * @brief DDS 话题发布者模板类
 *  
 * @tparam T 发布的数据类型（必须是DDS数据类型）
 */
template<typename T>
class TopicPublisher {
    static_assert(!std::is_void_v<T>, "T must be a valid DDS data type");
    static_assert(std::is_class_v<T> || std::is_arithmetic_v<T>, 
                  "T must be a class or arithmetic type");
public:
    TopicPublisher(const dds::domain::DomainParticipant& participant, 
                  const std::string& topic_name,
                  const dds::pub::qos::DataWriterQos& qos = dds::pub::qos::DataWriterQos())
        : participant_(participant)
        , publisher_(participant)
        , topic_(participant, topic_name)
        , writer_(publisher_, topic_, qos)
        , topic_name_(topic_name)
        , qos_(qos)
        , is_initialized_(true)
    {
        if (publisher_.is_nil() || topic_.is_nil() || writer_.is_nil()) {
            std::cerr << "Failed to initialize TopicPublisher for topic: " << topic_name_ << std::endl;
            is_initialized_ = false;
        }
        // NOTE: Disabled to reduce log verbosity
        // else {
        //     std::cout << "TopicPublisher initialized successfully for topic: " << topic_name_ << std::endl;
        // }
    }
    
    /**
     * @brief 析构函数
     */
    ~TopicPublisher() {
        is_initialized_ = false;
    }
    
    // 禁用拷贝
    TopicPublisher(const TopicPublisher&) = delete;
    TopicPublisher& operator=(const TopicPublisher&) = delete;
    
    // 允许移动
    TopicPublisher(TopicPublisher&&) = default;
    TopicPublisher& operator=(TopicPublisher&&) = default;
    
    /**
     * @brief 发布数据
     */
    bool publish(const T& data) {
        if (!is_initialized_ || writer_.is_nil()) {
            std::cerr << "Publisher not initialized or invalid for topic: " << topic_name_ << std::endl;
            return false;
        }
        
        try {
            writer_.write(data);
            return true;
        } catch (const dds::core::Exception& e) {
            std::cerr << "DDS exception while publishing to " << topic_name_ << ": " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Exception while publishing to " << topic_name_ << ": " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief 异步发布
     */
    bool publish_async(const T& data) {
        if (!is_initialized_ || writer_.is_nil()) {
            std::cerr << "Publisher not initialized or invalid for topic: " << topic_name_ << std::endl;
            return false;
        }
        
        try {
            writer_.write(data, dds::core::InstanceHandle(), dds::core::Time::invalid());
            return true;
        } catch (const dds::core::Exception& e) {
            std::cerr << "DDS exception while async publishing to " << topic_name_ << ": " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Exception while async publishing to " << topic_name_ << ": " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief 获取主题名称
     */
    const std::string& topic_name() const { return topic_name_; }
    
    /**
     * @brief 检查有效性
     */
    bool is_valid() const { return is_initialized_ && !writer_.is_nil(); }
    
    /**
     * @brief 获取DataWriter
     */
    dds::pub::DataWriter<T>& writer() {
        if (!is_initialized_ || writer_.is_nil()) {
            throw std::runtime_error("Publisher not initialized or DataWriter invalid");
        }
        return writer_;
    }
    
    /**
     * @brief 获取DataWriter（const版本）
     */
    const dds::pub::DataWriter<T>& writer() const {
        if (!is_initialized_ || writer_.is_nil()) {
            throw std::runtime_error("Publisher not initialized or DataWriter invalid");
        }
        return writer_;
    }
    
    /**
     * @brief 等待订阅者
     */
    bool wait_for_subscribers(uint32_t count, uint32_t timeout_ms = 1000) {
        if (!is_initialized_ || writer_.is_nil()) return false;
        
        try {
            auto start = std::chrono::steady_clock::now();
            auto timeout = std::chrono::milliseconds(timeout_ms);
            
            while (std::chrono::steady_clock::now() - start < timeout) {
                auto status = writer_.publication_matched_status();
                if (status.current_count() >= count) return true;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return false;
        } catch (const dds::core::Exception& e) {
            std::cerr << "DDS exception while waiting for subscribers: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief 获取订阅者数量
     */
    uint32_t subscriber_count() {
        if (!is_initialized_ || writer_.is_nil()) return 0;
        
        try {
            auto status = writer_.publication_matched_status();
            return status.current_count();
        } catch (const dds::core::Exception& e) {
            std::cerr << "DDS exception while getting subscriber count: " << e.what() << std::endl;
            return 0;
        }
    }

private:
    dds::domain::DomainParticipant participant_;
    dds::pub::Publisher publisher_;
    dds::topic::Topic<T> topic_;
    dds::pub::DataWriter<T> writer_;
    std::string topic_name_;
    dds::pub::qos::DataWriterQos qos_;
    std::atomic<bool> is_initialized_{false};
};

} // namesapce dds_common
} // namespace leju

#endif // LEJU_DDS_TOPIC_PUBLISHER_H_
