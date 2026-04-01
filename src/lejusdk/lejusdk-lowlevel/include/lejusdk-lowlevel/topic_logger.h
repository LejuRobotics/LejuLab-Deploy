#ifndef LEJU_TOPIC_LOGGER_H_
#define LEJU_TOPIC_LOGGER_H_

#include <string>
#include <vector>
#include <memory>

namespace leju {

/**
 * @brief 基于 DDS 的日志发布器
 *
 * 用于动态发布调试数据到 DDS Topics，支持向量和标量数据类型。
 * 自动管理 Topic 生命周期，线程安全。
 *
 * @code{.cpp}
 * // 创建 Logger (使用默认 domain id = 0)
 * auto logger = leju::TopicLogger::create();
 *
 * // 或指定 domain id
 * auto logger = leju::TopicLogger::create(1);
 *
 * // 发布数据
 * std::vector<double> joint_pos = {0.1, 0.2, 0.3};
 * logger->publishVector("/controller/joint_pos", joint_pos);
 * logger->publishValue("/monitor/loop_time", 0.5);
 * @endcode
 */
class TopicLogger {
public:
    /**
     * @brief 创建 TopicLogger 实例
     * @param domain_id DDS Domain ID，默认为 0
     * @return unique_ptr 指向 TopicLogger 实例
     */
    static std::unique_ptr<TopicLogger> create(int domain_id = 0);

    /**
     * @brief 析构函数
     */
    ~TopicLogger();

    // 禁用拷贝
    TopicLogger(const TopicLogger&) = delete;
    TopicLogger& operator=(const TopicLogger&) = delete;

    // 禁用移动（因为使用 PIMPL）
    TopicLogger(TopicLogger&&) = delete;
    TopicLogger& operator=(TopicLogger&&) = delete;

    /**
     * @brief 发布向量数据
     * @param topic_name Topic 名称
     * @param data 向量数据
     * @return 发布是否成功
     */
    bool publishVector(const std::string& topic_name, const std::vector<double>& data);

    /**
     * @brief 发布标量数据
     * @param topic_name Topic 名称
     * @param value 标量值
     * @return 发布是否成功
     */
    bool publishValue(const std::string& topic_name, double value);

    /**
     * @brief 获取已创建的向量 Topic 数量
     */
    size_t vectorTopicCount() const;

    /**
     * @brief 获取已创建的标量 Topic 数量
     */
    size_t valueTopicCount() const;

private:
    TopicLogger();

    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace leju

#endif // LEJU_TOPIC_LOGGER_H_
