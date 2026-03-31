#pragma once

#include <atomic>
#include <functional>

#include "lejusdk-lowlevel/leju_sdk.h"

namespace leju {

/**
 * @brief 手柄输入处理器
 *
 */
class JoyHandler {
 public:
  using Callback = std::function<void(const JoyData&, const JoyData::Buttons& prev)>;

  JoyHandler() = default;
  ~JoyHandler();

  // 禁止拷贝
  JoyHandler(const JoyHandler&) = delete;
  JoyHandler& operator=(const JoyHandler&) = delete;

  /**
   * @brief 设置数据回调
   *
   * 每帧手柄数据通过此回调输出。
   * 必须在 start() 之前调用。
   */
  void setCallback(Callback cb);

  /**
   * @brief 启动（订阅 SDK joy 回调）
   */
  void start();

  /**
   * @brief 停止
   */
  void stop();

  /**
   * @brief 是否正在运行
   */
  bool isRunning() const { return running_.load(); }

 private:
  std::atomic<bool> running_{false};
  JoyData::Buttons prev_buttons_{};
  Callback callback_;
};

}  // namespace leju
