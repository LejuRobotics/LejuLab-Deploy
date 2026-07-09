#ifndef LEJUSDK_UTILS_CALLBACK_WRAPPER_HPP_
#define LEJUSDK_UTILS_CALLBACK_WRAPPER_HPP_

#include <mutex>
#include <utility>
#include <vector>

namespace leju {
namespace internal {

/**
 * @brief Thread-safe callback wrapper template
 *
 * This template provides a thread-safe way to store and invoke callbacks.
 * It uses internal mutex protection to ensure thread safety when setting,
 * getting, or calling callbacks.
 *
 * @tparam CallbackType The type of callback function to wrap
 */
template<typename CallbackType>
struct CallbackWrapper {
  mutable std::mutex mutex;
  CallbackType callback;

  CallbackWrapper() : callback(nullptr) {}

  /**
   * @brief Set a new callback
   * @param cb The callback to set
   */
  void set(const CallbackType& cb) {
    std::lock_guard<std::mutex> lock(mutex);
    callback = cb;
  }

  /**
   * @brief Clear the callback (set to nullptr)
   */
  void clear() {
    std::lock_guard<std::mutex> lock(mutex);
    callback = nullptr;
  }

  /**
   * @brief Get a copy of the current callback
   * @return A copy of the stored callback
   */
  CallbackType get() const {
    std::lock_guard<std::mutex> lock(mutex);
    return callback;
  }

  /**
   * @brief Check if a callback is set
   * @return true if callback is not nullptr, false otherwise
   */
  bool is_set() const {
    std::lock_guard<std::mutex> lock(mutex);
    return callback != nullptr;
  }

  /**
   * @brief Call the callback if it's set, with given arguments
   * @tparam Args Argument types
   * @param args Arguments to pass to the callback
   * @return true if callback was called, false if no callback was set
   */
  template<typename... Args>
  bool call_if(Args&&... args) const {
    CallbackType cb;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!callback) {
        return false;
      }
      cb = callback;
    }
    // Call without holding the lock to avoid potential deadlock
    cb(std::forward<Args>(args)...);
    return true;
  }
};

/**
 * @brief Thread-safe callback vector wrapper template
 *
 * This template provides a thread-safe way to store and invoke multiple callbacks.
 * It manages a vector of callbacks with thread-safe operations for adding,
 * removing, clearing, and invoking all callbacks.
 *
 * @tparam CallbackType The type of callback function to wrap
 */
template<typename CallbackType>
struct CallbackVector {
  mutable std::mutex mutex;
  std::vector<CallbackType> callbacks;

  /**
   * @brief Add a callback to the vector
   * @param cb The callback to add
   */
  void add(const CallbackType& cb) {
    std::lock_guard<std::mutex> lock(mutex);
    callbacks.push_back(cb);
  }

  /**
   * @brief Clear all callbacks
   */
  void clear() {
    std::lock_guard<std::mutex> lock(mutex);
    callbacks.clear();
  }

  /**
   * @brief Get the number of callbacks
   * @return The number of stored callbacks
   */
  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex);
    return callbacks.size();
  }

  /**
   * @brief Check if the vector is empty
   * @return true if no callbacks are stored
   */
  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex);
    return callbacks.empty();
  }

  /**
   * @brief Call all stored callbacks with given arguments
   * @tparam Args Argument types
   * @param args Arguments to pass to all callbacks
   * @return The number of callbacks called
   */
  template<typename... Args>
  size_t call_all(Args&&... args) const {
    std::vector<CallbackType> callbacks_copy;
    {
      std::lock_guard<std::mutex> lock(mutex);
      callbacks_copy = callbacks;
    }

    // Call without holding the lock to avoid potential deadlock
    for (const auto& cb : callbacks_copy) {
      if (cb) {
        cb(std::forward<Args>(args)...);
      }
    }
    return callbacks_copy.size();
  }
};

}  // namespace internal
}  // namespace leju

#endif  // LEJUSDK_UTILS_CALLBACK_WRAPPER_HPP_