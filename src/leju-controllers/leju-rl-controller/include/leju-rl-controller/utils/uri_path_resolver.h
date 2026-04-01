#pragma once

#include <stdexcept>
#include <string>
#include <vector>

namespace leju {

/**
 * @brief URI 解析错误异常
 */
class UriResolveError : public std::runtime_error {
 public:
  explicit UriResolveError(const std::string& msg) : std::runtime_error(msg) {}
};

/**
 * @brief URI 路径解析器
 *
 * 支持的路径格式：
 * 1. URI: file://path
 * 2. 绝对路径: /path/to/file
 * 3. 相对路径: path/to/file (相对于 base_dir)
 */
class UriPathResolver {
 public:
  /**
   * @brief 解析路径为本地绝对路径或完整 URL
   * @param uri_path 输入的 URI 或本地路径 (file://path, /abs/path, rel/path)
   * @param base_dir 基础目录，用于解析相对路径
   * @return 解析后的绝对路径
   * @throws UriResolveError 解析失败时抛出异常
   */
  static std::string resolve(const std::string& uri_path,
                             const std::string& base_dir = "");

  /**
   * @brief 检查是否为支持的 URI scheme
   * @param uri_path 输入的路径
   * @return true 如果是支持的 URI (如 file://)
   */
  static bool isSupportedUri(const std::string& uri_path);

};

}  // namespace leju
