#include "leju-rl-controller/utils/uri_path_resolver.h"

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace leju {

namespace {

// 字符串转小写
std::string toLower(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return str;
}

// 提取 URI scheme
std::string extractScheme(const std::string& uri_path) {
  auto pos = uri_path.find("://");
  if (pos == std::string::npos || pos == 0) return "";

  // 验证 scheme 格式: [a-zA-Z][a-zA-Z0-9+.-]*
  for (size_t i = 0; i < pos; ++i) {
    char c = uri_path[i];
    if (i == 0) {
      if (!std::isalpha(c)) return "";
    } else {
      if (!std::isalnum(c) && c != '+' && c != '-' && c != '.') return "";
    }
  }
  return toLower(uri_path.substr(0, pos));
}

// 提取 URI 路径部分（去掉 scheme://）
std::string extractPath(const std::string& uri) {
  auto pos = uri.find("://");
  if (pos == std::string::npos) return uri;
  return uri.substr(pos + 3);
}

// 路径是否绝对路径
bool isAbsolute(const std::string& path) {
  return !path.empty() && path[0] == '/';
}

// 规范化路径（去除 .. 等）
std::string normalize(const std::string& path) {
  return std::filesystem::path(path).lexically_normal().string();
}

}  // namespace

std::string UriPathResolver::resolve(const std::string& uri_path,
                                      const std::string& base_dir) {
  if (uri_path.empty()) {
    throw UriResolveError("Empty path");
  }

  auto scheme = extractScheme(uri_path);

  // file:// 协议处理
  if (scheme == "file") {
    auto file_path = extractPath(uri_path);
    if (file_path.empty()) {
      throw UriResolveError("Empty path in file:// URI");
    }

    if (isAbsolute(file_path)) {
      return normalize(file_path);
    }

    if (base_dir.empty()) {
      throw UriResolveError("Relative file path requires base_dir: " + uri_path);
    }

    return normalize(base_dir + "/" + file_path);
  }

  // TODO: 网络协议支持 (ftp://, http://, https://)
  // 当前不支持，后续扩展时在此添加处理逻辑
  if (scheme == "ftp" || scheme == "http" || scheme == "https") {
    throw UriResolveError("Network URI not yet supported '" + scheme + "': " + uri_path);
  }

  // 未知协议
  if (!scheme.empty()) {
    throw UriResolveError("Unsupported URI scheme '" + scheme + "': " + uri_path);
  }

  // 普通本地路径
  if (isAbsolute(uri_path)) {
    return normalize(uri_path);
  }

  if (!base_dir.empty()) {
    return normalize(base_dir + "/" + uri_path);
  }

  return uri_path;
}

bool UriPathResolver::isSupportedUri(const std::string& uri_path) {
  auto scheme = extractScheme(uri_path);
  // 当前仅支持 file://，网络协议后续扩展
  return scheme == "file";
}

}  // namespace leju
