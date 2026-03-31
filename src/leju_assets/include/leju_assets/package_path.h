#pragma once

#include <cstddef>
#include <string>

namespace leju {
namespace leju_assets {

/** Gets the path to the package source directory. */
std::string getPath();
std::string getAbsolutePathFromRosPackage(const std::string &package_name, const std::string &path);
}  // namespace robotic_assets
}  // namespace leju
