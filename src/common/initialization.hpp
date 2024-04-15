/**
 * @file initialization.hpp
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SRC_COMMON_INITIALIZATION_HPP
#define SRC_COMMON_INITIALIZATION_HPP
#include <unistd.h>

#include <iostream>
#include <thread>

namespace Common {
namespace Init {

inline bool GetInstallPath(std::string &path) {
  char *nav3dInstallPath = std::getenv("MULTI_LIVOX_MERGE_INSTALL_PATH");
  if (nullptr == nav3dInstallPath) {
    std::cerr << "\033[31m"
              << "[LogCleaner]:No env 'MULTI_LIVOX_MERGE_INSTALL_PATH'"
              << "\033[0m" << std::endl;
    return false;
  } else {
    path = std::string(nav3dInstallPath);
    return true;
  }
}

inline std::string GetConfigPath(const std::string &install_path) {
  return std::string(install_path + "/../config");
}

}  // namespace Init
}  // namespace Common

#endif