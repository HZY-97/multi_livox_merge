/**
 * @file Node.cpp
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <glog/logging.h>

#include <rclcpp/rclcpp.hpp>

#include "common/initialization.hpp"

int main(int argc, char **argv) {
  std::string installPath;
  std::string configPath;
  Common::Init::GetInstallPath(installPath);
  Common::Init::GetConfigPath(configPath);

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  //   FLAGS_minloglevel = google::FATAL;
  FLAGS_logbufsecs = 0;
  FLAGS_colorlogtostderr = true;
  LOG(INFO) << "multi_livox_merge Log Init Success!";

  return 0;
}