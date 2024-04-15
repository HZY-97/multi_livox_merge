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
#include "paramLoad/LivoxConfig.h"

int main(int argc, char** argv) {
  std::string installPath;
  std::string configPath;
  bool isGetInstallPath = Common::Init::GetInstallPath(installPath);
  if (!isGetInstallPath) {
    return -1;
  }
  configPath = Common::Init::GetConfigPath(installPath);

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  //   FLAGS_minloglevel = google::FATAL;
  FLAGS_logbufsecs = 0;
  FLAGS_colorlogtostderr = true;
  LOG(INFO) << "multi_livox_merge Log Init Success!";

  rclcpp::InitOptions rcl_options;
  rcl_options.auto_initialize_logging(false);
  rclcpp::init(argc, argv, rcl_options);

  // TODO
  auto livox_config = paramLoad::LivoxConfig::GetInstance();
  livox_config->SetConfigFilePath(configPath);
  livox_config->LoadConfig();

  rclcpp::shutdown();
  return 0;
}