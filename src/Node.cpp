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

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "common/initialization.hpp"
#include "manager/NodeManager.h"
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
  // FLAGS_minloglevel = google::FATAL;
  FLAGS_logbufsecs = 0;
  FLAGS_colorlogtostderr = true;
  LOG(INFO) << "multi_livox_merge Log Init Success!";

  rclcpp::InitOptions rcl_options;
  rcl_options.auto_initialize_logging(false);
  rclcpp::init(argc, argv, rcl_options);

  auto livox_config = paramLoad::LivoxConfig::GetInstance();
  livox_config->SetConfigFilePath(configPath);
  livox_config->LoadConfig();

  auto node_manager_ptr = manager::NodeManager::GetInstance();
  node_manager_ptr->CreateTopicManager();
  if (paramLoad::LivoxConfig::GetInstance()->mode == 1) {
    node_manager_ptr->CreateProcess();
    std::thread t = std::thread([&]() {
      while (true) {
        auto r = node_manager_ptr->m_caliPtr->Exectute();
        if (r.x != 0.0f && r.y != 0.0f && r.z != 0.0f && r.roll != 0.0f &&
            r.pitch != 0.0f && r.yaw != 0.0f) {
          std::cout << r.ShowResult() << std::endl;
        } else {
          LOG_EVERY_N(INFO, 50) << "Waiting for calculation...";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    });

    node_manager_ptr->Spin();
    t.join();
  } else if (paramLoad::LivoxConfig::GetInstance()->mode == 0) {
    node_manager_ptr->CreateProcess();
    std::thread t = std::thread([&]() {
      while (true) {
        node_manager_ptr->m_mergePtr->MergeCloud();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });

    node_manager_ptr->Spin();
    t.join();
  }

  rclcpp::shutdown();
  return 0;
}