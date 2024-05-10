/**
 * @file NodeManager.cpp
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "NodeManager.h"

namespace manager {

NodeManager* NodeManager::livoxConfigPtr = nullptr;
mutex NodeManager::livoxConfigPtrMtx;

NodeManager* NodeManager::GetInstance() {
  if (livoxConfigPtr == nullptr) {
    std::lock_guard<mutex> lock(livoxConfigPtrMtx);
    if (livoxConfigPtr == nullptr) {
      livoxConfigPtr = new NodeManager();
    }
  }
  return livoxConfigPtr;
}

NodeManager::NodeManager() : m_caliPtr(nullptr) {
  LOG(INFO) << "NodeManager ctor";
  m_options.use_intra_process_comms(true);
  m_options.enable_rosout(false);
  m_options.start_parameter_services(false);
  m_options.start_parameter_event_publisher(false);
  LOG(INFO) << "NodeManager ctor finished";
}

NodeManager::~NodeManager() {
}

void NodeManager::CreateTopicManager() {
  if (m_topicManagerPtr == nullptr) {
    m_topicManagerPtr =
        std::make_shared<TopicManager>("multi_livox_merge", m_options);
  }
}

void NodeManager::CreateProcess() {
  if (paramLoad::LivoxConfig::GetInstance()->mode == 1) {
    if (m_caliPtr == nullptr) {
      m_caliPtr = std::make_shared<calibrate::IcpMethodCali>();
    }
  } else if (paramLoad::LivoxConfig::GetInstance()->mode == 0) {
    if (m_mergePtr == nullptr) {
      m_mergePtr = std::make_shared<merge::LidarMerge>();
    }
  }
}

void NodeManager::Spin() {
  LOG(INFO) << "NodeManager spin";
  rclcpp::spin(m_topicManagerPtr);
}

}  // namespace manager