/**
 * @file NodeManager.h
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef NODEMANAGER_H
#define NODEMANAGER_H

#pragma once
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "calibrate/IcpMethodCali.h"
#include "manager/TopicManager.h"
#include "merge/LidarMerge.h"
#include "paramLoad/LivoxConfig.h"

using std::mutex;

namespace manager {
class NodeManager {
 public:
  ~NodeManager();
  NodeManager(const NodeManager&) = delete;
  NodeManager& operator=(const NodeManager&) = delete;
  NodeManager(NodeManager&&) = delete;
  NodeManager& operator=(NodeManager&&) = delete;

  static NodeManager* GetInstance();

  void CreateTopicManager();
  void CreateProcess();
  void Spin();

 private:
  NodeManager();

 public:
  std::shared_ptr<TopicManager> m_topicManagerPtr;
  std::shared_ptr<calibrate::CalibrateBase> m_caliPtr;

  std::shared_ptr<merge::LidarMerge> m_mergePtr;

 private:
  static NodeManager* livoxConfigPtr;
  static mutex livoxConfigPtrMtx;

  rclcpp::NodeOptions m_options;
};
}  // namespace manager

#endif