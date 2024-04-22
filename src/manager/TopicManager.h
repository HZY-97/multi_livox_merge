/**
 * @file TopicManager.h
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SRC_MANAGER_TOPIC_MANAGER_H
#define SRC_MANAGER_TOPIC_MANAGER_H

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

#include "DataManager.h"
#include "common/sysQos.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "paramLoad/LivoxConfig.h"

using std::vector;

namespace manager {

class TopicManager : public rclcpp::Node {
 public:
  explicit TopicManager(std::string nodeName,
                        const rclcpp::NodeOptions &options);
  virtual ~TopicManager();

 private:
  void InitTopic();

  void TMCloudHandlerLivox_0_merge(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg_0);
  void TMCloudHandlerLivox_1_merge(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg_1);

  void TMCloudHandlerLivox_0_calibrate(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg_0);
  void TMCloudHandlerLivox_1_calibrate(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg_1);

  void Create_1_sub_merge(vector<Common::Type::LivoxInfo> &livoxInfoVec);
  void Create_2_sub_merge(vector<Common::Type::LivoxInfo> &livoxInfoVec);

  void Create_2_sub_calibrate(vector<Common::Type::LivoxInfo> &livoxInfoVec);

 public:
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_mergeLivox;

 private:
  vector<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr>
      m_sub_Livox_vec;
};

}  // namespace manager

#endif