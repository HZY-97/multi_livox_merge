/**
 * @file TopicManager.cpp
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "manager/TopicManager.h"

#include <string>

using paramLoad::LivoxConfig;

namespace manager {
TopicManager::TopicManager(std::string nodeName,
                           const rclcpp::NodeOptions& options)
    : Node(nodeName, options) {
  LOG(INFO) << "TopicManager ctor";
  InitTopic();
  LOG(INFO) << "TopicManager ctor finished";
}

TopicManager::~TopicManager() {
}

void TopicManager::InitTopic() {
  auto livoxInfoVec = LivoxConfig::GetInstance()->m_livoxInfoVec;
  auto useLidarId = LivoxConfig::GetInstance()->use_lidar_id;
  auto useImuId = LivoxConfig::GetInstance()->use_imu_id;
  auto mode = LivoxConfig::GetInstance()->mode;

  if (0 == mode) {
    LOG(WARNING) << "***********************************";
    LOG(WARNING) << "**mode 0: merge all lidar to id 0**";
    LOG(WARNING) << "***********************************";
    if (livoxInfoVec.size() == 1) {
      Create_1_sub_merge(livoxInfoVec);
    } else if (livoxInfoVec.size() == 2) {
      Create_2_sub_merge(livoxInfoVec);
    }

  } else if (1 == mode) {
    LOG(WARNING) << "*************************************";
    LOG(WARNING) << "**mode 1: calibration lidar to id:0**";
    LOG(WARNING) << "*************************************";
    Create_2_sub_calibrate(livoxInfoVec);
  }
}

void TopicManager::Create_1_sub_merge(
    vector<Common::Type::LivoxInfo>& livoxInfoVec) {
  common::qos::QosConfig qosConfig;
  auto lidarOpt = rclcpp::SubscriptionOptions();
  lidarOpt.callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto lidarInfo_0 = livoxInfoVec[0];
  string topicName_0 = "/livox/lidar_" + lidarInfo_0.ip;
  auto subLaserCloudLivox_0 =
      create_subscription<livox_ros_driver2::msg::CustomMsg>(
          topicName_0, qosConfig.qos_lidar,
          std::bind(&TopicManager::TMCloudHandlerLivox_0_merge, this,
                    std::placeholders::_1),
          lidarOpt);
  m_sub_Livox_vec.push_back(subLaserCloudLivox_0);
}

void TopicManager::Create_2_sub_merge(
    vector<Common::Type::LivoxInfo>& livoxInfoVec) {
  common::qos::QosConfig qosConfig;
  auto lidarOpt = rclcpp::SubscriptionOptions();
  lidarOpt.callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto lidarInfo_0 = livoxInfoVec[0];
  auto lidarInfo_1 = livoxInfoVec[1];

  string topicName_0 = "/livox/lidar_" + lidarInfo_0.ip;
  string topicName_1 = "/livox/lidar_" + lidarInfo_1.ip;

  auto subLaserCloudLivox_0 =
      create_subscription<livox_ros_driver2::msg::CustomMsg>(
          topicName_0, qosConfig.qos_lidar,
          std::bind(&TopicManager::TMCloudHandlerLivox_0_merge, this,
                    std::placeholders::_1),
          lidarOpt);

  auto subLaserCloudLivox_1 =
      create_subscription<livox_ros_driver2::msg::CustomMsg>(
          topicName_1, qosConfig.qos_lidar,
          std::bind(&TopicManager::TMCloudHandlerLivox_1_merge, this,
                    std::placeholders::_1),
          lidarOpt);

  m_sub_Livox_vec.push_back(subLaserCloudLivox_0);
  m_sub_Livox_vec.push_back(subLaserCloudLivox_1);
}

void TopicManager::TMCloudHandlerLivox_0_merge(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg_0) {
  double ms = laserCloudMsg_0->header.stamp.nanosec / 1000.0;
  LOG(INFO) << "000==" << ms;
}

void TopicManager::TMCloudHandlerLivox_1_merge(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg_1) {
  double ms = laserCloudMsg_1->header.stamp.nanosec / 1000.0;
  LOG(INFO) << "111==" << ms;
}

void TopicManager::Create_2_sub_calibrate(
    vector<Common::Type::LivoxInfo>& livoxInfoVec) {
  common::qos::QosConfig qosConfig;
  auto lidarOpt = rclcpp::SubscriptionOptions();
  lidarOpt.callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto lidarInfo_0 = livoxInfoVec[0];
  auto lidarInfo_1 = livoxInfoVec[1];

  string topicName_0 = "/livox/lidar_" + lidarInfo_0.ip;
  string topicName_1 = "/livox/lidar_" + lidarInfo_1.ip;

  auto subLaserCloudLivox_0 =
      create_subscription<livox_ros_driver2::msg::CustomMsg>(
          topicName_0, qosConfig.qos_lidar,
          std::bind(&TopicManager::TMCloudHandlerLivox_0_calibrate, this,
                    std::placeholders::_1),
          lidarOpt);

  auto subLaserCloudLivox_1 =
      create_subscription<livox_ros_driver2::msg::CustomMsg>(
          topicName_1, qosConfig.qos_lidar,
          std::bind(&TopicManager::TMCloudHandlerLivox_1_calibrate, this,
                    std::placeholders::_1),
          lidarOpt);

  m_sub_Livox_vec.push_back(subLaserCloudLivox_0);
  m_sub_Livox_vec.push_back(subLaserCloudLivox_1);
}

void TopicManager::TMCloudHandlerLivox_0_calibrate(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg_0) {
  DataManager::GetInstance()->SaveCaliData(0, laserCloudMsg_0);
  double ms = laserCloudMsg_0->header.stamp.nanosec / 1000000.0;
  // std::cout << "lidar==" << ms << std::endl;
}

void TopicManager::TMCloudHandlerLivox_1_calibrate(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg_1) {
  DataManager::GetInstance()->SaveCaliData(1, laserCloudMsg_1);
  double ms = laserCloudMsg_1->header.stamp.nanosec / 1000000.0;
  // std::cout << "111==" << ms << std::endl;
}

}  // namespace manager