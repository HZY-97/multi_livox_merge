/**
 * @file DataManager.h
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-18
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <cstdint>
#pragma once

#include <glog/logging.h>

#include <deque>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace manager {

class DataManager {
 public:
  static DataManager* GetInstance();
  ~DataManager();
  DataManager(const DataManager&) = delete;
  DataManager& operator=(const DataManager&) = delete;
  DataManager(DataManager&&) = delete;
  DataManager& operator=(DataManager&&) = delete;

  void SaveCaliData(uint8_t lidar_id,
                    livox_ros_driver2::msg::CustomMsg::SharedPtr data);
  livox_ros_driver2::msg::CustomMsg::SharedPtr GetFrontMsg(uint8_t lidar_id);

 private:
  DataManager();

 public:
  std::deque<livox_ros_driver2::msg::CustomMsg::SharedPtr> m_caliData_0_deq;
  std::deque<livox_ros_driver2::msg::CustomMsg::SharedPtr> m_caliData_1_deq;
  std::mutex m_caliData_0_deq_Mtx;
  std::mutex m_caliData_1_deq_Mtx;

 private:
  static DataManager* m_dataManagerPtr;
  static std::mutex m_mutex;
};

}  // namespace manager

#endif