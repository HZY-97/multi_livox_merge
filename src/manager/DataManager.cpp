/**
 * @file DataManager.cpp
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-18
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "manager/DataManager.h"

namespace manager {
DataManager* DataManager::m_dataManagerPtr = nullptr;
std::mutex DataManager::m_mutex;

DataManager* DataManager::GetInstance() {
  if (m_dataManagerPtr == nullptr) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_dataManagerPtr == nullptr) {
      m_dataManagerPtr = new DataManager();
    }
  }
  return m_dataManagerPtr;
}

DataManager::DataManager() {
}

DataManager::~DataManager() {
}

void DataManager::SaveCaliData(
    uint8_t lidar_id, livox_ros_driver2::msg::CustomMsg::SharedPtr data) {
  if (0 == lidar_id) {
    std::lock_guard<std::mutex> lock_0(m_caliData_0_deq_Mtx);
    m_caliData_0_deq.push_back(data);
  } else if (1 == lidar_id) {
    std::lock_guard<std::mutex> lock_1(m_caliData_1_deq_Mtx);
    m_caliData_1_deq.push_back(data);
  } else {
    LOG(ERROR) << "SaveCaliData unknow lidar_id";
  }
  // LOG(INFO) << "0_deq Size= " << m_caliData_0_deq.size();
  // LOG(INFO) << "1_deq Size= " << m_caliData_1_deq.size();
}

livox_ros_driver2::msg::CustomMsg::SharedPtr DataManager::GetFrontMsg(
    uint8_t lidar_id) {
  if (0 == lidar_id) {
    std::lock_guard<std::mutex> lock_0(m_caliData_0_deq_Mtx);
    if (m_caliData_0_deq.size() > 0) {
      auto cloud = m_caliData_0_deq.front();
      m_caliData_0_deq.pop_front();
      if (!m_caliData_0_deq.empty()) {
        m_caliData_0_deq.clear();
      }
      return cloud;
    }
  } else if (1 == lidar_id) {
    std::lock_guard<std::mutex> lock_1(m_caliData_1_deq_Mtx);
    if (m_caliData_1_deq.size() > 0) {
      auto cloud = m_caliData_1_deq.front();
      m_caliData_1_deq.pop_front();
      if (!m_caliData_1_deq.empty()) {
        m_caliData_1_deq.clear();
      }
      return cloud;
    }
  } else {
    LOG(ERROR) << "SaveCaliData unknow lidar_id";
  }
  return nullptr;
}

}  // namespace manager