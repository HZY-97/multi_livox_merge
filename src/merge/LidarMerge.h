#ifndef LIDARMERGE_H
#define LIDARMERGE_H

#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <atomic>
#include <deque>
#include <pcl/impl/point_types.hpp>
#include <std_msgs/msg/header.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "manager/DataManager.h"

namespace merge {
class LidarMerge {
 public:
  LidarMerge();
  ~LidarMerge();

  void MergeCloud();

 private:
  bool SyncCloud();

  void PubMeergeCloud();

 public:
 private:
  manager::DataManager* m_dataManager;

  livox_ros_driver2::msg::CustomMsg::SharedPtr cloud_0_ptr;
  livox_ros_driver2::msg::CustomMsg::SharedPtr cloud_1_ptr;

  std::string m_saveMergePcdPath;

  livox_ros_driver2::msg::CustomMsg::SharedPtr tmp_cloud_0_ptr;
  livox_ros_driver2::msg::CustomMsg::SharedPtr tmp_cloud_1_ptr;

#ifdef SHOW_TIME
  std::chrono::system_clock::time_point m_lastTime;
  std::chrono::system_clock::time_point m_currentTime;
#endif
};

}  // namespace merge

#endif