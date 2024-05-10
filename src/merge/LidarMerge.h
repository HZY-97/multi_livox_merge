#ifndef LIDARMERGE_H
#define LIDARMERGE_H

#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

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
  uint8_t CheckHeader(std_msgs::msg::Header header_0,
                      std_msgs::msg::Header header_1);

  void PubMeergeCloud();

 public:
 private:
  livox_ros_driver2::msg::CustomMsg::SharedPtr cloud_0_ptr;
  livox_ros_driver2::msg::CustomMsg::SharedPtr cloud_1_ptr;

  std::string m_saveMergePcdPath;
};

}  // namespace merge

#endif