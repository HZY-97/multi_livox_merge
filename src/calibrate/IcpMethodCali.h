/**
 * @file IcpMethodCali.h
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef ICPMETHODCALI_H
#define ICPMETHODCALI_H

#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <thread>

#include "CalibrateBase.h"
#include "paramLoad/LivoxConfig.h"

namespace calibrate {
class IcpMethodCali : public CalibrateBase {
 public:
  IcpMethodCali();
  ~IcpMethodCali();

  virtual Result Exectute() override;

 private:
  void SyncCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud);

  uint8_t CheckHeader(std_msgs::msg::Header header_0,
                      std_msgs::msg::Header header_1);

  void GetIntensityCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud);

 public:
 private:
  Result m_lastResult;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_source_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_target_cloud;
};
}  // namespace calibrate
#endif