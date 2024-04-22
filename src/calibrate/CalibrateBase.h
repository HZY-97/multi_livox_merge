/**
 * @file CalibrateBase.h
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-18
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef CALIBRATEBASE_H
#define CALIBRATEBASE_H

#include <string>
#pragma once

#include <glog/logging.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <iomanip>
#include <thread>
#include <vector>

#include "common/initialization.hpp"
#include "manager/DataManager.h"
#include "pcl/impl/point_types.hpp"

using std::vector;

namespace calibrate {

struct Result {
  Result() {
    x = 0.0f, y = 0.0f, z = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  }
  float x;
  float y;
  float z;
  float roll;   // rad
  float pitch;  // rad
  float yaw;    // rad

  std::string ShowResult() {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);  // 设置小数点后保留6位
    oss << "\033[1;32m\n|* x = " << x << std::setw(9) << " y = " << y
        << std::setw(9) << " z = " << z << std::setw(9)
        << " *|\n|* roll = " << roll * 180.0f / M_PI << std::setw(9)
        << " pitch = " << pitch * 180.0f / M_PI << std::setw(9)
        << " yaw = " << yaw * 180.0f / M_PI << " *|\033[0m" << std::setw(9);

    return oss.str();  // 将流中的内容转换为字符串并返回
  }
};

class CalibrateBase {
 public:
  CalibrateBase();
  virtual ~CalibrateBase();

  virtual Result Exectute() = 0;

  pcl::PointCloud<pcl::PointXYZI>::Ptr TransformPointCloud(
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, Result transformIn);

 protected:
  vector<Result> m_resultsVec;

  std::string m_saveCaliedPcdPath;

 private:
};
}  // namespace calibrate

#endif