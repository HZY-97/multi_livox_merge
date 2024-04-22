/**
 * @file CalibrateBase.cpp
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-18
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "CalibrateBase.h"

#include <cstdlib>

namespace calibrate {
CalibrateBase::CalibrateBase() {
  Common::Init::GetInstallPath(m_saveCaliedPcdPath);
  m_saveCaliedPcdPath = m_saveCaliedPcdPath + "/CaliedPcd/";
  std::string command = "rm -rf " + m_saveCaliedPcdPath;
  system(command.c_str());
  command = "mkdir -p " + m_saveCaliedPcdPath;
  system(command.c_str());
}

CalibrateBase::~CalibrateBase() {
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CalibrateBase::TransformPointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, Result transformIn) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(
      new pcl::PointCloud<pcl::PointXYZI>());

  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);

  Eigen::Affine3f transCur = pcl::getTransformation(
      transformIn.x, transformIn.y, transformIn.z, transformIn.roll,
      transformIn.pitch, transformIn.yaw);

#pragma omp parallel for num_threads(8)
  for (int i = 0; i < cloudSize; ++i) {
    const auto &pointFrom = cloudIn->points[i];
    cloudOut->points[i].x = transCur(0, 0) * pointFrom.x +
                            transCur(0, 1) * pointFrom.y +
                            transCur(0, 2) * pointFrom.z + transCur(0, 3);
    cloudOut->points[i].y = transCur(1, 0) * pointFrom.x +
                            transCur(1, 1) * pointFrom.y +
                            transCur(1, 2) * pointFrom.z + transCur(1, 3);
    cloudOut->points[i].z = transCur(2, 0) * pointFrom.x +
                            transCur(2, 1) * pointFrom.y +
                            transCur(2, 2) * pointFrom.z + transCur(2, 3);
    cloudOut->points[i].intensity = pointFrom.intensity;
  }
  return cloudOut;
}

}  // namespace calibrate