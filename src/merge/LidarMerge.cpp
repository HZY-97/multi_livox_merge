#include "merge/LidarMerge.h"

#include <pcl/io/pcd_io.h>

#include <pcl/impl/point_types.hpp>

#include "manager/NodeManager.h"

namespace merge {
LidarMerge::LidarMerge() {
  if (paramLoad::LivoxConfig::GetInstance()->save_pcd) {
    Common::Init::GetInstallPath(m_saveMergePcdPath);
    m_saveMergePcdPath = m_saveMergePcdPath + "/MergePcd/";
    std::string command = "mkdir -p " + m_saveMergePcdPath;
    system(command.c_str());
  }
}

LidarMerge::~LidarMerge() {
}

void LidarMerge::MergeCloud() {
  if (!SyncCloud()) {
    return;
  }
  PubMeergeCloud();
}

bool LidarMerge::SyncCloud() {
  cloud_0_ptr.reset();
  cloud_1_ptr.reset();
  auto data = manager::DataManager::GetInstance();
  livox_ros_driver2::msg::CustomMsg::SharedPtr tmp_cloud_0_ptr = nullptr;
  livox_ros_driver2::msg::CustomMsg::SharedPtr tmp_cloud_1_ptr = nullptr;
  tmp_cloud_0_ptr = data->GetFrontMsg(0);
  tmp_cloud_1_ptr = data->GetFrontMsg(1);
  if (tmp_cloud_0_ptr == nullptr || tmp_cloud_1_ptr == nullptr) {
    LOG(ERROR) << "No cloud";
    return false;
  }
  bool is_sync = false;
  while (is_sync) {
    if (CheckHeader(tmp_cloud_0_ptr->header, tmp_cloud_1_ptr->header) == 1) {
      LOG(WARNING) << "wait sync id:1";
      tmp_cloud_1_ptr = data->GetFrontMsg(1);
    } else if (CheckHeader(tmp_cloud_0_ptr->header, tmp_cloud_1_ptr->header) ==
               0) {
      LOG(WARNING) << "wait sync id:0";
      tmp_cloud_0_ptr = data->GetFrontMsg(0);
    } else if (CheckHeader(tmp_cloud_0_ptr->header, tmp_cloud_1_ptr->header) ==
               100) {
      is_sync = true;
    }
  }
  cloud_0_ptr = tmp_cloud_0_ptr;
  cloud_1_ptr = tmp_cloud_1_ptr;
  return true;
}

uint8_t LidarMerge::CheckHeader(std_msgs::msg::Header header_0,
                                std_msgs::msg::Header header_1) {
  double time_ms_0 = header_0.stamp.nanosec / 1000000.0;
  double time_ms_1 = header_1.stamp.nanosec / 1000000.0;
  if ((time_ms_0 - time_ms_1) > 0.0 && (time_ms_0 - time_ms_1) > 100.0) {
    // 1 too old
    return 1;
  } else if ((time_ms_0 - time_ms_1) < 0.0 &&
             (time_ms_0 - time_ms_1) < -100.0) {
    // 0 too old
    return 0;
  } else {
    return 100;
  }
}

void LidarMerge::PubMeergeCloud() {
  livox_ros_driver2::msg::CustomMsg::SharedPtr full_cloud_ptr;
  full_cloud_ptr = cloud_0_ptr;
  LOG(INFO) << "cloud_0_ptr size= " << cloud_0_ptr->points.size();
  LOG(INFO) << "cloud_1_ptr size= " << cloud_1_ptr->points.size();
  // for (size_t i = 0; i < cloud_1_ptr->points.size(); i++) {
  //   full_cloud_ptr->points.push_back(cloud_1_ptr->points[i]);
  // }
  full_cloud_ptr->points.insert(full_cloud_ptr->points.end(),
                                cloud_1_ptr->points.begin(),
                                cloud_1_ptr->points.end());
  LOG(INFO) << "full_cloud_ptr size= " << full_cloud_ptr->points.size();

  manager::NodeManager::GetInstance()->m_topicManagerPtr->m_mergeLivox->publish(
      *full_cloud_ptr);

  if (paramLoad::LivoxConfig::GetInstance()->save_pcd) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_full_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < full_cloud_ptr->points.size(); i++) {
      pcl_full_cloud->points.push_back(pcl::PointXYZI(
          full_cloud_ptr->points[i].x, full_cloud_ptr->points[i].y,
          full_cloud_ptr->points[i].z, full_cloud_ptr->points[i].reflectivity));
    }

    pcl::io::savePCDFileBinary(m_saveMergePcdPath + "0.pcd", *pcl_full_cloud);
  }

  full_cloud_ptr.reset();
}

}  // namespace merge