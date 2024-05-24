#include "merge/LidarMerge.h"

#include <glog/logging.h>
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

  m_dataManager = manager::DataManager::GetInstance();

  tmp_cloud_0_ptr = nullptr;
  tmp_cloud_1_ptr = nullptr;
  cloud_0_ptr = nullptr;
  cloud_1_ptr = nullptr;
}

LidarMerge::~LidarMerge() {
}

void LidarMerge::MergeCloud() {
  if (!SyncCloud()) {
    // LOG(WARNING) << "Sync Fail!";
    return;
  }
  PubMeergeCloud();
#ifdef SHOW_TIME
  /*calculate time*/
  static bool isFirst = true;
  if (isFirst) {
    isFirst = false;
    m_lastTime = std::chrono::high_resolution_clock::now();
    return;
  }
  m_currentTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> duration =
      m_currentTime - m_lastTime;
  LOG(INFO) << "use time: " << duration.count() << " ms";
  LOG(INFO) << "pub hz: " << 1000.0 / duration.count() << " hz";
  m_lastTime = m_currentTime;
/*calculate time*/
#endif
  LOG(INFO) << "Publish Success!";
}

bool LidarMerge::SyncCloud() {
  if (nullptr == tmp_cloud_0_ptr) {
    tmp_cloud_0_ptr = m_dataManager->GetFrontMsg(0);
  }

  if (nullptr == tmp_cloud_1_ptr) {
    tmp_cloud_1_ptr = m_dataManager->GetFrontMsg(1);
  }

  if (tmp_cloud_0_ptr == nullptr || tmp_cloud_1_ptr == nullptr) {
    // LOG(ERROR) << "No cloud";
    return false;
  }

  is_sync.store(false);

  while (!is_sync.load()) {
    auto header_0 = tmp_cloud_0_ptr->header;
    auto header_1 = tmp_cloud_1_ptr->header;

    double time_ms_0 =
        header_0.stamp.sec * 1000.0 + header_0.stamp.nanosec / 1000000.0;
    double time_ms_1 =
        header_1.stamp.sec * 1000.0 + header_1.stamp.nanosec / 1000000.0;

    LOG(INFO) << "time_ms_0= " << time_ms_0;
    LOG(INFO) << "time_ms_1= " << time_ms_1;
    LOG(INFO) << "time_diff 0-1 = " << time_ms_0 - time_ms_1;
    if (CheckTime(time_ms_0, time_ms_1) == 1) {
      // LOG(WARNING) << "wait sync id:1";
      LOG_FIRST_N(INFO, 15) << "wait sync id:1";
      tmp_cloud_1_ptr.reset();
      while (tmp_cloud_1_ptr == nullptr) {
        tmp_cloud_1_ptr = m_dataManager->GetFrontMsg(0);
      }
    } else if (CheckTime(time_ms_0, time_ms_1) == 0) {
      // LOG(WARNING) << "wait sync id:0";
      LOG_FIRST_N(INFO, 15) << "wait sync id:0";
      tmp_cloud_0_ptr.reset();
      while (tmp_cloud_0_ptr == nullptr) {
        tmp_cloud_0_ptr = m_dataManager->GetFrontMsg(0);
      }
    } else if (CheckTime(time_ms_0, time_ms_1) == 100) {
      is_sync.store(true);
    } else {
      LOG(ERROR) << "reset all cloud";
      tmp_cloud_0_ptr.reset();
      tmp_cloud_1_ptr.reset();
      is_sync.store(true);
      return false;
    }
  }
  cloud_0_ptr = tmp_cloud_0_ptr;
  cloud_1_ptr = tmp_cloud_1_ptr;

  return true;
}

uint8_t LidarMerge::CheckTime(double time_0_ms, double time_1_ms) {
  double time_diff = time_0_ms - time_1_ms;
  if (std::abs(time_diff) < 50.0) {
    // sync ok
    return 100;
  } else if (time_diff >= 100.0 && time_diff <= 300.0) {
    // 1 is old
    LOG(WARNING) << " 1 is old time_diff = " << time_diff;
    return 1;
  } else if (time_diff <= -100.0 && time_diff >= -300.0) {
    // 0 is old
    LOG(WARNING) << " 0 is old time_diff = " << time_diff;
    return 0;
  } else {
    LOG(ERROR) << "time_diff = " << time_diff;
    return 231;
  }
}

void LidarMerge::PubMeergeCloud() {
  auto start = std::chrono::high_resolution_clock::now();
  livox_ros_driver2::msg::CustomMsg::SharedPtr full_cloud_ptr;
  full_cloud_ptr = cloud_0_ptr;
  // LOG(INFO) << "cloud_0_ptr size= " << cloud_0_ptr->points.size();
  // LOG(INFO) << "cloud_1_ptr size= " << cloud_1_ptr->points.size();
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
  tmp_cloud_0_ptr.reset();
  tmp_cloud_1_ptr.reset();
  cloud_0_ptr.reset();
  cloud_1_ptr.reset();
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> duration = start - end;
  LOG(INFO) << "PubMeergeCloud use time= " << duration.count();
}

}  // namespace merge