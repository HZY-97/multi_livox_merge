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
  std::lock_guard<std::mutex> lock0(m_dataManager->m_caliData_0_deq_Mtx);
  std::lock_guard<std::mutex> lock1(m_dataManager->m_caliData_1_deq_Mtx);

  while (!m_dataManager->m_caliData_0_deq.empty() &&
         !m_dataManager->m_caliData_1_deq.empty()) {
    tmp_cloud_0_ptr = m_dataManager->m_caliData_0_deq.front();
    tmp_cloud_1_ptr = m_dataManager->m_caliData_1_deq.front();

    double time_diff = (tmp_cloud_0_ptr->header.stamp.sec * 1000.0 +
                        tmp_cloud_0_ptr->header.stamp.nanosec / 1000000.0) -
                       (tmp_cloud_1_ptr->header.stamp.sec * 1000.0 +
                        tmp_cloud_1_ptr->header.stamp.nanosec / 1000000.0);
    LOG(INFO) << "time_diff 0-1= " << time_diff;
    if (std::abs(time_diff) < 50.0) {
      cloud_0_ptr = tmp_cloud_0_ptr;
      cloud_1_ptr = tmp_cloud_1_ptr;

      m_dataManager->m_caliData_0_deq.pop_front();
      m_dataManager->m_caliData_1_deq.pop_front();
      return true;
    } else {
      if (time_diff > 0) {
        LOG(ERROR) << "id:1 is old wait...";
        m_dataManager->m_caliData_1_deq.pop_front();
      } else {
        LOG(ERROR) << "id:0 is old wait...";
        m_dataManager->m_caliData_0_deq.pop_front();
      }
    }
  }

  return false;
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

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_full_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (paramLoad::LivoxConfig::GetInstance()->save_pcd) {
    for (size_t i = 0; i < full_cloud_ptr->points.size(); i++) {
      pcl_full_cloud->points.push_back(pcl::PointXYZI(
          full_cloud_ptr->points[i].x, full_cloud_ptr->points[i].y,
          full_cloud_ptr->points[i].z, full_cloud_ptr->points[i].reflectivity));
    }
    pcl::io::savePCDFileBinary(m_saveMergePcdPath + "0.pcd", *pcl_full_cloud);
  }
  if (manager::NodeManager::GetInstance()
          ->m_topicManagerPtr->m_showMergeLivox->get_subscription_count() !=
      0) {
    auto pub_show = manager::NodeManager::GetInstance()
                        ->m_topicManagerPtr->m_showMergeLivox;
    sensor_msgs::msg::PointCloud2 tmpCloud;

    pcl::toROSMsg(*pcl_full_cloud, tmpCloud);
    tmpCloud.header.frame_id = "map";
    pub_show->publish(tmpCloud);
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