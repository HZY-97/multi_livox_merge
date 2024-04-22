#ifndef SRC_PARAM_LOAD_LIVOX_CONFIG_H
#define SRC_PARAM_LOAD_LIVOX_CONFIG_H

#pragma once

#include <glog/logging.h>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "common/json.hpp"
#include "common/livox_info.hpp"

using nlohmann::json;
using std::mutex;
using std::string;
using std::vector;

namespace paramLoad {

class LivoxConfig final {
 public:
  static LivoxConfig* GetInstance();
  void SetConfigFilePath(const string& config_file_path);
  void LoadConfig();
  ~LivoxConfig();

  LivoxConfig(const LivoxConfig&) = delete;
  LivoxConfig& operator=(const LivoxConfig&) = delete;
  LivoxConfig(LivoxConfig&&) = delete;
  LivoxConfig& operator=(LivoxConfig&&) = delete;

 private:
  LivoxConfig();

  string ReplaceIpDotsWithUnderscores(const string& ip);

 public:
  int mode;
  vector<int> use_lidar_id;
  vector<int> use_imu_id;
  vector<Common::Type::LivoxInfo> m_livoxInfoVec;

 private:
  string m_configPath;
  static LivoxConfig* livoxConfigPtr;
  static mutex livoxConfigPtrMtx;
};

}  // namespace paramLoad

#endif