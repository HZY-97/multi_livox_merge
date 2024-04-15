/**
 * @file LivoxConfig.cpp
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "LivoxConfig.h"

#include <cstddef>

using std::lock_guard;

namespace paramLoad {

LivoxConfig* LivoxConfig::livoxConfigPtr = nullptr;
mutex LivoxConfig::livoxConfigPtrMtx;

LivoxConfig* LivoxConfig::GetInstance() {
  if (livoxConfigPtr == nullptr) {
    lock_guard<mutex> lock(livoxConfigPtrMtx);
    if (livoxConfigPtr == nullptr) {
      livoxConfigPtr = new LivoxConfig();
    }
  }

  return livoxConfigPtr;
}

void LivoxConfig::SetConfigFilePath(const string& config_file_path) {
  LOG(INFO) << "set livox_config.json path:" << config_file_path;
  m_configPath = config_file_path;
}

LivoxConfig::LivoxConfig() {
}

LivoxConfig::~LivoxConfig() {
}

void LivoxConfig::LoadConfig() {
  string json_path = m_configPath + "/livox_config.json";
  std::ifstream parseJson(json_path);
  auto livox_config_json = nlohmann::json::parse(parseJson);
  parseJson.close();

  LOG(INFO) << "Load livox_config.json";
  LOG(INFO) << livox_config_json.dump(2);
  LOG(INFO) << "============================================";

  try {
    mode = livox_config_json["working_mode"]["mode"];
    size_t use_lidar_count = livox_config_json["use_lidar_id"].size();
    LOG(INFO) << "use_lidar_count = " << use_lidar_count;
    for (size_t i = 0; i < use_lidar_count; i++) {
      use_lidar_id.push_back(livox_config_json["use_lidar_id"][i].get<int>());
      LOG(INFO) << "use_lidar_id = " << use_lidar_id[i];
    }

    size_t use_imu_count = livox_config_json["use_imu_id"].size();
    LOG(INFO) << "use_imu_count = " << use_imu_count;
    for (size_t i = 0; i < use_imu_count; i++) {
      use_imu_id.push_back(livox_config_json["use_imu_id"][i].get<int>());
      LOG(INFO) << "use_imu_id = " << use_imu_id[i];
    }

    size_t mid_360_count = livox_config_json["mid_360"].size();
    LOG(INFO) << "mid_360_count = " << mid_360_count;
    LOG(INFO) << "==========================================";
    for (size_t i = 0; i < mid_360_count; i++) {
      Common::Type::LivoxInfo livox_info;
      livox_info.id = livox_config_json["mid_360"][i]["id"].get<int>();
      livox_info.ip = livox_config_json["mid_360"][i]["ip"].get<string>();
      livox_info.x = livox_config_json["mid_360"][i]["x"].get<int>();
      livox_info.y = livox_config_json["mid_360"][i]["y"].get<int>();
      livox_info.z = livox_config_json["mid_360"][i]["z"].get<int>();
      livox_info.roll = livox_config_json["mid_360"][i]["roll"].get<int>();
      livox_info.pitch = livox_config_json["mid_360"][i]["pitch"].get<int>();
      livox_info.yaw = livox_config_json["mid_360"][i]["yaw"].get<int>();
      m_livoxInfoVec.push_back(livox_info);
      LOG(INFO) << livox_info.ShowLivoxInfo();
      LOG(INFO) << "==========================================";
    }

  } catch (const nlohmann::json::exception& e) {
    std::cerr << "\033[31m" << e.what() << "\033[0m" << '\n';
  }
}

}  // namespace paramLoad