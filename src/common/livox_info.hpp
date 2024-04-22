/**
 * @file livox_info.hpp
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SRC_COMMON_LIVOX_INFO_HPP
#define SRC_COMMON_LIVOX_INFO_HPP

#include <string>

#include "common/json.hpp"

using std::string;

namespace Common {
namespace Type {
class LivoxInfo {
 public:
  explicit LivoxInfo()
      : id(-1),
        ip(""),
        x(0.0),
        y(0.0),
        z(0.0),
        roll(0.0),
        pitch(0.0),
        yaw(0.0) {
  }
  explicit LivoxInfo(const int &id, const string &ip, const double &x,
                     const double &y, const double &z, const double &roll,
                     const double &pitch, const double &yaw) {
    this->id = id;
    this->ip = ip;
    this->x = x;
    this->y = y;
    this->z = z;
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;
  }

  LivoxInfo(const LivoxInfo &other) {
    this->id = other.id;
    this->ip = other.ip;
    this->x = other.x;
    this->y = other.y;
    this->z = other.z;
    this->roll = other.roll;
    this->pitch = other.pitch;
    this->yaw = other.yaw;
  }
  LivoxInfo(const LivoxInfo &&other) {
    this->id = other.id;
    this->ip = other.ip;
    this->x = other.x;
    this->y = other.y;
    this->z = other.z;
    this->roll = other.roll;
    this->pitch = other.pitch;
    this->yaw = other.yaw;
  }

  LivoxInfo &operator=(const LivoxInfo &other) {
    if (this != &other) {
      this->id = other.id;
      this->ip = other.ip;
      this->x = other.x;
      this->y = other.y;
      this->z = other.z;
      this->roll = other.roll;
      this->pitch = other.pitch;
      this->yaw = other.yaw;
    }
    return *this;
  }

  ~LivoxInfo() {
  }

  std::string ShowLivoxInfo() {
    return "\nid:" + std::to_string(id) + " ip:" + ip +
           "\nx:" + std::to_string(x) + " y:" + std::to_string(y) +
           " z:" + std::to_string(z) + "\nroll:" + std::to_string(roll) +
           " pitch:" + std::to_string(pitch) + " yaw:" + std::to_string(yaw);
  }

  void ToDegree() {
    roll = roll * 180.0f / M_PI;
    pitch = pitch * 180.0f / M_PI;
    yaw = yaw * 180.0f / M_PI;
  }

  void ToRadian() {
    roll = roll * M_PI / 180.0f;
    pitch = pitch * M_PI / 180.0f;
    yaw = yaw * M_PI / 180.0f;
  }

 public:
  int id;
  /** @brief 此处需存储的格式为192_168_3_110*/
  string ip;
  /** @brief xyz单位是米,roll/pitch/yaw单位是角度*/
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;

 private:
};
}  // namespace Type
}  // namespace Common

#endif