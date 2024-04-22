/**
 * @file sysQos.hpp
 * @author huizeyu (huizeyu@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2024-01-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SYS_QOS_HPP
#define SYS_QOS_HPP

#include <rclcpp/rclcpp.hpp>

namespace common::qos {
class QosConfig {
 public:
  QosConfig() {
  }
  ~QosConfig() {
  }
  rmw_qos_profile_t qos_profile{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                1,
                                RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                RMW_QOS_DEADLINE_DEFAULT,
                                RMW_QOS_LIFESPAN_DEFAULT,
                                RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                false};

  //   rclcpp::QoS qos = rclcpp::QoS(
  //       rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
  //       qos_profile);
  rclcpp::QoS qos = rclcpp::QoS(1);

  rmw_qos_profile_t qos_profile_imu{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                    2000,
                                    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                    RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                    RMW_QOS_DEADLINE_DEFAULT,
                                    RMW_QOS_LIFESPAN_DEFAULT,
                                    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                    false};

  //   rclcpp::QoS qos_imu = rclcpp::QoS(
  //       rclcpp::QoSInitialization(qos_profile_imu.history,
  //       qos_profile_imu.depth), qos_profile_imu);
  rclcpp::QoS qos_imu = rclcpp::QoS(2000);

  rmw_qos_profile_t qos_profile_lidar{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                      5,
                                      RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                      RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                      RMW_QOS_DEADLINE_DEFAULT,
                                      RMW_QOS_LIFESPAN_DEFAULT,
                                      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                      false};

  //   rclcpp::QoS qos_lidar =
  //       rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_lidar.history,
  //                                             qos_profile_lidar.depth),
  //                   qos_profile_lidar);
  rclcpp::QoS qos_lidar = rclcpp::QoS(5);
};
}  // namespace common::qos

#endif
