
/**
 * @file qos_profiles.hpp
 * @brief AEIROBOT Toolbox QoS 프로필 정의
 *
 * 이 헤더는 다양한 통신 요구사항(제어, 센서, 토픽, 서비스, 액션, 파라미터)에 맞춘
 * rclcpp::QoS 객체를 생성하여 제공합니다.
 * 기본 프로필은 rmw_qos_profile_default로 초기화되고, 각 프로필별로 keep_last,
 * reliability, durability, deadline, lifespan, liveliness 등을 설정합니다.
 */
#ifndef AEIROBOT_TOOLBOX_QOS_PROFILES_HPP
#define AEIROBOT_TOOLBOX_QOS_PROFILES_HPP

#include "rclcpp/rclcpp.hpp"
#include "aeirobot_toolbox/basic_tools.hpp"
#include <yaml-cpp/yaml.h>

namespace aeirobot
{
  /**
   * @brief 제어 루프용 QoS 프로필
   *
   * control_time 파라미터를 기반으로 deadline을 설정하며,
   * keep_last 10, Reliable, Volatile, Automatic liveliness를 사용합니다.
   *
   * @return rclcpp::QoS 제어 메시지 전송에 적합한 QoS 설정
   */
  const rclcpp::QoS qos_control_profile = []()
  {
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.keep_last(10);
#if ROS_DISTRO == foxy
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
#else
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile.deadline(rclcpp::Duration::from_seconds(aeirobot::GetParameter<double>("time_parameter", "control_time")));
    qos_profile.lifespan(rclcpp::Duration::from_seconds(1));
    qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
#endif
    return qos_profile;
  }();

  /**
   * @brief 센서 데이터 전송용 QoS 프로필
   *
   * keep_last 5, BestEffort, Volatile, Automatic liveliness를 사용합니다.
   *
   * @return rclcpp::QoS 센서 메시지 전송에 적합한 QoS 설정
   */
  const rclcpp::QoS qos_sensor_profile = []()
  {
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.keep_last(5);
#if ROS_DISTRO == foxy
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
#else
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile.deadline();
    qos_profile.lifespan();
    qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);
#endif
    return qos_profile;
  }();

  /**
   * @brief 일반 토픽 통신용 QoS 프로필
   *
   * keep_last 5, BestEffort, Volatile, Automatic liveliness를 사용합니다.
   *
   * @return rclcpp::QoS 일반 토픽 통신에 적합한 QoS 설정
   */
  const rclcpp::QoS qos_topic_profile = []()
  {
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.keep_last(5);
#if ROS_DISTRO == foxy
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
#else
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile.deadline();
    qos_profile.lifespan();
    qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);
#endif
    return qos_profile;
  }();

  /**
   * @brief 서비스 호출용 QoS 프로필
   *
   * keep_last 10, Reliable, Volatile, Automatic liveliness를 사용합니다.
   *
   * @return rclcpp::QoS 서비스 통신에 적합한 QoS 설정
   */
  const rclcpp::QoS qos_service_profile = []()
  {
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.keep_last(10);
#if ROS_DISTRO == foxy
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
#else
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile.deadline();
    qos_profile.lifespan();
    qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);
#endif
    return qos_profile;
  }();

  /**
   * @brief 액션 서버용 QoS 프로필
   *
   * keep_last 1, Reliable, TransientLocal, Automatic liveliness를 사용합니다.
   *
   * @return rclcpp::QoS 액션 서버 통신에 적합한 QoS 설정
   */
  const rclcpp::QoS qos_action_profile = []()
  {
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.keep_last(1);
#if ROS_DISTRO == foxy
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
#else
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
    qos_profile.deadline();
    qos_profile.lifespan();
    qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);
#endif
    return qos_profile;
  }();

  /**
   * @brief 파라미터 통신용 QoS 프로필
   *
   * keep_last 1000, Reliable, Volatile, Automatic liveliness를 사용합니다.
   *
   * @return rclcpp::QoS 파라미터 통신에 적합한 QoS 설정
   */
  const rclcpp::QoS qos_parameter_profile = []()
  {
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.keep_last(1000);
#if ROS_DISTRO == foxy
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
#else
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile.deadline();
    qos_profile.lifespan();
    qos_profile.liveliness(rclcpp::LivelinessPolicy::Automatic);
#endif
    return qos_profile;
  }();
}

#endif // AEIROBOT_TOOLBOX_QOS_PROFILES_HPP
