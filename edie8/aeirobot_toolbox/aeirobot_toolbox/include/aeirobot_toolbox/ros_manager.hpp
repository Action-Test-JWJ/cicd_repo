/**
 * @file ros_manager.hpp
 * @brief ROS 2 노드 관리 클래스 정의
 *
 * 다양한 토픽 퍼블리셔, 서브스크라이버, 서비스 서버/클라이언트,
 * 그리고 월 타이머를 관리하는 RosManager 클래스 인터페이스를 제공합니다.
 */

#ifndef ROS_MANAGER_HPP
#define ROS_MANAGER_HPP

#include <unordered_map>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "aeirobot_toolbox/qos_profiles.hpp"

using namespace std;

namespace aeirobot
{

  /**
   * @class RosManager
   * @brief ROS 2 노드 기능을 통합하여 편리하게 사용하기 위한 관리 클래스
   *
   * 이 클래스는 퍼블리셔, 서브스크라이버, 서비스 서버/클라이언트,
   * 그리고 월 타이머를 내부적으로 저장 및 관리하며,
   * 동적으로 추가·삭제할 수 있는 인터페이스를 제공합니다.
   */
  class RosManager : public rclcpp::Node
  {
  private:
    unordered_map<string, shared_ptr<void>> publishers;              /**< 주제별 퍼블리셔 저장소 */
    unordered_map<string, shared_ptr<void>> subscribers;             /**< 주제별 서브스크라이버 저장소 */
    unordered_map<string, shared_ptr<void>> services;                /**< 서비스 서버 저장소 */
    unordered_map<string, shared_ptr<void>> clients;                 /**< 서비스 클라이언트 저장소 */
    unordered_map<string, rclcpp::TimerBase::SharedPtr> wall_timers; /**< 이름별 월 타이머 저장소 */

  public:
    /**
     * @brief 토픽에 메시지를 퍼블리시합니다. 퍼블리셔가 없으면 자동으로 생성합니다.
     * @tparam MsgType 퍼블리시할 메시지 타입
     * @param topic_name 토픽 이름
     * @param msg 퍼블리시할 메시지 객체
     * @return 항상 true를 반환합니다.
     */
    template <typename MsgType>
    bool Publish(const string &topic_name, MsgType msg)
    {
      auto it = publishers.find(topic_name);
      if (it == publishers.end())
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Publisher does not exist. Add it now. (\"" << topic_name << "\")");
        AddPublisher<MsgType>(topic_name);
      }

      auto publisher = GetPublisher<MsgType>(topic_name);
      publisher->publish(msg);
      return true;
    }

    /**
     * @brief 새로운 퍼블리셔를 생성하고 저장소에 추가합니다.
     * @tparam MsgType 퍼블리시할 메시지 타입
     * @param topic_name 토픽 이름
     * @param qos QoS 프로필 (기본값: qos_topic_profile)
     * @return 생성된 퍼블리셔의 shared_ptr
     */
    template <typename MsgType>
    shared_ptr<rclcpp::Publisher<MsgType>> AddPublisher(const string &topic_name,
                                                        const rclcpp::QoS &qos = qos_topic_profile)
    {
      auto pub = this->create_publisher<MsgType>(topic_name, qos);
      auto result = publishers.insert({topic_name, pub});

      if (!result.second)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Publisher already exist. (\"" << topic_name << "\")");
      }
      else
      {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Publisher added. (\"" << topic_name << "\")");
      }

      return GetPublisher<MsgType>(topic_name);
    }

    /**
     * @brief 저장된 퍼블리셔를 반환합니다.
     * @tparam MsgType 퍼블리시할 메시지 타입
     * @param topic_name 토픽 이름
     * @return 퍼블리셔의 shared_ptr
     */
    template <typename MsgType>
    shared_ptr<rclcpp::Publisher<MsgType>> GetPublisher(const string &topic_name)
    {
      return static_pointer_cast<rclcpp::Publisher<MsgType>>(publishers[topic_name]);
    }

    /**
     * @brief 퍼블리셔를 저장소에서 제거합니다.
     * @param topic_name 토픽 이름
     * @return 성공 시 true, 존재하지 않을 경우 false
     */
    bool RemovePublisher(const string &topic_name)
    {
      auto it = publishers.find(topic_name);
      if (it != publishers.end())
      {
        publishers.erase(topic_name);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Publisher removed. (\"" << topic_name << "\")");
        return true;
      }
      else
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Publisher does not exist. (\"" << topic_name << "\")");
        return false;
      }
    }

    /**
     * @brief 새로운 서브스크라이버를 생성하고 저장소에 추가합니다.
     * @tparam MsgType 구독할 메시지 타입
     * @param topic_name 토픽 이름
     * @param callback 메시지 수신 시 호출할 콜백 함수
     * @param qos QoS 프로필 (기본값: qos_topic_profile)
     * @return 생성된 서브스크라이버의 shared_ptr
     */
    template <typename MsgType>
    shared_ptr<rclcpp::Subscription<MsgType>> AddSubscriber(
        const string &topic_name,
        function<void(const typename MsgType::SharedPtr)> callback,
        const rclcpp::QoS &qos = qos_topic_profile)
    {
      auto sub = this->create_subscription<MsgType>(topic_name, qos, callback);
      auto result = subscribers.insert({topic_name, sub});

      if (!result.second)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Subscriber already exist. (\"" << topic_name << "\")");
      }
      else
      {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Subscriber added. (\"" << topic_name << "\")");
      }
      return GetSubscriber<MsgType>(topic_name);
    }

    /**
     * @brief 저장된 서브스크라이버를 반환합니다.
     * @tparam MsgType 구독할 메시지 타입
     * @param topic_name 토픽 이름
     * @return 서브스크라이버의 shared_ptr
     */
    template <typename MsgType>
    shared_ptr<rclcpp::Subscription<MsgType>> GetSubscriber(const string &topic_name)
    {
      return static_pointer_cast<rclcpp::Subscription<MsgType>>(subscribers[topic_name]);
    }

    /**
     * @brief 서브스크라이버를 저장소에서 제거합니다.
     * @param topic_name 토픽 이름
     * @return 성공 시 true, 존재하지 않을 경우 false
     */
    bool RemoveSubscriber(const string &topic_name)
    {
      auto it = subscribers.find(topic_name);
      if (it != subscribers.end())
      {
        subscribers.erase(topic_name);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Subscriber removed. (\"" << topic_name << "\")");
        return true;
      }
      else
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Subscriber does not exist. (\"" << topic_name << "\")");
        return false;
      }
    }

    /**
     * @brief 새로운 서비스 서버를 생성하고 저장소에 추가합니다.
     * @tparam MsgType 서비스 타입
     * @param service_name 서비스 이름
     * @param callback 서비스 요청 시 호출할 콜백 함수
     * @return 생성된 서비스 서버의 shared_ptr
     */
    template <typename MsgType>
    shared_ptr<rclcpp::Service<MsgType>> AddService(
        const string &service_name,
        function<bool([[maybe_unused]]const typename MsgType::Request::ConstSharedPtr,
                      typename MsgType::Response::SharedPtr)> callback)
    {
      auto srv = this->create_service<MsgType>(service_name, callback);
      auto result = services.insert({service_name, srv});

      if (!result.second)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Service already exist. (\"" << service_name << "\")");
      }
      else
      {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Service added. (\"" << service_name << "\")");
      }
      return GetService<MsgType>(service_name);
    }

    /**
     * @brief 저장된 서비스 서버를 반환합니다.
     * @tparam MsgType 서비스 타입
     * @param service_name 서비스 이름
     * @return 서비스 서버의 shared_ptr
     */
    template <typename MsgType>
    shared_ptr<rclcpp::Service<MsgType>> GetService(const string &service_name)
    {
      return static_pointer_cast<rclcpp::Service<MsgType>>(services[service_name]);
    }

    /**
     * @brief 서비스 서버를 저장소에서 제거합니다.
     * @param service_name 서비스 이름
     * @return 성공 시 true, 존재하지 않을 경우 false
     */
    bool RemoveService(const string &service_name)
    {
      auto it = services.find(service_name);
      if (it != services.end())
      {
        services.erase(service_name);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Service removed. (\"" << service_name << "\")");
        return true;
      }
      else
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Service does not exist. (\"" << service_name << "\")");
        return false;
      }
    }

    /**
     * @brief 새로운 서비스 클라이언트를 생성하고 저장소에 추가합니다.
     * @tparam MsgType 서비스 타입
     * @param client_name 클라이언트 이름
     * @return 생성된 서비스 클라이언트의 shared_ptr
     */
    template <typename MsgType>
    shared_ptr<rclcpp::Client<MsgType>> AddClient(const string &client_name)
    {
      auto client = this->create_client<MsgType>(client_name);
      auto result = clients.insert({client_name, client}); // 수정: services → clients

      if (!result.second)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Client already exist. (\"" << client_name << "\")");
      }
      else
      {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Client added. (\"" << client_name << "\")");
      }
      return GetClient<MsgType>(client_name);
    }

    /**
     * @brief 저장된 서비스 클라이언트를 반환합니다.
     * @tparam MsgType 서비스 타입
     * @param client_name 클라이언트 이름
     * @return 서비스 클라이언트의 shared_ptr
     */
    template <typename MsgType>
    shared_ptr<rclcpp::Client<MsgType>> GetClient(const string &client_name)
    {
      return static_pointer_cast<rclcpp::Client<MsgType>>(clients[client_name]);
    }

    /**
     * @brief 서비스 클라이언트를 저장소에서 제거합니다.
     * @param client_name 클라이언트 이름
     * @return 성공 시 true, 존재하지 않을 경우 false
     */
    bool RemoveClient(const string &client_name)
    {
      auto it = clients.find(client_name);
      if (it != clients.end())
      {
        clients.erase(client_name);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Client removed. (\"" << client_name << "\")");
        return true;
      }
      else
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Client does not exist. (\"" << client_name << "\")");
        return false;
      }
    }

    /**
     * @brief 월 타이머를 생성하고 저장소에 추가합니다. 이 타이머는 무조건 시스템 시간을 기준으로 작동합니다.
     * @param name 타이머 식별 이름
     * @param interval 타이머 간격 (milliseconds)
     * @param callback 타이머 만료 시 호출할 콜백 함수
     * @return 추가 성공 시 true, 이미 존재하면 false
     */
    bool AddWallTimer(const string &name,
                      chrono::milliseconds interval,
                      function<void()> callback)
    {
      auto wall_timer = this->create_wall_timer(interval, callback);
      auto result = wall_timers.insert({name, static_pointer_cast<rclcpp::TimerBase>(wall_timer)});

      if (!result.second)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Timer already exist. (\"" << name << "\")");
        return false;
      }
      else
      {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "WallTimer added. (\"" << name << "\")");
        return true;
      }
    }

    /**
     * @brief 적응형 타이머를 생성하고 저장소에 추가합니다. 이 타이머는 시뮬레이션 시간 사용여부에 따라 시간 기준을 적용합니다. 
     * @param name 타이머 식별 이름
     * @param interval 타이머 간격 (milliseconds)
     * @param callback 타이머 만료 시 호출할 콜백 함수
     * @return 추가 성공 시 true, 이미 존재하면 false
     */
    bool AddTimer(const string &name,
                  chrono::milliseconds interval,
                  function<void()> callback)
    {
      auto wall_timer = rclcpp::create_timer(this, this->get_clock(), interval, callback);
      auto result = wall_timers.insert({name, static_pointer_cast<rclcpp::TimerBase>(wall_timer)});

      if (!result.second)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Timer already exist. (\"" << name << "\")");
        return false;
      }
      else
      {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Adaptive Timer added. (\"" << name << "\")");
        return true;
      }
    }

    /**
     * @brief 월 타이머를 저장소에서 제거하고 취소합니다.
     * @param name 타이머 식별 이름
     * @return 제거 성공 시 true, 존재하지 않을 경우 false
     */
    bool RemoveWallTimer(const string &name)
    {
      auto it = wall_timers.find(name);
      if (it != wall_timers.end())
      {
        wall_timers[name]->cancel();
        wall_timers.erase(name);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "WallTimer removed. (\"" << name << "\")");
        return true;
      }
      else
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "WallTimer does not exist. (\"" << name << "\")");
        return false;
      }
    }

    /**
     * @brief RosManager 생성자
     * @param node_name ROS 2 노드 이름
     */
    RosManager(const string &node_name)
        : Node(node_name)
    {
    }
  };

} // namespace aeirobot

#endif // ROS_MANAGER_HPP
