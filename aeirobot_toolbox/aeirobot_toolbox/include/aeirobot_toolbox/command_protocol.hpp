/**
 * @file command_protocol.hpp
 * @brief 명령 프로토콜 관련 메시지 및 액션 변환 유틸 정의
 */
#ifndef COMMAND_PROTOCOL_HPP
#define COMMAND_PROTOCOL_HPP

#include <rclcpp_action/rclcpp_action.hpp>
#include "aeirobot_msgs/msg/command.hpp"
#include "aeirobot_msgs/action/command.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using Command = aeirobot_msgs::action::Command;
using GoalHandleCommand = rclcpp_action::ServerGoalHandle<Command>;

namespace aeirobot
{
  /**
   * @class Command
   * @brief 명령 프로토콜에서 사용되는 기본 명령 유형 상수를 정의하는 클래스
   */
  class Command
  {
  public:
    /** @brief 정지 명령 */
    static constexpr uint8_t k_stop = 0;
    /** @brief 모션 명령 */
    static constexpr uint8_t k_motion = 1;
    /** @brief 보행 명령 */
    static constexpr uint8_t k_walking = 2;
    /** @brief 추적 명령 */
    static constexpr uint8_t k_tracking = 3;
    /** @brief 킥 명령 */
    static constexpr uint8_t k_kick = 4;
    /** @brief 토크 제어 명령 */
    static constexpr uint8_t k_torque = 5;
    /** @brief 모드 설정 명령 */
    static constexpr uint8_t k_setmode = 6;
    /** @brief 튜닝 명령 */
    static constexpr uint8_t k_tuning = 7;
    /** @brief 팔 제어 명령 */
    static constexpr uint8_t k_arms = 8;
    /** @brief 손 제어 명령 */
    static constexpr uint8_t k_hands = 9;
    /** @brief 정의된 명령 종류 수 */
    static constexpr uint8_t k_max_size = 10;
  };

  /**
   * @brief 명령 스타일(세부 유형)을 정의하는 클래스 모음
   */
  class Style
  {
  public:
    /** @brief 스타일 없음 */
    static constexpr uint8_t k_none = 0;

    /**
     * @brief 모션 스타일 상수 정의
     */
    class Motion
    {
    public:
      /** @brief 전신 모션 */
      static constexpr uint8_t k_wholebody = 0;
      /** @brief 상체 모션 */
      static constexpr uint8_t k_torso = 1;
      /** @brief 팔 모션 */
      static constexpr uint8_t k_arms = 2;
      /** @brief 다리 모션 */
      static constexpr uint8_t k_legs = 3;
      /** @brief 팔+상체 모션 */
      static constexpr uint8_t k_arms_torso = 4;
      /** @brief RAIM 모션 */
      static constexpr uint8_t k_raim = 5;
      /** @brief 모션 완료 상태 */
      static constexpr uint8_t k_finish_1 = 6;
      /** @brief 정의된 모션 스타일 수 */
      static constexpr uint8_t k_max_size = 7;
    };

    /**
     * @brief 보행 스타일 상수 정의
     */
    class Walking
    {
    public:
      static constexpr uint8_t k_omni = 0;            /**< @brief 옴니 방향 보행 */
      static constexpr uint8_t k_parallel = 1;        /**< @brief 평행 보행 */
      static constexpr uint8_t k_forward = 2;         /**< @brief 전진 */
      static constexpr uint8_t k_backward = 3;        /**< @brief 후진 */
      static constexpr uint8_t k_left = 4;            /**< @brief 좌 이동 */
      static constexpr uint8_t k_right = 5;           /**< @brief 우 이동 */
      static constexpr uint8_t k_turn_left = 6;       /**< @brief 제자리 좌회전 */
      static constexpr uint8_t k_turn_right = 7;      /**< @brief 제자리 우회전 */
      static constexpr uint8_t k_centered_left = 8;   /**< @brief 중심 기준 좌 이동 */
      static constexpr uint8_t k_centered_right = 9;  /**< @brief 중심 기준 우 이동 */
      static constexpr uint8_t k_spot = 10;           /**< @brief 제자리 보행 */
      static constexpr uint8_t k_dribble = 11;        /**< @brief 드리블 */
      static constexpr uint8_t k_stop = 12;           /**< @brief 보행 정지 */
      static constexpr uint8_t k_speed_position = 13; /**< @brief 속도-위치 모드 */
      static constexpr uint8_t k_speed_velocity = 14; /**< @brief 속도-속도 모드 */
      static constexpr uint8_t k_gymnastics = 15;     /**< @brief 체조 모드 */
      static constexpr uint8_t k_max_size = 16;       /**< @brief 정의된 보행 스타일 수 */
    };

    /**
     * @brief 추적 스타일 상수 정의
     */
    class Tracking
    {
    public:
      static constexpr uint8_t k_point = 0;    /**< @brief 점 추적 */
      static constexpr uint8_t k_object = 1;   /**< @brief 객체 추적 */
      static constexpr uint8_t k_angle = 2;    /**< @brief 각도 추적 */
      static constexpr uint8_t k_max_size = 3; /**< @brief 정의된 추적 스타일 수 */
    };

    /**
     * @brief 킥 스타일 상수 정의
     */
    class Kick
    {
    public:
      static constexpr uint8_t k_pass = 0;                /**< @brief 패스 */
      static constexpr uint8_t k_left_kick = 1;           /**< @brief 왼발 킥 */
      static constexpr uint8_t k_right_kick = 2;          /**< @brief 오른발 킥 */
      static constexpr uint8_t k_left_elude_kick = 3;     /**< @brief 왼발 회피 킥 */
      static constexpr uint8_t k_right_elude_kick = 4;    /**< @brief 오른발 회피 킥 */
      static constexpr uint8_t k_left_elude_dribble = 5;  /**< @brief 왼발 회피 드리블 */
      static constexpr uint8_t k_right_elude_dribble = 6; /**< @brief 오른발 회피 드리블 */
      static constexpr uint8_t k_squat = 7;               /**< @brief 스쿼트 */
      static constexpr uint8_t k_squat_up = 8;            /**< @brief 스쿼트 업 */
      static constexpr uint8_t k_squat_down = 9;          /**< @brief 스쿼트 다운 */
      static constexpr uint8_t k_max_size = 10;           /**< @brief 정의된 킥 스타일 수 */
    };

    /**
     * @brief 토크 제어 스타일 상수 정의
     */
    class Torque
    {
    public:
      static constexpr uint8_t k_on = 0;       /**< @brief 토크 켬 */
      static constexpr uint8_t k_off = 1;      /**< @brief 토크 끔 */
      static constexpr uint8_t k_max_size = 2; /**< @brief 정의된 토크 스타일 수 */
    };

    /**
     * @brief 모드 설정 스타일 상수 정의
     */
    class SetMode
    {
    public:
      static constexpr uint8_t k_whole_active = 0; /**< @brief 전체 활성화 */
      static constexpr uint8_t k_part_active = 1;  /**< @brief 부분 활성화 */
      static constexpr uint8_t k_max_size = 2;     /**< @brief 정의된 모드 스타일 수 */
    };

    /**
     * @brief 튜닝 스타일 상수 정의
     */
    class Tuning
    {
    public:
      static constexpr uint8_t k_save_offset = 1;      /**< @brief 오프셋 저장 */
      static constexpr uint8_t k_set_position = 2;     /**< @brief 위치 설정 */
      static constexpr uint8_t k_zero_offset_pose = 3; /**< @brief 제로 오프셋 자세 */
      static constexpr uint8_t k_pre_offset_pose = 4;  /**< @brief 이전 오프셋 자세 */
      static constexpr uint8_t k_offset_on = 5;        /**< @brief 오프셋 켬 */
      static constexpr uint8_t k_offset_off = 6;       /**< @brief 오프셋 끔 */
      static constexpr uint8_t k_refresh = 7;          /**< @brief 갱신 */
      static constexpr uint8_t k_max_size = 8;         /**< @brief 정의된 튜닝 스타일 수 */
    };

    /**
     * @brief 팔 제어 스타일 상수 정의
     */
    class Arm
    {
    public:
      static constexpr uint8_t k_ik = 0;       /**< @brief 역기구학 모드 */
      static constexpr uint8_t k_fk = 1;       /**< @brief 순기구학 모드 */
      static constexpr uint8_t k_swing = 2;    /**< @brief 스윙 모드 */
      static constexpr uint8_t k_stop = 3;     /**< @brief 정지 */
      static constexpr uint8_t k_max_size = 4; /**< @brief 정의된 팔 제어 수 */
    };

    /**
     * @brief 손 제어 스타일 상수 정의
     */
    class Hand
    {
    public:
      static constexpr uint8_t k_position = 0; /**< @brief 위치 제어 */
      static constexpr uint8_t k_velocity = 1; /**< @brief 속도 제어 */
      static constexpr uint8_t k_force = 2;    /**< @brief 힘 제어 */
      static constexpr uint8_t k_motion = 3;   /**< @brief 모션 제어 */
      static constexpr uint8_t k_max_size = 4; /**< @brief 정의된 손 제어 수 */
    };
  };

  /**
   * @brief 명령 값(옵션 등) 상수를 정의하는 클래스 모음
   */
  class Value
  {
  public:
    /** @brief 값 없음 */
    static constexpr double k_none = 0.0;

    /**
     * @brief 팔 IK 값 상수 정의
     */
    class ArmIk
    {
    public:
      static constexpr double k_elbow_auto = 0.0;       /**< @brief 자동 팔꿈치 조절 */
      static constexpr double k_elbow_manual = 1.0;     /**< @brief 수동 팔꿈치 조절 */
      static constexpr double k_left_arm = 2.0;         /**< @brief 왼팔 사용 */
      static constexpr double k_right_arm = 3.0;        /**< @brief 오른팔 사용 */
      static constexpr double k_both_arms = 4.0;        /**< @brief 양팔 사용 */
      static constexpr double k_left_arm_object = 5.0;  /**< @brief 왼팔 + 객체 조작 */
      static constexpr double k_right_arm_object = 6.0; /**< @brief 오른팔 + 객체 조작 */
      static constexpr double k_both_arms_object = 7.0; /**< @brief 양팔 + 객체 조작 */
      static constexpr double k_open_the_door = 8.0;    /**< @brief 문 열기 */
      static constexpr double k_max_size = 9.0;         /**< @brief 정의된 IK 값 수 */
    };

    /**
     * @brief 보행 속도 값 상수 정의
     */
    class SpeedWalking
    {
    public:
      static constexpr double k_start = 0.0;    /**< @brief 보행 시작 */
      static constexpr double k_stop = 1.0;     /**< @brief 보행 정지 */
      static constexpr double k_max_size = 2.0; /**< @brief 정의된 속도 값 수 */
    };

    /**
     * @brief 체조 동작 값 상수 정의
     */
    class Gymnastics
    {
    public:
      static constexpr double k_warm_up = 0.0;       /**< @brief 워밍업 */
      static constexpr double k_pedal_cycling = 1.0; /**< @brief 사이클 페달링 */
      static constexpr double k_strength_hold = 2.0; /**< @brief 근력 유지 */
      static constexpr double k_all_actions = 3.0;   /**< @brief 모든 동작 */
      static constexpr double k_stop = 4.0;          /**< @brief 동작 정지 */
      static constexpr double k_max_size = 5.0;      /**< @brief 정의된 체조 값 수 */
    };

    /**
     * @brief 전체 모션 베이스 값 상수 정의
     */
    class WholeBodyMotion
    {
    public:
      static constexpr double k_base = 0.0;     /**< @brief 기본 모션 */
      static constexpr double k_max_size = 1.0; /**< @brief 정의된 베이스 값 수 */
    };

    /**
     * @brief 상체 모션 값 상수 정의
     */
    class TorsoMotion
    {
    public:
      static constexpr double k_base = 0.0;             /**< @brief 기본 상체 모션 */
      static constexpr double k_keeper_base = 1.0;      /**< @brief 골키퍼 상체 모션 */
      static constexpr double k_search = 2.0;           /**< @brief 탐색 모션 */
      static constexpr double k_waistless_search = 3.0; /**< @brief 허리 고정 탐색 */
      static constexpr double k_kickzone_test = 4.0;    /**< @brief 킥존 테스트 */
      static constexpr double k_kidnap_base = 5.0;      /**< @brief 납치 모션 */
      static constexpr double k_max_size = 6.0;         /**< @brief 정의된 상체 모션 수 */
    };

    /**
     * @brief 팔 모션 값 상수 정의
     */
    class ArmsMotion
    {
    public:
      static constexpr double k_base = 0.0;     /**< @brief 기본 팔 모션 */
      static constexpr double k_max_size = 1.0; /**< @brief 정의된 팔 모션 수 */
    };

    /**
     * @brief 다리 모션 값 상수 정의
     */
    class LegsMotion
    {
    public:
      static constexpr double k_base = 0.0;     /**< @brief 기본 다리 모션 */
      static constexpr double k_max_size = 1.0; /**< @brief 정의된 다리 모션 수 */
    };

    /**
     * @brief 팔+상체 모션 값 상수 정의
     */
    class ArmsTorsoMotion
    {
    public:
      static constexpr double k_base = 0.0;     /**< @brief 기본 팔+상체 모션 */
      static constexpr double k_max_size = 1.0; /**< @brief 정의된 팔+상체 수 */
    };

    /**
     * @brief 객체 종류 값 상수 정의
     */
    class Object
    {
    public:
      static constexpr double k_none = 0.0;             /**< @brief 객체 없음 */
      static constexpr double k_ball = 1.0;             /**< @brief 공 */
      static constexpr double k_red_candy_box = 2.0;    /**< @brief 빨간 사탕 상자 */
      static constexpr double k_yellow_candy_box = 3.0; /**< @brief 노란 사탕 상자 */
      static constexpr double k_blue_candy_box = 4.0;   /**< @brief 파란 사탕 상자 */
      static constexpr double k_cup = 5.0;              /**< @brief 컵 */
      static constexpr double k_tray = 6.0;             /**< @brief 쟁반 */
      static constexpr double k_person = 7.0;           /**< @brief 사람 */
      static constexpr double k_handle = 8.0;           /**< @brief 손잡이 */
      static constexpr double k_max_size = 9.0;         /**< @brief 정의된 객체 수 */
    };
  };

  /**
   * @brief 명령 메시지를 문자열로 변환하는 유틸리티 클래스
   */
  class CmdConverter
  {
  public:
    /**
     * @brief 실수 벡터를 문자열로 변환
     * @param values 실수 값 벡터
     * @return 콤마로 구분된 문자열
     */
    static std::string FloatArrToString(const std::vector<double> &values);

    /**
     * @brief Command 메시지에서 command 값을 문자열로 변환
     * @param msg 변환할 Command 메시지
     * @return command 문자열
     */
    static std::string CommandToString(const aeirobot_msgs::msg::Command &msg);

    /**
     * @brief Command 메시지에서 style 값을 문자열로 변환
     * @param msg 변환할 Command 메시지
     * @return style 문자열
     */
    static std::string StyleToString(const aeirobot_msgs::msg::Command &msg);

    /**
     * @brief Command 메시지에서 value 값을 문자열로 변환
     * @param msg 변환할 Command 메시지
     * @return value 문자열
     */
    static std::string ValueToString(const aeirobot_msgs::msg::Command &msg);

    /**
     * @brief Command 포인터 메시지에서 command 값을 문자열로 변환
     * @param msg 변환할 Command 포인터 메시지
     * @return command 문자열
     */
    static std::string CommandToString(const aeirobot_msgs::msg::Command *msg);

    /**
     * @brief Command 포인터 메시지에서 style 값을 문자열로 변환
     * @param msg 변환할 Command 포인터 메시지
     * @return style 문자열
     */
    static std::string StyleToString(const aeirobot_msgs::msg::Command *msg);

    /**
     * @brief Command 포인터 메시지에서 value 값을 문자열로 변환
     * @param msg 변환할 Command 포인터 메시지
     * @return value 문자열
     */
    static std::string ValueToString(const aeirobot_msgs::msg::Command *msg);

    /**
     * @brief SharedPtr 메시지에서 command 값을 문자열로 변환
     * @param msg 변환할 SharedPtr Command 메시지
     * @return command 문자열
     */
    static std::string CommandToString(const aeirobot_msgs::msg::Command::SharedPtr msg);

    /**
     * @brief SharedPtr 메시지에서 style 값을 문자열로 변환
     * @param msg 변환할 SharedPtr Command 메시지
     * @return style 문자열
     */
    static std::string StyleToString(const aeirobot_msgs::msg::Command::SharedPtr msg);

    /**
     * @brief SharedPtr 메시지에서 value 값을 문자열로 변환
     * @param msg 변환할 SharedPtr Command 메시지
     * @return value 문자열
     */
    static std::string ValueToString(const aeirobot_msgs::msg::Command::SharedPtr msg);

    /**
     * @brief action Goal 메시지에서 command 값을 문자열로 변환
     * @param goal 변환할 action Goal 메시지
     * @return command 문자열
     */
    static std::string CommandToString(const aeirobot_msgs::action::Command::Goal &goal);

    /**
     * @brief action Goal 메시지에서 style 값을 문자열로 변환
     * @param goal 변환할 action Goal 메시지
     * @return style 문자열
     */
    static std::string StyleToString(const aeirobot_msgs::action::Command::Goal &goal);

    /**
     * @brief action Goal 메시지에서 value 값을 문자열로 변환
     * @param goal 변환할 action Goal 메시지
     * @return value 문자열
     */
    static std::string ValueToString(const aeirobot_msgs::action::Command::Goal &goal);

    /**
     * @brief GoalHandleCommand에서 command 값을 문자열로 변환
     * @param goal_handle 변환할 GoalHandleCommand 객체
     * @return command 문자열
     */
    static std::string CommandToString(const std::shared_ptr<GoalHandleCommand> goal_handle);

    /**
     * @brief GoalHandleCommand에서 style 값을 문자열로 변환
     * @param goal_handle 변환할 GoalHandleCommand 객체
     * @return style 문자열
     */
    static std::string StyleToString(const std::shared_ptr<GoalHandleCommand> goal_handle);

    /**
     * @brief GoalHandleCommand에서 value 값을 문자열로 변환
     * @param goal_handle 변환할 GoalHandleCommand 객체
     * @return value 문자열
     */
    static std::string ValueToString(const std::shared_ptr<GoalHandleCommand> goal_handle);

    /**
     * @brief 문자열을 command 값으로 변환
     * @param command 변환할 command 문자열
     * @return 해당하는 command 상수
     */
    static uint8_t StringToCommand(const std::string& command);

    /**
     * @brief 문자열을 style 값으로 변환
     * @param style 변환할 style 문자열
     * @return 해당하는 style 상수
     */
    static uint8_t StringToStyle(const std::string& style);
  };

} // namespace aeirobot

#endif // COMMAND_PROTOCOL_HPP
