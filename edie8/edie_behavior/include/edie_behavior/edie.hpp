#ifndef EDIE_HPP
#define EDIE_HPP

#include <memory>
#include <map>
#include <atomic>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <aeirobot_toolbox/ros_manager.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <random>

#include "edie_behavior/action_nodes.hpp"
#include "edie_behavior/condition_nodes.hpp"
#include "edie_behavior/joy_manager.hpp"
#include "edie_behavior/transform.hpp"
#include "edie_behavior/object.hpp"
#include <edie_msgs/msg/face_detection_array.hpp>

using namespace std;

namespace aeirobot
{

class Edie
{
public:
  enum class ControlMode
  {
    k_manual, 
    k_auto,
    k_sim  // sim mode 추가
  };

  enum class ManualMode
  {
    k_remote,
    k_follow,
    k_home,
    k_llm
  };

  // remote 모드 내의 서브 모드 정의
  enum class RemoteSubMode
  {
    EARS,
    LEGS
  };

  enum class DockingStatus
  {
    k_idle,      // 시작전
    k_running,   // 진행중
    k_success,   // 성공
    k_failed     // 실패
  };

  const int version = 8;

  Edie(const Edie &) = delete;
  Edie &operator=(const Edie &) = delete;
  ControlMode control_mode;
  ManualMode manual_mode;
  
  static unique_ptr<Edie> instance;
  std::shared_ptr<RosManager> ros_manager;
  std::shared_ptr<JoyManager> joy_manager;
  BT::Tree behavior_tree;
  std::unique_ptr<BT::StdCoutLogger> logger_cout;

  double l_ear_joint;
  double r_ear_joint;
  double l_leg_joint;
  double r_leg_joint;

  Transform transform;
  Object target;
  sensor_msgs::msg::RegionOfInterest target_roi;
  rclcpp::Time target_last_saw;
  double last_face_detection_stamp = 0.0;


  int cur_action = -1;
  int cur_state = -1;
  int emotion_state = 0;
  float cam_width;
  float cam_height;
  vector<uint8_t> skin_state_values;
  vector<int16_t> laser_front_values;

  std::map<std::string, std::string> navigation_statuses;
  std::string station_status = "undocked";
  
  // Remote joystick station out command (A=3, Y=4)
  uint8_t station_out_command = 0;
  bool station_out = false; // station out 플래그

  bool is_new_emo_msg_received = false;
  bool is_infront_with_target = false;
  bool is_aligned_with_target = false;
  bool is_in_interaction_mode = false;
  bool is_surprised_runaway = false;  // 놀라서 도망가기 플래그
  int surprise_count = 0;             // 놀란 횟수 카운터
  bool is_action_ready = true;
  bool is_motion_ready = true;
  bool is_pose_correction_done = false;
  bool is_scan_result_received = false;
  bool is_home_init_done = false;
  bool is_bt_idle_ = true;
  
  DockingStatus docking_status_ = DockingStatus::k_idle;

  bool joy_option_trigger[2] = {false, false};

  bool is_motor_enable_command = true;
  bool is_motor_enable_state = false;
  uint8_t charging_status = 0;

  bool is_aruco_visible = false;
  double battery_percentage = 1.0;
  std::optional<geometry_msgs::msg::PoseStamped> last_aruco_pose;
  std::optional<geometry_msgs::msg::Pose> averaged_aruco_pose;
  std::optional<geometry_msgs::msg::PoseStamped> last_robot_pose;
  geometry_msgs::msg::PoseStamped last_scan_result; // 스캔 결과 저장용

  std::map<std::string, std::vector<double>> aruco_marker_position; // 마커 위치 저장용

  // Vora 통한 인식 모드 str 값 수신
  std::string vora_mode_state = "";
  // double last_emotion_mode_msg_stamp = 0.0; // 이 줄 삭제 (안 쓰임)

  // double last_emotion_msg_stamp = 0.0; // 이 줄 삭제 (안 쓰임)
  double emotion_fresh_timeout = 0.5;       // TTL: 마지막 수신 시각 보다 길어지면 이번 루프에서 판정 로직 수행 X
  double emotion_duration_checking_max_time = 10.0;

  // FSR 비교/디바운스/스테일 검사용
  double last_skin_state_msg_stamp = 0.0;
  double fsr_fresh_timeout = 0.5;           // TTL: 마지막 수신 시각 보다 길어지면 이번 루프에서 판정 로직 수행 X

  // FSR touch rising trigged flag
  bool is_fsr_touch_triggered_ = false;
  bool was_fsr_touched_previously_;

  int min_roi_width;

  // 여러 스레드(예: ROS 콜백 스레드, BT 틱 스레드)가 동시에 읽고/쓰기 해도 데이터 레이스 없이 안전한 불리언 플래그
  bool is_emotion_done = false;
  bool is_llm_state = false;
  bool is_stt_listening = false;
  bool is_mobile_for_emotion = false;        

  // Debouncing for is_infront_with_target
  bool debounced_is_infront_ = false;
  bool last_raw_is_infront_ = false;
  rclcpp::Time is_infront_last_changed_;

  // Robust debouncing state machine variables
  bool pending_value_;
  bool have_pending_;
  rclcpp::Time pending_since_;

  // Hysteresis for stopping logic
  bool is_linearly_stopped_ = false;
  bool is_angularly_stopped_ = false;
  
  // laser_front 관련 변수/파라미터
  double laser_stop_distance_mm = 0.0; // 이하면 정지
  double laser_release_distance_mm = 0.0; // 이 이상이면 정지 해제

  double last_front_laser_msg_stamp = 0.0;   // 마지막 레이저 수신 시각(sec)
  double laser_fresh_timeout = 0.0;      // TTL: 마지막 수신 시각 보다 길어지면 이번 루프에서 판정 로직 수행 X
  double laser_duration_checking_start_time = 0.0; // 레이저 체크 시작 시간(sec)
  double laser_duration_checking_max_time = 2.0; // 레이저 체크 최대 시간(sec)
  bool  is_laser_checking_started = false; // 레이저 체크 시작 플래그
  bool   is_blocked_by_laser = false;       // 히스테리시스 유지 플래그

  // Stage waiting variables
  std::map<int, double> wait_stage_durations_sec_;

  // human following
  rclcpp::Time last_human_roi_stamp_;
  bool is_random_walk_needed = false;
  bool is_human_visible_now = false;
  int target_lost_count = 0;

  // [수정] 3단계 대기 로직을 위한 상태 변수 (public으로 이동)
  int infront_waiting_stage_ = 0; // 0: 대기 안함, 1, 2, 3: 대기 단계, 4: 랜덤워크 요청
  int emotion_played_for_stage_ = 0; // 몇 단계의 감정을 이미 표현했는지 기억
  rclcpp::Time wait_stage_start_time_;

  // Target Following Parameters
  double follow_linear_min;
  double follow_linear_max;
  double follow_angular_min;
  double follow_far_enter;
  double follow_far_exit;
  double follow_far_max;
  double follow_search_spin_duration_sec;
  int    follow_target_lost_count_threshold;
  double follow_angular_align_threshold;
  double follow_h_fov_per_pixel_deg;
  double follow_v_fov_per_pixel_deg;
  double follow_target_lost_timeout_sec;
  double follow_infront_debounce_sec;

  // Interaction Mode 관련 플래그들
  bool is_doing_interaction_approach_ = false;

public: // Methods
  static Edie* GetInstance();
  Edie();
  void Run();
  void Init(std::shared_ptr<rclcpp::Node> node);

  void PubRemote(double accel, double steer);
  void PubMotorActivate(bool trigger);
  void PubEar(double left, double right);
  void PubLeg(double left, double right);
  void PubEmotion(uint8_t input);
  void PubFace(uint8_t input);
  void PubMotion(uint8_t input);
  void PubSound(uint8_t input);
  void PubNavigation(uint8_t input);
  void PubOperationMode(ControlMode mode);
  void PubScanCommand(bool command);
  void PubTargetMarkerId(const std::string& marker_id);
  void PubIsBtIdle(bool is_idle);
  void PubStartDocking(const std::string& dock_id);
  void PubVoraModeSwitchRequest(const std::string& mode);
  void PubIsInfrontWithTarget(bool is_infront);
  void PubMotionRecordCommand();

  void FollowTarget();
  bool TargetFaced();

  geometry_msgs::msg::Twist GetTwistTargetInfo();
  geometry_msgs::msg::Twist GetTwistToAvoidTarget();
  geometry_msgs::msg::Twist GetTwistForStaring();

private:
  void Update();
  void UpdateTwistCommand();
  void UpdateTargetDetection();
  void UpdateFsrTouch();
  void UpdateWaitLogic();
  void UpdateInteractionState();
  void RegisterRosTopics();
  void RegisterJoyFunctions();
  void ResetStateVariables();

  void RegisterBTNodes(BT::BehaviorTreeFactory &factory);

  rclcpp::TimerBase::SharedPtr aruco_visibility_timer;
  rclcpp::TimerBase::SharedPtr reset_request_timer;
  rclcpp::TimerBase::SharedPtr interaction_approach_timer;
  rclcpp::TimerBase::SharedPtr station_out_timer;
  rclcpp::Time target_lost_start_time_;

  // 원격 조종 서브 모드
  RemoteSubMode remote_sub_mode_;
};

} // namespace aeirobot

#endif // EDIE_HPP