#include "edie_behavior/edie.hpp"

#include <chrono>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <edie_msgs/srv/check_ready.hpp>
#include <edie_msgs/msg/face_detection_array.hpp>

// Add this for parameter handling
#include "rclcpp/parameter_client.hpp"
// #include <rclcpp/exceptions.hpp> // No longer needed

using namespace std;
using namespace chrono_literals;

// 타입명 너무 길어서 압축
using Bool              = std_msgs::msg::Bool;
using Int8              = std_msgs::msg::Int8;
using UInt8             = std_msgs::msg::UInt8;
using UInt8MultiArray   = std_msgs::msg::UInt8MultiArray;
using Int16MultiArray   = std_msgs::msg::Int16MultiArray;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using String            = std_msgs::msg::String;
using Point             = geometry_msgs::msg::Point;
using Twist             = geometry_msgs::msg::Twist;
using TwistStamped      = geometry_msgs::msg::TwistStamped;
using PoseStamped       = geometry_msgs::msg::PoseStamped;
using Image             = sensor_msgs::msg::Image;
using RegionOfInterest  = sensor_msgs::msg::RegionOfInterest;
using CameraInfo        = sensor_msgs::msg::CameraInfo;
using KeyValue          = diagnostic_msgs::msg::KeyValue;
using BatteryState      = sensor_msgs::msg::BatteryState;


namespace aeirobot
{
  // generic(int, char, bool 등) 하지 않은 static 클래스 멤버 변수는 외부에서만 초기화할 수 있다. 
  unique_ptr<Edie> Edie::instance = nullptr;

  // 생성자
  Edie::Edie()
  {
    control_mode = ControlMode::k_auto;
    manual_mode = ManualMode::k_remote;
    is_action_ready = true;
    is_motion_ready = true;
    is_pose_correction_done = false;
    is_scan_result_received = false;
    battery_percentage = 0.0;
    target_last_saw = rclcpp::Time(0, 0, RCL_ROS_TIME);
    is_bt_idle_ = true;
    
    // docking 상태 초기화
    docking_status_ = DockingStatus::k_idle;

    // Debounce variables initialization
    debounced_is_infront_ = false;
    last_raw_is_infront_ = false;
    is_infront_last_changed_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // Robust debouncing state machine variables initialization
    pending_value_ = false;
    have_pending_ = false;
    pending_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // FSR touch rising trigger variables
    is_fsr_touch_triggered_ = false;
    was_fsr_touched_previously_ = false;

    // 새로운 상태 변수 초기화
    is_random_walk_needed = false;
    is_human_visible_now = false;
    target_lost_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    vora_mode_state = "human"; // vora 모드 기본값 설정
    target_lost_count = 0;
    // time_stopped_infront_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    // is_touched_during_infront_ = false;
    // was_infront_previously_ = false;
    infront_waiting_stage_ = 0;
    emotion_played_for_stage_ = 0;
    wait_stage_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // 원격 조종 서브 모드를 귀 조종으로 초기화
    remote_sub_mode_ = RemoteSubMode::EARS;
  }
  
  void Edie::Init(std::shared_ptr<rclcpp::Node> node)
  {
    // behavior 시작 시 모터 활성화 명령을 항상 true로 설정
    is_motor_enable_command = true;

    ros_manager = make_shared<RosManager>("edie_behavior_tree");
    is_infront_last_changed_ = ros_manager->now(); // <<< Add this line
    RegisterRosTopics();

    joy_manager = make_shared<JoyManager>("edie_joy_manager");
    RegisterJoyFunctions();

    // --- Parameter Handling ---
    node->declare_parameter("sim_mode", false);
    node->declare_parameter("bt_file", "edie_bt_workshop.xml");
    node->declare_parameter("laser_stop_distance_mm", 0.0);
    node->declare_parameter("laser_release_distance_mm", 0.0);
    node->declare_parameter("laser_fresh_timeout", 0.0);
    node->declare_parameter("min_roi_width", 60);
    node->declare_parameter("robot_mode", "remote"); // Add default value

    bool sim_mode = node->get_parameter("sim_mode").as_bool();
    string bt_file = node->get_parameter("bt_file").as_string();
    laser_stop_distance_mm = node->get_parameter("laser_stop_distance_mm").as_double();
    laser_release_distance_mm = node->get_parameter("laser_release_distance_mm").as_double();
    laser_fresh_timeout = node->get_parameter("laser_fresh_timeout").as_double();
    min_roi_width = node->get_parameter("min_roi_width").as_int();
    string robot_mode = node->get_parameter("robot_mode").as_string();

    // Declare and get target following parameters
    node->declare_parameter("follow_target.linear_min", 0.2);
    node->declare_parameter("follow_target.linear_max", 0.45);
    node->declare_parameter("follow_target.angular_min", 1.3);
    node->declare_parameter("follow_target.far_enter", 0.25);
    node->declare_parameter("follow_target.far_exit", 0.30);
    node->declare_parameter("follow_target.far_max", 0.8);
    node->declare_parameter("follow_target.search_spin_duration_sec", 3.0);
    node->declare_parameter("follow_target.target_lost_count_threshold", 5);
    node->declare_parameter("follow_target.angular_align_threshold", 0.08);
    node->declare_parameter("follow_target.h_fov_per_pixel_deg", 0.140623598);
    node->declare_parameter("follow_target.v_fov_per_pixel_deg", 0.187497515);
    node->declare_parameter("follow_target.target_lost_timeout_sec", 1.0);
    node->declare_parameter("follow_target.infront_debounce_sec", 2.0);
    follow_linear_min = node->get_parameter("follow_target.linear_min").as_double();
    follow_linear_max = node->get_parameter("follow_target.linear_max").as_double();
    follow_angular_min = node->get_parameter("follow_target.angular_min").as_double();
    follow_far_enter = node->get_parameter("follow_target.far_enter").as_double();
    follow_far_exit = node->get_parameter("follow_target.far_exit").as_double();
    follow_far_max = node->get_parameter("follow_target.far_max").as_double();
    follow_search_spin_duration_sec = node->get_parameter("follow_target.search_spin_duration_sec").as_double();
    follow_target_lost_count_threshold = node->get_parameter("follow_target.target_lost_count_threshold").as_int();
    follow_angular_align_threshold = node->get_parameter("follow_target.angular_align_threshold").as_double();
    follow_h_fov_per_pixel_deg = node->get_parameter("follow_target.h_fov_per_pixel_deg").as_double();
    follow_v_fov_per_pixel_deg = node->get_parameter("follow_target.v_fov_per_pixel_deg").as_double();
    follow_target_lost_timeout_sec = node->get_parameter("follow_target.target_lost_timeout_sec").as_double();
    follow_infront_debounce_sec = node->get_parameter("follow_target.infront_debounce_sec").as_double();

    RCLCPP_INFO(ros_manager->get_logger(), "Parameters: [sim_mode: %s], [bt_file: %s], [laser_stop_distance_mm: %.1f], [laser_release_distance_mm: %.1f], [laser_fresh_timeout: %.2f], [min_roi_width: %d], [robot_mode: %s]",
                sim_mode ? "true" : "false", bt_file.c_str(), laser_stop_distance_mm, laser_release_distance_mm, laser_fresh_timeout, min_roi_width, robot_mode.c_str());
    
    // --- BT Loading ---
    BT::BehaviorTreeFactory factory;
    RegisterBTNodes(factory);

    string file_path = ament_index_cpp::get_package_share_directory("edie_behavior") + "/config/" + bt_file;
    factory.registerBehaviorTreeFromFile(file_path);
    RCLCPP_INFO(ros_manager->get_logger(), "Loading BT file: %s", file_path.c_str());

    behavior_tree = factory.createTree("Main");
    auto blackboard = behavior_tree.rootBlackboard();
    blackboard->set("initialization_done", false);
    blackboard->set("isWaitingForTouch", false);
    blackboard->set("isFindingFaceAfterTouch", false);

    // 이 logger_cout는 Edie() 생성자 함수 안에서만 존재하는 지역 변수입니다.
    BT::StdCoutLogger logger_cout(behavior_tree);
    
    // Apply initial robot_mode settings from parameters
    if (robot_mode == "follow") {
      control_mode = ControlMode::k_manual;
      manual_mode = ManualMode::k_follow;
    } else if (robot_mode == "home") {
      control_mode = ControlMode::k_manual;
      manual_mode = ManualMode::k_home;
    } else if (robot_mode == "llm") {
      control_mode = ControlMode::k_manual;
      manual_mode = ManualMode::k_llm;
    } else if (robot_mode == "remote") {
      control_mode = ControlMode::k_manual;
      manual_mode = ManualMode::k_remote;
    } else if (robot_mode == "auto") {
        control_mode = ControlMode::k_auto;
    } else if (robot_mode == "sim") {
        control_mode = ControlMode::k_sim;
    }


    // Apply sim_mode settings
    if (sim_mode) {
      is_motor_enable_state = true;  // sim mode일 때 모터 상태 강제 활성화
      RCLCPP_INFO(ros_manager->get_logger(), "Behavior node started in SIM mode");
    }
  }

  // Edie 싱글톤 호출
  Edie *Edie::GetInstance()
  {
    if (instance == nullptr)
      instance = make_unique<Edie>();
    return instance.get();
  }

  // 에디 작동 함수
  void Edie::Run()
  {
    // 행동트리 루프 월타이머 생성
    ros_manager->AddWallTimer("BehaviorTreeUpdate", 10ms, [this](){ behavior_tree.tickOnce(); });

    // 에디 루프 월타이머 생성
    ros_manager->AddWallTimer("EdieUpdate", 100ms, [this](){ Update(); });

    // std::thread joy_thread([&](){ rclcpp::spin(joy_manager); });
    // std::thread ros_thread([&](){ rclcpp::spin(ros_manager); });

    // rclcpp::spin(ros_manager);
    // joy_thread.join();
    // ros_thread.join();

    while (rclcpp::ok())
    {
      rclcpp::spin_some(joy_manager);
      rclcpp::spin_some(ros_manager);
    }

    behavior_tree.haltTree();
  }

  void Edie::ResetStateVariables()
  {
    // --- Target / Alignment / Infront ---
    is_infront_with_target = false;
    is_aligned_with_target = false;

    // --- Touch / FSR ---
    is_fsr_touch_triggered_ = false;
    was_fsr_touched_previously_ = false;
    skin_state_values.clear();
    last_skin_state_msg_stamp = 0.0;

    // --- Random walk / Lost tracking ---
    is_random_walk_needed = false;
    target_lost_start_time_ = rclcpp::Time(0,0,RCL_ROS_TIME);
    target_lost_count = 0;

    // --- Human visibility / ROI ---
    is_human_visible_now = false;
    target_last_saw = rclcpp::Time(0,0,RCL_ROS_TIME);
    target_roi = sensor_msgs::msg::RegionOfInterest(); // zero-init

    // --- Debounce (infront) ---
    debounced_is_infront_ = false;
    last_raw_is_infront_ = false;
    is_infront_last_changed_ = rclcpp::Time(0,0,RCL_ROS_TIME);
    have_pending_ = false;
    pending_value_ = false;
    pending_since_ = rclcpp::Time(0,0,RCL_ROS_TIME);

    // --- Wait state machine ---
    infront_waiting_stage_ = 0;
    emotion_played_for_stage_ = 0;
    wait_stage_start_time_ = rclcpp::Time(0,0,RCL_ROS_TIME);

    // --- Laser stop/release state ---
    is_blocked_by_laser = false;
    is_laser_checking_started = false;
    laser_duration_checking_start_time = 0.0;
    last_front_laser_msg_stamp = 0.0;

    // --- Emotion/Action readiness ---
    is_action_ready = true;
    is_motion_ready = true;
    is_new_emo_msg_received = false;
    is_mobile_for_emotion = false;
    is_surprised_runaway = false;
    surprise_count = 0;

    // --- VORA / Face / LLM ---
    // vora_mode_state = "human"; // 정책에 따라 주석 해제하여 사용
    last_face_detection_stamp = 0.0;
    is_llm_state = false;
    is_stt_listening = false;

    // --- ArUco / Scan / Docking ---
    is_aruco_visible = false;
    last_aruco_pose.reset(); // std::optional 초기화
    is_scan_result_received = false;
    last_scan_result = geometry_msgs::msg::PoseStamped(); // 기본 생성자로 초기화

    if (aruco_visibility_timer) aruco_visibility_timer->cancel();
    if (reset_request_timer)    reset_request_timer->cancel();
    if (interaction_approach_timer) interaction_approach_timer->cancel();
    if (station_out_timer) station_out_timer->cancel();

    // --- Interaction Mode ---
    is_in_interaction_mode = false;
    is_doing_interaction_approach_ = false;

    docking_status_ = DockingStatus::k_idle;

    // --- Navigation / direct control ---
    navigation_statuses.clear();
    PubNavigation(0); // IDLE
    remote_sub_mode_ = RemoteSubMode::EARS; // 기본 서브모드로
    station_out = false; // station_out 플래그 리셋

    RCLCPP_INFO(ros_manager->get_logger(), "All state variables have been reset (extended).");
  }

  // 주기적으로 업데이트해야하는 것들 여기에 정리
  void Edie::Update()
  {
    // 수동조작 모드일 때 계속 전달할 값. (귀, 다리)
    if(control_mode == ControlMode::k_manual && is_action_ready && is_motion_ready)
    {
      if (manual_mode == ManualMode::k_remote)
      {
        double accel = (double)joy_manager->GetAxesData(Key::KeyCodeF710::Axes::k_l_stick_y) * 0.3 ;
        double steer = (double)joy_manager->GetAxesData(Key::KeyCodeF710::Axes::k_l_stick_x) * 1.8;
        PubRemote(accel, steer);

        if (remote_sub_mode_ == RemoteSubMode::EARS)
        {
          // --- Analog Ear Control with Triggers ---
          double raw_lt = joy_manager->GetAxesData(Key::KeyCodeF710::Axes::k_lt); // Range: 1.0 (released) to -1.0 (fully pressed)
          double raw_rt = joy_manager->GetAxesData(Key::KeyCodeF710::Axes::k_rt); // Range: 1.0 (released) to -1.0 (fully pressed)

          // Map trigger range [1.0, -1.0] to ear position [0.0, 1.0]
          double ear_l_pos = (1.0 - raw_lt) / 2.0;
          double ear_r_pos = (1.0 - raw_rt) / 2.0;

          PubEar(ear_l_pos, ear_r_pos);
        }
        else if (remote_sub_mode_ == RemoteSubMode::LEGS)
        {
          // --- Analog Leg Control with Triggers ---
          double raw_lt = joy_manager->GetAxesData(Key::KeyCodeF710::Axes::k_lt); // Range: 1.0 to -1.0
          double raw_rt = joy_manager->GetAxesData(Key::KeyCodeF710::Axes::k_rt); // Range: 1.0 to -1.0

          // Map trigger range [1.0, -1.0] to leg position [0.0, 0.75]
          double leg_l_pos = ((1.0 - raw_lt) / 2.0) * 0.75;
          double leg_r_pos = ((1.0 - raw_rt) / 2.0) * 0.75;
          
          PubLeg(leg_l_pos, leg_r_pos);
        }
      }
    }
    if (manual_mode == ManualMode::k_follow && charging_status == 0)
    {
      UpdateFsrTouch();
      UpdateTargetDetection();
      UpdateInteractionState(); // <<< 새로운 상태 관리 함수 호출

      // 상호작용 모드가 아닐 때만 "터치 대기" 로직을 실행합니다.
      if (!is_in_interaction_mode)
      {
        UpdateWaitLogic(); 
      }
    }
    else if(control_mode == ControlMode::k_auto)
    {
      
    }

    // Update instantaneous human visibility
    if (vora_mode_state == "emotion") {
      RCLCPP_INFO(ros_manager->get_logger(), "[Stop] Reason: In 'emotion' mode.");
      PubRemote(0.0, 0.0);
    }

    PubOperationMode(control_mode);
    PubMotorActivate(is_motor_enable_command);
    PubIsBtIdle(is_bt_idle_);

  }

  void Edie::UpdateInteractionState()
  {
    // OFF 조건: 사람이 앞에 없으면 무조건 상호작용 모드를 끈다
    if (!debounced_is_infront_ && !is_mobile_for_emotion)
    {
      if (is_in_interaction_mode) {
        is_in_interaction_mode = false;
        is_doing_interaction_approach_ = false;
        is_fsr_touch_triggered_ = false;  // 트리거 플래그 리셋!
        if (interaction_approach_timer) interaction_approach_timer->cancel();
        RCLCPP_INFO(ros_manager->get_logger(), "[Interaction] Mode OFF: Target lost. Flags reset.");
      }
      return;
    }

    // ON 조건: 사람이 앞에 있고, 아직 상호작용 모드가 아닐 때만 확인
    // FSR 터치나 STT가 트리거되면 Interaction Mode로 진입
    if (!is_in_interaction_mode) 
    {
      if(is_fsr_touch_triggered_){
        is_in_interaction_mode = true;
        RCLCPP_INFO(ros_manager->get_logger(), "[Interaction] Mode ON: Triggered by touch.");
        
        // 짧게 앞으로 움직이기 시작 (0.15m/s)
        is_doing_interaction_approach_ = true;
        
        // 기존 타이머가 있으면 취소
        if (interaction_approach_timer) {
          interaction_approach_timer->cancel();
        }
        
        // 0.3초 후에 멈추는 타이머 생성
        interaction_approach_timer = ros_manager->create_wall_timer(
          std::chrono::milliseconds(1000), 
          [this]() {
            is_doing_interaction_approach_ = false;
            PubRemote(0.0, 0.0); // 명시적으로 멈춤 명령 전송
            interaction_approach_timer->cancel();
            RCLCPP_INFO(ros_manager->get_logger(), "[Interaction] Approach completed, stopping.");
          }
        );
        
        // 상호작용이 시작되었으므로, 대기 상태 리셋
        infront_waiting_stage_ = 0;
        emotion_played_for_stage_ = 0;
        
      } 
      else if (is_stt_listening) 
      {
        is_in_interaction_mode = true;
        RCLCPP_INFO(ros_manager->get_logger(), "[Interaction] Mode ON: Triggered by STT.");
        
        // 상호작용이 시작되었으므로, 대기 상태를 즉시 리셋
        infront_waiting_stage_ = 0;
        emotion_played_for_stage_ = 0;
      }
    }
    
    // Interaction Mode일 때 짧게 앞으로 가기
    if (is_in_interaction_mode) {
      // 1. 짧게 앞으로 가기 (접근 중)
      if (is_doing_interaction_approach_) {
        PubRemote(0.15, 0.0);
      }
      // 2. 회전 중이 아니면 사람을 계속 정면으로 align
      // else if (!is_mobile_for_emotion) {
      //   auto align_twist = GetTwistForStaring();
      //   PubRemote(align_twist.linear.x, align_twist.angular.z);
      // }
    }
  }

  // 이 함수를 새로 추가합니다.
  void Edie::UpdateWaitLogic()
  {
    // [NEW] 3단계 대기 상태 머신 로직
    if (debounced_is_infront_ && infront_waiting_stage_ < 4) {
      if (infront_waiting_stage_ == 0) { // 대기 시작
        infront_waiting_stage_ = 1;
        emotion_played_for_stage_ = 0; // 대기 시작 시 리셋
        wait_stage_start_time_ = ros_manager->now();
        RCLCPP_INFO(ros_manager->get_logger(), "[WaitLogic] Stage 1 started. Waiting for touch...");
      }

      // 대기 중 터치가 감지되면 모든 대기 상태 리셋
      if(is_fsr_touch_triggered_){
        infront_waiting_stage_ = 0;
        emotion_played_for_stage_ = 0;
        RCLCPP_INFO(ros_manager->get_logger(), "[WaitLogic] Touch detected. Resetting wait state.");
        is_fsr_touch_triggered_ = false; // 플래그를 소비하여 무한 루프 방지
      }

      // 현재 단계에서 설정된 시간이 지났으면 다음 단계로 전환
      if (infront_waiting_stage_ > 0 && infront_waiting_stage_ <= 3) { // 1, 2, 3 단계에 대해서만 타이머 체크
        double current_stage_duration = 10.0; // Default to 10 seconds
        auto it = wait_stage_durations_sec_.find(infront_waiting_stage_);
        if (it != wait_stage_durations_sec_.end()) {
          current_stage_duration = it->second;
        }

        if ((ros_manager->now() - wait_stage_start_time_).seconds() >= current_stage_duration) {
          infront_waiting_stage_++;
          wait_stage_start_time_ = ros_manager->now();
          if (infront_waiting_stage_ < 4) {
            RCLCPP_INFO(ros_manager->get_logger(), "[WaitLogic] No touch for %.1f seconds. Proceeding to Stage %d.", current_stage_duration, infront_waiting_stage_);
          }
        }
      }

    } else if (!debounced_is_infront_) {
      // 사람이 사라지면 대기 상태 리셋
      if (infront_waiting_stage_ != 0) {
        RCLCPP_INFO(ros_manager->get_logger(), "[WaitLogic] Target lost. Resetting wait state.");
        infront_waiting_stage_ = 0;
        emotion_played_for_stage_ = 0;
      }
    }
    
    // 최종 단계(4)에 도달하면 랜덤 워크 요청 -> BT에서 직접 처리하도록 변경
    if (infront_waiting_stage_ >= 4) {
      RCLCPP_INFO_ONCE(ros_manager->get_logger(), "[WaitLogic] Final stage reached. BT will trigger avoidance.");
    }
  }

  // FSR 센서가 눌렸다가 떼어지는 순간을 감지
  void Edie::UpdateFsrTouch()
  {
    // FSR Release Detection Logic
    if (skin_state_values.empty() || (ros_manager->now().seconds() - last_skin_state_msg_stamp) > fsr_fresh_timeout)
    {
        // 데이터가 없거나 오래되었으면 상태를 리셋합니다.
        was_fsr_touched_previously_ = false;
        return;
    }

    bool is_currently_touched = false;
    for (const auto& val : skin_state_values) {
      if (val == 2) { // StrongTouch
        is_currently_touched = true;
        break;
      }
    }

    // "이전에는 눌려 있었는데, 지금은 떨어졌을 때" 릴리즈 이벤트를 감지합니다.
    if (was_fsr_touched_previously_ && !is_currently_touched) {
      is_fsr_touch_triggered_ = true;
    }

    // 다음 주기를 위해 현재 터치 상태를 저장합니다.
    was_fsr_touched_previously_ = is_currently_touched;
  }

  void Edie::UpdateTargetDetection()
  {
    bool raw_infront_status;

    // First, determine the raw "in front" status based on vision
    if (ros_manager->now() < target_last_saw + rclcpp::Duration::from_seconds(follow_target_lost_timeout_sec))
    {
      // Target is considered visible. Check distance.
      double frame_size = sqrt(cam_width/2 * cam_height);
      double image_size = sqrt(target_roi.width * target_roi.height);
      double bbox_ratio  = image_size / frame_size;
      double how_far = Clamp(1 - bbox_ratio, 0, 1);
      
      // Hysteresis logic, updating the class member `is_infront_with_target`
      if (how_far < follow_far_enter) {
        is_infront_with_target = true;
      } else if (how_far > follow_far_exit) {
        is_infront_with_target = false;
      }
      raw_infront_status = is_infront_with_target;
    }
    else
    {
      // 타겟이 사라짐: 모든 관련 상태 플래그를 false로 설정합니다.
      is_human_visible_now = false;
      is_infront_with_target = false;
      raw_infront_status = false;
    }

    // Now, apply debouncing to the raw status
    const bool raw = raw_infront_status;
    const auto now = ros_manager->now();

    if (!have_pending_) {
      // A change is detected -> start "pending"
      if (raw != debounced_is_infront_) {
        pending_value_ = raw;
        pending_since_ = now;
        have_pending_  = true;
      }
    } else {
      // We are pending, if the value changes again -> reset pending (start new pending)
      if (raw != pending_value_) {
        pending_value_ = raw;
        pending_since_ = now;
      } else {
        // If the value is stable, check time
        if ((now - pending_since_).seconds() >= follow_infront_debounce_sec) {
          if (debounced_is_infront_ != pending_value_) {
            debounced_is_infront_ = pending_value_;
            PubIsInfrontWithTarget(debounced_is_infront_);
          }
          have_pending_ = false; // clear pending
        }
      }
    }
  }

  // ROS2 토픽 등록
  void Edie::RegisterRosTopics()
  {
    // AddPublisher, AddSubscriber, AddWallTimer 사용 예시
    // ros_manager->AddPublisher<std_msgs::msg::Bool>("", qos_topic_profile);
    // ros_manager->AddSubscriber<std_msgs::msg::Bool>("", [this](const std_msgs::msg::Bool::ShardPtr &msg) {}, qos_topic_profile);
    // ros_manager->AddWallTimer("", 100ms, [&]() {});

    ros_manager->AddPublisher<Bool>("/edie8/motor/enable/command", qos_control_profile);
    ros_manager->AddPublisher<TwistStamped>("/edie8/navigation/direct_vel", qos_control_profile);
    ros_manager->AddPublisher<Float64MultiArray>("/edie_l_ear_position_controller/commands", qos_control_profile);
    ros_manager->AddPublisher<Float64MultiArray>("/edie_r_ear_position_controller/commands", qos_control_profile);
    ros_manager->AddPublisher<Float64MultiArray>("/edie_l_leg_position_controller/commands", qos_control_profile);
    ros_manager->AddPublisher<Float64MultiArray>("/edie_r_leg_position_controller/commands", qos_control_profile);
    ros_manager->AddPublisher<UInt8>("/edie8/emotion/action_index", qos_control_profile);
    ros_manager->AddPublisher<UInt8>("/edie8/emotion/display_index", qos_control_profile);
    ros_manager->AddPublisher<UInt8>("/edie8/emotion/motion_index", qos_control_profile);
    ros_manager->AddPublisher<UInt8>("/edie8/emotion/sound_index", qos_control_profile);
    ros_manager->AddPublisher<UInt8>("/edie8/mobile/action/command", qos_control_profile);
    ros_manager->AddPublisher<UInt8>("/edie8/operation_mode", qos_control_profile); //0: auto, 1: remote, 2: follow, 3: home
    ros_manager->AddPublisher<Bool>("/edie8/localization/scan_command", qos_control_profile);
    ros_manager->AddPublisher<String>("/edie8/behavior/target_marker_id", qos_control_profile);
    ros_manager->AddPublisher<Bool>("/edie8/behavior/is_idle", qos_topic_profile);
    ros_manager->AddPublisher<String>("/edie8/behavior/start_docking", qos_control_profile);
    ros_manager->AddPublisher<String>("/edie8/vora/switch_mode_request", qos_control_profile);  // String: "human" | "emotion"
    ros_manager->AddPublisher<Bool>("/edie8/behavior/is_infront_with_target", qos_control_profile);
    ros_manager->AddPublisher<Bool>("/edie8/motion/record_command", qos_control_profile);

    ros_manager->AddSubscriber<Bool>("/edie8/llm/generating", 
    [this](const Bool::SharedPtr &msg)
    {
      is_llm_state = msg->data;
    }, qos_topic_profile);

    ros_manager->AddSubscriber<Bool>("/edie8/stt/listening", 
    [this](const Bool::SharedPtr &msg)
    {
      is_stt_listening = msg->data;
    }, qos_topic_profile);

    // Vora 통한 인식 모드 str 값 수신
    ros_manager->AddSubscriber<String>("vora/mode_state", 
    [this](const String::SharedPtr &msg)
    {
      const std::string payload = msg->data;
      std::string mode = "";
      // 간단한 문자열 검사로 파싱 (JSON 의존성 제거)
      try {
        const auto key_pos = payload.find("\"kind\"");
        if (key_pos != std::string::npos) {
          const auto window = payload.substr(key_pos, 64);
          if (window.find("emotion") != std::string::npos) mode = "emotion";
          else if (window.find("human") != std::string::npos) mode = "human";
        }
      } catch (...) {}

      if (!mode.empty()) {
        vora_mode_state = mode;
        RCLCPP_INFO(ros_manager->get_logger(), "Vora mode state received: %s", mode.c_str());
        // emotion 모드로 전환될 때 타임스탬프 초기화 (새 세션 시작)
        if (mode == "emotion") {
          last_face_detection_stamp = 0.0;
          RCLCPP_INFO(ros_manager->get_logger(), 
                      "[vora/mode_state] Entered emotion mode. Reset last_face_detection_stamp to 0.0");
          PubNavigation(0); // emotion 모드 진입 시 즉시 모든 네비게이션 동작 정지
          PubEmotion(1); //모드 변환 뚜뚜 루루루루
        }
        else if (mode == "human") {
        }

        // BT가 모든 상태 전환을 관리하므로 여기서는 mode 업데이트만 필요
      }
    }, qos_topic_profile);

    // fsr 값 수신
    ros_manager->AddSubscriber<UInt8MultiArray>("/edie8/sensor/skin_state", 
    [this](const UInt8MultiArray::SharedPtr &msg)
    { 
      skin_state_values = msg->data;
      last_skin_state_msg_stamp = ros_manager->now().seconds(); // 마지막 수신 시각 갱신
    }, qos_topic_profile);

    // laser_front 값 수신
    ros_manager->AddSubscriber<Int16MultiArray>("/edie8/sensor/front/laser", 
    [this](const Int16MultiArray::SharedPtr &msg)
    { 
      laser_front_values = msg->data;
      last_front_laser_msg_stamp = ros_manager->now().seconds(); // 마지막 수신 시각 갱신
    }, qos_topic_profile);

    // string으로 감정 전달받기
    // "Neutral", "Anger", "Happiness", "Surprise", "Disgust", "Sadness", "Fear", "Unknown" 중 하나. 
    ros_manager->AddSubscriber<String>("/edie8/vision/emotion_state", 
    [this](const String::SharedPtr &msg)
    {
      bool data_received = true;
      if(msg->data == "Neutral")
      {
        emotion_state = 0;
      }
      else if(msg->data == "Anger")
      {
        emotion_state = 1;
      }
      else if(msg->data == "Happiness")
      {
        emotion_state = 2;
      }
      else if(msg->data == "Surprise")
      {
        emotion_state = 3;
      }
      else if(msg->data == "Disgust")
      {
        emotion_state = 4;
      }
      else if(msg->data == "Sadness")
      {
        emotion_state = 5;
      }
      else if(msg->data == "Fear")
      {
        emotion_state = 6;
      }
      else if(msg->data == "Unknown")
      {
        emotion_state = 7;  // Unknown: FSR 부위별 반응
      }
      else // 예외처리
      {
        data_received = false;
        emotion_state = -1;
      }

      if(data_received)
      {
        // 새 메시지 도착 플래그 (BT 퍼블리시 에지용)
        is_new_emo_msg_received = true;
        // last_emotion_msg_stamp = ros_manager->now().seconds(); // 마지막 수신 시각 갱신
      }
    }, qos_topic_profile);
    
    // // 가장 가까운 사람의 화면상 좌표
    // // ┌───→ x+   0,0 º─────┐
    // // │              │     │
    // // ↓ y+           └─────º cam_width,cam_height
    // ros_manager->AddSubscriber<Point>("/edie8/vision/closest_human_point", 
    // [this](const Point::SharedPtr &msg)
    // {
    // }, qos_topic_profile);
    
    // 가장 가까운 사람의 화면상 좌표
    // ┌───→ x+   0,0 º─────┐
    // │              │     │
    // ↓ y+           └─────º cam_width,cam_height
    ros_manager->AddSubscriber<RegionOfInterest>("/edie8/vision/closest_human_roi", 
    [this](const RegionOfInterest::SharedPtr &msg)
    {
      if (msg->width < static_cast<unsigned int>(min_roi_width)) {
        // 너무 작아서 멀리 있는 것으로 간주하고 무시
        return; 
      }

      target_roi = *msg;
      target_last_saw = ros_manager->now();
      is_human_visible_now = true; // 사람이 보이면 즉시 플래그 ON
    }, qos_topic_profile);

    ros_manager->AddSubscriber<Bool>("/edie8/emotion/emotion_done",
      [this](const Bool::SharedPtr &msg)
      {
        is_action_ready = msg->data;
      
      }, qos_topic_profile);
    
    ros_manager->AddSubscriber<Bool>("/edie8/emotion/motion_done",
      [this](const Bool::SharedPtr &msg)
      {
        is_motion_ready = msg->data;
      }, qos_topic_profile);

    // /edie8/vision/face_detections 토픽 구독 (얼굴 감지 상태 모니터링)
    // Publisher가 RELIABLE이므로 Subscriber도 RELIABLE로 맞춰야 함
    ros_manager->AddSubscriber<edie_msgs::msg::FaceDetectionArray>("/edie8/vision/face_detections",
      [this](const edie_msgs::msg::FaceDetectionArray::SharedPtr &msg)
      {
        // 얼굴이 감지되었을 때만 타임스탬프 업데이트
        if (!msg->faces.empty()) {
          last_face_detection_stamp = ros_manager->now().seconds();
        }
      }, 10);  // qos_topic_profile 대신 숫자 10 사용 (RELIABLE, depth=10)

    // 카메라 이미지 정보
    ros_manager->AddSubscriber<CameraInfo>("/edie8/vision/camera_info", 
    [this](const CameraInfo::SharedPtr &msg)
    {
      cam_width  = (float)msg->width;
      cam_height = (float)msg->height;
      // 카메라 정보는 한 번만 받아오면 됨. 
      // auto this_subscription = ros_manager->GetSubscriber<CameraInfo>("/edie8/vision/camera_info");
      // this_subscription.reset();
    }, qos_topic_profile);

    // RosManager에 서비스 리퀘스트 기능 아직 없음..
    // ros_manager->AddClient<edie_msgs::srv::CheckReady>("/edie8/action_ready");
  
    // 네비게이션 상태 전달
    ros_manager->AddSubscriber<KeyValue>("/edie8/mobile/action/state", 
      [this](const KeyValue::SharedPtr &msg)
      {
        navigation_statuses[msg->key] = msg->value;
      }, qos_topic_profile);
      
    ros_manager->AddSubscriber<Bool>("/edie8/localization/reset_request", 
      [this](const Bool::SharedPtr &msg)
      {
        if(msg->data)
        {
          is_pose_correction_done = true;

          // 기존 타이머가 있다면 리셋
          if (reset_request_timer) {
            reset_request_timer->cancel();
          }

          // 1초 후에 is_aruco_visible을 false로 설정하는 일회성 타이머 생성
          reset_request_timer = ros_manager->create_wall_timer(0.5s, [this]() {
            is_pose_correction_done = false;
            reset_request_timer->cancel(); // 일회성으로 만들기 위해 스스로를 취소
          });
        }
      }, qos_topic_profile);

    ros_manager->AddSubscriber<Bool>("/edie8/motor/enable/state", 
      [this](const Bool::SharedPtr &msg)
      {
        is_motor_enable_state = msg->data;
      }, qos_topic_profile);

    ros_manager->AddSubscriber<PoseStamped>("/edie8/localization/aruco_marker/robot_pose", 
      [this](const PoseStamped::SharedPtr &msg)
      {
        last_aruco_pose = *msg;
        is_aruco_visible = true;

        
        // 기존 타이머가 있다면 리셋
        if (aruco_visibility_timer) {
          aruco_visibility_timer->cancel();
        }

        // 1초 후에 is_aruco_visible을 false로 설정하는 일회성 타이머 생성
        aruco_visibility_timer = ros_manager->create_wall_timer(0.5s, [this]() {
          is_aruco_visible = false;
          aruco_visibility_timer->cancel(); // 일회성으로 만들기 위해 스스로를 취소
        });

      }, qos_topic_profile);
    ros_manager->AddSubscriber<PoseStamped>("/edie8/localization/robot_pose", 
      [this](const PoseStamped::SharedPtr &msg)
      {
        last_robot_pose = *msg;
      }, qos_topic_profile);

    ros_manager->AddSubscriber<PoseStamped>("/edie8/localization/scan_result",
      [this](const PoseStamped::SharedPtr &msg)
      {
        RCLCPP_INFO(ros_manager->get_logger(), "Scan result received: position=(%.3f, %.3f)", 
                    msg->pose.position.x, msg->pose.position.y);
        is_scan_result_received = true;
        last_scan_result = *msg; // Save the scan result
        RCLCPP_INFO(ros_manager->get_logger(), "Scan result saved to last_scan_result");
      }, qos_topic_profile);

    // 배터리 충전 상태 구독 (0: Not Charging, 1: Charging, 2: Charged)
    ros_manager->AddSubscriber<UInt8>("/edie8/battery/charging_status", 
      [this](const UInt8::SharedPtr &msg)
      {
        uint8_t previous_status = charging_status;
        charging_status = msg->data;

        // charging_status가 변경되었을 때만 모드 전환
        if (previous_status != charging_status)
        {
          // 모드 전환 시 상태 변수 리셋
          ResetStateVariables();
        
          if(charging_status != 0)
          {
            // 충전 중 (1 또는 2) -> remote 모드로 전환
            control_mode = ControlMode::k_manual;
            manual_mode = ManualMode::k_remote;
            ResetStateVariables();
            RCLCPP_INFO(ros_manager->get_logger(), "charging_status changed to %d -> Switching to Remote mode (state variables reset)", charging_status);
          }
          else if (previous_status == 1 && charging_status == 0)
          {
            // 1 -> 0: 충전 중이었다가 충전 안함으로 변경 -> follow 모드로 즉시 전환
            control_mode = ControlMode::k_manual;
            manual_mode = ManualMode::k_follow;
            PubNavigation(0);  // 네비게이션 IDLE로 초기화
            RCLCPP_INFO(ros_manager->get_logger(), "charging_status changed 1->0 -> Switching to Follow mode (state variables reset)");
          }
          else if (previous_status == 2 && charging_status == 0)
          {
            // 2 -> 0: 완충에서 충전 안함으로 변경 -> 3초 타이머 시작
            RCLCPP_INFO(ros_manager->get_logger(), "charging_status changed 2->0 -> Creating 3-second timer for Follow mode transition");
            
            // 기존 타이머가 있으면 취소
            if (station_out_timer) {
              RCLCPP_INFO(ros_manager->get_logger(), "[2->0 Timer] Canceling existing timer");
              station_out_timer->cancel();
            }
            
            // 3초 후에 follow 모드로 전환하는 타이머 생성
            station_out_timer = ros_manager->create_wall_timer(
              std::chrono::milliseconds(3000), 
              [this]() {
                RCLCPP_INFO(ros_manager->get_logger(), "[2->0 Timer] Timer callback triggered after 3 seconds!");
                station_out = false;  // station_out 플래그 초기화
                ResetStateVariables();
                control_mode = ControlMode::k_manual;
                manual_mode = ManualMode::k_follow;
                PubNavigation(0);  // 네비게이션 IDLE로 초기화
                station_out_timer->cancel();
                RCLCPP_INFO(ros_manager->get_logger(), "[2->0 Timer] Switched to Follow mode, station_out reset");
              }
            );
            RCLCPP_INFO(ros_manager->get_logger(), "[2->0 Timer] Timer created successfully");
          }
        }
      }, qos_topic_profile);

    // docking 완료 상태 구독
    ros_manager->AddSubscriber<Bool>("/edie8/docking_status", 
      [this](const Bool::SharedPtr &msg)
      {
        if (msg->data) {
          docking_status_ = DockingStatus::k_success;  // 성공
        } else {
          docking_status_ = DockingStatus::k_failed;   // 실패
        }
        RCLCPP_INFO(ros_manager->get_logger(), "Docking completed: %s", 
                    msg->data ? "SUCCESS" : "FAILURE");
      }, qos_topic_profile);

    // GUI에서 operation mode 변경 요청
    ros_manager->AddSubscriber<String>("/edie8/gui/operation_mode", 
      [this](const String::SharedPtr &msg)
      {
        RCLCPP_INFO(ros_manager->get_logger(), "GUI operation mode command received: %s", msg->data.c_str());
        ResetStateVariables();
        if (msg->data == "auto") {
          control_mode = ControlMode::k_auto;
          RCLCPP_INFO_STREAM(ros_manager->get_logger(), "Operation Mode -> Auto (from GUI)");
        } else if (msg->data == "remote") {
          control_mode = ControlMode::k_manual;
          manual_mode = ManualMode::k_remote;
          RCLCPP_INFO_STREAM(ros_manager->get_logger(), "Operation Mode -> Remote (from GUI)");
        } else if (msg->data == "follow") {
          control_mode = ControlMode::k_manual;
          manual_mode = ManualMode::k_follow;
          PubNavigation(0);
          RCLCPP_INFO_STREAM(ros_manager->get_logger(), "Operation Mode -> Follow (from GUI)");
        } else if (msg->data == "home") {
          control_mode = ControlMode::k_manual;
          manual_mode = ManualMode::k_home;
          PubNavigation(0);
          RCLCPP_INFO_STREAM(ros_manager->get_logger(), "Operation Mode -> Home (from GUI)");
        } else {
          RCLCPP_WARN(ros_manager->get_logger(), "Unknown GUI operation mode command: %s", msg->data.c_str());
        }
      }, qos_topic_profile);

    // Remote joystick station out command (A=3, Y=4)
    ros_manager->AddSubscriber<UInt8>("/edie8/station/out", 
      [this](const UInt8::SharedPtr &msg)
      {
        station_out_command = msg->data;
        RCLCPP_INFO(ros_manager->get_logger(), 
                    "Remote Joystick: station_out=%d", msg->data);
        
        // 충전 중이고, Y 버튼(4)이 눌렸을 때, 그리고 타이머가 실행 중이지 않을 때만
        RCLCPP_INFO(ros_manager->get_logger(), 
                    "[DEBUG] Checking condition: charging_status=%d, msg->data=%d, station_out=%d",
                    charging_status, msg->data, station_out);
        
        if (charging_status != 0 && msg->data == 4 && !station_out)
        {
          station_out = true;
          RCLCPP_INFO(ros_manager->get_logger(), "Station out triggered (charging_status=%d)", charging_status);
          control_mode = ControlMode::k_manual;
          manual_mode = ManualMode::k_home;
          RCLCPP_INFO(ros_manager->get_logger(), "[Station Out] Mode changed to HOME");
          
          // PubNavigation(1) 한 번만 호출
          PubNavigation(1);
          RCLCPP_INFO(ros_manager->get_logger(), "[Station Out] PubNavigation(1) called");
        }
        else
        {
          RCLCPP_INFO(ros_manager->get_logger(), 
                      "[DEBUG] Condition NOT met - no action taken");
        }
      }, qos_topic_profile);
  }

  void Edie::PubMotorActivate(bool trigger)
  {
    Bool motor_enable_msg;
    motor_enable_msg.data = trigger;
    ros_manager->Publish("/edie8/motor/enable/command", motor_enable_msg);
  }

  bool Edie::TargetFaced()
  {
    // RCLCPP_INFO_STREAM(ros_manager->get_logger(), "TargetFaced: " << is_infront_with_target);
    // is_infront_with_target = true;

    return is_infront_with_target;
  }

  void Edie::FollowTarget()
  {
    auto msg = GetTwistTargetInfo();
    PubRemote(msg.linear.x, msg.angular.z);
    
    // Follow 모드 동안 5초마다 motion 4번 보내기
    static double last_motion_sent_time = 0.0;
    double now_sec = ros_manager->now().seconds();
    RCLCPP_INFO(ros_manager->get_logger(), "FollowTarget: now_sec: %f, last_motion_sent_time: %f", now_sec, last_motion_sent_time);
    if (now_sec - last_motion_sent_time >= 5.0) {
      RCLCPP_INFO(ros_manager->get_logger(), "FollowTarget: Sending motion 4");
      PubEmotion(3);
      last_motion_sent_time = now_sec;
    }
  }

  Twist Edie::GetTwistTargetInfo()
  {
    // 상태 기반 로깅을 위한 정적 변수
    enum class FollowState { IDLE, BLOCKED_BY_LASER, STOPPED_TOO_CLOSE, FOLLOWING, SEARCHING, LOST_REQUESTING_WALK };
    static FollowState last_state = FollowState::IDLE;
    FollowState current_state = last_state;

    Twist twist_msg;
    if(cam_width == 0 || cam_height == 0) return twist_msg;

    // ------------------- Laser-first stop logic -------------------
    const double now_sec = ros_manager->now().seconds();
    const bool laser_fresh = (now_sec - last_front_laser_msg_stamp) <= laser_fresh_timeout;
    
    if (laser_fresh && !laser_front_values.empty())
    {
      double sum = 0.0; int cnt = 0;
      for (int16_t d : laser_front_values) { if (d > 0) { sum += d; ++cnt; } }
      
      if (cnt > 0)
      {
        const double avg = sum / cnt;

        if (avg <= laser_stop_distance_mm) {
          if (!is_laser_checking_started){
            laser_duration_checking_start_time = now_sec;
            is_laser_checking_started = true;
          }
          if ((now_sec - laser_duration_checking_start_time) >= laser_duration_checking_max_time) {
            is_blocked_by_laser = true;
          }
        } else {
          is_laser_checking_started = false;
        }
        
        if (avg >= laser_release_distance_mm) {
          is_blocked_by_laser = false;
          is_laser_checking_started = false;
        }
        
        if (is_blocked_by_laser) {
          current_state = FollowState::BLOCKED_BY_LASER;
          if (current_state != last_state) {
            RCLCPP_INFO(ros_manager->get_logger(), "[FollowLogic] State -> BLOCKED_BY_LASER. Avg distance: %.1f mm", avg);
          }
          twist_msg.linear.x = 0.0;
          twist_msg.angular.z = 0.0;
          last_state = current_state;
          return twist_msg;
        }
      }
    }

    // ------------------- Target Following Logic -------------------
    // 파라미터를 사용하도록 수정합니다. (이전 수정사항이 반영되지 않아 다시 적용합니다)
    if(target_last_saw.nanoseconds() != 0 && target_last_saw + rclcpp::Duration::from_seconds(follow_target_lost_timeout_sec) > ros_manager->now())
    {
      // 1) Target Found
      current_state = FollowState::FOLLOWING;
      is_random_walk_needed = false;
      target_lost_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      target_lost_count = 0;

      // --- Linear velocity mapping (Simplified to prevent oscillation) ---
      double frame_size = sqrt(cam_width/2 * cam_height);
      double image_size = sqrt(target_roi.width * target_roi.height);
      double bbox_ratio  = image_size / frame_size;
      double how_far = Clamp(1 - bbox_ratio, 0, 1);
      
      // 속도 계산을 단순화하여, 경계에서 머뭇거리는 현상을 제거합니다.
      // 로봇은 최소 linear_min 속도로 접근하며, 최종 정지는 debounced_is_infront_가 담당
      double t = (Clamp(how_far, follow_far_exit, follow_far_max) - follow_far_exit) / (follow_far_max - follow_far_exit);
      twist_msg.linear.x = follow_linear_min + t * (follow_linear_max - follow_linear_min);

      // --- Angular velocity mapping ---
      double delta_x = (target_roi.x_offset + target_roi.width / 2.0) - (cam_width / 2.0);
      double h_fov_per_pixel_rad = DegToRad(follow_h_fov_per_pixel_deg);
      double yaw_error_rad = -delta_x * h_fov_per_pixel_rad;
      const double P_GAIN_ANGULAR = 1.8;
      double angular_z = yaw_error_rad * P_GAIN_ANGULAR;

      if(abs(yaw_error_rad) < follow_angular_align_threshold) {
        twist_msg.angular.z = 0;
        is_aligned_with_target = true;
      } else {
        twist_msg.angular.z = angular_z;
        is_aligned_with_target = false;
      }
    }
    else
    {
      // 2) Target Lost
      is_infront_with_target = false;
      is_aligned_with_target = false;

      if (target_lost_start_time_.seconds() == 0.0) {
        target_lost_start_time_ = ros_manager->now();
        target_lost_count++;
      }

      if (target_lost_count >= follow_target_lost_count_threshold) {
        if ((ros_manager->now() - target_lost_start_time_).seconds() < follow_search_spin_duration_sec) {
          current_state = FollowState::SEARCHING;
          // PubEmotion(4);
          twist_msg.angular.z = follow_angular_min;
          is_random_walk_needed = false;
        } else {
          current_state = FollowState::LOST_REQUESTING_WALK;
          twist_msg.angular.z = 0.0;
          is_random_walk_needed = true;
        }
      } else {
        current_state = FollowState::LOST_REQUESTING_WALK;
        twist_msg.angular.z = 0.0;
        is_random_walk_needed = true;
      }
    }

    // Final check for stopping if too close (overrides everything except laser stop)
    if (debounced_is_infront_) {
      current_state = FollowState::STOPPED_TOO_CLOSE;
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
    }

    // Log state change only when the state is actually changed.
    if (current_state != last_state) {
      std::string state_str = "UNKNOWN";
      switch(current_state) {
          case FollowState::FOLLOWING:            state_str = "FOLLOWING"; break;
          case FollowState::STOPPED_TOO_CLOSE:    state_str = "STOPPED_TOO_CLOSE"; break;
          case FollowState::SEARCHING:            state_str = "SEARCHING"; break;
          case FollowState::LOST_REQUESTING_WALK: state_str = "LOST_REQUESTING_WALK"; break;
          default: break; // BLOCKED_BY_LASER is handled above and returns early.
      }
      
      if (current_state == FollowState::SEARCHING) {
          RCLCPP_INFO(ros_manager->get_logger(), "[FollowLogic] State -> %s (Lost count: %d)", state_str.c_str(), target_lost_count);
      } else {
          RCLCPP_INFO(ros_manager->get_logger(), "[FollowLogic] State -> %s", state_str.c_str());
      }
    }
    last_state = current_state;

    // Throttled velocity log for debugging, prints every 500ms.
    RCLCPP_INFO_THROTTLE(ros_manager->get_logger(), *ros_manager->get_clock(), 500,
                       "[FollowLogic] Velocity -> linear.x: %.2f, angular.z: %.2f", 
                       twist_msg.linear.x, twist_msg.angular.z);

    return twist_msg;
  }

  Twist Edie::GetTwistToAvoidTarget()
  {
    Twist twist_msg;
    // If no human is visible, just spin in place.
    if (!is_human_visible_now || cam_width == 0 || cam_height == 0) {
      twist_msg.angular.z = follow_angular_min * 0.7;
      return twist_msg;
    }

    // --- Move backward while turning away from the detected human ---
    twist_msg.linear.x = -0.2; // Constant backward speed

    double delta_x = (target_roi.x_offset + target_roi.width / 2.0) - (cam_width / 2.0);
    double h_fov_per_pixel_rad = DegToRad(follow_h_fov_per_pixel_deg);
    
    double yaw_error_rad = -delta_x * h_fov_per_pixel_rad;
    const double P_GAIN_ANGULAR = 1.5;
    
    twist_msg.angular.z = -yaw_error_rad * P_GAIN_ANGULAR;

    RCLCPP_INFO_THROTTLE(ros_manager->get_logger(), *ros_manager->get_clock(), 500,
                       "[AvoidLogic] Escaping from human. Velocity -> linear.x: %.2f, angular.z: %.2f", 
                       twist_msg.linear.x, twist_msg.angular.z);

    return twist_msg;
  }

    //나중에 삭제
  Twist Edie::GetTwistForStaring()
  {
    Twist twist_msg;
    if (!is_human_visible_now || cam_width == 0 || cam_height == 0) {
      return twist_msg; // 0, 0 반환
    }

    // GetTwistTargetInfo에서 각속도 계산 로직만 가져옴
    double x_error = (target_roi.x_offset + target_roi.width / 2.0 - cam_width / 2.0) / (cam_width / 2.0);
    double P_gain = 2.0; // P 제어 게인 (반응성 조절)
    
    twist_msg.angular.z = x_error * P_gain;

    // 정렬 허용 오차 내에 있다면 회전 멈춤
    if (abs(x_error) < 0.05) {
      twist_msg.angular.z = 0.0;
    }
    
    return twist_msg;
  }

  // 주행 명령 전달
  void Edie::PubRemote(double accel, double steer)
  {
    TwistStamped msg;
    msg.header.stamp = ros_manager->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x       = accel; // <<< 전진,후진
    msg.twist.linear.y       = 0.0;
    msg.twist.linear.z       = 0.0;
    msg.twist.angular.x      = 0.0;
    msg.twist.angular.y      = 0.0;
    msg.twist.angular.z      = steer; // <<< 회전
    
    ros_manager->Publish("/edie8/navigation/direct_vel", msg);
  }

  // 귀 위치 명령 전달
  void Edie::PubEar(double left, double right)
  {
    Float64MultiArray msg;
    msg.data.push_back(left);
    ros_manager->Publish("/edie_l_ear_position_controller/commands", msg);
    msg.data[0] = right;
    ros_manager->Publish("/edie_r_ear_position_controller/commands", msg);
  }

  // 다리 위치 명령 전달
  void Edie::PubLeg(double left, double right)
  {
    Float64MultiArray msg;
    msg.data.push_back(left);
    ros_manager->Publish("/edie_l_leg_position_controller/commands", msg);
    msg.data[0] = right;
    ros_manager->Publish("/edie_r_leg_position_controller/commands", msg);
  }

  // 어떤 감정을 표현할지 명령 전달
  void Edie::PubEmotion(uint8_t input)
  {
    UInt8 msg;
    msg.data = input;
    ros_manager->Publish("/edie8/emotion/action_index", msg);
    
    is_action_ready = false;

    // 이번 퍼블리시로 "새 입력"을 소비
    is_new_emo_msg_received = false;

  }

  // 어떤 얼굴을 할지 명령 전달
  void Edie::PubFace(uint8_t input)
  {
    UInt8 msg;
    msg.data = input;
    ros_manager->Publish("/edie8/emotion/display_index", msg);
  }

  // 어떤 모션 재생할지 전달
  void Edie::PubMotion(uint8_t input)
  {
    UInt8 msg;
    msg.data = input;
    ros_manager->Publish("/edie8/emotion/motion_index", msg);
    is_motion_ready = false;
  }

  // 어떤 소리 낼지 명령 전달
  void Edie::PubSound(uint8_t input)
  {
    UInt8 msg;
    msg.data = input;
    ros_manager->Publish("/edie8/emotion/sound_index", msg);
  }

  void Edie::PubNavigation(uint8_t input)
  {
    RCLCPP_INFO_STREAM(ros_manager->get_logger(), "PubNavigation: " << static_cast<int>(input));
    UInt8 msg;
    msg.data = input;
    ros_manager->Publish("/edie8/mobile/action/command", msg);
  }

  void Edie::PubStartDocking(const std::string& dock_id)
  {
    RCLCPP_INFO_STREAM(ros_manager->get_logger(), "PubStartDocking: " << dock_id);
    String msg;
    msg.data = dock_id;
    ros_manager->Publish("/edie8/behavior/start_docking", msg);
  }

  void Edie::PubOperationMode(ControlMode mode)
  {
    UInt8 msg;
    if (mode == ControlMode::k_auto) 
    {
      msg.data = 0; // k_auto = 0
    } 
    else if (mode == ControlMode::k_sim) 
    {
      msg.data = 4; // k_sim = 4
    } 
    else 
    {
      // manual mode일 때는 세부 모드에 따라 구분
      switch (manual_mode) {
        case ManualMode::k_remote:
          msg.data = 1; // k_remote = 1
          break;
        case ManualMode::k_follow:
          msg.data = 2; // k_follow = 2
          break;
        case ManualMode::k_home:
          msg.data = 3; // k_home = 3
          break;
        case ManualMode::k_llm:
          msg.data = 4; // k_llm = 4
          break;
        default:
          msg.data = 1; // 기본값 k_remote
          break;
      }
    }
    // RCLCPP_INFO_STREAM(ros_manager->get_logger(), "[PubOperationMode] msg.data: " << msg.data);
    ros_manager->Publish("/edie8/operation_mode", msg);
  }

  void Edie::PubIsBtIdle(bool is_idle)
  {
    Bool msg;
    msg.data = is_idle;
    ros_manager->Publish("/edie8/behavior/is_idle", msg);
  }

  void Edie::PubScanCommand(bool command)
  {
    // RCLCPP_INFO_STREAM(ros_manager->get_logger(), "PubScanCommand: " << command);
    Bool msg;
    msg.data = command;
    ros_manager->Publish("/edie8/localization/scan_command", msg);
  }

  void Edie::PubTargetMarkerId(const std::string& marker_id)
  {
      String msg;
      msg.data = marker_id;
      ros_manager->Publish("/edie8/behavior/target_marker_id", msg);
  }

  // vora/switch_mode 서비스 클라이언트는 있지만, 토픽 구독은 아직 없음 
  // → 즉, edie8/vora/switch_mode_request 신호를 받아서 서비스 콜로 이어주는 브리지 부분 필요 
  void Edie::PubVoraModeSwitchRequest(const std::string& mode)
  {
    String msg;
    msg.data = mode;
    RCLCPP_INFO_STREAM(ros_manager->get_logger(), "pub vora mode : " << msg.data);
    ros_manager->Publish("/edie8/vora/switch_mode_request", msg);
  }

  void Edie::PubIsInfrontWithTarget(bool is_infront)
  {
    Bool msg;
    msg.data = is_infront;
    ros_manager->Publish("/edie8/behavior/is_infront_with_target", msg);
  }

  void Edie::PubMotionRecordCommand()
  {
    Bool msg;
    msg.data = true; // recorder는 true 메시지를 받을 때마다 토글합니다.
    ros_manager->Publish("/edie8/motion/record_command", msg);
  }

  // 조이스틱 버튼 입력에 대한 정의
  void Edie::RegisterJoyFunctions()
  {
      // LB, RB 옵션 트리거. 누르면 true 떼면 false
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_lb, Key::EventType::k_on_key_down, [this](){ 
        joy_option_trigger[0] = true; 
      });
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_lb, Key::EventType::k_on_key_up,   [this](){ joy_option_trigger[0] = false; });
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_rb, Key::EventType::k_on_key_down, [this](){ 
        joy_option_trigger[1] = true; 

        // remote 모드에서 귀/다리 조종 전환
        if (control_mode == ControlMode::k_manual && manual_mode == ManualMode::k_remote)
        {
          if (remote_sub_mode_ == RemoteSubMode::EARS)
          {
            remote_sub_mode_ = RemoteSubMode::LEGS;
            RCLCPP_INFO(ros_manager->get_logger(), "Switched to LEG Control Mode");
          }
          else
          {
            remote_sub_mode_ = RemoteSubMode::EARS;
            RCLCPP_INFO(ros_manager->get_logger(), "Switched to EAR Control Mode");
          }
        }
      });
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_rb, Key::EventType::k_on_key_up,   [this](){ joy_option_trigger[1] = false; });

      // 표정 전환(옵션 트리거 이용 예정)
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_arrow_up,    Key::EventType::k_on_key_down, 
      [this]()
      { /*화살표 상 키 입력시*/ 
        // 수동조작 모드가 아닐 경우, 작동하지 않음. 
        if(control_mode != ControlMode::k_manual) return;

      });
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_arrow_right, Key::EventType::k_on_key_down, 
      [this]()
      { /*화살표 우 키 입력시*/ 
        // 수동조작 모드가 아닐 경우, 작동하지 않음. 
        if(control_mode != ControlMode::k_manual) return;

      });
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_arrow_down,  Key::EventType::k_on_key_down, 
      [this]()
      { /*화살표 하 키 입력시*/ 
        // 수동조작 모드가 아닐 경우, 작동하지 않음. 
        if(control_mode != ControlMode::k_manual) return;

      });
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_arrow_left,  Key::EventType::k_on_key_down, 
      [this]()
      { /*화살표 좌 키 입력시*/ 
        // 수동조작 모드가 아닐 경우, 작동하지 않음. 
        if(control_mode != ControlMode::k_manual) return;

      });

      // 모션 실행(옵션 트리거 이용 예정)
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_a, Key::EventType::k_on_key_down, 
      [this]()
      { /*a키 입력시*/ 
        if(control_mode == ControlMode::k_manual)
        {
          ResetStateVariables();
          manual_mode = ManualMode::k_llm;
          PubNavigation(0);  // 네비게이션 IDLE로 초기화
          RCLCPP_INFO_STREAM(ros_manager->get_logger(), "Manual Mode -> LLM");
        }
      });
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_b, Key::EventType::k_on_key_down, 
      [this]()
      { /*b키 입력시*/ 
        if(control_mode == ControlMode::k_manual)
        {
          ResetStateVariables();
          manual_mode = ManualMode::k_home;
          PubNavigation(0);  // 네비게이션 IDLE로 초기화
          RCLCPP_INFO_STREAM(ros_manager->get_logger(), "Manual Mode -> Home");
        }
      });
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_x, Key::EventType::k_on_key_down, 
      [this]()
      { /*x키 입력시*/ 
        if(control_mode == ControlMode::k_manual)
        {
          ResetStateVariables();
          manual_mode = ManualMode::k_follow;
          PubNavigation(0);  // 네비게이션 IDLE로 초기화
          PubVoraModeSwitchRequest("human"); // Vora 모드를 human으로 초기화
          RCLCPP_INFO_STREAM(ros_manager->get_logger(), "Manual Mode -> Follow");
        }
      });
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_y, Key::EventType::k_on_key_down, 
      [this]()
      { /*y키 입력시*/ 
        if(control_mode == ControlMode::k_manual)
        {
          ResetStateVariables();
          manual_mode = ManualMode::k_remote;
          RCLCPP_INFO_STREAM(ros_manager->get_logger(), "Manual Mode -> Remote. VORA mode switch is not explicitly called.");
        }
      });

      // 움직임, 소리, 표정 녹화 시작, 정지
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_back, Key::EventType::k_on_key_down, 
      [this]()
      { /*back 키 입력시*/
        PubMotionRecordCommand(); // 녹화 토글 명령 발행
        // is_motor_enable_command = false; // 모터 비활성화는 별개로 둠
      });
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_start, Key::EventType::k_on_key_down, 
      [this]()
      { /*start 키 입력시*/ 
        PubMotionRecordCommand(); // 녹화 토글 명령 발행
        // is_motor_enable_command = true; // 모터 활성화는 별개로 둠
      });

      // 자동 모드와 수동 모드 전환
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_logo, Key::EventType::k_on_key_down, 
      [this]()
      { /*로지텍 로고키 입력시*/ 
        ResetStateVariables();
        if(control_mode == ControlMode::k_manual)
        {
          control_mode = ControlMode::k_auto;
          ResetStateVariables();
          RCLCPP_INFO_STREAM(ros_manager->get_logger(), "Manual mode Off");
        }
        else if(control_mode == ControlMode::k_auto)
        {
          control_mode = ControlMode::k_manual;
          manual_mode = ManualMode::k_remote;
          station_status = "undocked"; // manual 모드로 전환 시 undocked로 리셋
          ResetStateVariables();
          RCLCPP_INFO_STREAM(ros_manager->get_logger(), "Manual mode On");
        }
        else
        { 
          control_mode = ControlMode::k_manual;
          manual_mode = ManualMode::k_remote; // 수동 모드 진입 시 항상 remote로 초기화
          ResetStateVariables();
          RCLCPP_INFO_STREAM(ros_manager->get_logger(), "Manual Mode -> Remote");
        }
      });

      // 미정
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_l_stick, Key::EventType::k_on_key_down, [this](){ /*좌 스틱 클릭 입력시*/ });
      joy_manager->SetOnKeyEvent(Key::ButtonMap::k_r_stick, Key::EventType::k_on_key_down, [this](){ /*우 스틱 클릭 입력시*/ });
    }

} // namespace aeirobot