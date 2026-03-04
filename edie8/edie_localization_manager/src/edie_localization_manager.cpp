#include "edie_localization_manager/edie_localization_manager.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include <cmath>

using namespace pose_fusion;

EdieLocalizationManager::EdieLocalizationManager()
    : Node("edie_localization_manager")
{
    rclcpp::QoS ekf_pub_qos(rclcpp::KeepLast(10));
    ekf_pub_qos.reliable();
    ekf_pub_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    // reset_done(이벤트)용 QoS: Reliable + TransientLocal
    rclcpp::QoS evt_qos(rclcpp::KeepLast(1));
    evt_qos.reliable();
    evt_qos.durability(rclcpp::DurabilityPolicy::TransientLocal); // 늦게 붙은 구독자도 “마지막 이벤트” 1개는 받음
    evt_qos.lifespan(std::chrono::seconds(5));  // 너무 오래된 “리셋 완료(true)”가 새 구독자에게 뒤늦게 전달되는 걸 방지 (ex 5초)
    
    rclcpp::QoS viz_qos(rclcpp::KeepLast(1000)); // 10 -> 1000
    viz_qos.best_effort(); // 신뢰성 있는 QoS로 설정
    rclcpp::QoS qos_profile(2);
    // sub_openvins = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "/ov_msckf/odomimu", qos_profile, std::bind(&EdieLocalizationManager::OpenVinsOdomCallback, this, std::placeholders::_1));

    // Determine EKF odom topic based on sim_mode parameter
    this->declare_parameter<bool>("sim_mode", false);
    bool sim_mode = this->get_parameter("sim_mode").as_bool();
    std::string ekf_odom_topic;

    ekf_odom_topic = "/edie8/localization/ekf_odom";
    RCLCPP_INFO(this->get_logger(), "Real mode is ON. Subscribing to EKF odom at '%s'", ekf_odom_topic.c_str());

    sub_ekf_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        ekf_odom_topic, ekf_pub_qos, std::bind(&EdieLocalizationManager::EkfOdomCallback, this, std::placeholders::_1));
    
    
        // Absolute pose from ArUco marker
    sub_aruco_pose_info = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/edie8/localization/aruco_marker/robot_pose", qos_profile,
        std::bind(&EdieLocalizationManager::ArucoPoseInfoCallback, this, std::placeholders::_1));
    // ZUPT status from OpenVINS
    sub_zupt_status_ = this->create_subscription<std_msgs::msg::Bool>(
        "/ov_msckf/zupt_status", qos_profile,
        std::bind(&EdieLocalizationManager::ZuptStatusCallback, this, std::placeholders::_1));

    sub_ekf_stop_status_ = this->create_subscription<std_msgs::msg::Bool>(
        "/edie8/localization/ekf_is_stop", evt_qos,
        std::bind(&EdieLocalizationManager::EkfStopStatusCallback, this, std::placeholders::_1));

    // Scan command subscription
    sub_scan_command_ = this->create_subscription<std_msgs::msg::Bool>(
        "/edie8/localization/scan_command", qos_profile,
        std::bind(&EdieLocalizationManager::ScanCommandCallback, this, std::placeholders::_1));

    // BT idle status subscription
    sub_bt_idle_ = this->create_subscription<std_msgs::msg::Bool>(
        "/edie8/behavior/is_idle", aeirobot::qos_topic_profile,
        std::bind(&EdieLocalizationManager::BtIdleCallback, this, std::placeholders::_1));
    
    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Publish
    pub_reset_request_ = this->create_publisher<std_msgs::msg::Bool>(
        "/edie8/localization/reset_request", evt_qos);
    pub_robot_pose_state_ = this->create_publisher<std_msgs::msg::UInt8>(
        "/edie8/localization/robot_pose_state", 10);
    pub_result_odom = this->create_publisher<nav_msgs::msg::Odometry>(
        "/edie8/localization/manager_odom", aeirobot::qos_sensor_profile);

    // pub_result_pose = this->create_publisher<geometry_msgs::msg::Pose2D>(
    //     "/edie8/localization/pose", aeirobot::qos_sensor_profile);

    pub_result_pose_info = this->create_publisher<edie_msgs::msg::PoseWithInfoStamped>(
        "/edie8/localization/manager_pose", aeirobot::qos_sensor_profile);
    pub_robot_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/edie8/localization/robot_pose", aeirobot::qos_sensor_profile);
    
    //for aruco init

    pub_aruco_eval_ = this->create_publisher<edie_msgs::msg::PoseWithInfoStamped>(
        "/edie8/localization/eval_aruco", aeirobot::qos_sensor_profile);
    
    // Scan result publisher
    pub_scan_result_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/edie8/localization/scan_result", viz_qos);
        
    pub_is_stationary_ = this->create_publisher<std_msgs::msg::Bool>(
        "/edie8/localization/is_stop", evt_qos);

    Init(); // 파라미터 로드를 나중에 수행

    // 품질 평가 퍼블리셔
    CreateQualityPublishers("openvins");
    CreateQualityPublishers("ekf_odom");
    CreateQualityPublishers("raw_odom");

    // 초기 포즈
    edie_msgs::msg::PoseWithInfoStamped initial_pose;
    initial_pose.pose.x = 0.0;
    initial_pose.pose.y = 0.0;
    initial_pose.pose.theta = 0.0;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = this->get_clock()->now();

    pose_history_.push_back(initial_pose);

    // Initialize map_to_odom transform to identity
    map_to_odom_transform_.setIdentity();

    // Initialize scan state
    is_scanning_ = false;
    scan_poses_.clear();

    // TF Broadcast Timer
    tf_broadcast_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), // 20 Hz
        std::bind(&EdieLocalizationManager::BroadcastMapToOdom, this));
}
void EdieLocalizationManager::ArucoPoseInfoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received ArUco pose: (%.3f, %.3f)", msg->pose.position.x, msg->pose.position.y);
    
    // Convert PoseStamped to PoseWithInfoStamped for compatibility
    edie_msgs::msg::PoseWithInfoStamped pose_info;
    pose_info.header = msg->header;
    pose_info.pose.x = msg->pose.position.x;
    pose_info.pose.y = msg->pose.position.y;
    
    // Extract yaw from quaternion
    tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose_info.pose.theta = yaw;
    
    aruco_pose_history_.push_back(pose_info);
    if (aruco_pose_history_.size() > static_cast<size_t>(params_.eval_params.aruco_history_size))
    {
        aruco_pose_history_.pop_front();
    }
    has_new_aruco_pose_.store(true, std::memory_order_release);
    
    RCLCPP_DEBUG(this->get_logger(), "ArUco pose added to history, size: %zu", aruco_pose_history_.size());
}

void EdieLocalizationManager::BtIdleCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    is_bt_idle_.store(msg->data, std::memory_order_release);
}

void EdieLocalizationManager::EkfStopStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    is_ekf_stopped_.store(msg->data, std::memory_order_release);
}

void EdieLocalizationManager::ScanCommandCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && !is_scanning_) {
        // Start scanning
        RCLCPP_INFO(this->get_logger(), "Starting scan mode - collecting ArUco poses for area estimation");
        is_scanning_ = true;
        scan_poses_.clear();
    } else if (!msg->data && is_scanning_) {
        // Stop scanning and process collected data
        RCLCPP_INFO(this->get_logger(), "Stopping scan mode - processing %zu collected poses", scan_poses_.size());
        ProcessScanData();
        is_scanning_ = false;
    }
}

void EdieLocalizationManager::BroadcastMapToOdom()
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    t.transform = tf2::toMsg(map_to_odom_transform_);

    tf_broadcaster_->sendTransform(t);
}

void EdieLocalizationManager::ZuptStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) // ZUPT is true (stationary)
    {
        consecutive_zupt_true_count_++;
    }
    else // ZUPT is false (moving)
    {
        consecutive_zupt_true_count_ = 0;
        is_zupt_stopped_.store(false, std::memory_order_release);
    }

    if (consecutive_zupt_true_count_ >= params_.eval_params.aruco_zupt_true_threshold)
    {
        is_zupt_stopped_.store(true, std::memory_order_release);
    }
}

EdieLocalizationManager::~EdieLocalizationManager() {}

double EdieLocalizationManager::GetLoopRate() const
{
    return params_.loop_rate;
}

void EdieLocalizationManager::Init()
{
    RCLCPP_INFO(this->get_logger(), "Initializing Localization Manager...");

    // ===== 파라미터 선언 및 로드 =====
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.read_only = true; // 기본적으로 읽기 전용으로 설정

    // ===== 파라미터 선언 및 로드 =====
    edie_localization_manager::parameter_loader::DeclareAndLoadParameters(this, params_, topic_hz_);

    // fusion_strategies 문자열을 enum으로 변환
    for (const auto& str : params_.fusion_strategies_str) {
        if (str == "OutlierRejection") fusion_strategies.push_back(pose_fusion::Strategy::kOutlierRejection);
        else if (str == "SimpleWeightedAverage") fusion_strategies.push_back(pose_fusion::Strategy::kSimpleWeightedAverage);
        else if (str == "Smoothing") fusion_strategies.push_back(pose_fusion::Strategy::kSmoothing);
    }

    // ===== 외부 YAML 파일 로드 =====
    const std::string ws_root = ROS2_WS_ROOT;
    const std::string manager_param_full_path = ws_root + params_.manager_param_path;
    const std::string evaluation_param_full_path = ws_root + params_.evaluation_param_path;
    const std::string set_position_param_full_path = ws_root + params_.set_position_param_path;
    const std::string fusion_param_full_path = ws_root + params_.fusion_param_path;
    const std::string game_param_full_path = ws_root + params_.game_param_path;

    if (!tools::ReadYamlManager(manager_param_full_path, params_, topic_hz_, this->get_logger())) {
        RCLCPP_WARN(this->get_logger(), "YAML 매니저 파라미터 파일을 읽지 못함. 기본값 사용.");
    }

    if (!tools::ReadYamlEvaluation(evaluation_param_full_path, params_.eval_params, this->get_logger())) {
        RCLCPP_WARN(this->get_logger(), "YAML 평가 파라미터 파일을 읽지 못함. 기본값 사용.");
    }

    if (!tools::ReadYamlFusion(fusion_param_full_path, params_.fusion_params, this->get_logger())) {
        RCLCPP_WARN(this->get_logger(), "YAML 퓨전 파라미터 파일을 읽지 못함. 기본값 사용.");
    }

}

void EdieLocalizationManager::EkfOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (IsOdomValid(*msg))
    {
        // Odometry 데이터에서 x, y, theta 정보 추출
        edie_msgs::msg::PoseWithInfoStamped pose_data;
        pose_data = odom_tools::CreatePoseInfoFromOdom(*msg);
        size_t max_size = topic_hz_.at("ekf_odom") * params_.pose_buffer_seconds;
        // std::cout << "max_size: " << max_size << std::endl;
        AddBuffer(pose_data_buffers_["ekf_odom"], {pose_data, this->get_clock()->now()}, max_size);
    }
}

void EdieLocalizationManager::RawOdomCallback([[maybe_unused]] const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (IsOdomValid(*msg))
    {
        // Odometry 데이터에서 x, y, theta 정보 추출
        edie_msgs::msg::PoseWithInfoStamped pose_data;
        pose_data = odom_tools::CreatePoseInfoFromOdom(*msg);
        size_t max_size = topic_hz_.at("raw_odom") * params_.pose_buffer_seconds;
        AddBuffer(pose_data_buffers_["raw_odom"], {pose_data, this->get_clock()->now()}, max_size);
    }
}

void EdieLocalizationManager::OpenVinsOdomCallback([[maybe_unused]] const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (IsOdomValid(*msg))
    {
        // Odometry 데이터에서 x, y, theta 정보 추출
        edie_msgs::msg::PoseWithInfoStamped pose_data;
        pose_data = odom_tools::CreatePoseInfoFromOdom(*msg);

        size_t max_size = topic_hz_.at("openvins") * params_.pose_buffer_seconds;
        AddBuffer(pose_data_buffers_["openvins"], {pose_data, this->get_clock()->now()}, max_size);
    }
}

// ===== 데이터 분석 및 판단 (45Hz) =====

void EdieLocalizationManager::main()
{

    // bool is_stationary = is_zupt_stopped_.load(std::memory_order_acquire) && is_ekf_stopped_.load(std::memory_order_acquire);
    bool is_stationary = is_ekf_stopped_.load(std::memory_order_acquire);

    if (!is_stationary)
    {
        if (robot_pose_state_ == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Robot is moving, resetting pose state to UNSTABLE.");
        }
        robot_pose_state_ = 0; // Set to unstable if moving
    }
        
    std_msgs::msg::Bool is_stationary_msg;
    is_stationary_msg.data = is_stationary;
    pub_is_stationary_->publish(is_stationary_msg);

    std_msgs::msg::UInt8 state_msg;
    state_msg.data = robot_pose_state_;
    pub_robot_pose_state_->publish(state_msg);
    
    // Part 0: Scan mode data collection using existing quality evaluation system
    if (is_scanning_ && has_new_aruco_pose_.load(std::memory_order_acquire))
    {
        has_new_aruco_pose_.store(false, std::memory_order_release);
        
        RCLCPP_INFO(this->get_logger(), "Scan mode: Processing new ArUco pose, history size: %zu", aruco_pose_history_.size());
        
        // Use existing ArUco reliability evaluation system
        if (!aruco_pose_history_.empty())
        {
            auto aruco_eval_result = pose_evaluation::EvaluateArucoReliability(aruco_pose_history_, params_.eval_params, this->get_logger());
            
            RCLCPP_INFO(this->get_logger(), "ArUco reliability check: is_reliable=%s, is_stable_pos=%s, is_stable_yaw=%s", 
                       aruco_eval_result.is_reliable ? "true" : "false",
                       aruco_eval_result.is_stable_pos ? "true" : "false", 
                       aruco_eval_result.is_stable_yaw ? "true" : "false");
            
            // Publish evaluation results for monitoring (even during scan mode)
            auto eval_msg = aruco_pose_history_.back();
            eval_msg.header.stamp = this->get_clock()->now();
            eval_msg.info.clear(); // Clear previous info from ArUco node
            
            auto kv = [](const std::string& key, const std::string& value) {
                diagnostic_msgs::msg::KeyValue kv;
                kv.key = key;
                kv.value = value;
                return kv;
            };

            eval_msg.info.push_back(kv("is_reliable", std::to_string(aruco_eval_result.is_reliable)));
            eval_msg.info.push_back(kv("is_stable_pos", std::to_string(aruco_eval_result.is_stable_pos)));
            eval_msg.info.push_back(kv("is_stable_yaw", std::to_string(aruco_eval_result.is_stable_yaw)));
            eval_msg.info.push_back(kv("stddev_x", std::to_string(aruco_eval_result.stddev_x)));
            eval_msg.info.push_back(kv("stddev_y", std::to_string(aruco_eval_result.stddev_y)));
            eval_msg.info.push_back(kv("stddev_yaw", std::to_string(aruco_eval_result.stddev_yaw)));
            eval_msg.info.push_back(kv("scan_mode", "true")); // Mark as scan mode
            pub_aruco_eval_->publish(eval_msg);
            
            // Only collect poses that pass the existing quality evaluation
            if (aruco_eval_result.is_reliable)
            {
                const auto& latest_aruco = aruco_pose_history_.back();
                scan_poses_.push_back(latest_aruco);
                RCLCPP_INFO(this->get_logger(), "Added reliable ArUco pose to scan: (%.3f, %.3f), total scan poses: %zu", 
                            latest_aruco.pose.x, latest_aruco.pose.y, scan_poses_.size());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "ArUco pose not reliable enough for scan collection");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "ArUco pose history is empty during scan mode");
        }
    }
    else if (is_scanning_)
    {
        RCLCPP_DEBUG(this->get_logger(), "Scan mode active but no new ArUco pose received");
    }

    // Part 1: ArUco-based Global Pose Correction and Odometry Reset Trigger
    if (has_new_aruco_pose_.load(std::memory_order_acquire))
    {
        has_new_aruco_pose_.store(false, std::memory_order_release);
        
        auto aruco_eval_result = pose_evaluation::EvaluateArucoReliability(aruco_pose_history_, params_.eval_params, this->get_logger());
        
        // Publish evaluation results for monitoring
        if (!aruco_pose_history_.empty())
        {
            auto eval_msg = aruco_pose_history_.back();
            eval_msg.header.stamp = this->get_clock()->now();
            eval_msg.info.clear(); // Clear previous info from ArUco node
            
            auto kv = [](const std::string& key, const std::string& value) {
                diagnostic_msgs::msg::KeyValue kv;
                kv.key = key;
                kv.value = value;
                return kv;
            };

            eval_msg.info.push_back(kv("is_reliable", std::to_string(aruco_eval_result.is_reliable)));
            eval_msg.info.push_back(kv("is_stable_pos", std::to_string(aruco_eval_result.is_stable_pos)));
            eval_msg.info.push_back(kv("is_stable_yaw", std::to_string(aruco_eval_result.is_stable_yaw)));
            eval_msg.info.push_back(kv("stddev_x", std::to_string(aruco_eval_result.stddev_x)));
            eval_msg.info.push_back(kv("stddev_y", std::to_string(aruco_eval_result.stddev_y)));
            eval_msg.info.push_back(kv("stddev_yaw", std::to_string(aruco_eval_result.stddev_yaw)));
            pub_aruco_eval_->publish(eval_msg);
        }


        // bool bt_is_idle = is_bt_idle_.load(std::memory_order_acquire);
        if (aruco_eval_result.is_reliable && is_stationary) // && bt_is_idle)
        {
            // [FINAL CHECK] 2차 검사: Manager가 직접 데이터 경향성을 보고 최종 판단
            // aruco_pose_history_의 표준 편차를 여기서 직접 다시 계산합니다.
            double sum_x = 0, sum_y = 0, sum_theta = 0;
            for (const auto& pose_info : aruco_pose_history_) {
                sum_x += pose_info.pose.x;
                sum_y += pose_info.pose.y;
                sum_theta += pose_info.pose.theta;
            }
            double mean_x = sum_x / aruco_pose_history_.size();
            double mean_y = sum_y / aruco_pose_history_.size();
            double mean_theta = sum_theta / aruco_pose_history_.size();

            double sq_sum_x = 0, sq_sum_y = 0, sq_sum_theta = 0;
            for (const auto& pose_info : aruco_pose_history_) {
                sq_sum_x += std::pow(pose_info.pose.x - mean_x, 2);
                sq_sum_y += std::pow(pose_info.pose.y - mean_y, 2);
                double d_theta = pose_info.pose.theta - mean_theta;
                d_theta = std::atan2(std::sin(d_theta), std::cos(d_theta));
                sq_sum_theta += std::pow(d_theta, 2);
            }
            double manager_stddev_x = std::sqrt(sq_sum_x / aruco_pose_history_.size());
            double manager_stddev_y = std::sqrt(sq_sum_y / aruco_pose_history_.size());
            double manager_stddev_yaw = std::sqrt(sq_sum_theta / aruco_pose_history_.size());

            // 새로 추가한 '최종 검사' 임계값으로 이중 확인
            if (manager_stddev_x <= params_.eval_params.final_check_pos_x_stddev_thresh &&
                manager_stddev_y <= params_.eval_params.final_check_pos_y_stddev_thresh &&
                manager_stddev_yaw <= params_.eval_params.final_check_yaw_stddev_thresh)
            {
                // Only send reset request ONCE per event using the trigger
                if (!reset_request_triggered_)
                {
                    RCLCPP_INFO(this->get_logger(), "Passed manager's final stability check. Updating TF and sending reset request.");
                    
                    // Get the reliable ArUco pose (T_map_base)
                    const auto& aruco_pose = aruco_pose_history_.back();
                    tf2::Transform map_to_base_tf;
                    tf2::Quaternion q_aruco;
                    q_aruco.setRPY(0, 0, aruco_pose.pose.theta);
                    map_to_base_tf.setOrigin(tf2::Vector3(aruco_pose.pose.x, aruco_pose.pose.y, 0.0));
                    map_to_base_tf.setRotation(q_aruco);

                    // Get the latest EKF pose (T_odom_base)
                    if (!pose_data_buffers_["ekf_odom"].empty())
                    {
                        // Publish reset request to correcting_odom_node.
                        // This will reset the ekf_odom frame to (0,0,0) at the robot's current position.
                        std_msgs::msg::Bool reset_msg;
                        reset_msg.data = true;
                        pub_reset_request_->publish(reset_msg);
                        robot_pose_state_ = 1; // Set state to stable
                        RCLCPP_WARN(get_logger(), "Pose correction done. Pose state set to STABLE.");

                        // Because the odom frame is reset to the base_link frame at this moment,
                        // the transform from map to odom is simply the transform from map to base_link (from ArUco).
                        map_to_odom_transform_ = map_to_base_tf;
                        
                        RCLCPP_INFO(this->get_logger(), "Updated map->odom TF and triggered reset request.");
                        reset_request_triggered_ = true; // Set trigger to prevent re-sending
                    }
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "ArUco passed initial check but FAILED manager's final stability check. Skipping TF update. (stddev_x: %.4f, stddev_y: %.4f, stddev_yaw: %.4f)",
                    manager_stddev_x, manager_stddev_y, manager_stddev_yaw);
            }
        }
        else
        {
            if (!aruco_eval_result.is_reliable) RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "ArUco not reliable, skipping correction.");
            if (!is_stationary) RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Robot not stationary, skipping correction.");
            // if (!bt_is_idle) RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "BT is not idle, skipping correction.");
            // If conditions are no longer met, re-arm the trigger for the next event.
            if (reset_request_triggered_) {
                RCLCPP_INFO(this->get_logger(), "Conditions for correction lost. Re-arming reset request trigger.");
            }
            reset_request_triggered_ = false;
        }
    }

    // Part 2: Fuse sensor data to get the best pose estimate in the odom frame
    if (!odom_initialized_)
    {
        predicted_pose_ = pose_history_.front();
    }
    else
    {
        tf2::Transform odom_delta_sum;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_delta_sum = odom_delta_sum_;
            odom_delta_sum_ = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0));
        }
        if (allow_prediction_)
        {
            predicted_pose_ = pose_prediction::UpdatePoseWithPrediction(odom_initialized_, pose_history_, odom_delta_sum, this->get_clock());
        }
        else
        {
            predicted_pose_ = pose_history_.front();
        }
    }

    const auto result_evaluation = pose_evaluation::Evaluate(
        odom_initialized_, predicted_pose_, pose_data_buffers_, params_.eval_params,
        source_info_, topic_hz_, this, &quality_publishers_);
    source_info_ = result_evaluation.source_info;

    // For now, we assume no external "set pose" commands are given
    auto result_set_pose = predicted_pose_; 

    auto fusion_result_pose = pose_fusion::FusePoses(
        result_set_pose, fusion_strategies, source_info_, predicted_pose_,
        pose_history_, this, params_.fusion_params);

    pose_history_.push_front(fusion_result_pose);
    if (pose_history_.size() > static_cast<size_t>(params_.pose_history_size)) {
        pose_history_.pop_back();
    }

    // Part 3: Calculate and publish the final global pose (map -> base_fused)
    
    // Get T_odom_base_fused from the fusion result
    tf2::Transform odom_to_base_fused_tf;
    tf2::Quaternion q_local;
    q_local.setRPY(0, 0, fusion_result_pose.pose.theta);
    odom_to_base_fused_tf.setOrigin(tf2::Vector3(fusion_result_pose.pose.x, fusion_result_pose.pose.y, 0.0));
    odom_to_base_fused_tf.setRotation(q_local);

    // Calculate T_map_base_final = T_map_odom * T_odom_base_fused
    tf2::Transform map_to_base_fused_tf = map_to_odom_transform_ * odom_to_base_fused_tf;

    // [HARD OVERRIDE] If the robot is very close to the charging station, use the ArUco pose directly.
    // if (!aruco_pose_history_.empty())
    // {
    //     const auto& latest_aruco_pose = aruco_pose_history_.back();
    //     const double aruco_age = (this->get_clock()->now() - latest_aruco_pose.header.stamp).seconds();
        
    //     // Conditions for override
    //     const bool is_close_enough = latest_aruco_pose.pose.x > -0.65;
    //     const bool is_y_aligned = std::abs(latest_aruco_pose.pose.y) < 0.5;
    //     const bool is_yaw_aligned = std::abs(latest_aruco_pose.pose.theta) < 0.5;
    //     const bool is_recent = aruco_age < 0.5;

    //     if (is_close_enough && is_y_aligned && is_yaw_aligned && is_recent)
    //     {
    //         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "HARD OVERRIDE: Using ArUco pose as final robot pose.");
            
    //         tf2::Quaternion q_aruco_override;
    //         q_aruco_override.setRPY(0, 0, latest_aruco_pose.pose.theta);
    //         map_to_base_fused_tf.setOrigin(tf2::Vector3(latest_aruco_pose.pose.x, latest_aruco_pose.pose.y, 0.0));
    //         map_to_base_fused_tf.setRotation(q_aruco_override);
    //     }
    // }

    // Publish the new map -> base_fused TF
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_fused";
    t.transform = tf2::toMsg(map_to_base_fused_tf);
    tf_broadcaster_->sendTransform(t);
    
    // Also, publish the final global pose on topics for convenience
    edie_msgs::msg::PoseWithInfoStamped final_pose_msg;
    final_pose_msg.header.stamp = this->get_clock()->now();
    final_pose_msg.header.frame_id = "map";
    final_pose_msg.pose.x = map_to_base_fused_tf.getOrigin().x();
    final_pose_msg.pose.y = map_to_base_fused_tf.getOrigin().y();
    double roll, pitch, yaw;
    tf2::Matrix3x3(map_to_base_fused_tf.getRotation()).getRPY(roll, pitch, yaw);
    final_pose_msg.pose.theta = yaw;
    
    // pub_result_pose->publish(final_pose_msg.pose);
    pub_result_pose_info->publish(final_pose_msg);

    // also publish 3D Pose for compatibility
    geometry_msgs::msg::PoseStamped robot_pose_msg;
    robot_pose_msg.header.stamp = this->get_clock()->now();
    robot_pose_msg.header.frame_id = "base_fused";
    robot_pose_msg.pose.position.x = final_pose_msg.pose.x;
    robot_pose_msg.pose.position.y = final_pose_msg.pose.y;
    robot_pose_msg.pose.position.z = 0.0;
    robot_pose_msg.pose.orientation = tf2::toMsg(map_to_base_fused_tf.getRotation());
    pub_robot_pose->publish(robot_pose_msg);
}

void EdieLocalizationManager::ProcessScanData()
{
    if (scan_poses_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No scan poses collected during scan period");
        return;
    }

    // Calculate mean position only (no yaw)
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& pose : scan_poses_)
    {
        sum_x += pose.pose.x;
        sum_y += pose.pose.y;
    }
    double mean_x = sum_x / scan_poses_.size();
    double mean_y = sum_y / scan_poses_.size();

    // Calculate standard deviation for radius estimation
    double sum_sq_x = 0.0, sum_sq_y = 0.0;
    for (const auto& pose : scan_poses_)
    {
        sum_sq_x += std::pow(pose.pose.x - mean_x, 2);
        sum_sq_y += std::pow(pose.pose.y - mean_y, 2);
    }
    double std_x = std::sqrt(sum_sq_x / scan_poses_.size());
    double std_y = std::sqrt(sum_sq_y / scan_poses_.size());
    
    // Use 2-sigma as radius (covers ~95% of data points)
    double radius = 2.0 * std::max(std_x, std_y);

    RCLCPP_INFO(this->get_logger(), "Scan analysis complete: %zu poses, center=(%.3f, %.3f), radius=%.3f", 
                scan_poses_.size(), mean_x, mean_y, radius);


    // Publish scan result with position only (no yaw)
    geometry_msgs::msg::PoseStamped scan_result;
    scan_result.header.stamp = this->get_clock()->now();
    scan_result.header.frame_id = "map";
    scan_result.pose.position.x = mean_x;
    scan_result.pose.position.y = mean_y;
    scan_result.pose.position.z = 0.0;
    scan_result.pose.orientation.w = 1.0; // Identity quaternion (no rotation)
    scan_result.pose.orientation.x = 0.0;
    scan_result.pose.orientation.y = 0.0;
    scan_result.pose.orientation.z = 0.0;
    pub_scan_result_->publish(scan_result);
}

void EdieLocalizationManager::CreateQualityPublishers(const std::string& source_name)
{
    // Check if a publisher for this source already exists
    if (quality_publishers_.count(source_name))
    {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Creating quality publisher for source: %s", source_name.c_str());

    std::string topic_name = "/aeirobot/localization/eval_" + source_name;
    quality_publishers_[source_name] = this->create_publisher<edie_msgs::msg::PoseEvaluation>(topic_name, 10);
}


bool EdieLocalizationManager::IsPoseValid(const edie_msgs::msg::PoseWithInfoStamped& pose)
{
    // 1. NaN 체크
    if (std::isnan(pose.pose.x) || std::isnan(pose.pose.y) || std::isnan(pose.pose.theta))
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("edie_localization_manager"),
            "Invalid pose: (" << pose.pose.x << ", " << pose.pose.y << ", " << pose.pose.theta << ")");
        return false;
    }

    // 2. 축구장 필드 범위 체크 (필요시)
    // if (pose.pose.x < -15.0 || pose.pose.x > 15.0 || pose.pose.y < -10.0 || pose.pose.y > 10.0)
    // {
    //     return false;
    // }

    // 3. 각도 범위 체크??? 는 필요 없을라나 100으로 시작하는 뭐 트리거가 있는것 같은데 확인 필요
    // if (pose.pose.theta < -M_PI || pose.pose.theta > M_PI)
    // {
    //     return false;
    // }
    return true;
}

bool EdieLocalizationManager::IsOdomValid(const nav_msgs::msg::Odometry& odom)
{
    if (std::isnan(odom.pose.pose.position.x) || std::isnan(odom.pose.pose.position.y) || std::isnan(odom.pose.pose.position.z))
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("edie_localization_manager"),
            "Invalid odom: (" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ", " << odom.pose.pose.position.z << ")");
        return false;
    }
    return true; // 정상적인 odom 데이터이면 true 반환
}
// ===== 버퍼 관리 ===== 나중에 생각

template<typename T>
void EdieLocalizationManager::AddBuffer(std::deque<T>& buffer, const T& data, size_t max_size)
{
    if (buffer.size() >= max_size)
    {
        buffer.pop_front();
    }
    buffer.push_back(data);
}

template<typename T>
void EdieLocalizationManager::CleanData(std::deque<T>& buffer) //
{
  while (!buffer.empty())
  {
    // if () 나중에 어떻게 관리할지
    // {
    //     buffer.pop_front();
    // }

  }
}
