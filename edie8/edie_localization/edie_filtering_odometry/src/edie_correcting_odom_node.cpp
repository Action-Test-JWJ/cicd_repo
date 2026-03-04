#include "edie_filtering_odometry/edie_correcting_odom_node.hpp"

EdieCorrectingOdomNode::EdieCorrectingOdomNode()
    : Node("edie_correcting_odom_node"),
      is_reset_requested_(false),
      is_reset_done_(false),
      last_reset_request_(false),
      cur_stop_(false),
      prev_stop_(false),
      below_active_(false),
      stop_state_initialized_(false),
      lin_stop_th_(0.0),
      ang_stop_th_(0.0),
      stop_hold_time_th_(0.0),
      C_(Identity()),
      below_since_(0, 0, RCL_ROS_TIME),
      stop_enter_time_(0, 0, RCL_ROS_TIME),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
{
    DeclareParameters();
    InitializeParameters();

    rclcpp::QoS sub_ekf_qos(rclcpp::KeepLast(10)); // 1000 -> 10
    sub_ekf_qos.reliable();
    sub_ekf_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    
    rclcpp::QoS ekf_pub_qos(rclcpp::KeepLast(10));
    ekf_pub_qos.reliable();
    ekf_pub_qos.durability(rclcpp::DurabilityPolicy::Volatile);
 
    // reset_request(명령)용 QoS: Reliable + Volatile
    rclcpp::QoS reset_qos(rclcpp::KeepLast(1));
    reset_qos.reliable();
    reset_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

    // reset_done(이벤트)용 QoS: Reliable + TransientLocal
    rclcpp::QoS evt_qos(rclcpp::KeepLast(1));
    evt_qos.reliable();
    evt_qos.durability(rclcpp::DurabilityPolicy::TransientLocal); // 늦게 붙은 구독자도 “마지막 이벤트” 1개는 받음
    evt_qos.lifespan(std::chrono::seconds(5));  // 너무 오래된 “리셋 완료(true)”가 새 구독자에게 뒤늦게 전달되는 걸 방지 (ex 5초)

    // Subscriber 
    // [EKF] 15[Hz]
    sub_ekf_odom = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", sub_ekf_qos,
        std::bind(&EdieCorrectingOdomNode::EkfOdometryCallback, this, std::placeholders::_1));

    sub_reset_bool = create_subscription<std_msgs::msg::Bool>("/edie8/localization/reset_request", reset_qos,
        std::bind(&EdieCorrectingOdomNode::ResetRequestCallback, this, std::placeholders::_1));
    
    // Publisher 
    // [EKF] 15[Hz]
    pub_ekf_odom = create_publisher<nav_msgs::msg::Odometry>("/edie8/localization/ekf_odom", ekf_pub_qos);

    pub_reset_done_bool_stamped = create_publisher<edie_msgs::msg::BoolStamped>("/edie8/localization/reset_done", evt_qos);
    pub_stop_bool = create_publisher<std_msgs::msg::Bool>("/edie8/localization/ekf_is_stop", evt_qos);
}

EdieCorrectingOdomNode::~EdieCorrectingOdomNode()
{
}

void EdieCorrectingOdomNode::DeclareParameters()
{
    this->declare_parameter<double>("lin_stop_th", 0.01);
    this->declare_parameter<double>("ang_stop_th", 0.01);
    this->declare_parameter<double>("stop_hold_time_th", 0.01);
}

void EdieCorrectingOdomNode::InitializeParameters()
{
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("edie_filtering_odometry");
    std::string config_file = pkg_share_dir + "/config/stop_params.yaml";

    // YAML 파일 로드
    try {
        YAML::Node config = YAML::LoadFile(config_file);

        lin_stop_th_ = config["lin_stop_th"].as<double>();
        ang_stop_th_ = config["ang_stop_th"].as<double>();
        stop_hold_time_th_ = config["stop_hold_time_th"].as<double>();

        this->set_parameter(rclcpp::Parameter("lin_stop_th", lin_stop_th_));
        this->set_parameter(rclcpp::Parameter("ang_stop_th", ang_stop_th_));
        this->set_parameter(rclcpp::Parameter("stop_hold_time_th", stop_hold_time_th_));

        RCLCPP_INFO(this->get_logger(), "Config file loaded successfully from %s", config_file.c_str());
        RCLCPP_INFO_STREAM(this->get_logger(),
                            "\n[DEBUG] Loaded Parameters:" << "\n"
                            << " lin_stop_th: " << lin_stop_th_ << "\n"
                            << " ang_stop_th: " << ang_stop_th_ << "\n"
                            << " stop_hold_time_th: " << stop_hold_time_th_ << "\n"
        );
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading config file: %s", e.what());
    }
}

void EdieCorrectingOdomNode::ResetRequestCallback(const std_msgs::msg::Bool &msg)
{
    std::scoped_lock lk(mtx_);
    is_reset_requested_ = msg.data;
}

void EdieCorrectingOdomNode::EkfOdometryCallback(const nav_msgs::msg::Odometry &odom)
{
    std::scoped_lock lk(mtx_);

    // 1) 상승 에지에서 오프셋 C_ 갱신 (현재 EKF 포즈의 역변환) 
    if (!last_reset_request_ && is_reset_requested_) {
        const double yaw0 = odom_utils_.yawFromQuat(odom.pose.pose.orientation);
        const SE2 T0{odom.pose.pose.position.x, odom.pose.pose.position.y, yaw0};
        C_ = inverse(T0);                // 이후부터 현재가 (0,0,0)로 보이게 됨
        is_reset_done_ = true;           // 이번 루프에서 이벤트 1회 전송
        
        // --- 재무장: 다음 true를 이벤트로 받아들이기 위해 즉시 내림 ---
        is_reset_requested_ = false;
        last_reset_request_  = false;

        cur_stop_       = false;
        prev_stop_      = false;
        below_active_   = false;
        below_since_    = rclcpp::Time(0, 0, RCL_ROS_TIME);
        stop_enter_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        // std::cout << "Reset requested" << std::endl;
        // std::cout << "----------------------------------" << std::endl;
    }

    // 2) SE2 합성 후 ekf_odom 채우기
    const double yaw = odom_utils_.yawFromQuat(odom.pose.pose.orientation);
    const SE2 T_ekf{odom.pose.pose.position.x, odom.pose.pose.position.y, yaw};
    const SE2 T_out = compose(C_, T_ekf);

    geometry_msgs::msg::Pose ekf_pose_out =
        odom_utils_.GetPose(T_out.x, T_out.y, odom.pose.pose.position.z,
                            odom_utils_.NormalizeAngleToPi(T_out.yaw));
    odom_utils_.SetPubOdomInfo(odom, ekf_odom_pub_, ekf_pose_out, "base_ekf");
    ekf_odom_pub_.pose.covariance = odom.pose.covariance;
    ekf_odom_pub_.twist.covariance = odom.twist.covariance;

    // 3) 정지 판정 (twist 기반 + 홀드타임)
    const double vx = odom.twist.twist.linear.x;       // 요청대로 linear.x만 사용
    const double wz = odom.twist.twist.angular.z;

    const double lin_speed = std::fabs(vx);
    const double ang_speed = std::fabs(wz);
    const bool   is_below_th = (lin_speed <= lin_stop_th_) && (ang_speed <= ang_stop_th_);
    const rclcpp::Time stamp = odom.header.stamp;      // 데이터 시각

    if (is_below_th) 
    {
        if (!below_active_ || (stamp < below_since_)) 
        {   
            below_active_ = true;
            below_since_  = stamp;  // 임계 이하 진입 시각
        }
        cur_stop_ = (stamp - below_since_).seconds() >= stop_hold_time_th_;
    } 
    else 
    {
        below_active_ = false;
        cur_stop_ = false;
    }

    // 4) 상태 퍼블리시: 초기 1회 + 전이 시에만
    if (!stop_state_initialized_) 
    {
        // 초기 상태 1회 퍼블리시
        std_msgs::msg::Bool stop_msg; 
        stop_msg.data = cur_stop_;
        pub_stop_bool->publish(stop_msg);
        
        prev_stop_ = cur_stop_;
        stop_state_initialized_ = true;

        if (cur_stop_) {
            // 초기부터 정지였다면 진입 시각 세팅
            stop_enter_time_ = stamp;
        }
    } 
    else if (cur_stop_ != prev_stop_) 
    {
        // 전이 이벤트: 로깅/지표 + 상태 갱신 퍼블리시
        if (cur_stop_) 
        {
            stop_enter_time_ = stamp;  // 정지로 진입
        } 
        else 
        {
            const double dur = (stamp - stop_enter_time_).seconds();
            RCLCPP_INFO(get_logger(), "Stopped for %.3f s", dur);
        }

        std_msgs::msg::Bool stop_msg; stop_msg.data = cur_stop_;
        pub_stop_bool->publish(stop_msg);
        prev_stop_ = cur_stop_;
    }

    // 5) 퍼블리시 & TF
    geometry_msgs::msg::TransformStamped t_ekf;
    BroadcastTF(t_ekf, ekf_odom_pub_, "base_ekf");
    PubEkfOdom();

    // 6) reset_done 이벤트 1회 발행 (에폭 경계 알림)
    if (is_reset_done_) {
        reset_done_bool_stamped_pub_.header.stamp = this->now(); // 이벤트 시각
        reset_done_bool_stamped_pub_.data = true;
        PubResetDoneBoolStamped();
        is_reset_done_ = false;          // 한 번만 내보냄
    }
}

// 입력: 오도메트리 메시지
// 출력: TF 브로드캐스트
void EdieCorrectingOdomNode::BroadcastTF(
    geometry_msgs::msg::TransformStamped &t, 
    const nav_msgs::msg::Odometry &target_odom, 
    std::string child_frame_id)
{
    t.header.frame_id = target_odom.header.frame_id;            // "odom"
    t.header.stamp    = target_odom.header.stamp;               // 시간 동기
    t.child_frame_id  = child_frame_id;

    // modeled_odom_.pose 에 담긴 position/orientation 을 그대로 사용
    t.transform.translation.x = target_odom.pose.pose.position.x;
    t.transform.translation.y = target_odom.pose.pose.position.y;
    t.transform.translation.z = target_odom.pose.pose.position.z;
    t.transform.rotation      = target_odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
}

// 휠 인코더 오도메트리 퍼블리셔
void EdieCorrectingOdomNode::PubEkfOdom()
{
    pub_ekf_odom->publish(ekf_odom_pub_);
}

void EdieCorrectingOdomNode::PubResetDoneBoolStamped()
{
    pub_reset_done_bool_stamped->publish(reset_done_bool_stamped_pub_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdieCorrectingOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}