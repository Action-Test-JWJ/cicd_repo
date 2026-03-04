#include "edie_filtering_odometry/edie_filtering_odom_node.hpp"

EdieFilteringOdomNode::EdieFilteringOdomNode()
    : Node("edie_filtering_odom_node"),
      is_first_odom_(true),
      is_rotating_only_(false),
      slip_hold_(false),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
{   
    last_rot_end_time_ = this->get_clock()->now();

    DeclareParams();
    InitializeParams();

    // 각 Particle에 담길 모델링된 x,y,yaw 
    if(nr_samples_ > 0){
        samples_.poses = std::vector<geometry_msgs::msg::Pose>(nr_samples_, geometry_msgs::msg::Pose());
    }
    else{
        RCLCPP_FATAL_STREAM(get_logger(), "Invalid Number of Samples requested: "
            << nr_samples_ << "! Exit..");
        return;
    }

    rclcpp::QoS sub_qos(rclcpp::KeepLast(1000)); // 10 -> 1000
    sub_qos.best_effort(); // 신뢰성 있는 QoS로 설정

    rclcpp::QoS pub_qos(rclcpp::KeepLast(50)); // 10 -> 50
    pub_qos.reliable(); // 신뢰성 있는 QoS로 설정
    pub_qos.transient_local();

    // Subscriber
    sub_odom = create_subscription<nav_msgs::msg::Odometry>("/edie8/diff_drive_controller/odom", sub_qos,
        std::bind(&EdieFilteringOdomNode::OdometryCallback, this, std::placeholders::_1));

    // Publisher
    pub_pose_array = create_publisher<geometry_msgs::msg::PoseArray>("/edie8/localization/odom_samples", pub_qos);
    pub_lpf_odom = create_publisher<nav_msgs::msg::Odometry>("/edie8/localization/lpf_odom", pub_qos);
    pub_modeled_odom = create_publisher<nav_msgs::msg::Odometry>("/edie8/localization/modeled_odom", pub_qos);

    // 생성자에서 alias 생성: 앞으로 angle_diff_로 각도 차이를 계산합니다.
    angle_diff_ = [this](double a, double b) -> double {
        return odom_utils_.GetAngleDiff(a, b);
    };
}

EdieFilteringOdomNode::~EdieFilteringOdomNode()
{
}

// 파라미터 선언
void EdieFilteringOdomNode::DeclareParams()
{
    // 필요한 파라미터 선언 (기본값 설정)
    this->declare_parameter<bool>("use_modeled_odom", true);
    this->declare_parameter<int64_t>("nr_samples", 300);
    this->declare_parameter<double>("lpf_x_cutoff", 0.01);
    this->declare_parameter<double>("lpf_y_cutoff", 0.01);
    this->declare_parameter<double>("lpf_yaw_cutoff", 0.01);
    this->declare_parameter<double>("weight_pos", 1.0);
    this->declare_parameter<double>("weight_yaw", 1.0);
    this->declare_parameter<double>("translation_threshold", 0.1);
    this->declare_parameter<double>("rotation_threshold", 0.1);
    this->declare_parameter<double>("high_odom_yaw_noise", 20.0);
    this->declare_parameter<double>("low_odom_yaw_noise", 0.1);
    this->declare_parameter<double>("high_imu_yaw_noise", 0.02);
    this->declare_parameter<double>("low_imu_yaw_noise", 0.005);
}

// 파라미터 초기화
void EdieFilteringOdomNode::InitializeParams()
{
    // config.yaml 파일에서 파라미터를 직접 불러오기
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("edie_filtering_odometry");
    std::string config_file = pkg_share_dir + "/config/filtering_odometry.yaml";
    LoadConfigFromFile(config_file);

    lpf_position_x.initialize(0.02, lpf_x_cutoff_); // 50Hz
    lpf_position_y.initialize(0.02, lpf_y_cutoff_); 
    lpf_yaw.initialize(0.02, lpf_yaw_cutoff_); 
}

// YAML 파일 내 파라미터 값들 로드 
void EdieFilteringOdomNode::LoadConfigFromFile(const std::string& filename)
{
    try {
        // YAML 파일 로드
        YAML::Node base = YAML::LoadFile(filename);
        YAML::Node config;

        // edie_odometry_motion_model_node 내의 ros__parameters가 있으면 그 내부를 사용
        if (base["edie_filtering_odom_node"] && base["edie_filtering_odom_node"]["ros__parameters"])
            config = base["edie_filtering_odom_node"]["ros__parameters"];
        else
            config = base;
        
        if (config["use_modeled_odom"]) {
            use_modeled_odom_ = config["use_modeled_odom"].as<bool>();
            this->set_parameter(rclcpp::Parameter("use_modeled_odom", use_modeled_odom_));
        }
        if (config["nr_samples"]) {
            nr_samples_ = config["nr_samples"].as<int64_t>();
            this->set_parameter(rclcpp::Parameter("nr_samples", nr_samples_));
        }
        if (config["lpf_x_cutoff"]) {
            lpf_x_cutoff_ = config["lpf_x_cutoff"].as<double>();
            this->set_parameter(rclcpp::Parameter("lpf_x_cutoff", lpf_x_cutoff_));
        }
        if (config["lpf_y_cutoff"]) {
            lpf_y_cutoff_ = config["lpf_y_cutoff"].as<double>();
            this->set_parameter(rclcpp::Parameter("lpf_y_cutoff", lpf_y_cutoff_));
        }
        if (config["lpf_yaw_cutoff"]) {
            lpf_yaw_cutoff_ = config["lpf_yaw_cutoff"].as<double>();
            this->set_parameter(rclcpp::Parameter("lpf_yaw_cutoff", lpf_yaw_cutoff_));
        }
        if (config["weight_pos"]) {
            weight_pos_ = config["weight_pos"].as<double>();
            this->set_parameter(rclcpp::Parameter("weight_pos", weight_pos_));
        }
        if (config["weight_yaw"]) {
            weight_yaw_ = config["weight_yaw"].as<double>();
            this->set_parameter(rclcpp::Parameter("weight_yaw", weight_yaw_));
        }
        if (config["translation_threshold"]) {
            translation_threshold_ = config["translation_threshold"].as<double>();
            this->set_parameter(rclcpp::Parameter("translation_threshold", translation_threshold_));
        }
        if (config["rotation_threshold"]) {
            rotation_threshold_ = config["rotation_threshold"].as<double>();
            this->set_parameter(rclcpp::Parameter("rotation_threshold", rotation_threshold_));
        }
        if (config["high_odom_yaw_noise"]) {
            high_odom_yaw_noise_ = config["high_odom_yaw_noise"].as<double>();
            this->set_parameter(rclcpp::Parameter("high_odom_yaw_noise", high_odom_yaw_noise_));
        }
        if (config["low_odom_yaw_noise"]) {
            low_odom_yaw_noise_ = config["low_odom_yaw_noise"].as<double>();
            this->set_parameter(rclcpp::Parameter("low_odom_yaw_noise", low_odom_yaw_noise_));
        }
        if (config["high_imu_yaw_noise"]) {
            high_imu_yaw_noise_ = config["high_imu_yaw_noise"].as<double>();
            this->set_parameter(rclcpp::Parameter("high_imu_yaw_noise", high_imu_yaw_noise_));
        }
        if (config["low_imu_yaw_noise"]) {
            low_imu_yaw_noise_ = config["low_imu_yaw_noise"].as<double>();
            this->set_parameter(rclcpp::Parameter("low_imu_yaw_noise", low_imu_yaw_noise_));
        }

        RCLCPP_INFO(this->get_logger(), "Config file loaded successfully from %s", filename.c_str());

        RCLCPP_INFO_STREAM(this->get_logger(),
                            "\n[DEBUG] Loaded Parameters:" << "\n"
                            << " use_modeled_odom: " << use_modeled_odom_ << "\n"
                            << " nr_samples: " << nr_samples_ << "\n"
                            << " lpf_x_cutoff: " << lpf_x_cutoff_ << "\n"
                            << " lpf_y_cutoff: " << lpf_y_cutoff_ << "\n"
                            << " lpf_yaw_cutoff: " << lpf_yaw_cutoff_ << "\n"
                            << " weight_pos: " << weight_pos_ << "\n"
                            << " weight_yaw: " << weight_yaw_ << "\n"
                            << " translation_threshold: " << translation_threshold_ << "\n"
                            << " rotation_threshold: " << rotation_threshold_ << "\n"
                            << " high_odom_yaw_noise: " << high_odom_yaw_noise_ << "\n"
                            << " low_odom_yaw_noise: " << low_odom_yaw_noise_ << "\n"
                            << " high_imu_yaw_noise: " << high_imu_yaw_noise_ << "\n"
                            << " low_imu_yaw_noise: " << low_imu_yaw_noise_ << "\n"
        );
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading config file: %s", e.what());
    }
}

// 입력: Particle Filter 통한 가우시안 모델링된 오도메트리
// 출력: LPF 적용된 오도메트리
void EdieFilteringOdomNode::LPFOdometry(double position_x, double position_y, double yaw)
{
    lpf_odom_x = lpf_position_x.GetFilteredOutput(position_x);
    lpf_odom_y = lpf_position_y.GetFilteredOutput(position_y);
    lpf_odom_yaw = lpf_yaw.GetFilteredOutput(yaw);
}

bool EdieFilteringOdomNode::HandleFirstOdom(const nav_msgs::msg::Odometry &odometry)
{
    // 초기 조건 설정
    if(is_first_odom_)
    {
        samples_.header.frame_id = odometry.header.frame_id;
        samples_.header.stamp = odometry.header.stamp;

        // modeled_odom_ 메시지 설정
        modeled_odom_.header.frame_id = odometry.header.frame_id;
        modeled_odom_.header.stamp = odometry.header.stamp;
        modeled_odom_.child_frame_id = "modeled";

        // lpf_odom_ 메시지 설정
        lpf_odom_.header.frame_id = odometry.header.frame_id;
        lpf_odom_.header.stamp = odometry.header.stamp;
        lpf_odom_.child_frame_id = "lpf";

        // 마지막 측정된 위치 및 자세 업데이트
        last_odom_x_ = odometry.pose.pose.position.x;
        last_odom_y_ = odometry.pose.pose.position.y;
        last_odom_yaw_ = yaw_meas_;

        is_first_odom_ = false;
        return true;
    }
    return false;
}

// 입력: 오도메트리 메시지
// 출력: TF 브로드캐스트
void EdieFilteringOdomNode::BroadcastTF(
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

void EdieFilteringOdomNode::OdometryCallback(const nav_msgs::msg::Odometry &odom)
{
    odom_msg_sub_ = odom;

    // 측정된 처음 위치 RPY 초기화
    tf2::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_meas_ = yaw;

    // 초기 조건 설정
    if(HandleFirstOdom(odom)) return;

    // 1) translation, rotation delta 계산
    double dx = odom.pose.pose.position.x - last_odom_x_;
    double dy = odom.pose.pose.position.y - last_odom_y_;
    double delta_transl = std::hypot(dx, dy);
    double delta_rot1 = std::fabs(angle_diff_(last_odom_yaw_, yaw_meas_));
    
    bool was_rotating = is_rotating_only_;
    // 2) is_rotating_only_ flag 설정
    if (delta_transl < translation_threshold_ && delta_rot1 > rotation_threshold_) {
        is_rotating_only_ = true;
        // std::cout << "is_rotating_only_ = true" << std::endl;
        // std::cout << "delta_transl = " << delta_transl << std::endl;
        // std::cout << "delta_rot1 = " << delta_rot1 << std::endl;
        // std::cout << "translation_threshold_ = " << translation_threshold_ << std::endl;
        // std::cout << "rotation_threshold_ = " << rotation_threshold_ << std::endl;
        // std::cout << "1 \n"<< std::endl;
    } 
    else {
        // is_rotating_only_ = false;
        // // std::cout << "is_rotating_only_ = false" << std::endl;
        // // std::cout << "delta_transl = " << delta_transl << std::endl;
        // // std::cout << "delta_rot1 = " << delta_rot1 << std::endl;
        // // std::cout << "translation_threshold_ = " << translation_threshold_ << std::endl;
        // // std::cout << "rotation_threshold_ = " << rotation_threshold_ << std::endl;
        // // std::cout << "22 \n"<< std::endl;

        // “회전에서 주행 모드로 전환된 순간” 기록
        if (was_rotating && !is_rotating_only_) {
            last_rot_end_time_ = this->get_clock()->now();
        }
        is_rotating_only_ = false;
    }

    // 지금이 홀드 기간 안인지 계산
    auto now     = this->get_clock()->now();
    auto elapsed = now - last_rot_end_time_; 
    slip_hold_   = (elapsed < rot_hold_duration_);

    // ---------------------------------------- 오도메트리 필터링 로직 ----------------------------------------
    double rep_roll, rep_pitch, rep_yaw;
    if (use_modeled_odom_)
    {
        gaussian_filter_.CopyParams(last_odom_x_, last_odom_y_, last_odom_yaw_, yaw_meas_);

        // from unnormalized & noisy pose -> to normalized & noise-free pose
        gaussian_filter_.UpdateSamples(odom, samples_);
        // samples_들 중에서 최소 에러를 가지는 인덱스 찾기
        size_t rep_idx = odom_utils_.FindMinErrorIdx(samples_, weight_pos_, weight_yaw_);

        // 선택된 대표 Particle의 값을 대표값으로 사용
        rep_sample_ = samples_.poses[rep_idx];
        tf2::Quaternion rep_q(rep_sample_.orientation.x, rep_sample_.orientation.y,
                            rep_sample_.orientation.z, rep_sample_.orientation.w);
        tf2::Matrix3x3(rep_q).getRPY(rep_roll, rep_pitch, rep_yaw);     

        lpf_in_x_ = rep_sample_.position.x;
        lpf_in_y_ = rep_sample_.position.y;
        lpf_in_yaw_ = rep_yaw;
    }
    else
    {
        lpf_in_x_ = odom.pose.pose.position.x;
        lpf_in_y_ = odom.pose.pose.position.y;
        lpf_in_yaw_ = yaw_meas_;
    }
    
    // LPF 적용
    LPFOdometry(lpf_in_x_, lpf_in_y_, lpf_in_yaw_);

    if (use_modeled_odom_)
    {
        // 대표 Particle를 기반으로 modeled_odom_ 메시지 설정
        geometry_msgs::msg::Pose modeled_pose_ = odom_utils_.GetPose(rep_sample_.position.x, rep_sample_.position.y, odom.pose.pose.position.z, odom_utils_.NormalizeAngleToPi(rep_yaw));
        odom_utils_.SetPubOdomInfo(odom, modeled_odom_, modeled_pose_, "modeled");
        // odom → modeled TF 브로드캐스트
        geometry_msgs::msg::TransformStamped t_modeled;
        BroadcastTF(t_modeled, modeled_odom_, "modeled");
    }

    // LPF 적용한 lpf_odom_ 메시지 설정
    geometry_msgs::msg::Pose lpf_pose_ = odom_utils_.GetPose(lpf_odom_x, lpf_odom_y, odom.pose.pose.position.z, odom_utils_.NormalizeAngleToPi(lpf_odom_yaw));
    odom_utils_.SetPubOdomInfo(odom, lpf_odom_, lpf_pose_, "lpf");
    // odom → lpf TF 브로드캐스트   
    geometry_msgs::msg::TransformStamped t_lpf;
    BroadcastTF(t_lpf, lpf_odom_, "lpf");

    // Publish samples
    PubFilteredModeledOdom();
    // ---------------------------------------------------------------------------------------------------
                                    
    // 마지막 측정된 위치 업데이트
    last_odom_x_ = odom.pose.pose.position.x;
    last_odom_y_ = odom.pose.pose.position.y;
    last_odom_yaw_ = yaw_meas_;
}

// Particle Filter 통한 가우시안 모델링 + LPF 적용한 오도메트리 퍼블리셔
void EdieFilteringOdomNode::PubFilteredModeledOdom()
{
    if (use_modeled_odom_)
    {
        // samples_ 메시지 설정
        samples_.header.frame_id = odom_msg_sub_.header.frame_id;
        samples_.header.stamp = odom_msg_sub_.header.stamp;

        // modeled_odom_ 메시지 설정
        modeled_odom_.header.frame_id = odom_msg_sub_.header.frame_id;
        modeled_odom_.header.stamp = odom_msg_sub_.header.stamp;
        modeled_odom_.child_frame_id = "modeled";

        pub_pose_array->publish(samples_);
        pub_modeled_odom->publish(modeled_odom_);
    }

    // lpf_odom_ 메시지 설정
    lpf_odom_.header.frame_id = odom_msg_sub_.header.frame_id;
    lpf_odom_.header.stamp = odom_msg_sub_.header.stamp;
    lpf_odom_.child_frame_id = "lpf";

    pub_lpf_odom->publish(lpf_odom_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdieFilteringOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}