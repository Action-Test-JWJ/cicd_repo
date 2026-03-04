#include "edie_imu/edie_imu_node.hpp"

EdieImuNode::EdieImuNode()
    : Node("edie_imu_node"),
    is_first_sub(true),
    is_data_ready(false),    // t -> f
    ready_to_publish(false),
    kf_accel_updated_(false),
    kf_gyro_updated_(false),
    was_stationary(true),
    g_to_m_sec_sqrd(9.80665),
    stop_worker_(false)
{ 
    CalibrateTimeOffset();

    // 필요한 파라미터 선언 (기본값 설정)
    DeclareParams();
    // LPF. KF 파라미터 초기화
    InitializeParams();

    // rclcpp::QoS qos_best_effort(rclcpp::KeepLast(1000));
    // qos_best_effort.best_effort();
    // rclcpp::QoS qos_reliable(1000);  // depth=50으로 증가
    // qos_reliable.reliable();       // 신뢰성 있는 QoS로 설정
    // qos_reliable.durability(rclcpp::DurabilityPolicy::Volatile);   
    // qos_reliable.history(rclcpp::HistoryPolicy::KeepLast);
    auto imu_qos = rclcpp::SensorDataQoS()
                        .keep_last(50) // 너무 작으면 데이터 놓치고, 너무 크면 메모리+지연
                        .best_effort() //.reliable()
                        .deadline(rclcpp::Duration::from_seconds(0.0025)); // 400 Hz 기대

    // Publisher [qos_reliable -> imu_qos]
    pub_imu_data_lpf = this->create_publisher<sensor_msgs::msg::Imu>("/edie8/sensor/lpf_imu", imu_qos);
    pub_imu_data_kf = this->create_publisher<sensor_msgs::msg::Imu>("/edie8/sensor/calibrated_imu", imu_qos);
    // Subscriber [qos_best_effort -> imu_qos]
    sub_imu_data = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, imu_qos, 
                std::bind(&EdieImuNode::ImuCallback, this, std::placeholders::_1));

    // 센서 데이터 publish 주기는 무조건 빠르게
//
    // 1. 1k[Hz]
    // timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&EdieImuNode::TimerCallback, this));
    // 2. 500[Hz]
    // timer = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&EdieImuNode::TimerCallback, this));
    // 3. 200[Hz]
    // timer = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&EdieImuNode::TimerCallback, this));
    // 4. 300[Hz](= 3.33ms) 
    // timer = this->create_wall_timer(std::chrono::microseconds(3333), std::bind(&EdieImuNode::TimerCallback, this));
//
    // 5. 400[Hz](= 2.5ms)
    timer = this->create_wall_timer(std::chrono::microseconds(2500), std::bind(&EdieImuNode::TimerCallback, this));
   
    // 워커 스레드 시작 (무거운 연산을 별도 스레드에서 처리)
    worker_thread_ = std::thread(&EdieImuNode::ProcessingLoop, this);
}

EdieImuNode::~EdieImuNode()
{
    // 워커 스레드 종료 신호 전송
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        stop_worker_ = true;
    }
    // 락 없이 바로 다른 스레드가 stop_worker_ 플래그를 확인할 수 있도록 보장
    worker_cv_.notify_all();
    
    if (worker_thread_.joinable())
    {
        worker_thread_.join();
    }
}

void EdieImuNode::CalibrateTimeOffset()
{
    auto mono_now = mono_clock::now();
    auto sys_now  = sys_clock::now();
    // 절대 시간 - 상대 시간 차이 계산 -> 초반 절대 시간 offset 용도
    steady_to_system_offset_ = std::chrono::duration_cast<ns>(sys_now.time_since_epoch())
                            - std::chrono::duration_cast<ns>(mono_now.time_since_epoch());
}

// 타이머 콜백에서는 무거운 작업을 하지 않고,
// 단순히 현재 시각(a.k.a 작업 식별자)를 큐에 넣고 워커 스레드에 알림
void EdieImuNode::TimerCallback()
{
    // auto current_time = this->now();  
    auto t_start = mono_clock::now();
    auto sys_ns = std::chrono::duration_cast<ns>(t_start.time_since_epoch()) + steady_to_system_offset_;
    auto current_time = rclcpp::Time(sys_ns.count());
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        imu_time_queue_.push(current_time);
    }

    // 워커 스레드에 알림
    worker_cv_.notify_one();

    // // 타임스탬프 로그 출력
    // rclcpp::Time now_time = this->now();
    // RCLCPP_INFO(this->get_logger(), "Timestamp this->now(): %llu ns", static_cast<unsigned long long>(now_time.nanoseconds()));
    // RCLCPP_INFO(this->get_logger(), "t_start: %llu ns", static_cast<unsigned long long>(t_start.time_since_epoch().count()));
    // RCLCPP_INFO(this->get_logger(), "steady_to_system_offset_: %llu ns", static_cast<unsigned long long>(steady_to_system_offset_.count()));
    // RCLCPP_INFO(this->get_logger(), "Timestamp sys_ns     : %llu ns", static_cast<unsigned long long>(sys_ns.count()));
    // // 타임스탬프 차이 계산 및 밀리초 변환
    // int64_t diff_ns = static_cast<int64_t>(now_time.nanoseconds()) - static_cast<int64_t>(sys_ns.count());
    // double diff_ms = static_cast<double>(diff_ns) * 1e-6;
    // RCLCPP_INFO(this->get_logger(), "Timestamp difference  : %.3f ms", diff_ms);
    // RCLCPP_INFO(this->get_logger(), "--------------------------------");
}

void EdieImuNode::DeclareParams()
{
    this->declare_parameter<double>("sample_freq", 400);
    this->declare_parameter<bool>("use_low_pass_filter", false);
    this->declare_parameter<bool>("use_kalman_filter", true);
    this->declare_parameter<double>("Q_cov", 0.01);
    this->declare_parameter<double>("R_cov", 0.01);
    this->declare_parameter<double>("lpf_accel_cutoff", 0.01);
    this->declare_parameter<double>("lpf_gyro_cutoff", 0.01);
    this->declare_parameter<double>("min_cutoff", 1.0);
    this->declare_parameter<double>("beta", 0.0);
    this->declare_parameter<double>("dcutoff", 1.0);
    this->declare_parameter<double>("zero_vel_accel_threshold", 0.1);
    this->declare_parameter<double>("zero_vel_gyro_threshold", 1.0);
    // 새로운 imu_topic 파라미터 선언 (기본값: 실제 로봇 환경)
    this->declare_parameter<std::string>("imu_topic", "/edie8/sensor/offset_imu");
}

void EdieImuNode::InitializeParams()
{
    // imu_topic 매개변수 읽기 (런치 파일에서 시뮬레이션일 경우 오버라이드한 값이 들어감)
    this->get_parameter("imu_topic", imu_topic);
    RCLCPP_INFO(this->get_logger(), "Using IMU topic: %s", imu_topic.c_str());

    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("edie_imu");
    std::string config_file = "";
    // 시뮬레이션 환경에서의 Kalman filter 파라미터 불러오기
    if (imu_topic == "/edie8/sensor/imu")
    {
        config_file = pkg_share_dir + "/config/sim_filter_params.yaml";
    }
    // 실제 로봇 환경
    else
    {
        config_file = pkg_share_dir + "/config/real_filter_params.yaml";        
    }
    LoadConfigFromFile(config_file);

    // Kalman filter 추정값 초기화
    accl_x_est_ = 0.0; accl_y_est_ = 0.0; accl_z_est_ = 0.0;
    gyro_x_est_ = 0.0; gyro_y_est_ = 0.0; gyro_z_est_ = 0.0;
    // Kalman filter 인스턴스 초기화
    kalman_filter_.Initialize(6, 6, Q_cov_, R_cov_); // 6 states: 3 accel + 3 gyro
 
    // OneEuroFilter 인스턴스 초기화
    oef_accel_x.OEFInitialize(sample_freq_, 0.4, 0.5, dcutoff_);     // 샘플링 주파수 400[Hz], mincutoff 1.0[Hz], beta 0.0, dcutoff 1.0[Hz]
    oef_accel_y.OEFInitialize(sample_freq_, min_cutoff_, 0.0, dcutoff_);
    oef_accel_z.OEFInitialize(sample_freq_, 0.0001, 0.075, dcutoff_);     // min_cutoff_ : 1.0 [Hz], beta_: 1.0 
    oef_gyro_x.OEFInitialize(sample_freq_, min_cutoff_, 0.0, dcutoff_);       // 0.1, 0.04(by far best)
    oef_gyro_y.OEFInitialize(sample_freq_, min_cutoff_, 0.0, dcutoff_);
    oef_gyro_z.OEFInitialize(sample_freq_, min_cutoff_, 0.04, dcutoff_);
}

void EdieImuNode::LoadConfigFromFile(const std::string& filename)
{
    // YAML 파일 로드
    try {
        // YAML 파일 로드
        YAML::Node base = YAML::LoadFile(filename);
        YAML::Node config;

        // edie_imu_node 내의 ros__parameters가 있으면 그 내부를 사용
        if (base["edie_imu_node"] && base["edie_imu_node"]["ros__parameters"])
            config = base["edie_imu_node"]["ros__parameters"];
        else
            config = base;
        
        if (config["sample_freq"]) {
            sample_freq_ = config["sample_freq"].as<double>();
            this->set_parameter(rclcpp::Parameter("sample_freq", sample_freq_));
        }
        if (config["use_low_pass_filter"]) {
            use_low_pass_filter_ = config["use_low_pass_filter"].as<bool>();
            this->set_parameter(rclcpp::Parameter("use_low_pass_filter", use_low_pass_filter_));
        }
        if (config["use_kalman_filter"]) {
            use_kalman_filter_ = config["use_kalman_filter"].as<bool>();
            this->set_parameter(rclcpp::Parameter("use_kalman_filter", use_kalman_filter_));
        }
        if (config["Q_cov"]) {
            Q_cov_ = config["Q_cov"].as<double>();
            this->set_parameter(rclcpp::Parameter("Q_cov", Q_cov_));
        }
        if (config["R_cov"]) {
            R_cov_ = config["R_cov"].as<double>();
            this->set_parameter(rclcpp::Parameter("R_cov", R_cov_));
        }
        if (config["lpf_accel_cutoff"]) {
            lpf_accel_cutoff_ = config["lpf_accel_cutoff"].as<double>();
            this->set_parameter(rclcpp::Parameter("lpf_accel_cutoff", lpf_accel_cutoff_));
        }
        if (config["lpf_gyro_cutoff"]) {
            lpf_gyro_cutoff_ = config["lpf_gyro_cutoff"].as<double>();
            this->set_parameter(rclcpp::Parameter("lpf_gyro_cutoff", lpf_gyro_cutoff_));
        }
        if (config["min_cutoff"]) {
            min_cutoff_ = config["min_cutoff"].as<double>();
            this->set_parameter(rclcpp::Parameter("min_cutoff", min_cutoff_));
        }
        if (config["beta"]) {
            beta_ = config["beta"].as<double>();
            this->set_parameter(rclcpp::Parameter("beta", beta_));
        }
        if (config["dcutoff"]) {
            dcutoff_ = config["dcutoff"].as<double>();
            this->set_parameter(rclcpp::Parameter("dcutoff", dcutoff_));
        }
        if (config["zero_vel_accel_threshold"]) {
            zero_vel_accel_threshold_ = config["zero_vel_accel_threshold"].as<double>();
            this->set_parameter(rclcpp::Parameter("zero_vel_accel_threshold", zero_vel_accel_threshold_));
        }
        if (config["zero_vel_gyro_threshold"]) {
            zero_vel_gyro_threshold_ = config["zero_vel_gyro_threshold"].as<double>();
            this->set_parameter(rclcpp::Parameter("zero_vel_gyro_threshold", zero_vel_gyro_threshold_));
        }
        RCLCPP_INFO(this->get_logger(), "Config file loaded successfully from %s", filename.c_str());

        RCLCPP_INFO_STREAM(this->get_logger(),
                            "\n[DEBUG] Loaded Parameters:" << "\n"
                            << " sample_freq: " << sample_freq_ << "\n"
                            << " use_low_pass_filter: " << use_low_pass_filter_ << "\n"
                            << " use_kalman_filter: " << use_kalman_filter_ << "\n"
                            << " Q_cov: " << Q_cov_ << "\n"
                            << " R_cov: " << R_cov_ << "\n"
                            << " lpf_accel_cutoff: " << lpf_accel_cutoff_ << "\n"
                            << " lpf_gyro_cutoff: " << lpf_gyro_cutoff_ << "\n"
                            << " min_cutoff: " << min_cutoff_ << "\n"
                            << " beta: " << beta_ << "\n"
                            << " dcutoff: " << dcutoff_ << "\n"
                            << " zero_vel_accel_threshold: " << zero_vel_accel_threshold_ << "\n"
                            << " zero_vel_gyro_threshold: " << zero_vel_gyro_threshold_ << "\n"
        );
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading config file: %s", e.what());
    }
}

void EdieImuNode::ProcessingLoop()
{
    // 스레드가 종료 신호를 받을 때까지 계속해서 루프 실행
    while (true)
    {
        // 타이머 콜백에서 큐에 넣은 시각(타임스탬프)으로, 이벤트가 발생한 시점
        rclcpp::Time task_time;

        // 중괄호 블록을 벗어나면서 unique_lock 객체가 소멸되어 worker_mutex_가 자동으로 해제
        // 무거운 작업(후속 연산) 동안 락을 유지하지 않아 다른 스레드(예: 타이머 콜백)가 큐에 접근하는 것을 막지 않음
        {
            std::unique_lock<std::mutex> lock(worker_mutex_);
            // 두 가지 조건 중 하나가 만족될 때까지 워커 스레드를 대기 -> notify_one() 혹은 notify_all()에 의해 알림 받음
            worker_cv_.wait(lock, [this] { return !imu_time_queue_.empty() || stop_worker_; });
            if (stop_worker_ && imu_time_queue_.empty())
            {
                break;
            }

            task_time = imu_time_queue_.front();
            imu_time_queue_.pop();
        }

        // auto start = std::chrono::steady_clock::now();

        // 무거운 연산 수행: IMU 데이터 읽기 및 publish
        // (아래 함수들은 원래 코드의 구현체를 그대로 사용)
        ReadImuData();
        PubImuData();       

        // auto end = std::chrono::steady_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // double duration_s = duration * 1e-6;  // μs → s

        // std::cout << "ProcessingLoop duration: " << duration_s << " s" << std::endl;
        // std::cout << "--------------------------------" << std::endl;
    }
}

void EdieImuNode::ImuCallback(const sensor_msgs::msg::Imu &imu)
{
    // if(!is_data_ready){
    //     return;
    // }
    is_data_ready = true;

    imu_msg_sub = imu;
    
    // 1. Offset or Raw + LPF 적용 데이터 헤더 설정
    if (is_first_sub) {
        if (use_low_pass_filter_) {
            imu_msg_lpf.header.frame_id = "imu_link";
            imu_msg_lpf.header.stamp = imu_msg_sub.header.stamp;
            imu_msg_lpf.linear_acceleration_covariance = imu_msg_sub.linear_acceleration_covariance;
            imu_msg_lpf.angular_velocity_covariance = imu_msg_sub.angular_velocity_covariance;
            // 가속도
            imu_msg_lpf.linear_acceleration_covariance[0] = 1e-6; //1e-4; //1e-5; // 1e-7 
            imu_msg_lpf.linear_acceleration_covariance[4] = 1e-9; //1e-1;
            imu_msg_lpf.linear_acceleration_covariance[8] = 1e-9;
            // 자이로
            imu_msg_lpf.angular_velocity_covariance[0] = 1e-9;
            imu_msg_lpf.angular_velocity_covariance[4] = 1e-9;
            imu_msg_lpf.angular_velocity_covariance[8] = 1e-6; //8e-2; //7.68e-3(don't know) //5e-1; //7e-2; //5e-2; //1e-5;
            // 오도멘트리
            imu_msg_lpf.orientation.x = 0.0;
            imu_msg_lpf.orientation.y = 0.0;
            imu_msg_lpf.orientation.z = 0.0;
            imu_msg_lpf.orientation.w = 1.0;
            imu_msg_lpf.orientation_covariance[0] = 1e-9;
            imu_msg_lpf.orientation_covariance[4] = 1e-9;
            imu_msg_lpf.orientation_covariance[8] = 1e-6;
        }

        // 2. Offset or Raw + LPF + KF imu 데이터 헤더 설정
        if (use_kalman_filter_) {
            imu_msg_kf.header.frame_id = "imu_link";
            imu_msg_kf.header.stamp = imu_msg_sub.header.stamp;
            imu_msg_kf.linear_acceleration_covariance = imu_msg_sub.linear_acceleration_covariance;
            imu_msg_kf.angular_velocity_covariance = imu_msg_sub.angular_velocity_covariance;
        }
        is_first_sub = false;
    }

    double abs_time = imu_msg_sub.header.stamp.sec + imu_msg_sub.header.stamp.nanosec * 1e-9;
    if (base_time < 0.0) {
        base_time = abs_time;
    }
    latest_time = abs_time - base_time;
    // 상대 시간[sec]
    // std::cout << "latest_time: " << latest_time << std::endl;
    // std::cout << "--------------------------------" << std::endl;
}

void EdieImuNode::ReadImuData()
{
    // Read IMU data
    if (is_data_ready)
    {
        // 측정값
        // 가속도 [m/s^2] -> [g]
        accl_sub = {imu_msg_sub.linear_acceleration.x, imu_msg_sub.linear_acceleration.y, imu_msg_sub.linear_acceleration.z};
        accl_sub[0] = accl_sub[0] / g_to_m_sec_sqrd;
        accl_sub[1] = accl_sub[1] / g_to_m_sec_sqrd;
        accl_sub[2] = accl_sub[2] / g_to_m_sec_sqrd;
        // 각속도 [rad/s] -> [deg/s]
        gyro_sub = {imu_msg_sub.angular_velocity.x, imu_msg_sub.angular_velocity.y, imu_msg_sub.angular_velocity.z};
        gyro_sub[0] = aeirobot::RadToDeg(gyro_sub[0]);
        gyro_sub[1] = aeirobot::RadToDeg(gyro_sub[1]);
        gyro_sub[2] = aeirobot::RadToDeg(gyro_sub[2]);

        // 필터링 여부 확인
        CheckFiltering();
    }
}

void EdieImuNode::CheckFiltering()
{
    //---------------------------------------------------------------
    // 1. LPF 적용
    if (use_low_pass_filter_) {
        auto [accl_lpf, gyro_lpf] = ProcessLpfImu(accl_sub, gyro_sub, latest_time);
        if (accl_lpf[0] == 0.0f && accl_lpf[1] == 0.0f && accl_lpf[2] == 0.0f)
        {
            return;
        }

        // ------------------------ Offset + LPF ------------------------
        imu_msg_lpf.header.frame_id = "imu_link";
        imu_msg_lpf.header.stamp = imu_msg_sub.header.stamp;
        // 가속도 [g] -> [m/s^2]
        imu_msg_lpf.linear_acceleration.x = g_to_m_sec_sqrd * accl_lpf[0];
        imu_msg_lpf.linear_acceleration.y = g_to_m_sec_sqrd * accl_lpf[1];
        imu_msg_lpf.linear_acceleration.z = g_to_m_sec_sqrd * accl_lpf[2];
        // 각속도 [deg/s] -> [rad/s]
        imu_msg_lpf.angular_velocity.x = aeirobot::DegToRad(gyro_lpf[0]);
        imu_msg_lpf.angular_velocity.y = aeirobot::DegToRad(gyro_lpf[1]);
        imu_msg_lpf.angular_velocity.z = aeirobot::DegToRad(gyro_lpf[2]);

        accl_odom_pred_in = {imu_msg_lpf.linear_acceleration.x, imu_msg_lpf.linear_acceleration.y, imu_msg_lpf.linear_acceleration.z};
        gyro_odom_pred_in = {imu_msg_lpf.angular_velocity.x, imu_msg_lpf.angular_velocity.y, imu_msg_lpf.angular_velocity.z};
    }
    //---------------------------------------------------------------

    //---------------------------------------------------------------
    // 2. KF 적용
    if (use_kalman_filter_) {
        Eigen::VectorXd measurement(6);
        measurement << 
            accl_sub[0],
            accl_sub[1],
            accl_sub[2],
            gyro_sub[0],
            gyro_sub[1],
            gyro_sub[2];
        // Predict and Update
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(6, 6);

        kalman_filter_.StatePrediction(F);
        kalman_filter_.Update(measurement, H);

        Eigen::VectorXd filtered = kalman_filter_.GetState();

        // ------------------------ Offset + KF -------------------
        imu_msg_kf.header.frame_id = "imu_link";
        imu_msg_kf.header.stamp = imu_msg_sub.header.stamp;
        // 가속도 [g] -> [m/s^2]
        imu_msg_kf.linear_acceleration.x = g_to_m_sec_sqrd * filtered(0);
        imu_msg_kf.linear_acceleration.y = g_to_m_sec_sqrd * filtered(1);
        imu_msg_kf.linear_acceleration.z = g_to_m_sec_sqrd * filtered(2);
        // 각속도 [deg/s] -> [rad/s]
        imu_msg_kf.angular_velocity.x = aeirobot::DegToRad(filtered(3));  
        imu_msg_kf.angular_velocity.y = aeirobot::DegToRad(filtered(4));
        imu_msg_kf.angular_velocity.z = aeirobot::DegToRad(filtered(5));

        accl_odom_pred_in = {imu_msg_kf.linear_acceleration.x, imu_msg_kf.linear_acceleration.y, imu_msg_kf.linear_acceleration.z};
        gyro_odom_pred_in = {imu_msg_kf.angular_velocity.x, imu_msg_kf.angular_velocity.y, imu_msg_kf.angular_velocity.z};
    }
    //---------------------------------------------------------------
    
    PredictOdometry(latest_time, accl_odom_pred_in, gyro_odom_pred_in);
    ready_to_publish = true;
}


void EdieImuNode::PredictOdometry(double t, std::array<double, 3>& accel, std::array<double, 3>& gyro)
{   
    // 1) 시간 간격 계산
    if (prev_time < 1e-8) prev_time = t;  // 첫 호출만
    // double dt = 1/sample_freq_;           // 400[Hz](= 0.0025[sec]) 기준
    double dt = t - prev_time;
    prev_time = t;

    // 2) 자세 적분 (angular velocity → delta quaternion)
    //    IMU msg 에 들어있는 angular_velocity 는 rad/s
    double wx = gyro[0];
    double wy = gyro[1];
    double wz = gyro[2];
    // 회전 각 크기[rad/sec]
    double omega = std::sqrt(wx*wx + wy*wy + wz*wz);
    tf2::Quaternion dq;
    // std::cout << "omega: " << omega << std::endl;
    // std::cout << "--------------------------------" << std::endl;

    if (omega >= 0.01) 
    {
        // 단위 회전축
        double ux = wx / omega;
        double uy = wy / omega;
        double uz = wz / omega;
        double angle = omega * dt;  // 회전 각도
        dq.setRotation(tf2::Vector3(ux, uy, uz), angle);
        // std::cout << "dq: " << dq.x() << ", " << dq.y() << ", " << dq.z() << ", " << dq.w() << std::endl;
        // std::cout << "==" << std::endl;
    } 
    else 
    {
        dq = tf2::Quaternion::getIdentity();
    }

    // 누적 자세 갱신
    latest_Q = latest_Q * dq;
    latest_Q.normalize();

    // 3) 가속도(body frame → world frame) 및 중력 보상 [m/s^2]
    tf2::Vector3 acc_b(accel[0],
                       accel[1],
                       accel[2]);
    // 3-1) world frame 으로 회전
    tf2::Vector3 acc_w = tf2::quatRotate(latest_Q, acc_b);
    // std::cout << "Before acc_w: " << acc_w.x() << ", " << acc_w.y() << ", " << acc_w.z() << std::endl;

    // 3-2) 중력 보상
    acc_w -= tf2::Vector3(0, 0, g_to_m_sec_sqrd);
    // std::cout << "After acc_w: " << acc_w.x() << ", " << acc_w.y() << ", " << acc_w.z() << std::endl;
    // std::cout << "--------------------------------" << std::endl;

    // 4) Zero‑Velocity 검출
    bool is_stationary = IsStationary(acc_w, gyro);
    // std::cout << "is_stationary: " << is_stationary << std::endl;
    // std::cout << "====\n" << std::endl;

    if (is_stationary)
    {
        if (!was_stationary) {
            // 막 정지 상태로 들어온 순간, 마지막 움직일 때 자세 저장
            last_moving_Q = latest_Q;
        }
        latest_Q = last_moving_Q;
    }
    was_stationary = is_stationary;

    if (use_low_pass_filter_) {
        imu_msg_lpf.orientation = tf2::toMsg(latest_Q);
    }

    if (use_kalman_filter_) {
        imu_msg_kf.orientation = tf2::toMsg(latest_Q);
    }
}

bool EdieImuNode::IsStationary(
  const tf2::Vector3& accl,
  const std::array<double,3>& gyro)
{
    double acc_norm = std::sqrt(
        accl.x()*accl.x() + accl.y()*accl.y() + accl.z()*accl.z());
    double ang_norm = std::sqrt(
        gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);

    // std::cout << "acc_norm: " << double(acc_norm/g_to_m_sec_sqrd) << std::endl;
    // std::cout << "ang_norm: " << aeirobot::RadToDeg(ang_norm) << std::endl;
    // std::cout << "zero_vel_accel_threshold_: " << zero_vel_accel_threshold_ << std::endl;
    // std::cout << "zero_vel_gyro_threshold_: " << zero_vel_gyro_threshold_ << std::endl;
    // std::cout << "----------------------------------------------------" << std::endl;

    return (std::abs(double(acc_norm/g_to_m_sec_sqrd)) < zero_vel_accel_threshold_)
           && (aeirobot::RadToDeg(ang_norm) < zero_vel_gyro_threshold_);
    // return (std::abs(double(acc_norm/g_to_m_sec_sqrd)) < zero_vel_accel_threshold_)
    //         || (aeirobot::RadToDeg(ang_norm) < zero_vel_gyro_threshold_);
}

std::pair<std::array<double, 3>, std::array<double, 3>> EdieImuNode::ProcessLpfImu(std::array<double, 3>& accel, std::array<double, 3>& gyro, double timestamp)
{
    std::array<double, 3> accl_lpf;
    std::array<double, 3> gyro_lpf;

    accl_lpf[0] = oef_accel_x.ApplyOneEuroFilter(accel[0], timestamp); 
    accl_lpf[1] = oef_accel_y.ApplyOneEuroFilter(accel[1], timestamp);
    accl_lpf[2] = oef_accel_z.ApplyOneEuroFilter(accel[2], timestamp);
    gyro_lpf[0] = oef_gyro_x.ApplyOneEuroFilter(gyro[0], timestamp);
    gyro_lpf[1] = oef_gyro_y.ApplyOneEuroFilter(gyro[1], timestamp);
    gyro_lpf[2] = oef_gyro_z.ApplyOneEuroFilter(gyro[2], timestamp);

    return std::make_pair(accl_lpf, gyro_lpf);
}

void EdieImuNode::PubImuData()
{   
    if (!ready_to_publish) return;

    if (use_low_pass_filter_) {
        pub_imu_data_lpf->publish(imu_msg_lpf);
    }

    if (use_kalman_filter_) {
        pub_imu_data_kf->publish(imu_msg_kf);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdieImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}