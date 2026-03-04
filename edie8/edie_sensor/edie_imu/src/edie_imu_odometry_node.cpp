#include "edie_imu/edie_imu_odometry_node.hpp"

EdieImuOdomNode::EdieImuOdomNode()
    : Node("edie_imu_odometry_node"),
    is_data_ready(false),    // t -> f
    ready_to_publish(false),
    was_stationary(true),
    g_to_m_sec_sqrd(9.80665),
    sample_freq_(400),
    v_forward_(0.0),
    zero_vel_accel_threshold_(1.0), //(1.02),      // [g] 단위에서 오차 허용 범위
    zero_vel_gyro_threshold_(0.2) //1.0  // [deg/s] 단위에서 오차 허용 범위   
{ 
    rclcpp::QoS imu_sub_qos(rclcpp::KeepLast(1000));
    imu_sub_qos.best_effort();
    // rclcpp::QoS qos_reliable(rclcpp::KeepLast(1000));
    // qos_reliable.reliable();

    rclcpp::QoS odom_pub_qos(rclcpp::KeepLast(50));
    odom_pub_qos.best_effort();
    // odom_pub_qos.transient_local();

    // Publisher
    pub_odometry = this->create_publisher<nav_msgs::msg::Odometry>("/edie8/sensor/odometry_imu", odom_pub_qos);
    // Subscriber
    sub_imu_data = this->create_subscription<sensor_msgs::msg::Imu>("/edie8/sensor/lpf_imu", imu_sub_qos, 
                    std::bind(&EdieImuOdomNode::ImuCallback, this, std::placeholders::_1));
    // Timer; 20ms마다 오디멘트리 발행
    timer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&EdieImuOdomNode::TimerCallback, this));
}

EdieImuOdomNode::~EdieImuOdomNode()
{
}

void EdieImuOdomNode::TimerCallback()
{
    if (ready_to_publish) {
        pubLatestOdometry(latest_time, latest_P, latest_V, latest_Q);
    }
}


void EdieImuOdomNode::ImuCallback(const sensor_msgs::msg::Imu &imu)
{
    is_data_ready = true;
    imu_sub = imu;

    double abs_time = imu_sub.header.stamp.sec + imu_sub.header.stamp.nanosec * 1e-9;
    if (base_time < 0.0) {
        base_time = abs_time;
    }
    latest_time = abs_time - base_time;
    // 실제 시간[sec]
    // std::cout << "latest_time: " << latest_time << std::endl;
    // std::cout << "--------------------------------" << std::endl;

    accl_sub = {imu_sub.linear_acceleration.x, imu_sub.linear_acceleration.y, imu_sub.linear_acceleration.z};
    gyro_sub = {imu_sub.angular_velocity.x, imu_sub.angular_velocity.y, imu_sub.angular_velocity.z};

    PredictOdometry(latest_time, accl_sub, gyro_sub);

    ready_to_publish = true;
}

void EdieImuOdomNode::PredictOdometry(double t, std::array<double, 3>& accel, std::array<double, 3>& gyro)
{
    // 1) 시간 간격 계산
    if (prev_time < 1e-8) prev_time = t;  // 첫 호출만
    // double dt = 1/sample_freq_;           // 400[Hz](= 0.0025[sec]) 기준
    double dt = t - prev_time;
    prev_time = t;

    // std::cout << "dt: " << dt << std::endl;
    // std::cout << "--------------------------------" << std::endl;

    // 2) 자세 적분 (angular velocity → delta quaternion)
    //    IMU msg 에 들어있는 angular_velocity 는 rad/s
    double wx = gyro[0];
    double wy = gyro[1];
    double wz = gyro[2];
    // 회전 각 크기[rad/sec]
    double omega = std::sqrt(wx*wx + wy*wy + wz*wz);
    tf2::Quaternion dq;
    std::cout << "omega: " << omega << std::endl;
    std::cout << "--------------------------------" << std::endl;

    if (omega >= 0.01) 
    {
        // 단위 회전축
        double ux = wx / omega;
        double uy = wy / omega;
        double uz = wz / omega;
        double angle = omega * dt;  // 회전 각도
        dq.setRotation(tf2::Vector3(ux, uy, uz), angle);
        std::cout << "dq: " << dq.x() << ", " << dq.y() << ", " << dq.z() << ", " << dq.w() << std::endl;
        std::cout << "==" << std::endl;
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
    std::cout << "is_stationary: " << is_stationary << std::endl;
    std::cout << "====" << std::endl;

    if (is_stationary)
    {
        if (!was_stationary) {
            // 막 정지 상태로 들어온 순간, 마지막 움직일 때 위치 및 자세 저장
            last_moving_P = latest_P;
            last_moving_Q = latest_Q;
        }
        v_forward_ = 0.0;
        latest_V = {0,0,0};
        latest_P = last_moving_P;  // 항상 마지막 위치로 리셋
        latest_Q = last_moving_Q;
    }
    else
    {
        // // 4) 속도 적분
        // latest_V[0] += acc_w.x() * dt;
        // latest_V[1] += acc_w.y() * dt;
        // latest_V[2] += acc_w.z() * dt;

        // // 5) 위치 적분
        // latest_P[0] += latest_V[0]*dt + 0.5*acc_w.x()*dt*dt;
        // latest_P[1] += latest_V[1]*dt + 0.5*acc_w.y()*dt*dt;
        // latest_P[2] += latest_V[2]*dt + 0.5*acc_w.z()*dt*dt;
    
        // 5) body‐frame x축 속도 적분 (앞으로 가면 +, 뒤로 가면 –)
        v_forward_ += acc_w[0] * dt;

        if (v_forward_ > 0.0) {
            std::cout << "전진 " << "v_forward_: " << v_forward_ << std::endl;
        }
        else{
            std::cout << "후진 " << "v_forward_: " << v_forward_ << std::endl;
        }
        std::cout << "@@@@@@" << std::endl;
        
        // 6) 현재 yaw 추출
        double roll, pitch, yaw;
        tf2::Matrix3x3(latest_Q).getRPY(roll, pitch, yaw);

        // 7) planar 위치 적분
        latest_P[0] += v_forward_ * std::cos(yaw) * dt;
        latest_P[1] += v_forward_ * std::sin(yaw) * dt;

        // (원하면 최고속도 제한, drift 보정 로직 추가)
        latest_V = { v_forward_ * std::cos(yaw), v_forward_ * std::sin(yaw), 0.0 };
    }
    was_stationary = is_stationary;
    
    // // 오디멘트리 발행  
    // pubLatestOdometry(t, latest_P, latest_V, latest_Q);
}

bool EdieImuOdomNode::IsStationary(
  const tf2::Vector3& accl,
  const std::array<double,3>& gyro)
{
    double acc_norm = std::sqrt(
        accl.x()*accl.x() + accl.y()*accl.y() + accl.z()*accl.z());
    double ang_norm = std::sqrt(
        gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);

    std::cout << "acc_norm: " << double(acc_norm/g_to_m_sec_sqrd) << std::endl;
    std::cout << "ang_norm: " << aeirobot::RadToDeg(ang_norm) << std::endl;
    std::cout << "zero_vel_accel_threshold_: " << zero_vel_accel_threshold_ << std::endl;
    std::cout << "zero_vel_gyro_threshold_: " << zero_vel_gyro_threshold_ << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;

    return (std::abs(double(acc_norm/g_to_m_sec_sqrd)) < zero_vel_accel_threshold_)
           && (aeirobot::RadToDeg(ang_norm) < zero_vel_gyro_threshold_);
}

void EdieImuOdomNode::pubLatestOdometry(double t, std::array<double, 3>& P, std::array<double, 3>& V, tf2::Quaternion& Q)
{
    nav_msgs::msg::Odometry odometry;
    
    int sec_ts = (int)t;
    uint nsec_ts = (uint)((t - sec_ts) * 1e9);
    odometry.header.stamp.sec = sec_ts;
    odometry.header.stamp.nanosec = nsec_ts;

    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_footprint";
    // Position
    odometry.pose.pose.position.x = P[0];
    odometry.pose.pose.position.y = P[1];
    odometry.pose.pose.position.z = P[2];
    // Quaternion
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    // Velocity
    odometry.twist.twist.linear.x = V[0];
    odometry.twist.twist.linear.y = V[1];
    odometry.twist.twist.linear.z = V[2];

    pub_odometry->publish(odometry);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdieImuOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}