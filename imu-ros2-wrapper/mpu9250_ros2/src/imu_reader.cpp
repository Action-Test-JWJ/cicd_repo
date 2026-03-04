#include "mpu9250_ros2/imu_reader.hpp"

ImuReadNode::ImuReadNode()
    : Node("imu_read_node"),
    offset_init_done(false),
    ready_to_publish(false),
    is_gyro_cali_done(false),   // t -> f   
    is_accel_cali_done(false),  // t -> f  
    is_saved(false),            // t -> f  
    is_loaded(false),
    is_stop(true),
    is_first_offset(true),
    is_first_sample(true),
    buffer_size(1000),
    dynamic_buffer_size(100),
    target_interval_sec(0.005),
    gravity_to_meter_per_second_squared(9.80665),
    stop_state_threshold(1.11)
{
    rclcpp::QoS qos_reliable(rclcpp::KeepLast(1000));
    qos_reliable.reliable();
    rclcpp::QoS qos_best_effort(rclcpp::KeepLast(1000));
    qos_best_effort.best_effort();

    // IMU 초기화
    if (InitializeImu()) ROS_GREEN_STREAM("IMU Node Init Successfully.");
    else ROS_RED_STREAM("IMU Node Init Failed.");

    // Publisher
    pub_imu_data_raw = this->create_publisher<sensor_msgs::msg::Imu>("/edie8/sensor/raw_imu", qos_reliable);
    pub_imu_data_offset = this->create_publisher<sensor_msgs::msg::Imu>("/edie8/sensor/offset_imu", qos_reliable);    
    pub_offset_done = this->create_publisher<std_msgs::msg::Bool>("/edie8/sensor/offset_done", qos_reliable);
    // Subscriber
    sub_req_offset_init = this->create_subscription<std_msgs::msg::Bool>("/edie8/sensor/imu_req_offset_init", 10, 
                                                std::bind(&ImuReadNode::OffsetInitCallback, this, std::placeholders::_1));

    // Timer
    //// 1. 1k[Hz]
    // timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ImuReadNode::TimerCallback, this));
    // 2. 400[Hz](= 2.5ms) -> Allan Variance 측정용
    timer = this->create_wall_timer(std::chrono::microseconds(2500), std::bind(&ImuReadNode::TimerCallback, this));
}

ImuReadNode::~ImuReadNode()
{
    // buffer_size개의 각속도 및 가속도 데이터 수집에 사용
    gyro_buffer.clear();
    accel_buffer.clear();
}


void ImuReadNode::TimerCallback()
{
    ReadRawImuData();
    PubImuData();
}

void ImuReadNode::OffsetInitCallback(const std_msgs::msg::Bool &msg)
{
    if(msg.data)
    {
        if (!offset_init_done)
        {
            ROS_RED_STREAM("IMU Offset Init Request Received.");
            is_gyro_cali_done = false;
            is_accel_cali_done = false;
            is_saved = false;
            is_loaded = false;

            offset_init_done = true;
        }
    }
    else
    {
        // ROS_GREEN_STREAM("Use Previous IMU Offset.");
        is_gyro_cali_done = true;
        is_accel_cali_done = true;
        is_saved = true;
        
        offset_init_done = false;
    }
    
    offset_done.data = offset_init_done;
}

bool ImuReadNode::InitializeImu()
{
    if (mpu9250_basic_init(MPU9250_INTERFACE_IIC, MPU9250_ADDRESS_AD0_LOW) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU9250");
        // false이면 ROS2 노드 종료
        rclcpp::shutdown();
        return false;  
    }
    else
    {
        return true;
    }
}

void ImuReadNode::ReadRawImuData()
{
    std::array<float, 3> g, dps, ut;
    
    // Read IMU data
    if (mpu9250_basic_read(g.data(), dps.data(), ut.data()) == 0)
    {
        // 측정값
        // 가속도 [g]
        accel_measurement = {g[0], g[1], g[2]};
        // 각속도 [deg/s]
        gyro_measurement = {dps[0], dps[1], dps[2]};
        
        // RAW IMU 데이터 헤더 설정
        imu_msg_raw.header.frame_id = "imu_link";
        imu_msg_raw.header.stamp = this->now();

        // ------------------------ Raw ------------------------
        // 가속도 [g] -> [m/s^2]
        imu_msg_raw.linear_acceleration.x = gravity_to_meter_per_second_squared * accel_measurement[0];
        imu_msg_raw.linear_acceleration.y = gravity_to_meter_per_second_squared * accel_measurement[1];
        imu_msg_raw.linear_acceleration.z = gravity_to_meter_per_second_squared * accel_measurement[2];
        // 각속도 [deg/s] -> [rad/s]
        imu_msg_raw.angular_velocity.x = aeirobot::DegToRad(gyro_measurement[0]);  
        imu_msg_raw.angular_velocity.y = aeirobot::DegToRad(gyro_measurement[1]);
        imu_msg_raw.angular_velocity.z = aeirobot::DegToRad(gyro_measurement[2]);

        // ----------------------- Allan Variance 측정용 실제 dt 계산 -----------------------------
        rclcpp::Time now = imu_msg_raw.header.stamp;
        if (!is_first_sample) {
            double this_dt = (now - prev_stamp).seconds();
            dt_buffer.push_back(this_dt);
        }
        prev_stamp = now;
        is_first_sample = false;

        // 캘리브레이션 상태 확인        
        CheckCalibration();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to read data from MPU9250");
    }
}

void ImuReadNode::CheckCalibration()
{
    // 캘리브레이션 완료 후 오프셋 저장
    // 캘리브레이션 완료 후 오프셋 저장
    if (is_gyro_cali_done && is_accel_cali_done)
    {
        // ROS_GREEN_STREAM("Calibration Done.");
        if (!is_saved)
        {
            const char* home_dir = std::getenv("HOME");
            if (!home_dir)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get HOME environment variable");
                return;
            }
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("mpu9250_ros2");
            std::string offsets_path = pkg_share_dir + "/config/imu_offsets.yaml";
            imu_utils_.SaveOffsetsToYaml(offsets_path);

            std::filesystem::path currentFilePath(__FILE__);
            std::filesystem::path packageSourceDir = currentFilePath.parent_path().parent_path(); // 예: .../imu-ros2-wrapper/mpu9250_ros2
            // std::cout << "packageSourceDir: " << packageSourceDir << std::endl;
            std::filesystem::path offsetsPath = packageSourceDir / "config" / "imu_offsets.yaml";
            std::string offsets_path_to_src = offsetsPath.string();

            imu_utils_.SaveOffsetsToYaml(offsets_path_to_src);

            is_saved = true;
            ROS_GREEN_STREAM("Saved!");
        }
    }
    else if(!is_accel_cali_done)
    {
        accel_buffer.push_back(accel_measurement);
        if (int(accel_buffer.size()) >= buffer_size)
        {
            // ------------------------- x축 가속도 값에 대한 Allan Variance 측정 -----------------------
            // 1) dt_buffer 의 평균 값 계산
            double sum_dt = std::accumulate(dt_buffer.begin(), dt_buffer.end(), 0.0);
            double mean_dt = sum_dt / dt_buffer.size();
            // 2) accel_buffer 에서 X축만 추출 + [g] → [m/s^2] 변환
            std::vector<double> x_data;
            x_data.reserve(accel_buffer.size());
            for (auto &v : accel_buffer) {
                // v[0] 은 accel_measurement[0] → [g]
                x_data.push_back(v[0] * gravity_to_meter_per_second_squared);
            }
            // 3) Allan 분산 계산
            auto allan = imu_utils_.ComputeAllanVariance(x_data, mean_dt);
            // 4) 표준편차 계산 (RMS noise)
            double sum = 0;
            for (double xi : x_data) sum += xi;
            double mean = sum / x_data.size();
            double sq_sum = 0;
            for (double xi : x_data) sq_sum += (xi - mean) * (xi - mean);
            double stddev = std::sqrt(sq_sum / (x_data.size() - 1));

            // 5) 로그 출력
            RCLCPP_INFO(this->get_logger(),
            "Static accel X: stddev = %.6f m/s^2", stddev);
            for (auto &p : allan) {
            RCLCPP_INFO(this->get_logger(),
                "  Allan τ=%.3f s → var=%.6e, dev=%.6e",
                p.first, p.second, std::sqrt(p.second));
            }
            // -------------------------------------------------------------------------------------

            // buffer_size개의 데이터를 수집한 후 캘리브레이션 수행
            if (imu_utils_.CalibrateAccel(accel_buffer)) 
            {
                is_accel_cali_done = true;
            } 
            else 
            {
                ROS_RED_STREAM("IMU Accel Offset Failed.");
                is_accel_cali_done = false;
            }
            // 버퍼 초기화
            accel_buffer.clear();
        }
        else
        {
            ROS_GREEN_STREAM("Acceleration data get " << int(accel_buffer.size()%buffer_size) << "/" << buffer_size);
        }
    }
    else if (!is_gyro_cali_done)
    {
        gyro_buffer.push_back(gyro_measurement);
        if (int(gyro_buffer.size()) >= buffer_size)
        {
            // buffer_size개의 데이터를 수집한 후 캘리브레이션 수행
            if (imu_utils_.CalibrateGyro(gyro_buffer)) 
            {
                ROS_GREEN_STREAM("IMU Gyro Offset Successfully.");
                is_gyro_cali_done = true;
            }
            else
            {
                ROS_RED_STREAM("IMU Gyro Offset Failed.");
                is_gyro_cali_done = false;
            }
            // 버퍼 초기화
            gyro_buffer.clear();
        }
        else
        {
            ROS_GREEN_STREAM("Gyroscope data get " << int(gyro_buffer.size()%buffer_size) << "/" << buffer_size);
        }
    }

    // 오프셋 로드
    if (is_saved && !is_loaded)
    {
        const char* home_dir = std::getenv("HOME");
        if (!home_dir)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get HOME environment variable");
            return;
        }
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("mpu9250_ros2");
        std::string offsets_path = pkg_share_dir + "/config/imu_offsets.yaml";
        imu_utils_.LoadOffsetsFromYaml(offsets_path);

        std::filesystem::path currentFilePath(__FILE__);
        std::filesystem::path packageSourceDir = currentFilePath.parent_path().parent_path(); // 예: .../imu-ros2-wrapper/mpu9250_ros2
        // std::cout << "packageSourceDir: " << packageSourceDir << std::endl;
        std::filesystem::path offsetsPath = packageSourceDir / "config" / "imu_offsets.yaml";
        std::string offsets_path_to_src = offsetsPath.string();

        imu_utils_.LoadOffsetsFromYaml(offsets_path_to_src);

        is_loaded = true;
        ROS_GREEN_STREAM("Loaded!");
    }
}

void ImuReadNode::CheckMovingState()
{
    double accel_norm = std::sqrt(accel_measurement[0]*accel_measurement[0] + accel_measurement[1]*accel_measurement[1] + accel_measurement[2]*accel_measurement[2]);
    if (accel_norm <= stop_state_threshold){
        is_stop = true;
        // ROS_BLUE_STREAM("이때 덜그럭거림");
        // std::cout << "accel_norm: " << accel_norm << std::endl;
    }
    else{
        // ROS_GREEN_STREAM("Moving State Detected.");
        std::cout << "accel_norm: " << accel_norm << std::endl;
        std::cout << "stop_state_threshold: " << stop_state_threshold << std::endl;
        std::cout << "--------------------------------" << std::endl;
        
        is_stop = false;
        is_first_offset = false;
    }
}

void ImuReadNode::ApplyOffsets()
{
    // 가속도 [g]
    accl_offset = imu_utils_.ApplyAccelOffsets(accel_measurement);
    // 각속도 [deg/s]
    gyro_offset = imu_utils_.ApplyGyroOffsets(gyro_measurement);

    // Offset 데이터 헤더 설정
    imu_msg_offset.header.frame_id = "imu_link";
    imu_msg_offset.header.stamp = imu_msg_raw.header.stamp;
    
    // ------------------------ Offset ------------------------
    // 가속도 [g] -> [m/s^2]
    imu_msg_offset.linear_acceleration.x = gravity_to_meter_per_second_squared * accl_offset[0];
    imu_msg_offset.linear_acceleration.y = gravity_to_meter_per_second_squared * accl_offset[1];
    imu_msg_offset.linear_acceleration.z = gravity_to_meter_per_second_squared * accl_offset[2];
    // 각속도 [deg/s] -> [rad/s]
    imu_msg_offset.angular_velocity.x = aeirobot::DegToRad(gyro_offset[0]);
    imu_msg_offset.angular_velocity.y = aeirobot::DegToRad(gyro_offset[1]);
    imu_msg_offset.angular_velocity.z = aeirobot::DegToRad(gyro_offset[2]);
    ready_to_publish = true;
}

void ImuReadNode::PubImuData()
{   
    if (!ready_to_publish) return;
    pub_imu_data_raw->publish(imu_msg_raw);
    pub_imu_data_offset->publish(imu_msg_offset);
    pub_offset_done->publish(offset_done);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuReadNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}