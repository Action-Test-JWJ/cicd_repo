#include "icm20948_ros2/icm20948_imu_reader.hpp"

Icm20948ReadNode::Icm20948ReadNode()
    : Node("icm20948_read_node"),
    is_gyro_cali_done(false),   // t -> f   
    is_accel_cali_done(false),  // t -> f  
    is_saved(false),            // t -> f  
    is_loaded(false),
    is_stop(true),
    buffer_size(1000),
    gravity_to_meter_per_second_squared(9.80665)
{
    CalibrateTimeOffset();

    // rclcpp::QoS qos_reliable(rclcpp::KeepLast(1000));
    // qos_reliable.reliable();
    // rclcpp::QoS qos_best_effort(rclcpp::KeepLast(1000));
    // qos_best_effort.best_effort();

    auto imu_qos = rclcpp::SensorDataQoS()
                        .keep_last(50)  // 너무 작으면 데이터 놓치고, 너무 크면 메모리+지연
                        .best_effort() //.reliable()
                        .deadline(rclcpp::Duration::from_seconds(0.002)); // 500 Hz 기대
    // Python 임베딩 초기화
    InitializePython();

    // IMU 초기화
    if (InitializeImu()) ROS_GREEN_STREAM("IMU Node Init Successfully.");
    else ROS_RED_STREAM("IMU Node Init Failed.");

    // Publisher [qos_reliable -> imu_qos]
    pub_imu_data_raw = this->create_publisher<sensor_msgs::msg::Imu>("/edie8/sensor/raw_imu", imu_qos);
    pub_imu_data_offset = this->create_publisher<sensor_msgs::msg::Imu>("/edie8/sensor/offset_imu", imu_qos);    

    // 500[Hz](= 2ms). [1k -> 600 -> 400 -> 500]
    timer = this->create_wall_timer(std::chrono::microseconds(2000), std::bind(&Icm20948ReadNode::TimerCallback, this));
}

Icm20948ReadNode::~Icm20948ReadNode()
{
    if (Py_IsInitialized()) {
        Py_Finalize();
    }

    // buffer_size개의 각속도 및 가속도 데이터 수집에 사용
    gyro_buffer.clear();
    accel_buffer.clear();
}

void Icm20948ReadNode::CalibrateTimeOffset()
{
    auto mono_now = mono_clock::now();
    auto sys_now  = sys_clock::now();
    // 절대 시간 - 상대 시간 차이 계산 -> 초반 절대 시간 offset 용도
    steady_to_system_offset_ = std::chrono::duration_cast<ns>(sys_now.time_since_epoch())
                            - std::chrono::duration_cast<ns>(mono_now.time_since_epoch());
}

void Icm20948ReadNode::TimerCallback()
{
    ReadRawImuData();
    CheckCalibration();
    // 오프셋/회전행렬 로드 완료 후에만
    if (is_loaded) ApplyOffsets();
    PubImuData();
}

void Icm20948ReadNode::InitializePython()
{
    // 1) Python 인터프리터 초기화
    Py_Initialize();
    PyEval_InitThreads();  // GIL과 메인 스레드 상태 초기화

    PyGILState_STATE g = PyGILState_Ensure();
    
    // 2) 공유 디렉터리 경로 가져와서 sys.path 삽입
    std::string share_dir = ament_index_cpp::get_package_share_directory("icm20948_ros2");
    std::string py_path = share_dir + "/python_modules";
    PyRun_SimpleString(("import sys; sys.path.insert(0, \"" + py_path + "\")").c_str());

    // 3) qwiic_i2c 모듈 임포트 → get_i2c_driver 호출 (iBus=5 → /dev/i2c-5)
    pI2cMod_ = PyImport_ImportModule("qwiic_i2c");
    if (!pI2cMod_) {
        RCLCPP_ERROR(get_logger(), "Failed to import qwiic_i2c");
        PyGILState_Release(g);
        return;
    }
    pGetDriver_ = PyObject_GetAttrString(pI2cMod_, "get_i2c_driver");
    Py_DECREF(pI2cMod_);
    pDriver_ = PyObject_CallFunction(pGetDriver_, "i", 5);
    Py_DECREF(pGetDriver_);
    if (!pDriver_) {
        RCLCPP_ERROR(get_logger(), "Failed to call get_i2c_driver(iBus=5)");
        PyGILState_Release(g);
        return;
    }

    // 4) qwiic_icm20948.py 모듈 import
    pModule_ = PyImport_ImportModule("qwiic_icm20948");
    if (!pModule_) {
        RCLCPP_ERROR(get_logger(), "Failed to import qwiic_icm20948.py");
        Py_DECREF(pDriver_);
        PyGILState_Release(g);
        return;
    }

    // 4) 클래스 가져오기
    PyObject* pClass = PyObject_GetAttrString(pModule_, "QwiicIcm20948");
    if (!pClass) {
        RCLCPP_ERROR(get_logger(), "QwiicIcm20948 class not found");
        Py_DECREF(pDriver_);
        PyGILState_Release(g);
        return;
    }

    // 5) 생성자 호출: (address, i2c_driver)
    //    I2C_ADDR 는 헤더에 정의된 0x68
    pInstance_ = PyObject_CallObject(pClass, Py_BuildValue("(iO)", I2C_ADDR, pDriver_));
    Py_DECREF(pClass);
    Py_DECREF(pDriver_);
    if (!pInstance_) {
        RCLCPP_ERROR(get_logger(), "Failed to instantiate QwiicIcm20948(address, driver)");
        PyGILState_Release(g);
        return;
    }

    // GIL을 완전히 반납하고, 현재 스레드 상태(PyThreadState*)를 저장
    // 반납된 스레드는 다른 스레드가 파이썬 C-API를 호출할 때 사용될 수 있도록 준비됨
    PyGILState_Release(g);
 
}

bool Icm20948ReadNode::InitializeImu()
{
    // Python 드라이버 연결 체크
    PyObject* pConnected = PyObject_GetAttrString(pInstance_, "connected");
    if (!pConnected || !PyObject_IsTrue(pConnected)) {
        RCLCPP_ERROR(get_logger(), "ICM20948 not connected over I2C");
        Py_XDECREF(pConnected);
        return false;
    }
    Py_DECREF(pConnected);

    // begin() 호출
    PyObject_CallMethod(pInstance_, "begin", nullptr);
    // 풀스케일 설정
    //// Python 모듈에서 gpm4 = 0x01, dps500 = 0x01 로 정의되어 있음
    PyObject_CallMethod(pInstance_, "setFullScaleRangeAccel", "i", GPM4);
    PyObject_CallMethod(pInstance_, "setFullScaleRangeGyro",  "i", DPS500);
    
    return true;
}

void Icm20948ReadNode::ReadRawImuData()
{   
    // GIL 확보
    PyGILState_STATE gil = PyGILState_Ensure();

    // 0) 읽기 시작 시점: 커널 타임스탬프
    auto t_start = mono_clock::now();

    // Python API 사용하여 호출
    // 1) dataReady()
    PyObject* pReady = PyObject_CallMethod(pInstance_, "dataReady", nullptr);
    if (!pReady || !PyObject_IsTrue(pReady)) {
        Py_XDECREF(pReady);
        RCLCPP_WARN(this->get_logger(), "Failed to read data from ICM20948");
        
        // GIL 반납
        PyGILState_Release(gil);
        return;
    }
    Py_DECREF(pReady);

    // 2) getAgmt()
    PyObject_CallMethod(pInstance_, "getAgmt", nullptr);

    // 0) 읽기 종료 시점: 커널 타임스탬프
    auto t_end = mono_clock::now();

    // 0) 중간점 계산
    auto mid = t_start + (t_end - t_start) / 2;  // 중간 시점 as time_point

    // 0) 절대 시간 offset 적용
    auto sys_ns = std::chrono::duration_cast<ns>(mid.time_since_epoch()) + steady_to_system_offset_;

    // 3) Python 속성에서 raw 값 꺼내기
    //  ICM-20948 센서가 출력한 “원시(raw) 카운트 [LSB]
    double ax = PyFloat_AsDouble(PyObject_GetAttrString(pInstance_, "axRaw"));
    double ay = PyFloat_AsDouble(PyObject_GetAttrString(pInstance_, "ayRaw"));
    double az = PyFloat_AsDouble(PyObject_GetAttrString(pInstance_, "azRaw"));
    double gx = PyFloat_AsDouble(PyObject_GetAttrString(pInstance_, "gxRaw"));
    double gy = PyFloat_AsDouble(PyObject_GetAttrString(pInstance_, "gyRaw"));
    double gz = PyFloat_AsDouble(PyObject_GetAttrString(pInstance_, "gzRaw"));

    // 4) GIL 반납 — Python 호출이 모두 끝난 직후
    PyGILState_Release(gil);

    // 5) 원시 카운트 [LSB] → [g] 또는 [deg/s]
    ax = ax / 8192.0;
    ay = ay / 8192.0;
    az = az / 8192.0;
    gx = gx / 65.5;
    gy = gy / 65.5;
    gz = gz / 65.5;

    // 6) ROS 메시지에 값 세팅 및 나머지 로직
    accel_measurement = {ax, ay, az};
    gyro_measurement  = {gx, gy, gz};

    // timestamp [ns] & frame 설정
    imu_msg_raw.header.stamp = rclcpp::Time(sys_ns.count());
    imu_msg_raw.header.frame_id = "imu_link";

     // 가속도 [g] → [m/s^2]
    imu_msg_raw.linear_acceleration.x = gravity_to_meter_per_second_squared * accel_measurement[0];
    imu_msg_raw.linear_acceleration.y = gravity_to_meter_per_second_squared * accel_measurement[1];
    imu_msg_raw.linear_acceleration.z = gravity_to_meter_per_second_squared * accel_measurement[2];
    imu_msg_raw.linear_acceleration_covariance[0] = 1e-6; //1e-4; //1e-5; // 1e-7 
    imu_msg_raw.linear_acceleration_covariance[4] = 1e-9; //1e-1;
    imu_msg_raw.linear_acceleration_covariance[8] = 1e-9;

    // 각속도 [deg/s] → [rad/s]
    imu_msg_raw.angular_velocity.x = aeirobot::DegToRad(gyro_measurement[0]);    
    imu_msg_raw.angular_velocity.y = aeirobot::DegToRad(gyro_measurement[1]);
    imu_msg_raw.angular_velocity.z = aeirobot::DegToRad(gyro_measurement[2]);
    imu_msg_raw.angular_velocity_covariance[0] = 1e-9;
    imu_msg_raw.angular_velocity_covariance[4] = 1e-9;
    imu_msg_raw.angular_velocity_covariance[8] = 1e-6; //8e-2; //7.68e-3(don't know) //5e-1; //7e-2; //5e-2; //1e-5;

    // 오도멘트리
    imu_msg_raw.orientation.x = 0.0;
    imu_msg_raw.orientation.y = 0.0;
    imu_msg_raw.orientation.z = 0.0;
    imu_msg_raw.orientation.w = 1.0;
    imu_msg_raw.orientation_covariance[0] = -1.0;
/*
    // 타임스탬프 로그 출력
    rclcpp::Time now_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Timestamp this->now(): %llu ns", static_cast<unsigned long long>(now_time.nanoseconds()));
    RCLCPP_INFO(this->get_logger(), "mid: %llu ns", static_cast<unsigned long long>(mid.time_since_epoch().count()));
    RCLCPP_INFO(this->get_logger(), "steady_to_system_offset_: %llu ns", static_cast<unsigned long long>(steady_to_system_offset_.count()));
    RCLCPP_INFO(this->get_logger(), "Timestamp sys_ns     : %llu ns", static_cast<unsigned long long>(sys_ns.count()));
    
    // 타임스탬프 차이 계산 및 밀리초 변환
    int64_t diff_ns = static_cast<int64_t>(now_time.nanoseconds()) - static_cast<int64_t>(sys_ns.count());
    double diff_ms = static_cast<double>(diff_ns) * 1e-6;
    RCLCPP_INFO(this->get_logger(), "Timestamp difference  : %.3f ms", diff_ms);
    RCLCPP_INFO(this->get_logger(), "--------------------------------");
*/
}

void Icm20948ReadNode::CheckCalibration()
{
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
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("icm20948_ros2");
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
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("icm20948_ros2");
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

void Icm20948ReadNode::ApplyOffsets()
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
    imu_msg_offset.linear_acceleration_covariance = imu_msg_raw.linear_acceleration_covariance;

    // 각속도 [deg/s] -> [rad/s]
    imu_msg_offset.angular_velocity.x = aeirobot::DegToRad(gyro_offset[0]);
    imu_msg_offset.angular_velocity.y = aeirobot::DegToRad(gyro_offset[1]);
    imu_msg_offset.angular_velocity.z = aeirobot::DegToRad(gyro_offset[2]);
    imu_msg_offset.angular_velocity_covariance = imu_msg_raw.angular_velocity_covariance;

    // 오도멘트리
    imu_msg_offset.orientation = imu_msg_raw.orientation;
    imu_msg_offset.orientation_covariance = imu_msg_raw.orientation_covariance;
}

void Icm20948ReadNode::PubImuData()
{   
    pub_imu_data_raw->publish(imu_msg_raw);
    if (is_loaded) pub_imu_data_offset->publish(imu_msg_offset);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Icm20948ReadNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}