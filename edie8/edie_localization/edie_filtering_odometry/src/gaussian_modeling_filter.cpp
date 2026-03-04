#include "edie_filtering_odometry/gaussian_modeling_filter.hpp"

GaussianModelingFilter::GaussianModelingFilter()
{
    // DeclareParams();
    InitializeParams();

    // 생성자에서 alias 생성: 앞으로 angle_diff_로 각도 차이를 계산합니다.
    angle_diff_ = [this](double a, double b) -> double {
        return odom_utils_.GetAngleDiff(a, b);
    };
}

GaussianModelingFilter::~GaussianModelingFilter()
{
}

void GaussianModelingFilter::DeclareParams()
{
    // 필요한 파라미터 선언 (기본값 설정)
    alpha1_ = 0.01;
    alpha2_ = 0.01;
    alpha3_x_ = 0.01;
    alpha3_y_ = 0.005;
    alpha3_ = 0.1;
    alpha4_ = 0.01;
    translation_threshold_ = 0.01;
}

// 파라미터 초기화
void GaussianModelingFilter::InitializeParams()
{
    // yaml 파일에서 파라미터를 직접 불러오기
    std::filesystem::path currentFilePath(__FILE__);
    std::filesystem::path packageSourceDir = currentFilePath.parent_path().parent_path(); // 예: .../edie_localization/edie_filtering_odometry
    // std::cout << "packageSourceDir: " << packageSourceDir << std::endl;
    std::filesystem::path offsetsPath = packageSourceDir / "config" / "gaussian_modeling_thresholds.yaml";
    std::string offsets_path_to_src = offsetsPath.string();
    LoadConfigFromFile(offsets_path_to_src);
}

// YAML 파일 내 파라미터 값들 로드 
void GaussianModelingFilter::LoadConfigFromFile(const std::string& filename)
{
    try {
        // YAML 파일 로드
        YAML::Node config = YAML::LoadFile(filename);

        if (config["alpha1"]) {
            alpha1_ = config["alpha1"].as<double>();
        }
        if (config["alpha2"]) {
            alpha2_ = config["alpha2"].as<double>();
        }
        if (config["alpha3_x"]) {
            alpha3_x_ = config["alpha3_x"].as<double>();
        }
        if (config["alpha3_y"]) {
            alpha3_y_ = config["alpha3_y"].as<double>();
        }
        if (config["alpha3"]) {
            alpha3_ = config["alpha3"].as<double>();
        }
        if (config["alpha4"]) {
            alpha4_ = config["alpha4"].as<double>();
        }
        if (config["translation_threshold"]) {
            translation_threshold_ = config["translation_threshold"].as<double>();
        }

        std::cout << "\n--------------------------------" << std::endl;
        std::cout << "Loaded offsets from " << filename << ":" << std::endl;
        std::cout << "alpha1: " << alpha1_ << std::endl;
        std::cout << "alpha2: " << alpha2_ << std::endl;
        std::cout << "alpha3_x: " << alpha3_x_ << std::endl;
        std::cout << "alpha3_y: " << alpha3_y_ << std::endl;
        std::cout << "alpha3: " << alpha3_ << std::endl;
        std::cout << "alpha4: " << alpha4_ << std::endl;
        std::cout << "translation_threshold: " << translation_threshold_ << std::endl;
        std::cout << "--------------------------------\n" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error loading config file: " << e.what() << std::endl;
    }
}

void GaussianModelingFilter::CopyParams(double lx, double ly, double lyaw, double myaw)
{
    last_odom_x_ = lx;
    last_odom_y_ = ly;
    last_odom_yaw_ = lyaw;
    yaw_meas_ = myaw;
}

void GaussianModelingFilter::UpdateSamples(const nav_msgs::msg::Odometry &odometry, geometry_msgs::msg::PoseArray &samples)
{
    if (last_odom_x_ == 0.0 && last_odom_y_ == 0.0 && last_odom_yaw_ == 0.0) return;

    //// 참고: Thrun et al.(Book: Probabilistic Robotics)
    //// Odometry 변화량 계산
    double odom_x_increment = odometry.pose.pose.position.x - last_odom_x_;
    double odom_y_increment = odometry.pose.pose.position.y - last_odom_y_;
    double odom_yaw_increment = angle_diff_(yaw_meas_, last_odom_yaw_);
    
    // (Apr 1)절대 이동 거리 계산
    double delta_transl_abs = std::sqrt(std::pow(odom_x_increment, 2) + std::pow(odom_y_increment, 2));
    // (Apr 1)진행 방향 결정: 이전 로봇 heading과 이동 벡터의 내적
    double dot = odom_x_increment * std::cos(last_odom_yaw_) + odom_y_increment * std::sin(last_odom_yaw_);
    // std::cout << "dot: " << dot << '\n';
    // std::cout << "-----------------------------" << '\n';
    double direction_sign = (dot >= 0) ? 1.0 : -1.0;       
    // (Apr 1)부호를 고려한 이동 거리
    delta_transl = direction_sign * delta_transl_abs;
    // (Apr 1)회전 변화량 계산 (후진이면 진행 방향을 반대로 보정)
    delta_rot1 = 0.0;
    if (delta_transl_abs >= translation_threshold_)
    {
        // std::cout << "--------------------------------" << '\n';
        // std::cout << "delta_transl_abs: " << delta_transl_abs << '\n';
        // std::cout << "translation_threshold_: " << translation_threshold_ << '\n';
        // std::cout << "================================================" << '\n';
        if (direction_sign < 0) {
            // 후진일 경우, 이동 벡터에 180도 보정
            delta_rot1 = angle_diff_(atan2(odom_y_increment, odom_x_increment) + M_PI, yaw_meas_);
        } else {
            delta_rot1 = angle_diff_(atan2(odom_y_increment, odom_x_increment), yaw_meas_);
        }
    }
    else{
        // std::cout << "delta_transl_abs: " << delta_transl_abs << '\n';

    }
    delta_rot2 = angle_diff_(odom_yaw_increment, delta_rot1);

    // // (Apr 1)
    double transl_std_x = alpha3_x_ * std::fabs(delta_transl) + alpha4_ * (std::fabs(delta_rot1) + std::fabs(delta_rot2));
    double transl_std_y = alpha3_y_ * std::fabs(delta_transl) + alpha4_ * (std::fabs(delta_rot1) + std::fabs(delta_rot2));
    // double transl_std = alpha3_ * std::fabs(delta_transl) + alpha4_ * (std::fabs(delta_rot1) + std::fabs(delta_rot2));
    double rot1_std = alpha1_ * delta_rot1 + alpha2_ * delta_transl; 
    double rot2_std = alpha1_ * delta_rot2 + alpha2_ * delta_transl;

    // seed [nano sec]
    unsigned long seed = odometry.header.stamp.sec * 1000000000UL + odometry.header.stamp.nanosec;
    noise_generator_ = std::default_random_engine(seed);

     // 비정규화된 노이즈 -> 정규화된 노이즈 -> 노이즈 제거된 samples_(pose array)
    NoiseGaussianModeling(transl_std_x, transl_std_y, rot1_std, rot2_std, samples);  
}

void GaussianModelingFilter::NoiseGaussianModeling(
    double transl_std_x, double transl_std_y, double rot1_std, double rot2_std, geometry_msgs::msg::PoseArray &samples_)
{
    // Measurement Noise -> Characteristic Gaussian Noise
    std::normal_distribution<double> transl_noise_x(0.0, transl_std_x);
    std::normal_distribution<double> transl_noise_y(0.0, transl_std_y);
    // std::normal_distribution<double> transl_noise(0.0, transl_std);
    std::normal_distribution<double> rot1_noise(0.0, rot1_std);
    std::normal_distribution<double> rot2_noise(0.0, rot2_std);

    // 각 Particle에 대한 모델링된 Noise-Free 위치 및 방향 계산
    for (auto & sample : samples_.poses)
    {
      double delta_transl_noise_free_x = delta_transl - transl_noise_x(noise_generator_);
      double delta_transl_noise_free_y = delta_transl - transl_noise_y(noise_generator_);
    //   double delta_transl_noise_free = delta_transl - transl_noise(noise_generator_);
      double delta_rot1_noise_free = angle_diff_(delta_rot1, rot1_noise(noise_generator_));
      double delta_rot2_noise_free = angle_diff_(delta_rot2, rot2_noise(noise_generator_));

      // 각 Particle의 모델링된 나중 위치 RPY 초기화
      tf2::Quaternion sample_q(sample.orientation.x, sample.orientation.y,
                                sample.orientation.z, sample.orientation.w);
      tf2::Matrix3x3 sample_m(sample_q);
      double sample_roll, sample_pitch, sample_yaw;
      sample_m.getRPY(sample_roll, sample_pitch, sample_yaw);

      // 모델링된 Noise-Free 위치 계산
      sample.position.x += delta_transl_noise_free_x * std::cos(sample_yaw + delta_rot1_noise_free);
      sample.position.y += delta_transl_noise_free_y * std::sin(sample_yaw + delta_rot1_noise_free);
    //   sample.position.x += delta_transl_noise_free * std::cos(sample_yaw + delta_rot1_noise_free);
    //   sample.position.y += delta_transl_noise_free * std::sin(sample_yaw + delta_rot1_noise_free);

      // 모델링된 Noise-Free 방향 계산
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, sample_yaw + delta_rot1_noise_free + delta_rot2_noise_free);
      sample.orientation.x = q.getX();
      sample.orientation.y = q.getY();
      sample.orientation.z = q.getZ();
      sample.orientation.w = q.getW();
    }
}