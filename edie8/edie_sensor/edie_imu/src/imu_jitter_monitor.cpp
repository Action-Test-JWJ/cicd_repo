#include "edie_imu/imu_jitter_monitor.hpp"

ImuJitterMonitor::ImuJitterMonitor()
    : Node("imu_jitter_monitor"),
    first_time(true),
    is_plotted(false)
{
    rclcpp::QoS qos_(rclcpp::SensorDataQoS().keep_last(200)); // 50 -> 100 -> 200
    // qos_.reliable();
    qos_.best_effort();

    // Subscriber
    // 400 [Hz] == 2.5 [ms]
    sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("/edie8/sensor/lpf_imu", qos_, 
        std::bind(&ImuJitterMonitor::ImuJitterCallback, this, std::placeholders::_1));

    // Timer 
    // 200 [Hz] == 5 [ms]
    // timer = create_wall_timer(std::chrono::milliseconds(5), std::bind(&ImuJitterMonitor::ReportJitter, this));
    // 400 [Hz] == 2.5 [ms]
    timer = this->create_wall_timer(std::chrono::microseconds(2500), std::bind(&ImuJitterMonitor::ReportJitter, this));
}

ImuJitterMonitor::~ImuJitterMonitor()
{
    buffer_delta_t.clear();
}


void ImuJitterMonitor::ImuJitterCallback(const sensor_msgs::msg::Imu &msg)
{
    struct timespec now_time;
    clock_gettime(CLOCK_MONOTONIC, &now_time);
    if (!first_time) {
      double dt = (now_time.tv_sec - prev_time.tv_sec)
                    + (now_time.tv_nsec - prev_time.tv_nsec) * 1e-9;
      buffer_delta_t.push_back(dt);
    }
    prev_time = now_time;
    first_time = false;
}

void ImuJitterMonitor::ReportJitter()
{
    if (buffer_delta_t.size() > report_threshold) {            
        if (!is_plotted) {
            ShowTimeSeriesPlot(buffer_delta_t);
            is_plotted = true;
        }

        buffer_delta_t.clear();
    }
}

void ImuJitterMonitor::ShowTimeSeriesPlot(const std::vector<double>& v)
{
    // 1) 통계 계산
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();
    double var = 0;
    for (double d : v) var += (d - mean) * (d - mean);
    var /= v.size();
    double stddv = std::sqrt(var);
    double data_min = *std::min_element(v.begin(), v.end());
    double data_max = *std::max_element(v.begin(), v.end());

    // 0.5ms 이하 또는 4.5ms 초과 샘플 수 계산 (10,000개 기준)
    const double low_th = 0.0005;  // 0.5ms
    const double high_th = 0.0045; // 4.5ms
    size_t outliers = std::count_if(
        v.begin(), v.end(),
        [&](double dt) { return dt < low_th || dt > high_th; }
    );

    RCLCPP_INFO(get_logger(),
        "IMU Δt: mean=%.3f ms, std=%.3f ms", mean*1000, stddv*1000);
    RCLCPP_INFO(get_logger(),
        "IMU jitter outliers: %zu / %zu samples (dt < 0.5ms or dt > 4.5ms)",
        outliers, report_threshold);
    RCLCPP_INFO(get_logger(),
        "--------------------------------");

    // 2) 이미지 세팅 (세로 늘리고 margin_bottom 추가)
    const int plot_w        = 1000;
    const int plot_h        = 400;
    const int margin_left   = 60;
    const int margin_top    = 20;
    const int margin_bottom = 60;  // 세로 여유
    const cv::Scalar bg(255,255,255),
                     black(0,0,0),
                     red(0,0,255),
                     blue(255,0,0),
                     green(0,200,0);
    cv::Mat img(plot_h + margin_top + margin_bottom,
                plot_w + margin_left + 20,
                CV_8UC3, bg);

    // 3) 매핑 람다: 데이터→픽셀 Y
    auto mapY = [&](double y){
        return margin_top + plot_h
             - int((y - data_min) / (data_max - data_min) * plot_h);
    };

    // 4) 축 그리기
    cv::line(img,
             {margin_left, margin_top},
             {margin_left, margin_top + plot_h},
             black);
    cv::line(img,
             {margin_left, margin_top + plot_h},
             {margin_left + plot_w, margin_top + plot_h},
             black);
    // mean 기준 +2ms, -2ms 수평선 그리기
    {
        int y_high = mapY(mean + 0.002);  // +2ms
        int y_low  = mapY(mean - 0.002);  // -2ms
        cv::line(img, {margin_left, y_high},   {margin_left + plot_w, y_high},  green, 2);
        cv::line(img, {margin_left, y_low},    {margin_left + plot_w, y_low},  green, 2);
    }
    // y축에 ±2ms 위치에 틱 및 숫자 라벨 표시
    {
        int y_high2 = mapY(mean + 0.002);
        int y_low2  = mapY(mean - 0.002);
        const int tick_len = 5;
        const int label_x = margin_left - 50;
        double val_high_ms = (mean + 0.002) * 1000;
        double val_low_ms  = (mean - 0.002) * 1000;
        auto str_high = cv::format("%.1fms", val_high_ms);
        auto str_low  = cv::format("%.1fms", val_low_ms);
        // tick lines
        cv::line(img, {margin_left - tick_len, y_high2}, {margin_left + tick_len, y_high2}, black, 1);
        cv::line(img, {margin_left - tick_len, y_low2},  {margin_left + tick_len, y_low2},  black, 1);
        // labels
        cv::putText(img, str_high, {label_x, y_high2 + 5}, cv::FONT_HERSHEY_SIMPLEX, 0.5, black, 1);
        cv::putText(img, str_low,  {label_x, y_low2 + 5},  cv::FONT_HERSHEY_SIMPLEX, 0.5, black, 1);
    }

    // 5) 시계열 데이터 폴리라인
    std::vector<cv::Point> pts;
    pts.reserve(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        int x = margin_left + int(double(i) / (v.size()-1) * plot_w);
        int y = mapY(v[i]);
        pts.emplace_back(x, y);
    }
    cv::polylines(img, pts, false, blue, 1);

    // 6) max 강조 (빨간 원 + 텍스트)
    {
      auto it = std::max_element(v.begin(), v.end());
      size_t idx = std::distance(v.begin(), it);
      double val = *it;
      int x = margin_left + int(double(idx) / (v.size()-1) * plot_w);
      int y = mapY(val);
      cv::circle(img, {x, y}, 5, red, 2);
      std::ostringstream ss;
      ss << "max["<<idx<<"]=" << std::fixed<<std::setprecision(1)
         << (val * 1000) << "ms";
      cv::putText(img, ss.str(),
                  {x + 10, y - 10},
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.5, red, 2);
    }

    // 7) min 강조 (초록 원 + 텍스트)
    {
      auto it = std::min_element(v.begin(), v.end());
      size_t idx = std::distance(v.begin(), it);
      double val = *it;
      int x = margin_left + int(double(idx) / (v.size()-1) * plot_w);
      int y = mapY(val);
      cv::circle(img, {x, y}, 5, green, 2);
      std::ostringstream ss;
      ss << "min["<<idx<<"]=" << std::fixed<<std::setprecision(1)
         << (val * 1000) << "ms";
      cv::putText(img, ss.str(),
                  {x + 10, y + 15},
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.5, green, 2);
    }

    // 8) mean, +1σ, -1σ 레이블 (작은 폰트, 수직 오프셋)
    struct Stat { double val; const char* label; };
    std::vector<Stat> highlights = {
      { mean,        "mean"  },
      { mean+stddv,  "+1 std_dev"   },
      { mean-stddv,  "-1 std_dev"   }
    };
    // 우상단에 범례(legend) 표시
    const double fs   = 0.4;     // fontScale
    const int th      = 1;       // thickness
    const int legendX = margin_left + plot_w - 200;
    const int legendY = margin_top + 20;
    for (size_t i = 0; i < highlights.size(); ++i) {
        std::ostringstream ss;
        ss << highlights[i].label << ":"
           << cv::format("%.2f", highlights[i].val * 1000) << "ms";
        int x = legendX;
        int y = legendY + static_cast<int>(i * (fs * 30));
        cv::putText(img,
                    ss.str(),
                    {x, y},
                    cv::FONT_HERSHEY_SIMPLEX,
                    fs, red, th);
    }

    // 9) 축 레이블
    cv::putText(img, "Sample Index",
                {margin_left + plot_w/2 - 50,
                 margin_top + plot_h + 40},
                cv::FONT_HERSHEY_SIMPLEX,
                0.6, black, 2);
    cv::putText(img, "dt [ms]",
                {margin_left - 40,
                 margin_top - 10},
                cv::FONT_HERSHEY_SIMPLEX,
                0.6, black, 2);

    // 10) 화면 표시
    cv::imshow("IMU Δt Time Series", img);
    cv::waitKey(0);
}

int main(int argc, char *argv[])
{
    // 1) ROS 2 초기화
    rclcpp::init(argc, argv);
    
    // 2) 스케줄러 정책 및 우선순위 설정 (SCHED_FIFO, priority=80)
    {
        struct sched_param sch_param{};
        sch_param.sched_priority = 80;  // 1~99 사이에서 선택 (높을수록 우선순위 높음)
        
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch_param) != 0) {
            std::cerr << "pthread_setschedparam failed: "
                      << std::strerror(errno) << std::endl;
        }

        // 확인용: 방금 설정된 정책과 우선순위 읽어오기
        int policy_check;
        struct sched_param param_check;
        if (pthread_getschedparam(pthread_self(), &policy_check, &param_check) == 0) {
            const char* policy_name =
                (policy_check == SCHED_FIFO ? "SCHED_FIFO" :
                 policy_check == SCHED_RR   ? "SCHED_RR"   : "SCHED_OTHER");
            std::cout << "스케줄링 확인: policy=" << policy_name
                      << ", priority=" << param_check.sched_priority
                      << std::endl;
        } else {
            std::cerr << "pthread_getschedparam failed: "
                      << std::strerror(errno) << std::endl;
        }
    }

    // 3) 노드 생성 및 spin
    auto node = std::make_shared<ImuJitterMonitor>();
    rclcpp::spin(node);

    // 4) 정리 및 종료
    rclcpp::shutdown();
    return 0;
}