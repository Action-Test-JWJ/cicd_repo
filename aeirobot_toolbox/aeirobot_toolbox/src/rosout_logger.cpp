#include "aeirobot_toolbox/rosout_logger.hpp"
#include "aeirobot_toolbox/basic_tools.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <sys/stat.h>
#include <filesystem>
#include <unistd.h>
#include <linux/limits.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace aeirobot_toolbox
{

void RosoutLogger::LoadNodeGroupMapping()
{
    // 파라미터에서 설정 파일 경로 가져오기
    std::string config_file = this->get_parameter("config_file_path").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Loading node group mapping from: %s", config_file.c_str());
    
    try
    {
        if (!std::filesystem::exists(config_file))
        {
            RCLCPP_WARN(this->get_logger(), "Config file not found: %s", config_file.c_str());
            return;
        }
        
        YAML::Node config = YAML::LoadFile(config_file);
        
        if (!config["log_groups"])
        {
            RCLCPP_WARN(this->get_logger(), "No log_groups found in config file");
            return;
        }
        
        // 모든 그룹과 노드를 순회하며 매핑 생성
        YAML::Node groups = config["log_groups"];
        for (const auto& group_it : groups)
        {
            std::string group_name = group_it.first.as<std::string>();
            YAML::Node nodes = group_it.second["nodes"];
            
            if (!nodes)
            {
                RCLCPP_WARN(this->get_logger(), "No nodes found in group: %s", group_name.c_str());
                continue;
            }
            
            for (const auto& node : nodes)
            {
                std::string node_name = node.as<std::string>();
                node_to_group_map_[node_name] = group_name;
                RCLCPP_DEBUG(this->get_logger(), "Mapped node '%s' to group '%s'", 
                          node_name.c_str(), group_name.c_str());
            }
            
            RCLCPP_INFO(this->get_logger(), "Added %zu nodes to group '%s'", 
                     nodes.size(), group_name.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "Node group mapping loaded with %zu entries", node_to_group_map_.size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading config file: %s", e.what());
    }
}

std::string RosoutLogger::GetNodeGroup(const std::string& node_name)
{
    // 노드 이름에 해당하는 그룹 검색
    auto it = node_to_group_map_.find(node_name);
    
    if (it != node_to_group_map_.end())
    {
        return it->second;
    }
    
    // 그룹을 찾지 못한 경우 기본 그룹 반환
    return "ungrouped";
}

RosoutLogger::RosoutLogger()
    : Node("rosout_logger")
    , log_level_filter_(rcl_interfaces::msg::Log::INFO)
    , log_dir_("./logs")
    , log_to_console_(false)
    , use_colored_log_(true)
    , rollover_interval_(3600.0)
    , last_rollover_time_(0) // 0으로 초기화 후 Initialize()에서 현재 시간으로 설정
{
    Initialize();
}

RosoutLogger::~RosoutLogger()
{
    Shutdown();
}

void RosoutLogger::Initialize()
{
    // 파라미터 로드
    std::vector<std::string> default_nodes;
    this->declare_parameter("target_nodes", default_nodes);
    this->declare_parameter("log_level", rcl_interfaces::msg::Log::INFO);
    this->declare_parameter("log_dir", ament_index_cpp::get_package_share_directory("aeirobot_toolbox") + "/logs");
    this->declare_parameter("log_to_console", false);
    this->declare_parameter("use_colored_log", true);
    this->declare_parameter("rollover_interval", 3600.0);
    this->declare_parameter("config_file_path", "");

    target_nodes_ = this->get_parameter("target_nodes").as_string_array();
    log_level_filter_ = this->get_parameter("log_level").as_int();
    log_dir_ = this->get_parameter("log_dir").as_string();
    log_to_console_ = this->get_parameter("log_to_console").as_bool();
    use_colored_log_ = this->get_parameter("use_colored_log").as_bool();
    rollover_interval_ = this->get_parameter("rollover_interval").as_double();
    
    // 현재 시간으로 last_rollover_time_ 초기화
    last_rollover_time_ = this->now();
    
    // YAML 설정 파일 로드 및 노드-그룹 맵핑 구성
    LoadNodeGroupMapping();

    // 로그 디렉토리 생성
    struct stat st;
    memset(&st, 0, sizeof(st));
    if (stat(log_dir_.c_str(), &st) == -1)
    {
        mkdir(log_dir_.c_str(), 0700);
    }

    // 로그 파일 초기화
    OpenLogFiles();

    // 구독 시작
    log_sub_ = this->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", 1000,
        std::bind(&RosoutLogger::LogCallback, this, std::placeholders::_1));

    last_rollover_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "RosoutLogger initialized. Logging %zu nodes to %s",
             target_nodes_.size(), log_dir_.c_str());
}

void RosoutLogger::Shutdown()
{
    // ROS2에서는 구독자가 자동으로 정리됩니다
    CloseLogFiles();
}

void RosoutLogger::LogCallback(const rcl_interfaces::msg::Log::SharedPtr msg)
{
    // 로그 레벨 필터링
    if (msg->level < log_level_filter_)
    {
        return;
    }

    // 노드 필터링
    if (!IsNodeAllowed(msg->name))
    {
        return;
    }

    // 로그 롤오버 확인
    CheckLogFileRollover();

    // 현재 시스템 시간 가져오기
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&in_time_t);

    // 밀리초 계산
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch() % std::chrono::seconds(1)).count();

    // 시스템 시간 포맷팅
    std::stringstream system_time_ss;
    system_time_ss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S") << '.'
                  << std::setfill('0') << std::setw(3) << milliseconds;

    // ROS 타임스탬프 포맷팅
    std::stringstream ros_timestamp_ss;
    ros_timestamp_ss << std::fixed << std::setprecision(3)
                << rclcpp::Time(msg->stamp).seconds();

    // 로그 메시지 구성
    std::stringstream log_ss;
    log_ss << "[" << system_time_ss.str() << "] [ROS:" << ros_timestamp_ss.str() << "] ["
           << GetLogLevelString(msg->level) << "] "
           << msg->msg;

    // 노드의 로그 파일 가져오기 (없으면 새로 생성)
    std::ofstream& log_file = GetLogFileForNode(msg->name);
    
    // 로그 파일에 기록
    if (log_file.is_open())
    {
        log_file << log_ss.str() << std::endl;
        log_file.flush();
    }

    // 콘솔 출력 (옵션)
    if (log_to_console_)
    {
        std::cout << "[" << msg->name << "] " << log_ss.str() << std::endl;
    }
}

void RosoutLogger::OpenLogFiles()
{
    std::lock_guard<std::mutex> lock(file_mutex_);

    // 현재 시간을 디렉토리 및 파일 이름에 포함
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    std::string timestamp_str = ss.str();

    // ROS2 워크스페이스 루트 찾기
    std::string workspace_dir = GetROS2WorkspaceRoot();
    
    // 로그 디렉토리 구성 (workspace/log/YYYYMMDD_HHMMSS)
    std::string log_dir = workspace_dir + "/log/" + timestamp_str;
    
    // 디렉토리 생성
    try {
        std::filesystem::create_directories(log_dir);
        RCLCPP_INFO(this->get_logger(), "Created log directory: %s", log_dir.c_str());
    } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create log directory: %s - %s", 
                     log_dir.c_str(), e.what());
        return;
    }
    
    // 로그 디렉토리 업데이트
    log_dir_ = log_dir;
    
    // 노드별 그룹 분류 (사전에 그룹별 디렉토리만 생성)
    RCLCPP_INFO(this->get_logger(), "Preparing log structure for %zu target nodes", target_nodes_.size());
    
    // 그룹 리스트 생성 (그룹별 디렉토리만 미리 생성)
    std::set<std::string> unique_groups;
    for (const auto& node_name : target_nodes_)
    {
        std::string group_name = GetNodeGroup(node_name);
        unique_groups.insert(group_name);
    }
    
    // 그룹별 디렉토리만 미리 생성 (파일은 로그 입력 시 생성)
    for (const auto& group_name : unique_groups)
    {
        // 그룹별 디렉토리 생성
        std::string group_dir = log_dir_ + "/" + group_name;
        try {
            std::filesystem::create_directories(group_dir);
            RCLCPP_INFO(this->get_logger(), "Created group directory: %s", group_dir.c_str());
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create group directory: %s - %s", 
                         group_dir.c_str(), e.what());
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Log directories prepared. Files will be created when logs are received.");
} // OpenLogFiles 함수 끝

// 노드별 로그 파일을 열거나 이미 열려있으면 반환하는 함수
std::ofstream& RosoutLogger::GetLogFileForNode(const std::string& node_name)
{
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    // 이미 해당 노드의 로그 파일이 열려 있으면 반환
    auto it = log_files_.find(node_name);
    if (it != log_files_.end() && it->second.is_open())
    {
        return it->second;
    }
    
    // 새로 파일을 열어야 함
    std::string group_name = GetNodeGroup(node_name);
    std::string group_dir = log_dir_ + "/" + group_name;
    
    // 그룹 디렉토리가 없으면 생성
    if (!std::filesystem::exists(group_dir))
    {
        try {
            std::filesystem::create_directories(group_dir);
            RCLCPP_INFO(this->get_logger(), "Created new group directory: %s", group_dir.c_str());
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create group directory: %s - %s", 
                        group_dir.c_str(), e.what());
            // 오류 시 빈 참조 반환 방지를 위한 정적 로거 파일 생성
            static std::ofstream null_stream;
            return null_stream;
        }
    }
    
    // 로그 파일 경로
    std::string filename = group_dir + "/" + node_name + ".log";
    
    // 파일 열기
    log_files_[node_name].open(filename, std::ios::out | std::ios::app);
    
    if (!log_files_[node_name].is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open log file for node %s: %s",
                    node_name.c_str(), filename.c_str());
        // 오류 시 빈 참조 반환 방지를 위한 정적 로거 파일 생성
        static std::ofstream null_stream;
        return null_stream;
    }
    
    // 파일 초기화 메시지 기록
    RCLCPP_INFO(this->get_logger(), "Successfully opened log file: %s", filename.c_str());
    std::stringstream timestamp;
    std::time_t current_time = std::time(nullptr);
    timestamp << std::put_time(std::localtime(&current_time), "%Y-%m-%d %H:%M:%S");
    log_files_[node_name] << "[" << node_name << "] Log file initialized at " << 
                    timestamp.str() << std::endl;
    
    return log_files_[node_name];
}

void RosoutLogger::CloseLogFiles()
{
    std::lock_guard<std::mutex> lock(file_mutex_);

    for (auto& file_pair : log_files_)
    {
        if (file_pair.second.is_open())
        {
            file_pair.second.close();
        }
    }

    log_files_.clear();
}

void RosoutLogger::CheckLogFileRollover()
{
    rclcpp::Time current_time = this->now();

    // 롤오버 간격 체크
    if ((current_time - last_rollover_time_).seconds() >= rollover_interval_)
    {
        RCLCPP_INFO(this->get_logger(), "Log file rollover triggered");

        CloseLogFiles();
        OpenLogFiles();

        last_rollover_time_ = current_time;
    }
}

bool RosoutLogger::IsNodeAllowed(const std::string& node_name)
{
    // 타겟 노드가 비어있으면 모든 노드 허용
    if (target_nodes_.empty())
    {
        return true;
    }

    // 노드 이름 검색
    for (const auto& target : target_nodes_)
    {
        if (node_name == target || node_name.find(target) != std::string::npos)
        {
            return true;
        }
    }

    return false;
}

std::string RosoutLogger::GetLogLevelString(int8_t level)
{
    if (!use_colored_log_)
    {
        // 색상 없이 일반 텍스트로 반환
        switch (level)
        {
            case rcl_interfaces::msg::Log::DEBUG:
                return "DEBUG";
            case rcl_interfaces::msg::Log::INFO:
                return "INFO";
            case rcl_interfaces::msg::Log::WARN:
                return "WARN";
            case rcl_interfaces::msg::Log::ERROR:
                return "ERROR";
            case rcl_interfaces::msg::Log::FATAL:
                return "FATAL";
            default:
                return "UNKNOWN";
        }
    }
    else
    {
        // 색상이 있는 텍스트로 반환 - basic_tools.hpp의 PRINT_COLOR 사용
        std::stringstream ss;
        switch (level)
        {
            case rcl_interfaces::msg::Log::DEBUG:
                ss << aeirobot::PRINT_COLOR::CYAN << "DEBUG" << aeirobot::PRINT_COLOR::ENDCOLOR;
                break;
            case rcl_interfaces::msg::Log::INFO:
                ss << aeirobot::PRINT_COLOR::GREEN << "INFO" << aeirobot::PRINT_COLOR::ENDCOLOR;
                break;
            case rcl_interfaces::msg::Log::WARN:
                ss << aeirobot::PRINT_COLOR::YELLOW << "WARN" << aeirobot::PRINT_COLOR::ENDCOLOR;
                break;
            case rcl_interfaces::msg::Log::ERROR:
                ss << aeirobot::PRINT_COLOR::RED << "ERROR" << aeirobot::PRINT_COLOR::ENDCOLOR;
                break;
            case rcl_interfaces::msg::Log::FATAL:
                ss << aeirobot::PRINT_COLOR::MAGENTA << "FATAL" << aeirobot::PRINT_COLOR::ENDCOLOR;
                break;
            default:
                ss << aeirobot::PRINT_COLOR::WHITE << "UNKNOWN" << aeirobot::PRINT_COLOR::ENDCOLOR;
                break;
        }
        return ss.str();
    }
}

std::string RosoutLogger::GetROS2WorkspaceRoot()
{
    // 1. 현재 실행 파일 경로 확인
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    std::string exec_path = std::string(result, (count > 0) ? count : 0);
    
    // 경로에서 install/bin 부분 찾기
    size_t pos = exec_path.find("/install/");
    if (pos != std::string::npos) {
        // install 디렉토리 위가 워크스페이스 루트
        return exec_path.substr(0, pos);
    }
    
    // 2. 현재 디렉토리 기준으로 상위 경로 찾기
    std::filesystem::path current_path = std::filesystem::current_path();
    while (!current_path.empty()) {
        if (std::filesystem::exists(current_path / "src") && 
            std::filesystem::exists(current_path / "build") &&
            std::filesystem::exists(current_path / "install")) {
            return current_path.string();
        }
        current_path = current_path.parent_path();
    }
    
    // 3. ROS_WORKSPACE 환경 변수 확인
    const char* ros_ws = std::getenv("ROS_WORKSPACE");
    if (ros_ws != nullptr) {
        return ros_ws;
    }
    
    // 4. 현재 작업 디렉토리 기본 사용
    RCLCPP_WARN(this->get_logger(), "Failed to detect ROS2 workspace root. Using current directory.");
    return std::filesystem::current_path().string();
}

} // aeirobot_toolbox

// 노드 진입점
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<aeirobot_toolbox::RosoutLogger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
