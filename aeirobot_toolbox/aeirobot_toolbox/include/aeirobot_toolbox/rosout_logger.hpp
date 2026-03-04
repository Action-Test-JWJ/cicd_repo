#ifndef AEIROBOT_ROSOUT_LOGGER_HPP
#define AEIROBOT_ROSOUT_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include "aeirobot_toolbox/basic_tools.hpp"  // 색상 코드를 위한 헬퍼

namespace aeirobot_toolbox
{

class RosoutLogger : public rclcpp::Node
{
public:
    RosoutLogger();
    ~RosoutLogger();

    void Initialize();
    void Shutdown();

private:
    // 로그 메시지 콜백 함수
    void LogCallback(const rcl_interfaces::msg::Log::SharedPtr msg);
    
    // 로그 파일 관리 함수
    void OpenLogFiles();
    void CloseLogFiles();
    void CheckLogFileRollover();
    
    // 노드 이름이 허용된 노드 목록에 있는지 확인
    bool IsNodeAllowed(const std::string& node_name);
    
    // ROS2 워크스페이스 루트 디렉토리 경로 획득 함수
    std::string GetROS2WorkspaceRoot();
    
    // 로그 레벨 문자열 변환 함수
    std::string GetLogLevelString(int8_t level);
    
    // 노드의 로그 파일 가져오기 (없으면 생성)
    std::ofstream& GetLogFileForNode(const std::string& node_name);

    // 노드-그룹 맵핑 관련 함수
    void LoadNodeGroupMapping();
    std::string GetNodeGroup(const std::string& node_name);

    // ROS2 관련 멤버 변수
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;
    
    // 설정 관련 변수
    std::vector<std::string> target_nodes_;
    int log_level_filter_;
    std::string log_dir_;
    
    // 노드-그룹 맵핑 변수
    std::map<std::string, std::string> node_to_group_map_; // 노드 이름 -> 그룹 이름 맵핑
    bool log_to_console_;
    bool use_colored_log_;
    double rollover_interval_;
    rclcpp::Time last_rollover_time_;
    
    // 로그 파일 관련 변수
    std::map<std::string, std::ofstream> log_files_;
    std::mutex file_mutex_;
};

} // aeirobot_toolbox

#endif // AEIROBOT_ROSOUT_LOGGER_HPP
