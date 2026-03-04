#include "edie_behavior/action_nodes.hpp"

namespace aeirobot
{

BT::NodeStatus Docking::onStart()
{
    auto edie = Edie::GetInstance();
    
    auto dock_id_opt = getInput<std::string>("dock_id");
    std::string dock_id = dock_id_opt.value_or("charging_station_1");
    
    // 현재 상태 확인
    if (edie->docking_status_ == Edie::DockingStatus::k_idle) {
        // 시작전: docking 시작
        RCLCPP_INFO(edie->ros_manager->get_logger(), "Starting docking for dock: %s", dock_id.c_str());
        
        // docking 상태를 진행중으로 변경
        edie->docking_status_ = Edie::DockingStatus::k_running;
        
        // 토픽 발행
        edie->PubStartDocking(dock_id);
        
        return BT::NodeStatus::RUNNING;
    }
    
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Docking::onRunning()
{
    auto edie = Edie::GetInstance();
    
    // 현재 상태 확인
    if (edie->docking_status_ == Edie::DockingStatus::k_running) {
        // 진행중: 계속 대기
        return BT::NodeStatus::RUNNING;
    }
    else if (edie->docking_status_ == Edie::DockingStatus::k_success) {
        // 성공: 상태 리셋 후 성공 반환
        edie->docking_status_ = Edie::DockingStatus::k_idle;
        RCLCPP_INFO(edie->ros_manager->get_logger(), "Docking completed successfully!");
        return BT::NodeStatus::SUCCESS;
    }
    else if (edie->docking_status_ == Edie::DockingStatus::k_failed) {
        // 실패: 상태 리셋 후 실패 반환
        edie->docking_status_ = Edie::DockingStatus::k_idle;
        RCLCPP_ERROR(edie->ros_manager->get_logger(), "Docking failed!");
        return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::RUNNING;
}

void Docking::onHalted()
{
    auto edie = Edie::GetInstance();
    
    // 중단되었을 때 상태 리셋
    if (edie->docking_status_ == Edie::DockingStatus::k_running) {
        edie->docking_status_ = Edie::DockingStatus::k_idle;
        RCLCPP_WARN(edie->ros_manager->get_logger(), "Docking was halted!");
    }
}

} // namespace aeirobot
