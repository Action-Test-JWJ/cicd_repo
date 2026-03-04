#ifndef ACTION_NODES_HPP
#define ACTION_NODES_HPP

#include <string>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <vector>

#include "edie_behavior/edie.hpp"
// #include "edie_behavior/edie_behavior_node.hpp"
// #include "edie_behavior/singleton.hpp"

using namespace std;

namespace aeirobot
{
  
  class Edie;

  #pragma region SyncActionNodes

  // class SetAction : public BT::SyncActionNode
  // {
  // public:
  //   SetAction(const string &name, const BT::NodeConfig &config) 
  //   : BT::SyncActionNode(name, config){}
  //   static BT::PortsList providedPorts();
  //   BT::NodeStatus tick();
  // };

  // class SetState : public BT::SyncActionNode
  // {
  // public:
  //   SetState(const string &name, const BT::NodeConfig &config) 
  //   : BT::SyncActionNode(name, config){}
  //   static BT::PortsList providedPorts();
  //   BT::NodeStatus tick();
  // };

  #pragma endregion // SyncActionNodes

  #pragma region StatefulActionNodes

  class RemoteControlPlay : public BT::StatefulActionNode
  {
  public:
    RemoteControlPlay(const string &name)
        : BT::StatefulActionNode(name, {}) {};
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
  };

  class MoveTo : public BT::StatefulActionNode
  {
  public:
    MoveTo(const string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config) {};
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
  };

  class Explore : public BT::StatefulActionNode
  {
  public:
    Explore(const string &name)
        : BT::StatefulActionNode(name, {}) {};
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
  };

  class TryCharge : public BT::StatefulActionNode
  {
  public:
    TryCharge(const string &name)
        : BT::StatefulActionNode(name, {}) {};
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
  };

  class TryInteract : public BT::StatefulActionNode
  {
  public:
    TryInteract(const string &name)
        : BT::StatefulActionNode(name, {}) {};
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
  };

  class SetHomeInitDone : public BT::SyncActionNode
  {
  public:
    SetHomeInitDone(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
      return {};
    }

    BT::NodeStatus tick() override;
  };

  class SendEmergencySignal : public BT::StatefulActionNode
  {
  public:
    SendEmergencySignal(const string &name)
        : BT::StatefulActionNode(name, {}) {};
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
  };

  class TryNavigation : public BT::StatefulActionNode
  {
  public:
    TryNavigation(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
      return { 
        BT::InputPort<uint8_t>("command"),
        BT::InputPort<std::string>("status_key")
      };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
  };

  class Docking : public BT::StatefulActionNode
  {
  public:
    Docking(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
      return { 
        BT::InputPort<std::string>("dock_id")
      };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
  };

  class SendScanCommand : public BT::SyncActionNode
  {
  public:
    SendScanCommand(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
      return{ BT::InputPort<bool>("command") };
    }

    BT::NodeStatus tick() override;
  };

  class FindClosestMarker : public BT::SyncActionNode
  {
  public:
    FindClosestMarker(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
      // Add an output port to write the marker ID to the blackboard
      return{ BT::OutputPort<std::string>("found_marker_id") };
    }

    BT::NodeStatus tick() override;
  };

  class SetTargetMarkerId : public BT::SyncActionNode
  {
  public:
    SetTargetMarkerId(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
      return{ BT::InputPort<std::string>("marker_id") };
    }

    BT::NodeStatus tick() override;
  };

  class CollectArucoPoseData : public BT::StatefulActionNode
  {
  public:
    CollectArucoPoseData(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>("sample_count") };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    int target_count_;
    std::vector<geometry_msgs::msg::Pose> collected_poses_;
    geometry_msgs::msg::Pose computeAveragedPose();
    std::vector<double> removeOutliers(const std::vector<double>& data);
    double computeMean(const std::vector<double>& data);
  };

  class SwitchVoraMode : public BT::SyncActionNode
  {
  public:
    SwitchVoraMode(const std::string &name, const BT::NodeConfig &config) 
    : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<std::string>("mode") };
    }

    BT::NodeStatus tick() override;
  };

  // Add this enum for clarity
  enum class TouchedArea { NONE, HEAD, RIGHT_BUTT, SIDE };

  // Modify ProcessFsrTouch to be a SyncActionNode with state
  class ProcessFsrTouch : public BT::SyncActionNode
  {
  public:
    ProcessFsrTouch(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    // State variables to remember the last touch
    TouchedArea _last_touched_area{TouchedArea::NONE};
    size_t _current_sequence_index{0};
  };

  class WaitAction : public BT::StatefulActionNode
  {
  public:
    WaitAction(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::chrono::steady_clock::time_point deadline_;
  };

  // Plays a specific emotion action and waits for completion.
  class PlayEmotion : public BT::StatefulActionNode
  {
  public:
    PlayEmotion(const std::string& name, const BT::NodeConfiguration& config);
  
    static BT::PortsList providedPorts();
  
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
  };

  class StareAtTarget : public BT::StatefulActionNode
  {
  public:
    StareAtTarget(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    rclcpp::Time start_time_;
    double duration_sec_;
  };

  class SetRandomWalkFlag : public BT::SyncActionNode
  {
  public:
    SetRandomWalkFlag(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
  };

  class ResetWaitingStatus : public BT::SyncActionNode
  {
  public:
    ResetWaitingStatus(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
  };

  class MoveAwayFromTarget : public BT::StatefulActionNode
  {
  public:
    MoveAwayFromTarget(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    rclcpp::Time start_time_;

    // "회전 -> 전진" 상태 관리를 위한 변수 추가
    enum class Phase { TURNING, MOVING_FORWARD };
    Phase current_phase_;
    double turn_duration_sec_;
    double turn_velocity_;
    double forward_duration_sec_;
    double forward_velocity_; // 전진 속도 멤버 변수 추가
  };

  class SetWaitDuration : public BT::SyncActionNode
  {
  public:
    SetWaitDuration(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
  };

  class ResetAngryRunaway : public BT::SyncActionNode
  {
  public:
    ResetAngryRunaway(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
  };


  #pragma endregion // StatefulActionNodes

} // namespace aeirobot

#endif // ACTION_NODES_HPP