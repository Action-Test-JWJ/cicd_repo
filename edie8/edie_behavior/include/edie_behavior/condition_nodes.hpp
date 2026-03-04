#ifndef CONDITION_NODES_HPP
#define CONDITION_NODES_HPP

#include <string>
#include <behaviortree_cpp/condition_node.h>

#include "edie_behavior/edie.hpp"
// #include "edie_behavior/edie_behavior_node.hpp"
// #include "edie_behavior/singleton.hpp"

using namespace std;

namespace aeirobot
{

  class Edie;

  class IsMotorReady : public BT::ConditionNode
  {
  public:
    IsMotorReady(const string &name) 
    : BT::ConditionNode(name, {}) {}
    BT::NodeStatus tick();
  };


  class IsReadyToAction : public BT::ConditionNode
  {
  public:
    IsReadyToAction(const string &name) 
    : BT::ConditionNode(name, {}) {}
    BT::NodeStatus tick();
  };

  class IsRobotMode : public BT::ConditionNode
  {
  public:
    IsRobotMode(const string &name, const BT::NodeConfig &config) 
    : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick();
  };

  class IsEmergencyDetected : public BT::ConditionNode
  {
  public:
    IsEmergencyDetected(const string &name) 
    : BT::ConditionNode(name, {}) {}
    BT::NodeStatus tick();
  };

  class IsBatteryChargedAtLeast : public BT::ConditionNode
  {
  public:
    IsBatteryChargedAtLeast(const string &name, const BT::NodeConfig &config) 
    : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick();
  };

  class IsInteractionTargetDetected : public BT::ConditionNode
  {
  public:
    IsInteractionTargetDetected(const string &name) 
    : BT::ConditionNode(name, {}) {}
    BT::NodeStatus tick();
  };

  class IsNavigationDone : public BT::ConditionNode
  {
  public:
  IsNavigationDone(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { 
      BT::InputPort<std::string>("state_key"),
      BT::InputPort<std::string>("state_value")
    };
  }
  
  BT::NodeStatus tick() override;
  };

  class IsPoseCorrectionDone : public BT::ConditionNode
  {
  public:
    IsPoseCorrectionDone(const string &name) 
    : BT::ConditionNode(name, {}) {}
    BT::NodeStatus tick();
  };
  class IsArucoVisible : public BT::ConditionNode
  {
  public:
    IsArucoVisible(const std::string &name) 
    : BT::ConditionNode(name, {}) {}
    BT::NodeStatus tick() override;
  };
  
  class IsStationArrived : public BT::ConditionNode
  {
  public:
    IsStationArrived(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
      return{ BT::InputPort<std::string>("status") };
    }

    BT::NodeStatus tick() override;
  };

class IsHomeInitDone : public BT::ConditionNode
{
public:
  IsHomeInitDone(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

class IsRealMiddle : public BT::ConditionNode
{
public:
  IsRealMiddle(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<double>("y_thresh"),
      BT::InputPort<double>("yaw_thresh"),
      BT::InputPort<double>("recent_sec")
    };
  }

  BT::NodeStatus tick() override;
};

class IsScanResultReceived : public BT::ConditionNode
{
public:
  IsScanResultReceived(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

class IsRobotInArea : public BT::ConditionNode
{
public:
  IsRobotInArea(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("area") };
  }

  BT::NodeStatus tick() override;
};

class IsCharging : public BT::ConditionNode
{
public:
  IsCharging(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class IsSTTListening : public BT::ConditionNode
{
public:
  IsSTTListening(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class IsLLMState : public BT::ConditionNode
{
public:
  IsLLMState(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class IsVoraMode : public BT::ConditionNode
{
public:
  IsVoraMode(const std::string &name, const BT::NodeConfig &config) 
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("mode") };
  }

  BT::NodeStatus tick() override;
};

class IsFaceDetectionActive : public BT::StatefulActionNode
{
public:
  IsFaceDetectionActive(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("timeout_sec", 10.0, "Timeout in seconds") };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Time start_time_;
};

class IsInfrontWithTarget : public BT::ConditionNode
{
public:
  IsInfrontWithTarget(const std::string &name, const BT::NodeConfig &config) 
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

class IsRandomWalkNeeded : public BT::ConditionNode
{
public:
  IsRandomWalkNeeded(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class IsHumanVisible : public BT::ConditionNode
{
public:
  IsHumanVisible(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class IsWaitingForTouch : public BT::ConditionNode
{
public:
  IsWaitingForTouch(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class IsInInteractionMode : public BT::ConditionNode
{
public:
  IsInInteractionMode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class IsAngryRunaway : public BT::ConditionNode
{
public:
  IsAngryRunaway(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class IsStationOutValue : public BT::ConditionNode
{
public:
  IsStationOutValue(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("value", "Expected station_out value (e.g., 3 for A button, 4 for Y button)") };
  }

  BT::NodeStatus tick() override;
};

} // namespace aeirobot

#endif // CONDITION_NODES_HPP