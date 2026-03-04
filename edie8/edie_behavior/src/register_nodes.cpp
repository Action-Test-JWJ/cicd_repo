#include "edie_behavior/edie.hpp"
#include "edie_behavior/action_nodes.hpp"
#include "edie_behavior/condition_nodes.hpp"

using namespace std;

void aeirobot::Edie::RegisterBTNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<aeirobot::RemoteControlPlay>("RemoteControlPlay");
  factory.registerNodeType<aeirobot::MoveTo>("MoveTo");
  factory.registerNodeType<aeirobot::Explore>("Explore");  // 이후에는 서브트리로 구현하는게 맞을듯
  factory.registerNodeType<aeirobot::TryCharge>("TryCharge");
  factory.registerNodeType<aeirobot::TryInteract>("TryInteract");
  factory.registerNodeType<aeirobot::SendEmergencySignal>("SendEmergencySignal");
  factory.registerNodeType<aeirobot::TryNavigation>("TryNavigation");
  factory.registerNodeType<aeirobot::SetHomeInitDone>("SetHomeInitDone");
  factory.registerNodeType<aeirobot::SendScanCommand>("SendScanCommand");
  factory.registerNodeType<aeirobot::FindClosestMarker>("FindClosestMarker");
  factory.registerNodeType<aeirobot::SetTargetMarkerId>("SetTargetMarkerId");
  factory.registerNodeType<aeirobot::Docking>("Docking");
  factory.registerNodeType<aeirobot::CollectArucoPoseData>("CollectArucoPoseData");
  factory.registerNodeType<aeirobot::SwitchVoraMode>("SwitchVoraMode");
  factory.registerNodeType<aeirobot::WaitAction>("WaitAction");
  factory.registerNodeType<aeirobot::StareAtTarget>("StareAtTarget");
  factory.registerNodeType<SetRandomWalkFlag>("SetRandomWalkFlag");
  factory.registerNodeType<ProcessFsrTouch>("ProcessFsrTouch"); 
  factory.registerNodeType<PlayEmotion>("PlayEmotion");
  factory.registerNodeType<IsWaitingForTouch>("IsWaitingForTouch");
  factory.registerNodeType<ResetWaitingStatus>("ResetWaitingStatus");
  factory.registerNodeType<MoveAwayFromTarget>("MoveAwayFromTarget");
  factory.registerNodeType<SetWaitDuration>("SetWaitDuration");
  factory.registerNodeType<ResetAngryRunaway>("ResetAngryRunaway");
  
  factory.registerNodeType<aeirobot::IsReadyToAction>("IsReadyToAction");
  factory.registerNodeType<aeirobot::IsMotorReady>("IsMotorReady");
  factory.registerNodeType<aeirobot::IsRobotMode>("IsRobotMode");
  factory.registerNodeType<aeirobot::IsHomeInitDone>("IsHomeInitDone");
  factory.registerNodeType<aeirobot::IsEmergencyDetected>("IsEmergencyDetected");
  factory.registerNodeType<aeirobot::IsBatteryChargedAtLeast>("IsBatteryChargedAtLeast");
  factory.registerNodeType<aeirobot::IsInteractionTargetDetected>("IsInteractionTargetDetected");
  factory.registerNodeType<aeirobot::IsLLMState>("IsLLMState");
  factory.registerNodeType<aeirobot::IsNavigationDone>("IsNavigationDone");
  factory.registerNodeType<aeirobot::IsPoseCorrectionDone>("IsPoseCorrectionDone");
  factory.registerNodeType<aeirobot::IsArucoVisible>("IsArucoVisible");
  factory.registerNodeType<aeirobot::IsRealMiddle>("IsRealMiddle");
  factory.registerNodeType<aeirobot::IsScanResultReceived>("IsScanResultReceived");
  factory.registerNodeType<aeirobot::IsRobotInArea>("IsRobotInArea");
  factory.registerNodeType<aeirobot::IsCharging>("IsCharging");
  factory.registerNodeType<aeirobot::IsSTTListening>("IsSTTListening");
  factory.registerNodeType<aeirobot::IsInfrontWithTarget>("IsInfrontWithTarget");
  factory.registerNodeType<aeirobot::IsVoraMode>("IsVoraMode");
  factory.registerNodeType<aeirobot::IsRandomWalkNeeded>("IsRandomWalkNeeded");
  factory.registerNodeType<aeirobot::IsFaceDetectionActive>("IsFaceDetectionActive");
  factory.registerNodeType<aeirobot::IsHumanVisible>("IsHumanVisible");
  factory.registerNodeType<aeirobot::IsInInteractionMode>("IsInInteractionMode");
  factory.registerNodeType<aeirobot::IsAngryRunaway>("IsAngryRunaway");
  factory.registerNodeType<aeirobot::IsStationOutValue>("IsStationOutValue");


}
