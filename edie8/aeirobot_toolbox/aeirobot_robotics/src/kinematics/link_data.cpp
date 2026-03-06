#include "aeirobot_robotics/kinematics/link_data.h"

namespace aeirobot
{
  LinkData::LinkData()
  {
    name_ = "";

    parent_ = -1;
    sibling_ = -1;
    child_ = -1;

    mass_ = 0.0;

    relative_position_ = aeirobot::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_axis_ = aeirobot::getTransitionXYZ(0.0, 0.0, 0.0);
    center_of_mass_ = aeirobot::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_center_of_mass_ = aeirobot::getTransitionXYZ(0.0, 0.0, 0.0);
    inertia_ = aeirobot::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_limit_max_ = 100.0;
    joint_limit_min_ = -100.0;

    joint_angle_ = 0.0;
    joint_velocity_ = 0.0;
    joint_acceleration_ = 0.0;

    position_ = aeirobot::getTransitionXYZ(0.0, 0.0, 0.0);
    orientation_ = aeirobot::convertRPYToRotation(0.0, 0.0, 0.0);
    transformation_ = aeirobot::getTransformationXYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  LinkData::~LinkData() {}
}