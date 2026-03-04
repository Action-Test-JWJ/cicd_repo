#ifndef AEIROBOT_TOOLBOX_EULER_ANGLE_HPP
#define AEIROBOT_TOOLBOX_EULER_ANGLE_HPP

#include <vector>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <aeirobot_msgs/msg/pose_xyzrpy.hpp>
#include "aeirobot_toolbox/vector3.hpp"

namespace aeirobot
{

class EulerAngle {
public:
  double roll = 0.0; // Roll angle in radians
  double pitch = 0.0; // Pitch angle in radians
  double yaw = 0.0; // Yaw angle in radians

  EulerAngle() = default;
  EulerAngle(double r, double p, double y) : roll(r), pitch(p), yaw(y) {}
  EulerAngle(const std::vector<double> &angles)
  {
    if (angles.size() >= 3)
    {
      roll = angles[0];
      pitch = angles[1];
      yaw = angles[2];
    }
  }

  EulerAngle(const geometry_msgs::msg::Quaternion &q)
  {
    *this = FromQuaternion(q);
  }

  EulerAngle(const geometry_msgs::msg::Pose &pose)
  {
    *this = FromQuaternion(pose.orientation);
  }

  EulerAngle(const geometry_msgs::msg::Pose2D &pose)
  {
    roll = 0.0; pitch = 0.0; yaw = pose.theta;
  }

  EulerAngle& operator=(const std::vector<double>& angles)
  {
    if (angles.size() >= 3)
    {
      roll = angles[0]; 
      pitch = angles[1]; 
      yaw = angles[2];
    }
    else
    {
      roll = pitch = yaw = 0.0;
    }
    return *this;
  }

  EulerAngle& operator=(const Vector3 &vec)
  {
    roll = vec.x; 
    pitch = vec.y; 
    yaw = vec.z;
    return *this;
  }

  EulerAngle& operator=(const aeirobot_msgs::msg::PoseXYZRPY &pose)
  {
    roll = pose.roll;
    pitch = pose.pitch;
    yaw = pose.yaw;
    return *this;
  }

  EulerAngle& operator=(const geometry_msgs::msg::Quaternion& q)
  {
    *this = FromQuaternion(q);
    return *this;
  }

  static EulerAngle FromQuaternion(const geometry_msgs::msg::Quaternion &q)
  {
    EulerAngle ea;
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);

    ea.roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                         1.0 - 2.0 * (q.x * q.x + q.y * q.y));

    ea.pitch = (std::abs(sinp) >= 1.0)
                 ? std::copysign(M_PI / 2, sinp)
                 : std::asin(sinp);

    ea.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return ea;
  }
};

} // namespace aeirobot

#endif // AEIROBOT_TOOLBOX_EULER_ANGLE_HPP