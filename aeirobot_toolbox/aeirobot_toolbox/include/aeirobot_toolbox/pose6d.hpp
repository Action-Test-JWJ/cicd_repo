#ifndef AEIROBOT_TOOLBOX_POSE6D_HPP
#define AEIROBOT_TOOLBOX_POSE6D_HPP

#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <aeirobot_msgs/msg/pose_xyzrpy.hpp>
#include "aeirobot_toolbox/vector3.hpp"
#include "aeirobot_toolbox/euler_angle.hpp"

namespace aeirobot
{

  class Pose6D
  {
  public:
    Vector3 position;
    EulerAngle angle;

    Pose6D() = default;
    Pose6D(const Vector3 &pos, const EulerAngle &ang) : position(pos), angle(ang) {}
    Pose6D(const geometry_msgs::msg::Pose &pose)
    {
      position.x = pose.position.x;
      position.y = pose.position.y;
      position.z = pose.position.z;

      Eigen::Quaterniond q(
          pose.orientation.w,
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z);

      Eigen::Matrix3d R = q.normalized().toRotationMatrix();

      // 오일러앵글 추출 (Roll-Pitch-Yaw, ZYX 순서)
      Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);
      // euler[0] = yaw (Z), euler[1] = pitch (Y), euler[2] = roll (X)

      angle.roll = euler[2];
      angle.pitch = euler[1];
      angle.yaw = euler[0];
    }
    Pose6D(const geometry_msgs::msg::Pose2D &pose)
    {
      position.x = pose.x;
      position.y = pose.y;
      // position.z = 0.0; // Z is not used in Pose2D
      // angle.roll = 0.0; // Roll is not used in Pose2D
      // angle.pitch = 0.0; // Pitch is not used in Pose2D
      angle.yaw = pose.theta; // Yaw is used as theta in Pose2D
    }
    Pose6D(const aeirobot_msgs::msg::PoseXYZRPY &pose)
    {
      position.x = pose.x;
      position.y = pose.y;
      position.z = pose.z;
      angle.roll = pose.roll;
      angle.pitch = pose.pitch;
      angle.yaw = pose.yaw;
    }
    Pose6D(const std::vector<double> &vec)
    {
      if (vec.size() >= 6)
      {
        position.x = vec[0];
        position.y = vec[1];
        position.z = vec[2];
        angle.roll = vec[3];
        angle.pitch = vec[4];
        angle.yaw = vec[5];
      }
    }
    Pose6D &operator=(const geometry_msgs::msg::Pose &pose)
    {
      position.x = pose.position.x;
      position.y = pose.position.y;
      position.z = pose.position.z;

      Eigen::Quaterniond q(
          pose.orientation.w,
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z);

      Eigen::Matrix3d R = q.normalized().toRotationMatrix();

      // 오일러각 추출 (Yaw-Pitch-Roll, ZYX 순서)
      Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);
      // euler[0] = yaw(Z), euler[1] = pitch(Y), euler[2] = roll(X)

      angle.roll = euler[2];
      angle.pitch = euler[1];
      angle.yaw = euler[0];

      return *this;
    }
    Pose6D &operator=(const geometry_msgs::msg::Pose2D &pose)
    {
      position.x = pose.x;
      position.y = pose.y;
      // position.z = 0.0; // Z is not used in Pose2D
      // angle.roll = 0.0; // Roll is not used in Pose2D
      // angle.pitch = 0.0; // Pitch is not used in Pose2D
      angle.yaw = pose.theta; // Yaw is used as theta in Pose2D
      return *this;
    }
    Pose6D &operator=(const aeirobot_msgs::msg::PoseXYZRPY &pose)
    {
      position.x = pose.x;
      position.y = pose.y;
      position.z = pose.z;
      angle.roll = pose.roll;
      angle.pitch = pose.pitch;
      angle.yaw = pose.yaw;
      return *this;
    }
    Pose6D &operator=(const std::vector<double> &vec)
    {
      if (vec.size() >= 6)
      {
        position.x = vec[0];
        position.y = vec[1];
        position.z = vec[2];
        angle.roll = vec[3];
        angle.pitch = vec[4];
        angle.yaw = vec[5];
      }
      return *this;
    }
    geometry_msgs::msg::Pose ToPoseMsg() const
    {
      geometry_msgs::msg::Pose pose;

      pose.position.x = position.x;
      pose.position.y = position.y;
      pose.position.z = position.z;

      // RPY -> Quaternion 변환
      Eigen::AngleAxisd roll_angle(angle.roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitch_angle(angle.pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yaw_angle(angle.yaw, Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;

      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();

      return pose;
    }
    geometry_msgs::msg::Pose2D ToPose2DMsg() const
    {
      geometry_msgs::msg::Pose2D pose2d;
      pose2d.x = position.x;
      pose2d.y = position.y;
      pose2d.theta = angle.yaw; // Yaw is used as theta in Pose2D
      return pose2d;
    }
    aeirobot_msgs::msg::PoseXYZRPY ToPoseXYZRPYMsg() const
    {
      aeirobot_msgs::msg::PoseXYZRPY pose_xyzrpy;
      pose_xyzrpy.x = position.x;
      pose_xyzrpy.y = position.y;
      pose_xyzrpy.z = position.z;
      pose_xyzrpy.roll = angle.roll;
      pose_xyzrpy.pitch = angle.pitch;
      pose_xyzrpy.yaw = angle.yaw;
      return pose_xyzrpy;
    }
    geometry_msgs::msg::Vector3 AngleToVector3Msg() const
    {
      geometry_msgs::msg::Vector3 vector3;
      vector3.x = angle.roll;
      vector3.y = angle.pitch;
      vector3.z = angle.yaw;
      return vector3;
    }
    std::vector<double> ToVector() const
    {
      return {position.x, position.y, position.z, angle.roll, angle.pitch, angle.yaw};
    }
    std::vector<double> ToVector2D() const
    {
      return {position.x, position.y, angle.yaw}; // Z, roll, and pitch are not used in 2D
    }
    bool operator==(const Pose6D &other) const
    {
      return (position.x == other.position.x &&
              position.y == other.position.y &&
              position.z == other.position.z &&
              angle.roll == other.angle.roll &&
              angle.pitch == other.angle.pitch &&
              angle.yaw == other.angle.yaw);
    }
    bool operator!=(const Pose6D &other) const
    {
      return !(*this == other);
    }
    bool IsValid() const
    {
      return (std::isfinite(position.x) && std::isfinite(position.y) &&
              std::isfinite(position.z) && std::isfinite(angle.roll) &&
              std::isfinite(angle.pitch) && std::isfinite(angle.yaw));
    }
  };

} // namespace aeirobot

#endif // AEIROBOT_TOOLBOX_POSE6D_HPP
