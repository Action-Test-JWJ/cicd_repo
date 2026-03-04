#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <string>
#include <chrono>

#include "rclcpp/time.hpp"

#include "edie_behavior/transform.hpp"

namespace aeirobot
{

class Object
{
public:
  std::string name;
  Transform transform;
  Vector3 velocity;
  rclcpp::Time last_saw;

  void SetParent(Object* p)
  {
    transform.SetParent(&(p->transform));
  }

  void SetParent(Transform* t)
  {
    transform.SetParent(t);
  }

  bool IsSeenWithin(std::chrono::duration<double> sec, const rclcpp::Time &now)
  {
    return ((last_saw + sec).seconds() > now.seconds()) ? true : false;
  }

  bool IsSeenWithin(double sec, const rclcpp::Time &now)
  {
    return IsSeenWithin(std::chrono::duration<double>(sec), now);
  }

  bool IsSeenWithin(int sec, const rclcpp::Time &now)
  {
    return IsSeenWithin(std::chrono::duration<double>((double)sec), now);
  }
};
  
} // namespace aeirobot

#endif