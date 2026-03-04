#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include <tf2/LinearMath/Vector3.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/vector3.hpp>

// #include <aeirobot_toolbox/vector3.hpp>
#include <aeirobot_toolbox/basic_tools.hpp>
#include <aeirobot_math/math_tool.hpp>

namespace aeirobot
{
enum Space
{
  k_local,
  k_global
};

class Transform
{
private:
  // Vector3 x,y means position x,y.
  // Vector3 z means yaw.
  Vector3 local_position;
  Vector3 position;

  std::shared_ptr<Transform> parent;
  Vector3 last_parent_pos;

public:
  // Transform()
  // {
  // };
  Transform(Transform *p = nullptr)
  {
    SetParent(p);
  };

  static std::shared_ptr<Transform> GetGround()
  {
    static std::shared_ptr<Transform> ground = std::make_shared<Transform>();
    return ground;
  }

  Vector3  GetPosition(const Space space = aeirobot::Space::k_global)
  {
    if(space == Space::k_global || parent == GetGround())
    {
      return position;
    }
    else
    {
      if(ParentChanged())
        GlobalToLocal();
      return local_position;
    }
  }

  void SetPosition(Transform &t)
  {
    SetPosition(t.GetPosition());
  }

  void SetPosition(const Vector3 &pos, const Space space = aeirobot::Space::k_global)
  {
    if(space == aeirobot::Space::k_global)
    {
      position = pos;
      GlobalToLocal();
    }
    else
    {
      local_position = pos;
      LocalToGlobal();
    }
  }

  void SetPosition(double x, double y, double z, Space space = aeirobot::Space::k_global)
  {
    SetPosition(Vector3(x, y, z), space);
  }

  void SetParent(Transform *p)
  {
    parent.reset(p);
    if(p == nullptr)
    {
      local_position = position;
    }
    else
    {
      GlobalToLocal();
    }
  }

  std::shared_ptr<Transform> GetParent()
  {
    if(this != GetGround().get() && parent == nullptr)
    {
      parent = GetGround();
    }
    return parent;
  }

  double Distance() const
  {
    return aeirobot::Distance(local_position);
  }

  double DistanceTo(const Vector3 &v, const Space s = aeirobot::k_global) const
  {
    if(s == aeirobot::Space::k_global)
    {
      return aeirobot::Distance(position, v);
    }
    else
    {
      return aeirobot::Distance(v);
    }
  }

  double DistanceTo(Transform &t)
  {
    return aeirobot::Distance(position, t.GetPosition());
  }

  // get angle to the v (atan2 = -PI ~ PI)
  double AngleTo(const Vector3 &v, const Space s = aeirobot::k_global) const
  {
    double angle;

    if(s == aeirobot::Space::k_global)
    {
      Vector3 v1 = position;
      Vector3 v2 = v;

      // double distance_to_destination = aeirobot::Distance(v1, v2);

      Vector3 v3 = v2 - v1;

      double angle_to_destination_global = atan2(v3.y, v3.x);

      // if(db.angle_to_destination_global < 0)
      //   db.angle_to_destination = db.angle_to_destination_global+PI*2;

      Vector3 to_dir, cur_dir;
      to_dir.x  = cos(angle_to_destination_global);
      to_dir.y  = sin(angle_to_destination_global);
      cur_dir.x = cos(v1.z);
      cur_dir.y = sin(v1.z);

      double cross = (to_dir.x * cur_dir.y) - (to_dir.y * cur_dir.x);

      // use acos function to get 0 ~ PI value
      angle = acos((to_dir.x * cur_dir.x) + (to_dir.y * cur_dir.y));

      if(cross > 0)
        angle = -angle;
    }
    else
    {
      angle = atan2(v.y, v.x);
    }

    return angle;
  }

  // get angle to the t (-PI ~ PI)
  double AngleTo(Transform t) 
  {
    t.SetParent(this);
    return AngleTo(t.GetPosition(aeirobot::k_local), aeirobot::k_local);
  }

private:
  Vector3 GlobalToLocal()
  {
    auto p = GetParent();

    local_position.x = ((position.x - p->position.x) * cos(-p->position.z)) - ((position.y - p->position.y) * sin(-p->position.z));
    local_position.y = ((position.x - p->position.x) * sin(-p->position.z)) + ((position.y - p->position.y) * cos(-p->position.z));
    local_position.z = position.z - p->position.z;

    last_parent_pos = p->position;
    return local_position;
  }

  Vector3 LocalToGlobal()
  {
    auto p = GetParent();

    position.x = p->position.x + (cos(p->position.z) * local_position.x) - (sin(p->position.z) * local_position.y);
    position.y = p->position.y + (sin(p->position.z) * local_position.x) + (cos(p->position.z) * local_position.y);
    position.z = p->position.z + local_position.z;

    last_parent_pos = p->position;
    return position;
  }

  bool ParentChanged()
  {
    auto p = GetParent();

    if( last_parent_pos.x == p->position.x &&
        last_parent_pos.y == p->position.y &&
        last_parent_pos.z == p->position.z )
    {
      return false;
    }
    else
    {
      last_parent_pos = p->position;
      return true;
    }
  }
};

} // namespace aeirobot

#endif//TRANSFORM_HPP