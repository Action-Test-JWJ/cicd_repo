#include "edie_mobile/edie_mobile_main.hpp"

std::array<double, 4> ref_tendency_x = {0, 0, 0, 0};
std::array<double, 4> ref_tendency_y = {0, 0, 0, 0};

double DegreeToRadian(double degree) { return degree * (pi / 180); }
double RadianToDegree(double radian) { return radian * (180 / pi); }
double NormalizeAngle(double angle)
{
  if (abs(angle) > M_PI)
  {
    if (angle > 0.0)
      angle = angle - 2*M_PI;
    else
      angle = angle + 2*M_PI;
  }
  return angle;
}

double EdieMobileNode::StanleyControl(
    double x, double y, double yaw, double v, Path2D path2d)
{
  // constants
  double L = Length;
  double k = k_gain;
  double max_steering = Max_Steering;

  // find nearest point
  double min_dist = 1e9;
  min_index = 0;
  int n_points = path2d.x.size();

  double front_x = x + L * std::cos(yaw); // x = back wheel position
  double front_y = y + L * std::sin(yaw); // y = back wheel position

  for (int i = 0; i < n_points; i++)
  { // find x coordinate index
    double dx = front_x - path2d.x[i];
    double dy = front_y - path2d.y[i];

    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_index = i;
    }
  }

  // compute cte at front axle
  if (min_index < 96)
  {
    ref_tendency_x = {path2d.x[min_index], path2d.x[min_index + 1], path2d.x[min_index + 2], path2d.x[min_index + 3]};
    ref_tendency_y = {path2d.y[min_index], path2d.y[min_index + 1], path2d.y[min_index + 3], path2d.y[min_index + 3]};
  }

  double ref_x = path2d.x[min_index];
  double ref_y = path2d.y[min_index];
  double ref_yaw = path2d.theta[min_index];
  double dx = ref_x - front_x;
  double dy = ref_y - front_y;

  std::vector<double> perp_vec = {std::cos(ref_yaw + M_PI / 2),
                                  std::sin(ref_yaw + M_PI / 2)};
  double cte1 = std::inner_product(
      std::begin(perp_vec), std::end(perp_vec), std::begin({dx, dy}), 0.0);
  double cte2 = std::sqrt(dx * dx + dy * dy);
  if (ref_y < front_y)
  {
    cte2 *= -1;
  }

  // control law
  double psi = NormalizeAngle(ref_yaw - yaw);
  double cte_term = std::atan2(k * cte1, k_s + v);

  // steering
  double steer = psi + cte_term;
  //steeer = steer;
  steer = std::max(std::min(steer, max_steering),
                   -max_steering); // limit the steering angle
  // return std::make_tuple(steer, ref_x, ref_y);
  // psi1 = psi;
  // cte_term1 = cte_term;
  // ref_yaw1 = ref_yaw;
  // cte11 = cte1;
  // v1 = v;
  return steer;
}

double EdieMobileNode::CalcLinearVel(double steer, double e_s, double e_theta)
{
  // 음수 방지 (목표 지나면 0)
  double d = std::max(0.0, e_s);

  // 거리 비례 속도
  double linear_x = stanley_linear_gain * d;

  // 최대 속도 제한
  if (linear_x > max_vel) linear_x = max_vel;

  if (d < 0.05) linear_x = 0.0;

  return linear_x;
}

double EdieMobileNode::CalculateDistance(const Pose2D &pointA, const Pose2D &pointB)
{
  return sqrt(pow((pointA.x - pointB.x), 2) + pow((pointA.y - pointB.y), 2));
}

PID::PID()
{
    P = 0.0;
    I = 0.0;
    D = 0.0;
    state_P = 0.0;
    state_I = 0.0;
    state_D = 0.0;

    max_state = 0.0;
    min_state = 0.0;
    pre_state = 0.0;
    dt = 0.0;
    integrated_state = 0.0;
}
PID::~PID()
{
}

double PID::process(double state)
{
    if (dt == 0.0)
    {
      state_D = 0.0;
    }
    else
    {
      state_D = (state - pre_state) / dt;
      state_I = state + integrated_state;
    }

    double out = P * state + D * state_D + I * state_I * dt;

    if (out > max_state)
    {
      out = max_state;
    }
    else if (out < min_state)
    {
      out = min_state;
    }
    pre_state = state;
    integrated_state = state_I;

    return out;
}
