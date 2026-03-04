#include "aeirobot_math/trajectory/bezier_curve.hpp"

namespace aeirobot
{
  BezierCurve::BezierCurve()
  {
  }

  BezierCurve::~BezierCurve()
  {
  }

  void BezierCurve::setBezierControlPoints(const std::vector<Point2D> &points)
  {
    control_points_.clear();
    control_points_ = points;
  }

  Point2D BezierCurve::getPoint(double t)
  {
    if (t > 1)
      t = 1;
    else if (t < 0)
      t = 0;

    int points_num = control_points_.size();
    Point2D point_at_t;
    point_at_t.x = 0;
    point_at_t.y = 0;

    if (points_num < 2)
      return point_at_t;

    point_at_t.x = control_points_[0].x * powDI(1 - t, points_num - 1);
    point_at_t.y = control_points_[0].y * powDI(1 - t, points_num - 1);

    unsigned int points_num_unsigned = static_cast<unsigned int>(points_num);
    for (unsigned int i = 1; i < (points_num_unsigned - 1); i++)
    {
      point_at_t.x += control_points_[i].x * combination(points_num - 1, i) * powDI(1 - t, points_num - 1 - i) * powDI(t, i);
      point_at_t.y += control_points_[i].y * combination(points_num - 1, i) * powDI(1 - t, points_num - 1 - i) * powDI(t, i);
    }

    point_at_t.x += control_points_[points_num - 1].x * powDI(t, points_num - 1);
    point_at_t.y += control_points_[points_num - 1].y * powDI(t, points_num - 1);

    return point_at_t;
  }
}