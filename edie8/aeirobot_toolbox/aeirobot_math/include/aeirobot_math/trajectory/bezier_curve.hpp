/**
 * @file bezier_curve.hpp
 * @brief 베지어 곡선 연산을 위한 제어점 설정 및 곡선 상 점 계산 클래스 선언
 */
#ifndef AEIROBOT_MATH_BEZIER_CURVE_H_
#define AEIROBOT_MATH_BEZIER_CURVE_H_

#include <vector>
#include "aeirobot_math/algebra/linear_algebra.hpp"

namespace aeirobot
{
  /**
   * @class BezierCurve
   * @brief 주어진 2D 제어점을 기반으로 베지어 곡선 상의 좌표를 계산
   */
  class BezierCurve
  {
  public:
    /**
     * @brief 생성자
     */
    BezierCurve();

    /**
     * @brief 소멸자
     */
    ~BezierCurve();

    /**
     * @brief 베지어 곡선의 제어점 설정
     * @param points 2D 제어점 벡터
     */
    void setBezierControlPoints(const std::vector<Point2D> &points);

    /**
     * @brief 파라미터 t에 해당하는 곡선 상의 점 계산
     * @param t [0.0, 1.0] 범위의 파라미터
     * @return 계산된 2D 점 좌표
     */
    Point2D getPoint(double t);

  private:
    /** @brief 설정된 제어점 저장 벡터 */
    std::vector<Point2D> control_points_;
  };

} // namespace aeirobot

#endif // AEIROBOT_MATH_BEZIER_CURVE_H_
