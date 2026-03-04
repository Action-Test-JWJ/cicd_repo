/**
 * @file spline.hpp
 * @brief B-스플라인 기저 함수 및 클램프된 3차 B-스플라인 평가 함수 선언
 */
#ifndef SPLINE_HPP_
#define SPLINE_HPP_

#include <vector>

namespace aeirobot
{
    /**
     * @class Spline
     * @brief B-스플라인 기저 함수 및 클램프된 3차 B-스플라인 계산 기능 제공
     */
    class Spline
    {
    public:
        /**
         * @brief Cox–de Boor 재귀식을 이용한 B-스플라인 기저 함수 계산
         * @param i       기저 함수 인덱스
         * @param degree  스플라인 차수 (예: 3은 3차)
         * @param knots   결절점 벡터 (크기 = 제어점 수 + degree + 1)
         * @param t       평가 파라미터 (0 <= t <= T)
         * @return        N_{i,degree}(t)의 값
         */
        static double BSplineBasisRecursive(
            int i,
            int degree,
            const std::vector<double> &knots,
            double t);

        /**
         * @brief 클램프된(clamped) 3차 B-스플라인을 t에서 계산
         * @param control_points  제어점 값 벡터 (끝점 클램프 포함, 예: {y0,y0, ..., yN,yN})
         * @param t               평가할 시간 (0 <= t <= T)
         * @param T               전체 구간 길이
         * @return                보간된 스플라인 값
         */
        static double EvaluateClampedCubicBSpline(
            const std::vector<double> &control_points,
            double t,
            double T);
    };

} // namespace aeirobot

#endif // SPLINE_HPP_
