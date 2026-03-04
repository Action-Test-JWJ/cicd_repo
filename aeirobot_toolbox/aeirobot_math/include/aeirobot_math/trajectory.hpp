/**
 * @file aeirobot_trajectory.hpp
 * @brief Trajectory 관련 헤더 파일을 한 곳에서 포함하는 유틸리티
 */
#ifndef AEIROBOT_TRAJECTORY_H_
#define AEIROBOT_TRAJECTORY_H_

/** @ingroup Trajectory
 *  @{ */

#include "trajectory/fifth_order_polynomial_trajectory.hpp"      /**< 5차 다항 궤적 계산 클래스 */
#include "trajectory/minimum_jerk_trajectory_with_via_point.hpp" /**< 최소 저크 궤적 (via-point 포함) 클래스 */
#include "trajectory/minimum_jerk_trajectory.hpp"                /**< 기본 최소 저크 궤적 클래스 */
#include "trajectory/ninth_order_polynomial_trajectory.hpp"      /**< 9차 다항 궤적 계산 클래스 */
#include "trajectory/seventh_order_polynomial_trajectory.hpp"    /**< 7차 다항 궤적 계산 클래스 */
#include "trajectory/simple_trapezoidal_velocity_profile.hpp"    /**< 트랩에조이달 속도 프로파일 클래스 */
#include "trajectory/spline.hpp"                                 /**< B-스플라인 보간 유틸리티 */
#include "trajectory/trajectory_calculator.hpp"                  /**< 다양한 궤적 계산 함수 모음 */
#include "trajectory/bezier_curve.hpp"                           /**< 베지어 곡선 계산 클래스 */

/** @} */ // end of Trajectory group

#endif /* AEIROBOT_TRAJECTORY_H_ */
