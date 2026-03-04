/**
 * @file aeirobot_math_tool.hpp
 * @brief 수학 연산 및 유틸리티 함수, 상수, 타입 정의 헤더
 */
#ifndef AEIROBOT_MATH_TOOL_HPP
#define AEIROBOT_MATH_TOOL_HPP

#include "rclcpp/rclcpp.hpp"
#include "aeirobot_toolbox/vector3.hpp"
#include "aeirobot_msgs/msg/pose_xyzrpy.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <filesystem>
#include <cmath>
#include <string>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

/** @brief 도(degree)에서 라디안(radian) 변환 상수 */
#define DEGREE2RADIAN (M_PI / 180.0)
/** @brief 라디안(radian)에서 도(degree) 변환 상수 */
#define RADIAN2DEGREE (180.0 / M_PI)

namespace aeirobot
{
    /** @brief 원주율(pi) 상수 */
    const double k_pi = M_PI;

    /**
     * @brief 도(degree)를 라디안(radian)으로 변환
     * @tparam T 수치형 타입
     * @param degree 도 값
     * @return 라디안 값
     */
    template <typename T>
    inline T DegToRad(T degree) { return degree * DEGREE2RADIAN; }

    /**
     * @brief 라디안(radian)을 도(degree)로 변환
     * @tparam T 수치형 타입
     * @param radian 라디안 값
     * @return 도 값
     */
    template <typename T>
    inline T RadToDeg(T radian) { return radian * RADIAN2DEGREE; }

    /**
     * @brief 2D 평면상 두 점(a, b) 간 거리 계산
     * @param a 시작 좌표
     * @param b 종료 좌표
     * @return 거리
     */
    inline double Distance(const aeirobot::Vector3 &a, const aeirobot::Vector3 &b)
    {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    /**
     * @brief 원점으로부터 점(v)까지 거리 계산 (x, y 평면)
     * @param v 대상 좌표
     * @return 거리
     */
    inline double Distance(const aeirobot::Vector3 &v)
    {
        return std::sqrt(std::pow(v.x, 2) + std::pow(v.y, 2));
    }

    /**
     * @brief 각도(degree)를 -180~180 범위로 정규화
     * @param angle 도 값
     * @return 정규화된 도 값
     */
    inline double NormalizeAngle(double angle)
    {
        angle = std::fmod(angle, 360.0);
        if (std::abs(angle) > 180.0)
            angle += (angle > 0 ? -360.0 : 360.0);
        return angle;
    }

    /**
     * @brief 라디안을 -π~π 범위로 정규화
     * @param radian 라디안 값
     * @return 정규화된 라디안 값
     */
    inline double NormalizeRadian(double radian)
    {
        radian = std::fmod(radian + M_PI, 2.0 * M_PI);
        if (radian < 0)
            radian += 2.0 * M_PI;
        return radian - M_PI;
    }

    /**
     * @brief 쿼터니언을 롤(roll)로 변환하는 함수
     *
     * @param x 쿼터니언 x 성분
     * @param y 쿼터니언 y 성분
     * @param z 쿼터니언 z 성분
     * @param w 쿼터니언 w 성분
     * @return double 롤(roll) 각도 [rad]
     */
    inline double QToRoll(double x, double y, double z, double w)
    {
        return std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    }

    /**
     * @brief 쿼터니언을 피치(pitch)로 변환하는 함수
     *
     * @param x 쿼터니언 x 성분
     * @param y 쿼터니언 y 성분
     * @param z 쿼터니언 z 성분
     * @param w 쿼터니언 w 성분
     * @return double 피치(pitch) 각도 [rad]
     */
    inline double QToPitch(double x, double y, double z, double w)
    {
        double sinp = 2.0 * (w * y - z * x);
        return (std::fabs(sinp) >= 1.0) ? std::copysign(M_PI / 2, sinp) : std::asin(sinp);
    }

    /**
     * @brief 쿼터니언을 요(yaw)로 변환하는 함수
     *
     * @param x 쿼터니언 x 성분
     * @param y 쿼터니언 y 성분
     * @param z 쿼터니언 z 성분
     * @param w 쿼터니언 w 성분
     * @return double 요(yaw) 각도 [rad]
     */
    inline double QToYaw(double x, double y, double z, double w)
    {
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    /**
     * @brief 지수(pow) 계산 (정수 지수에 대한 재귀적 구현)
     *
     * @param a 밑수(base)
     * @param b 지수(exponent, int형)
     * @return double a의 b제곱 (a^b) 값
     */
    inline double powDI(double a, int b)
    {
        return (b == 0) ? 1.0 : (b > 0 ? a * powDI(a, b - 1) : 1.0 / powDI(a, -b));
    }

    /**
     * @brief 부호(sign) 함수
     *
     * @param x 입력값
     * @return double x < 0일 때 -1, x > 0일 때 1, x == 0일 때 0 반환
     */
    inline double sign(double x)
    {
        return (x < 0.0) ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
    }

    /**
     * @brief 이항 계수(조합) 계산 함수 (nCr)
     *
     * @param n 전체 개수
     * @param r 선택 개수
     * @return int n개 중 r개를 선택하는 조합의 수 (이항 계수)
     */
    inline int combination(int n, int r)
    {
        if (r == 0 || n == r)
            return 1;
        return combination(n - 1, r - 1) + combination(n - 1, r);
    }

    /**
     * @struct Pose3D
     * @brief 3D 위치(x, y, z)와 자세(roll, pitch, yaw)를 저장하는 구조체
     */
    typedef struct
    {
        double x;     /**3차원 공간의 x 좌표 */
        double y;     /**3차원 공간의 y 좌표 */
        double z;     /**3차원 공간의 z 좌표 */
        double roll;  /**롤(roll) 각도 [rad] */
        double pitch; /**피치(pitch) 각도 [rad] */
        double yaw;   /**요(yaw) 각도 [rad] */
    } Pose3D;

    /**
     * @struct Point2D
     * @brief 2D 평면에서 x, y 좌표를 저장하는 구조체
     */
    typedef struct
    {
        double x; /**x 좌표 */
        double y; /**y 좌표 */
    } Point2D;

} // namespace aeirobot

#endif // AEIROBOT_MATH_TOOL_HPP
