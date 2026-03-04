/**
 * @file vector3.hpp
 * @brief 3차원 벡터 클래스를 정의하는 헤더 파일
 */

#ifndef VECTOR3_HPP
#define VECTOR3_HPP

#include <iostream>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace aeirobot
{

  /**
   * @class Vector3
   * @brief 3차원 벡터를 표현하고 다양한 연산을 지원하는 클래스
   */
  class Vector3
  {
  public:
    double x; /**@brief 벡터의 X 성분 */
    double y; /**@brief 벡터의 Y 성분 */
    double z; /**@brief 벡터의 Z 성분 */

    /**
     * @brief 기본 생성자, 모든 성분을 0으로 초기화
     */
    Vector3() : x(0), y(0), z(0) {}

    /**
     * @brief 임의 타입의 값으로 초기화하는 템플릿 생성자
     * @tparam T 초기화할 수치 타입
     * @param _x X 성분 값
     * @param _y Y 성분 값
     * @param _z Z 성분 값
     */
    template <typename T>
    Vector3(T _x, T _y, T _z)
    {
      Set<T>(_x, _y, _z);
    }

    /**
     * @brief std::vector<double>로부터 초기화
     * @param ref 크기가 최소 3 이상인 double 벡터
     */
    Vector3(const std::vector<double> &ref)
        : x(ref[0]), y(ref[1]), z(ref[2]) {}

    /**
     * @brief geometry_msgs::msg::Vector3_로부터 초기화
     * @param ref ROS2 Vector3 메시지
     */
    Vector3(const geometry_msgs::msg::Vector3_<std::allocator<void>> &ref)
        : x(ref.x), y(ref.y), z(ref.z) {}

    /**
     * @brief geometry_msgs::msg::Pose2D_로부터 초기화
     * @param ref ROS2 Pose2D 메시지 (theta를 Z 성분으로 사용)
     */
    Vector3(const geometry_msgs::msg::Pose2D_<std::allocator<void>> &ref)
        : x(ref.x), y(ref.y), z(ref.theta) {}

    /**
     * @brief geometry_msgs::msg::Point_로부터 초기화
     * @param ref ROS2 Point 메시지
     */
    Vector3(const geometry_msgs::msg::Point_<std::allocator<void>> &ref)
        : x(ref.x), y(ref.y), z(ref.z) {}

    /**
     * @brief 벡터 성분을 설정하는 템플릿 함수
     * @tparam T 설정할 수치 타입
     * @param _x X 성분 값
     * @param _y Y 성분 값
     * @param _z Z 성분 값
     */
    template <typename T>
    void Set(T _x, T _y, T _z)
    {
      x = static_cast<double>(_x);
      y = static_cast<double>(_y);
      z = static_cast<double>(_z);
    }

    /**
     * @brief 특정 2D 사각형 영역 내부에 벡터가 있는지 검사
     * @tparam T 영역 경계 및 크기 타입
     * @param _x 영역의 왼쪽 X 좌표
     * @param _y 영역의 아래 Y 좌표
     * @param _w 영역의 너비
     * @param _h 영역의 높이
     * @return 내부에 있으면 true, 아니면 false
     */
    template <typename T>
    bool IsIn(T _x, T _y, T _w, T _h) const
    {
      return (x > static_cast<double>(_x) &&
              x < static_cast<double>(_x + _w) &&
              y > static_cast<double>(_y) &&
              y < static_cast<double>(_y + _h));
    }

    /**
     * @brief std::vector<double>를 할당 연산자로 복사
     * @param ref 크기가 최소 1 이상인 double 벡터
     * @return 복사된 자기 자신
     */
    Vector3 &operator=(const std::vector<double> &ref)
    {
      if (ref.size() > 0)
        x = ref[0];
      if (ref.size() > 1)
        y = ref[1];
      if (ref.size() > 2)
        z = ref[2];
      return *this;
    }

    /**
     * @brief geometry_msgs::msg::Vector3_를 할당 연산자로 복사
     * @param ref ROS2 Vector3 메시지
     * @return 복사된 자기 자신
     */
    Vector3 &operator=(const geometry_msgs::msg::Vector3_<std::allocator<void>> &ref)
    {
      x = ref.x;
      y = ref.y;
      z = ref.z;
      return *this;
    }

    /**
     * @brief geometry_msgs::msg::Pose2D_를 할당 연산자로 복사
     * @param ref ROS2 Pose2D 메시지
     * @return 복사된 자기 자신
     */
    Vector3 &operator=(const geometry_msgs::msg::Pose2D_<std::allocator<void>> &ref)
    {
      x = ref.x;
      y = ref.y;
      z = ref.theta;
      return *this;
    }

    /**
     * @brief geometry_msgs::msg::Point_를 할당 연산자로 복사
     * @param ref ROS2 Point 메시지
     * @return 복사된 자기 자신
     */
    Vector3 &operator=(const geometry_msgs::msg::Point_<std::allocator<void>> &ref)
    {
      x = ref.x;
      y = ref.y;
      z = ref.z;
      return *this;
    }

    /**
     * @brief 벡터 덧셈 연산자
     * @param ref 더할 Vector3 객체
     * @return 결과 Vector3 객체
     */
    Vector3 operator+(const Vector3 &ref) const
    {
      return Vector3(x + ref.x, y + ref.y, z + ref.z);
    }

    /**
     * @brief std::vector<double>와 벡터 덧셈
     * @param ref 크기 최소 3 이상의 double 벡터
     * @return 결과 Vector3 객체
     */
    Vector3 operator+(const std::vector<double> &ref) const
    {
      if (ref.size() < 3)
      {
        std::cerr << "Vector3 + vector<double> size error\n";
        return *this;
      }
      return Vector3(x + ref[0], y + ref[1], z + ref[2]);
    }

    /**
     * @brief ROS2 Vector3 메시지와 벡터 덧셈
     * @param ref geometry_msgs::msg::Vector3_ 객체
     * @return 결과 Vector3 객체
     */
    Vector3 operator+(const geometry_msgs::msg::Vector3_<std::allocator<void>> &ref) const
    {
      return Vector3(x + ref.x, y + ref.y, z + ref.z);
    }

    /**
     * @brief ROS2 Pose2D 메시지와 벡터 덧셈
     * @param ref geometry_msgs::msg::Pose2D_ 객체
     * @return 결과 Vector3 객체
     */
    Vector3 operator+(const geometry_msgs::msg::Pose2D_<std::allocator<void>> &ref) const
    {
      return Vector3(x + ref.x, y + ref.y, z + ref.theta);
    }

    /**
     * @brief ROS2 Point 메시지와 벡터 덧셈
     * @param ref geometry_msgs::msg::Point_ 객체
     * @return 결과 Vector3 객체
     */
    Vector3 operator+(const geometry_msgs::msg::Point_<std::allocator<void>> &ref) const
    {
      return Vector3(x + ref.x, y + ref.y, z + ref.z);
    }

    /**
     * @brief 벡터 뺄셈 연산자
     * @param ref 뺄 Vector3 객체
     * @return 결과 Vector3 객체
     */
    Vector3 operator-(const Vector3 &ref) const
    {
      return Vector3(x - ref.x, y - ref.y, z - ref.z);
    }

    /**
     * @brief std::vector<double>와 벡터 뺄셈
     * @param ref 크기 최소 3 이상의 double 벡터
     * @return 결과 Vector3 객체
     */
    Vector3 operator-(const std::vector<double> &ref) const
    {
      if (ref.size() < 3)
      {
        std::cerr << "Vector3 - vector<double> size error\n";
        return *this;
      }
      return Vector3(x - ref[0], y - ref[1], z - ref[2]);
    }

    /**
     * @brief ROS2 Vector3 메시지와 벡터 뺄셈
     * @param ref geometry_msgs::msg::Vector3_ 객체
     * @return 결과 Vector3 객체
     */
    Vector3 operator-(const geometry_msgs::msg::Vector3_<std::allocator<void>> &ref) const
    {
      return Vector3(x - ref.x, y - ref.y, z - ref.z);
    }

    /**
     * @brief ROS2 Pose2D 메시지와 벡터 뺄셈
     * @param ref geometry_msgs::msg::Pose2D_ 객체
     * @return 결과 Vector3 객체
     */
    Vector3 operator-(const geometry_msgs::msg::Pose2D_<std::allocator<void>> &ref) const
    {
      return Vector3(x - ref.x, y - ref.y, z - ref.theta);
    }

    /**
     * @brief ROS2 Point 메시지와 벡터 뺄셈
     * @param ref geometry_msgs::msg::Point_ 객체
     * @return 결과 Vector3 객체
     */
    Vector3 operator-(const geometry_msgs::msg::Point_<std::allocator<void>> &ref) const
    {
      return Vector3(x - ref.x, y - ref.y, z - ref.z);
    }

    /**
     * @brief 스칼라 곱 연산자
     * @param ref 곱할 스칼라 값
     * @return 결과 Vector3 객체
     */
    Vector3 operator*(const double ref) const
    {
      return Vector3(x * ref, y * ref, z * ref);
    }

    /**
     * @brief 스칼라 나눗셈 연산자
     * @param ref 나눌 스칼라 값
     * @return 결과 Vector3 객체
     */
    Vector3 operator/(const double ref) const
    {
      return Vector3(x / ref, y / ref, z / ref);
    }

    /**
     * @brief 다른 벡터와의 내적 연산
     * @param v 내적할 Vector3 객체
     * @return 내적 결과값
     */
    double Dot(const Vector3 &v) const
    {
      return x * v.x + y * v.y + z * v.z;
    }

    /**
     * @brief Vector3 메시지로 변환하는 형변환 연산자
     * @return geometry_msgs::msg::Vector3_ 객체
     */
    operator geometry_msgs::msg::Vector3_<std::allocator<void>>() const
    {
      geometry_msgs::msg::Vector3_<std::allocator<void>> result;
      result.x = x;
      result.y = y;
      result.z = z;
      return result;
    }

    /**
     * @brief Pose2D 메시지로 변환하는 형변환 연산자
     * @return geometry_msgs::msg::Pose2D_ 객체
     */
    operator geometry_msgs::msg::Pose2D_<std::allocator<void>>() const
    {
      geometry_msgs::msg::Pose2D_<std::allocator<void>> result;
      result.x = x;
      result.y = y;
      result.theta = z;
      return result;
    }

    /**
     * @brief Point 메시지로 변환하는 형변환 연산자
     * @return geometry_msgs::msg::Point_ 객체
     */
    operator geometry_msgs::msg::Point_<std::allocator<void>>() const
    {
      geometry_msgs::msg::Point_<std::allocator<void>> result;
      result.x = x;
      result.y = y;
      result.z = z;
      return result;
    }

    /**
     * @brief std::vector<double>로 변환하는 형변환 연산자
     * @return 크기 3의 double 벡터
     */
    operator std::vector<double>() const
    {
      return std::vector<double>{x, y, z};
    }

    /**
     * @brief 다른 벡터를 향한 2D 각도(−π~π) 계산
     * @param v 목표 벡터
     * @return atan2(v.y−y, v.x−x) 값
     */
    double AngleTo(const Vector3 &v) const
    {
      if (v.x == x && v.y == y)
        return 0;
      return std::atan2(v.y - y, v.x - x);
    }
  };

  /**
   * @brief 두 벡터 A→B 간의 2D 각도(−π~π) 계산
   * @param a 기준 벡터 A
   * @param b 목표 벡터 B
   * @return atan2(b.y−a.y, b.x−a.x) 값
   */
  inline double AngleAToB(const Vector3 &a, const Vector3 &b)
  {
    if (a.x == b.x && a.y == b.y)
      return 0;
    return std::atan2(b.y - a.y, b.x - a.x);
  }

} // namespace aeirobot

#endif // VECTOR3_HPP
