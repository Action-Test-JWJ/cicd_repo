/**
 * @file aeirobot_toolbox_basic_tools.hpp
 * @brief 로봇 제어를 위한 다양한 기본 유틸리티 함수와 구조체, 상수, 매크로 등을 정의
 * @author
 * @date
 */

#ifndef AEIROBOT_TOOLBOX_BASIC_TOOLS_HPP
#define AEIROBOT_TOOLBOX_BASIC_TOOLS_HPP

#include "rclcpp/rclcpp.hpp"
#include "vector3.hpp"
#include "aeirobot_msgs/msg/pose_xyzrpy.hpp"
#include <iostream>
#include <filesystem>
#include <cmath>
#include <string>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sstream> // ⬅︎ SplitString 사용을 위한 헤더 (필요 최소 추가)

namespace aeirobot
{
  /**
   * @struct JointData
   * @brief 단일 관절의 상태 정보를 저장하는 구조체
   * @details
   * 관절 이름, 위치, 속도, 전류(effort), 가속도 정보를 저장합니다.
   */
  struct JointData
  {
    std::string name; /// 관절 이름
    double pos;       /// 관절 위치
    double vel;       /// 관절 속도
    double effort;    /// 관절 전류
    double acc;       /// 관절 가속도
  };

  /**
   * @brief 로봇의 전체 상태 데이터를 저장하는 구조체
   */
  struct RobotStateData
  {
    /* ----- 일반화 좌표 ----- */
    Eigen::VectorXd q{Eigen::VectorXd::Zero(0)};      ///< 관절 위치 벡터 (size = model.nq)
    Eigen::VectorXd q_dot{Eigen::VectorXd::Zero(0)};  ///< 관절 속도 벡터 (size = model.nv)
    Eigen::VectorXd q_ddot{Eigen::VectorXd::Zero(0)}; ///< 관절 가속도 벡터

    /* ----- 발바닥 힘/FSR ----- */
    double left_foot_force{0.0};                  ///< 왼발 힘 센서 값
    double right_foot_force{0.0};                 ///< 오른발 힘 센서 값
    std::vector<Eigen::Vector3d> fsr_positions{}; ///< FSR 센서 위치 정보

    /* ----- CoM / Body ----- */
    Eigen::Vector3d com_pos{Eigen::Vector3d::Zero()};      ///< 무게중심 위치
    Eigen::Vector3d com_vel{Eigen::Vector3d::Zero()};      ///< 무게중심 속도
    Eigen::Vector3d body_pos{Eigen::Vector3d::Zero()};     ///< 바디 위치
    Eigen::Vector3d body_vel{Eigen::Vector3d::Zero()};     ///< 바디 속도
    Eigen::Matrix3d body_R{Eigen::Matrix3d::Identity()};   ///< 바디 회전행렬
    Eigen::Vector3d body_ang{Eigen::Vector3d::Zero()};     ///< 바디 각도
    Eigen::Vector3d body_ang_vel{Eigen::Vector3d::Zero()}; ///< 바디 각속도

    /* ----- 스윙발 ----- */
    Eigen::VectorXd swing_foot_pos{Eigen::VectorXd::Zero(6)};  ///< 스윙발 위치 [x y z r p y]
    Eigen::VectorXd swing_foot_vel{Eigen::VectorXd::Zero(6)};  ///< 스윙발 속도
    Eigen::Matrix3d swing_foot_R{Eigen::Matrix3d::Identity()}; ///< 스윙발 회전행렬
  };

  /**
   * @brief 입력값이 NaN 혹은 Inf인지 확인
   * @param n 확인할 double 값
   * @return NaN 또는 Inf이면 true, 아니면 false
   */
  inline bool IsNanOrInf(double n)
  {
    return isnan(n) || isinf(n);
  }

  /**
   * @brief geometry_msgs::msg::Pose2D 타입이 NaN/Inf인지 확인
   * @param n Pose2D 값
   * @return NaN 또는 Inf 포함시 true
   */
  inline bool IsNanOrInf(geometry_msgs::msg::Pose2D n)
  {
    return IsNanOrInf(n.x) || IsNanOrInf(n.y) || IsNanOrInf(n.theta);
  }

  /**
   * @brief aeirobot::Vector3 타입이 NaN/Inf인지 확인
   * @param n Vector3 값
   * @return NaN 또는 Inf 포함시 true
   */
  inline bool IsNanOrInf(aeirobot::Vector3 n)
  {
    return IsNanOrInf(n.x) || IsNanOrInf(n.y) || IsNanOrInf(n.z);
  }

  /**
   * @brief vector<double> 값들 중 NaN/Inf 존재여부 확인
   * @param n double vector
   * @return NaN 또는 Inf 포함시 true
   */
  inline bool IsNanOrInf(std::vector<double> n)
  {
    for (auto i : n)
    {
      if (IsNanOrInf(i))
        return true;
    }
    return false;
  }

  /**
   * @brief 값(val)을 min과 max 범위로 제한(clamp)
   * @param val 입력값
   * @param min 최소값
   * @param max 최대값
   * @return min <= val <= max가 되도록 제한된 값
   */
  inline double Clamp(double val, double min, double max)
  {
    if (val < min)
      val = min;
    else if (val > max)
      val = max;
    return val;
  } // val 값을 min과 max 사이로 조정

  /**
   * @brief 구분자를 기준으로 문자열 분할
   * @param str 분할할 문자열
   * @param delimiter 구분자 문자
   * @return 분할된 문자열 벡터
   */
  inline std::vector<std::string> SplitString(const std::string &str, const char &delimiter)
  {
    std::istringstream iss(str);
    std::string buffer;
    std::vector<std::string> result;
    while (std::getline(iss, buffer, delimiter))
    {
      result.push_back(buffer);
    }
    return result;
  }

  /**
   * @brief (내부 유틸) 점표기 경로("a.b.c")로 YAML 노드 탐색 후 존재 여부/노드 반환
   */
  inline YAML::Node GetYamlNodeByPath(const YAML::Node &root, const std::string &path)
  {
    if (!root)
      return YAML::Node();
    if (path.empty())
      return YAML::Node();

    YAML::Node cur = root;
    auto keys = SplitString(path, '.');
    for (const auto &k : keys)
    {
      if (!cur || !cur[k])
        return YAML::Node();
      cur = cur[k];
    }
    return cur;
  }

  /**
   * @brief 패키지명, 파라미터 yaml 파일명, 파라미터명을 입력받아 파라미터 반환
   * @tparam T 반환 타입
   * @param package_name 패키지명
   * @param parameter_file_name yaml 파일명(확장자 제외)
   * @param parameter_name 파라미터 이름
   * @return 읽어온 파라미터 값(T)
   * @throws runtime_error: 파일 없음/파라미터 없음/변환 실패
   */
  template <typename T>
  inline T GetParameter(const std::string &package_name, const std::string &parameter_file_name, const std::string &parameter_name)
  {
    std::string parameter_path = ament_index_cpp::get_package_share_directory(package_name) + "/config/" + parameter_file_name + ".yaml";
    YAML::Node doc;
    try
    {
      doc = YAML::LoadFile(parameter_path.c_str());
    }
    catch (const YAML::BadFile &e)
    {
      throw std::runtime_error("Failed to load parameter file: " + parameter_path);
    }

    // 1) 점표기 경로로 직접 탐색 (예: "inekf_node.ros__parameters.odometry_mode" 또는 "noise_axis.gyroscope_std_sim")
    YAML::Node node = GetYamlNodeByPath(doc, parameter_name);

    // 2) 최상위 키로 탐색 실패 시, 일반적인 ROS2 노드 네임스페이스 경로로 폴백
    if (!node)
    {
      // 우선 고정된 노드명 시도: "inekf_node.ros__parameters.<name>"
      node = GetYamlNodeByPath(doc, std::string("inekf_node.ros__parameters.") + parameter_name);

      // 여전히 실패하면, 최상위의 모든 노드에 대해 "*.ros__parameters.<name>" 시도
      if (!node && doc.IsMap())
      {
        for (auto it = doc.begin(); it != doc.end(); ++it)
        {
          if (!it->first.IsScalar())
            continue;
          const std::string top_key = it->first.as<std::string>();
          YAML::Node cand = GetYamlNodeByPath(doc, top_key + ".ros__parameters." + parameter_name);
          if (cand)
          {
            node = cand;
            break;
          }
        }
      }
    }

    if (!node)
    {
      // 마지막으로, 전통적인 단일 키 접근도 시도
      if (doc[parameter_name])
      {
        node = doc[parameter_name];
      }
    }

    if (!node)
    {
      throw std::runtime_error("Parameter not found: " + parameter_name);
    }

    try
    {
      return node.as<T>();
    }
    catch (const YAML::TypedBadConversion<T> &e)
    {
      throw std::runtime_error("Failed to convert parameter: " + parameter_name + " to the requested type.");
    }
  }

  /**
   * @brief 기본 패키지 이름 상수 (alice4_parameters)
   */
  const std::string default_package_name = "alice4_parameters";

  /**
   * @brief 패키지명을 입력하지 않고 파라미터 파일명, 파라미터명만 입력 시 파라미터 반환 (동적 경로 탐색)
   * @tparam T 반환 타입
   * @param parameter_file_name yaml 파일명(확장자 제외)
   * @param parameter_name 파라미터 이름
   * @return 읽어온 파라미터 값(T)
   * @throws runtime_error: 환경변수, 파일 없음/파라미터 없음/변환 실패
   */
  template <typename T>
  inline T GetParameter(const std::string &parameter_file_name, const std::string &parameter_name)
  {
    YAML::Node doc;

    // ROS_WS 환경 변수에서 작업 공간 경로를 가져옴
    const char *ros_ws_env = std::getenv("ROS_WS");
    if (!ros_ws_env)
    {
      throw std::runtime_error("ROS_WS 환경 변수가 설정되지 않았습니다.");
    }

    // 기본 경로로 parameter_path 설정
    std::string parameter_path = std::string(ros_ws_env) + "/src/" + default_package_name + "/config/" + parameter_file_name + ".yaml";

    // 파일 로딩 시도 함수
    auto try_load_file = [&](const std::string &file_path) -> bool
    {
      try
      {
        YAML::Node tmp = YAML::LoadFile(file_path.c_str());
        if (tmp)
        {
          doc = tmp;
          return true;
        }
      }
      catch (const YAML::BadFile &e)
      {
        ;
      }
      return false;
    };

    // 1. 기본 경로에서 파일 찾기 시도
    if (!try_load_file(parameter_path))
    {
      // 기본 경로에서 파일을 찾지 못한 경우
      // 2. ROS_WS 경로에서 '_parameters'가 포함된 폴더 검색
      std::string ros_ws_path = std::string(ros_ws_env) + "/src/";
      bool file_found = false;

      for (const auto &entry : std::filesystem::directory_iterator(ros_ws_path))
      {
        if (entry.is_directory() && entry.path().filename().string().find("_parameters") != std::string::npos)
        {
          // _parameters가 포함된 폴더를 발견하면, default_package_name을 해당 폴더로 변경
          std::string package_name = entry.path().filename().string();
          // 새로운 경로로 파일을 다시 찾음
          parameter_path = entry.path().string() + "/config/" + parameter_file_name + ".yaml";
          // 파일이 있으면 성공
          if (try_load_file(parameter_path))
          {
            file_found = true;
            break;
          }
        }
      }
      // 파일을 찾지 못하면 오류 발생
      if (!file_found)
      {
        throw std::runtime_error("파라미터 파일을 찾을 수 없습니다: " + parameter_file_name + ".yaml (기본 경로와 ROS_WS 경로 모두에서 찾을 수 없음)");
      }
    }

    // ==== 여기부터 키 탐색 로직 (상동) ====
    // 1) 점표기 경로로 직접 탐색
    YAML::Node node = GetYamlNodeByPath(doc, parameter_name);

    // 2) ROS2 노드 네임스페이스 폴백: "inekf_node.ros__parameters.<name>"
    if (!node)
    {
      node = GetYamlNodeByPath(doc, std::string("inekf_node.ros__parameters.") + parameter_name);

      // 3) 최상위 모든 노드에 대해 "*.ros__parameters.<name>" 시도
      if (!node && doc.IsMap())
      {
        for (auto it = doc.begin(); it != doc.end(); ++it)
        {
          if (!it->first.IsScalar())
            continue;
          const std::string top_key = it->first.as<std::string>();
          YAML::Node cand = GetYamlNodeByPath(doc, top_key + ".ros__parameters." + parameter_name);
          if (cand)
          {
            node = cand;
            break;
          }
        }
      }
    }

    // 4) 마지막으로 전통적인 단일 키
    if (!node && doc[parameter_name])
      node = doc[parameter_name];

    if (!node)
    {
      throw std::runtime_error("파라미터를 찾을 수 없습니다: " + parameter_name);
    }

    // 파라미터를 요청된 타입으로 변환 시도
    try
    {
      return node.as<T>();
    }
    catch (const YAML::TypedBadConversion<T> &e)
    {
      throw std::runtime_error("파라미터 변환 실패: " + parameter_name + " 요청된 타입으로 변환할 수 없습니다.");
    }
  }

  /**
   * @brief 골반 모드 (PelvisMode) 열거형
   */
  enum PelvisMode
  {
    LIPM = 0, ///< Linear Inverted Pendulum Model
    SLIP = 1  ///< Spring Loaded Inverted Pendulum
  };

  /**
   * @brief 콘솔 출력 색상 지정용 열거형
   */
  enum PRINT_COLOR
  {
    BLACK,
    RED,
    GREEN,
    YELLOW,
    BLUE,
    MAGENTA,
    CYAN,
    WHITE,
    ENDCOLOR
  };

  /**
   * @brief 색상 코드에 맞게 ostream에 ANSI 코드 삽입
   * @param os 출력 스트림
   * @param c 색상 enum 값
   * @return 수정된 ostream
   */
  inline std::ostream &operator<<(std::ostream &os, PRINT_COLOR c)
  {
    switch (c)
    {
    case BLACK:
      os << "\033[1;30m";
      break;
    case RED:
      os << "\033[1;31m";
      break;
    case GREEN:
      os << "\033[1;32m";
      break;
    case YELLOW:
      os << "\033[1;33m";
      break;
    case BLUE:
      os << "\033[1;34m";
      break;
    case MAGENTA:
      os << "\033[1;35m";
      break;
    case CYAN:
      os << "\033[1;36m";
      break;
    case WHITE:
      os << "\033[1;37m";
      break;
    case ENDCOLOR:
      os << "\033[0m";
      break;
    default:
      os << "\033[1;37m";
    }
    return os;
  }

  /**
   * @brief 문자열을 PRINT_COLOR enum으로 변환
   * @param color 색상명(문자열)
   * @return PRINT_COLOR 값
   */
  inline PRINT_COLOR ConvertStringToColor(const std::string &color)
  {
    if (color == "BLACK")
      return BLACK;
    else if (color == "RED")
      return RED;
    else if (color == "GREEN")
      return GREEN;
    else if (color == "YELLOW")
      return YELLOW;
    else if (color == "BLUE")
      return BLUE;
    else if (color == "MAGENTA")
      return MAGENTA;
    else if (color == "CYAN")
      return CYAN;
    else if (color == "WHITE")
      return WHITE;
    else
      return ENDCOLOR;
  }

  /**
   * @brief RCLCPP_INFO_STREAM으로 메시지 출력
   * @tparam T 출력할 메시지 타입
   * @param msg 출력할 메시지
   */
  template <typename T>
  inline void PrintMessage(const T &msg)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("aeirobot_toolbox"), "print message \n"
                                                                   << to_yaml(msg));
  }

  /**
   * @brief 색상을 지정하여 메시지 출력
   * @tparam T 출력할 메시지 타입
   * @param msg 출력할 메시지
   * @param color 출력 색상 문자열
   */
  template <typename T>
  inline void PrintMessage(const T &msg, const std::string &color)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("aeirobot_toolbox"), aeirobot::PRINT_COLOR(ConvertStringToColor(color))
                                                                   << "print message \n"
                                                                   << to_yaml(msg) << aeirobot::PRINT_COLOR(ENDCOLOR));
  }

  /**
   * @brief 행렬의 크기와 자료형을 출력하는 디버깅 함수
   */
  template <typename Derived>
  inline void PrintMatrixSizeAndType(const std::string &name, const Eigen::MatrixBase<Derived> &mat)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("aeirobot_toolbox"), "\n[Matrix: " << name << "]\n"
                                                                             << "  Rows: " << mat.rows() << ", Cols: " << mat.cols() << "\n"
                                                                             << "  Scalar type: " << typeid(typename Derived::Scalar).name() << "\n"
                                                                             << "--------------------------");
  }

#define ROS_BLACK_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("aeirobot_toolbox"), aeirobot::BLACK << x << aeirobot::ENDCOLOR)     // 검정색(Black)으로 메시지 출력
#define ROS_RED_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("aeirobot_toolbox"), aeirobot::RED << x << aeirobot::ENDCOLOR)         // 빨간색(Red)으로 메시지 출력
#define ROS_GREEN_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("aeirobot_toolbox"), aeirobot::GREEN << x << aeirobot::ENDCOLOR)     // 초록색(Green)으로 메시지 출력
#define ROS_YELLOW_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("aeirobot_toolbox"), aeirobot::YELLOW << x << aeirobot::ENDCOLOR)   // 노란색(Yellow)으로 메시지 출력
#define ROS_BLUE_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("aeirobot_toolbox"), aeirobot::BLUE << x << aeirobot::ENDCOLOR)       // 파란색(Blue)으로 메시지 출력
#define ROS_MAGENTA_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("aeirobot_toolbox"), aeirobot::MAGENTA << x << aeirobot::ENDCOLOR) // 자홍색(Magenta)으로 메시지 출력
#define ROS_CYAN_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("aeirobot_toolbox"), aeirobot::CYAN << x << aeirobot::ENDCOLOR)       // 청록색(Cyan)으로 메시지 출력

  // 조건부 컬러 출력 매크로(주석 처리)
  // #define ROS_BLACK_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, aeirobot::BLACK << x << aeirobot::ENDCOLOR)
  // #define ROS_RED_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, aeirobot::RED << x << aeirobot::ENDCOLOR)
  // #define ROS_GREEN_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, aeirobot::GREEN << x << aeirobot::ENDCOLOR)
  // #define ROS_YELLOW_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, aeirobot::YELLOW << x << aeirobot::ENDCOLOR)
  // #define ROS_BLUE_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, aeirobot::BLUE << x << aeirobot::ENDCOLOR)
  // #define ROS_MAGENTA_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, aeirobot::MAGENTA << x << aeirobot::ENDCOLOR)
  // #define ROS_CYAN_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, aeirobot::CYAN << x << aeirobot::ENDCOLOR)
}

#endif // AEIROBOT_TOOLBOX_BASIC_TOOLS_HPP
