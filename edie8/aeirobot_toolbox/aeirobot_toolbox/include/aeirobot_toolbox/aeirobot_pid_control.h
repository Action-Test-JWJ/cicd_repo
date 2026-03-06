/**
 * @file aeirobot_pid_control.h
 * @brief PD/PID 제어 클래스 정의
 *
 * @details
 *   - PDController: 비례-미분 제어 구현
 *   - PIDController: 비례-적분-미분 제어 구현
 */
#ifndef AEIROBOT_TOOLBOX_AEIROBOT_PID_CONTROL_H_
#define AEIROBOT_TOOLBOX_AEIROBOT_PID_CONTROL_H_

#include "aeirobot_toolbox/basic_tools.hpp"

namespace aeirobot
{
  /**
   * @class PDController
   * @brief 비례(P) 및 미분(D) 제어기
   * @details
   * 비례-미분 제어 방식으로 목표값과 현재값의 차이(오차)를 이용해 제어 신호를 산출
   */
  class PDController
  {
  public:
    /**
     * @brief 생성자: 제어 주기 파라미터 초기화
     * @return 반환값 없음
     */
    PDController();
    /**
     * @brief 소멸자
     * @return 반환값 없음
     */
    ~PDController();

    /**
     * @brief 목표값 설정
     * @param desired 목표 센서 값
     * @return 반환값 없음
     */
    void SetDesired(double desired) { desired_ = desired; }

    /**
     * @brief 제어 이득 설정
     * @param Kp 비례 이득
     * @param Kd 미분 이득
     * @return 반환값 없음
     */
    void SetGains(double Kp, double Kd)
    {
      p_gain_ = Kp;
      d_gain_ = Kd;
    }

    /**
     * @brief 현재 센서 출력으로부터 제어 입력 계산
     * @param present_sensor_output 현재 센서 값
     * @return double 제어 신호
     */
    double GetFeedBack(double present_sensor_output);

  private:
    double desired_{0.0};          /**목표 센서 값 */
    double p_gain_{0.0};           /**비례 이득 */
    double d_gain_{0.0};           /**미분 이득 */
    double control_time_sec_{0.0}; /**제어 주기 (초) */
    double curr_err_{0.0};         /**현재 오차 */
    double prev_err_{0.0};         /**이전 오차 */
  };

  /**
   * @class PIDController
   * @brief 비례-적분-미분 제어기
   * @details
   * 비례, 적분, 미분 이득을 모두 사용하는 PID 제어기
   */
  class PIDController
  {
  public:
    /**
     * @brief 생성자: 제어 주기 파라미터 초기화
     * @return 반환값 없음
     */
    PIDController();
    /**
     * @brief 소멸자
     * @return 반환값 없음
     */
    ~PIDController();

    /**
     * @brief 목표값 설정
     * @param desired 목표 센서 값
     * @return 반환값 없음
     */
    void SetDesired(double desired) { desired_ = desired; }

    /**
     * @brief 제어 이득 설정
     * @param Kp 비례 이득
     * @param Ki 적분 이득
     * @param Kd 미분 이득
     * @return 반환값 없음
     */
    void SetPidGains(double Kp, double Ki, double Kd);

    /**
     * @brief 적분 항 리셋
     * @return 반환값 없음
     */
    void ResetPidIntegral();

    /**
     * @brief PID 제어 처리 함수
     * @param desired 목표 값
     * @param present 현재 값
     * @return double 제어 신호
     */
    double PidProcess(double desired, double present);

    /**
     * @brief 현재 센서 출력으로부터 제어 입력 계산
     * @param present_sensor_output 현재 센서 값
     * @return double 제어 신호
     */
    double GetFeedBack(double present_sensor_output);

  private:
    double desired_{0.0};          /**목표 센서 값 */
    double p_gain_{0.0};           /**비례 이득 */
    double i_gain_{0.0};           /**적분 이득 */
    double d_gain_{0.0};           /**미분 이득 */
    double control_time_sec_{0.0}; /**제어 주기 (초) */
    double curr_err_{0.0};         /**현재 오차 */
    double prev_err_{0.0};         /**이전 오차 */
    double sum_err_{0.0};          /**누적 오차 */
  };

} // namespace aeirobot

#endif // AEIROBOT_TOOLBOX_AEIROBOT_PID_CONTROL_H_