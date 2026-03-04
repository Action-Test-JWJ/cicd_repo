/**
 * @file aeirobot_signal_processing.h
 * @brief 각종 신호처리(필터) 클래스 및 함수 정의 (Moving Average, Low Pass, Scalar EKF 등)
 * @author Crowban
 */

#ifndef AEIROBOT_TOOLBOX_AEIROBOT_SIGNAL_PROCESSING_H_
#define AEIROBOT_TOOLBOX_AEIROBOT_SIGNAL_PROCESSING_H_

#include <cmath>
#include <deque>
#include <vector>
#include <numeric>
#include <Eigen/Dense>
#include "aeirobot_toolbox/basic_tools.hpp"

namespace aeirobot
{
  /**
   * @class MovingAverageFilter
   * @brief 이동평균(Moving Average) 필터 클래스
   */
  class MovingAverageFilter
  {
  public:
    /**
     * @brief 생성자 - 내부 버퍼 및 변수 초기화
     */
    MovingAverageFilter();

    /**
     * @brief 소멸자
     */
    ~MovingAverageFilter();

    /**
     * @brief 버퍼 크기(윈도우 길이) 초기화 및 필터 내부 상태 리셋
     * @param input_buffer_size - 이동평균 버퍼 크기(샘플 개수)
     * @return 없음
     */
    void Initialize(int input_buffer_size);

    /**
     * @brief 버퍼 크기(윈도우 길이) 변경 및 필터 내부 상태 리셋
     * @param input_buffer_size - 이동평균 버퍼 크기(샘플 개수)
     * @return 없음
     */
    void SetBufferSize(int input_buffer_size);

    /**
     * @brief 현재 버퍼 크기(윈도우 크기) 반환
     * @return 버퍼 크기
     */
    int GetBufferSize() const;

    /**
     * @brief 이동평균 필터를 적용한 결과값 반환
     * @param present_raw_value - 현재 입력값(필터에 넣을 원시 데이터)
     * @return 이동평균이 적용된 출력값
     */
    double GetFilteredOutput(double present_raw_value);

  private:
    /**
     * @brief 이동평균 버퍼(윈도우) 크기
     */
    int buffer_size;

    /**
     * @brief 입력값 저장용 순환 버퍼(윈도우)
     */
    std::vector<double> buffer;

    /**
     * @brief 현재 버퍼 내 삽입 인덱스
     */
    int index;

    /**
     * @brief 현재 저장된 샘플 개수 (최대 buffer_size)
     */
    int count;

    /**
     * @brief 버퍼 내 샘플 총합(이동평균 계산용)
     */
    double sum;
  };

  /**
   * @brief std::deque 기반의 간단 이동평균 필터 함수
   * @param filter_queue - 입력값 저장용 큐(참조)
   * @param new_value - 새로 추가할 입력값
   * @param queue_size - 버퍼 최대 크기(윈도우 길이)
   * @return 이동평균이 적용된 출력값
   */
  double ApplyMovingAverageFilter(std::deque<double> &filter_queue, double new_value, size_t queue_size);

  /**
   * @class LowPassFilter
   * @brief 1차 저역통과(로우패스) 필터 클래스
   */
  class LowPassFilter
  {
  public:
    /**
     * @brief 생성자 - 파라미터 기본값으로 초기화
     */
    LowPassFilter();

    /**
     * @brief 생성자 - 제어주기/차단주파수 직접 지정
     * @param input_control_time - 제어주기(초)
     * @param input_cutoff_frequency - 차단주파수(Hz)
     */
    LowPassFilter(double input_control_time, double input_cutoff_frequency);

    /**
     * @brief 생성자 - 제어주기/차단주파수/최초 결과 값 직접 지정
     * @param input_control_time - 제어주기(초)
     * @param input_cutoff_frequency - 차단주파수(Hz)
     * @param initial_ouput - 최초 결과 값
     */
    LowPassFilter(double input_control_time, double input_cutoff_frequency, double initial_output);

    /**
     * @brief 소멸자
     */
    ~LowPassFilter();

    /**
     * @brief 제어주기, 차단주파수 재설정 및 내부 상태 리셋
     * @param input_control_time - 제어주기(초)
     * @param input_cutoff_frequency - 차단주파수(Hz)
     * @return 없음
     */
    void initialize(double input_control_time, double input_cutoff_frequency);

    /**
     * @brief 제어주기, 차단주파수, 최초 결과 값 재설정 및 내부 상태 리셋
     * @param input_control_time - 제어주기(초)
     * @param input_cutoff_frequency - 차단주파수(Hz)
     * @param initial_output - 최초 결과 값
     * @return 없음
     */
    void initialize(double input_control_time, double input_cutoff_frequency, double initial_output);

    /**
     * @brief 차단주파수 재설정
     * @param input_cutoff_frequency - 차단주파수(Hz)
     * @return 없음
     */
    void SetCutOffFrequency(double input_cutoff_frequency);

    /**
     * @brief 현재 차단주파수 반환
     * @return 차단주파수(Hz)
     */
    double GetCutOffFrequency() const;

    /**
     * @brief 이전 결과를 반환
     * @return 이전 출력값
     */
    double GetPreviousOutput() const;

    /**
     * @brief 입력 신호에 대해 로우패스 필터를 적용한 결과 반환
     * @param present_raw_value - 현재 입력값(필터에 넣을 원시 데이터)
     * @return 필터가 적용된 출력값
     */
    double GetFilteredOutput(double present_raw_value);

    /**
     * @brief 필터 계수(alpha) 재계산(주기/차단주파수 반영)
     * @return 없음
     */
    void CalculateAlpha();

  private:
    /**
     * @brief 차단주파수(Hz)
     */
    double cutoff_frequency;

    /**
     * @brief 제어주기(초)
     */
    double control_time;

    /**
     * @brief 이전 출력값(재귀 계산용)
     */
    double previous_output;

    /**
     * @brief 필터 계수(alpha)
     */
    double alpha;
  };

  /**
   * @class ScalarEKF
   * @brief 1차원 스칼라 확장 칼만 필터(EKF) - 위치/속도 추정
   */
  class ScalarEKF
  {
  public:
    /**
     * @brief 생성자
     * @param initial_value - 초기 위치값
     * @param dt - 시간 간격(초)
     * @param process_noise - 프로세스(모델) 노이즈 공분산
     * @param measurement_noise - 측정 노이즈 공분산
     */
    inline ScalarEKF(double initial_value, double dt, double process_noise, double measurement_noise)
        : dt_(dt), R_(measurement_noise)
    {
      x_ << initial_value, 0.0;
      P_ = Eigen::Matrix2d::Identity() * 1.0;
      F_ << 1.0, dt_,
          0.0, 1.0;
      H_ << 1.0, 0.0;
      Q_ = Eigen::Matrix2d::Identity() * process_noise;
    }

    /**
     * @brief 현재 측정값으로 EKF 업데이트, 필터링된 위치값 반환
     * @param measurement - 측정값
     * @return 필터링된 위치값
     */
    inline double update(double measurement)
    {
      Eigen::Vector2d x_pred = F_ * x_;
      Eigen::Matrix2d P_pred = F_ * P_ * F_.transpose() + Q_;
      double y = measurement - (H_ * x_pred);
      double S = (H_ * P_pred * H_.transpose())(0, 0) + R_;
      Eigen::Vector2d K = P_pred * H_.transpose() / S;
      x_ = x_pred + K * y;
      Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
      P_ = (I - K * H_) * P_pred;
      return x_(0);
    }

    /**
     * @brief 예측 상태(위치/속도) 반환
     * @return 예측된 상태 벡터 [위치, 속도]^T
     */
    inline Eigen::Vector2d predict() const
    {
      return F_ * x_;
    }

  private:
    /**
     * @brief 시간 간격(초)
     */
    double dt_;

    /**
     * @brief 측정 노이즈 공분산(스칼라)
     */
    double R_;

    /**
     * @brief 상태 벡터 [위치, 속도]^T
     */
    Eigen::Vector2d x_;

    /**
     * @brief 상태 공분산 행렬
     */
    Eigen::Matrix2d P_;

    /**
     * @brief 상태 전이 행렬
     */
    Eigen::Matrix2d F_;

    /**
     * @brief 측정 행렬 (위치만 관측)
     */
    Eigen::RowVector2d H_;

    /**
     * @brief 프로세스 노이즈 공분산 행렬
     */
    Eigen::Matrix2d Q_;
  };

  /**
   * @class OneEuroFilter
   * @brief 원 유로 필터(One Euro Filter) 클래스
   */
  class OneEuroFilter
  {
  public:
    /**
     * @brief 생성자 - 파라미터 기본값으로 초기화
     */
    OneEuroFilter();

    /**
     * @brief 생성자
     * @param min_cutoff - 최소 차단주파수(Hz)
     * @param beta - 반응성 계수
     * @param dcutoff - 변화율 필터의 차단주파수(Hz)
     * @param control_time - 제어 주기(초)
     */
    OneEuroFilter(double min_cutoff, double beta, double dcutoff, double control_time);

    /**
     * @brief 생성자
     * @param min_cutoff - 최소 차단주파수(Hz)
     * @param beta - 반응성 계수
     * @param dcutoff - 변화율 필터의 차단주파수(Hz)
     * @param control_time - 제어 주기(초)
     * @param initial_output - 최초 결과 값
     */
    OneEuroFilter(double min_cutoff, double beta, double dcutoff, double control_time, double initial_output);

    /**
     * @brief 소멸자
     */
    ~OneEuroFilter();

    /**
     * @brief 제어주기, 차단주파수 재설정 및 내부 상태 리셋
     * @param min_cutoff - 최소 차단주파수(Hz)
     * @param beta - 반응성 계수
     * @param dcutoff - 변화율 필터의 차단주파수(Hz)
     * @param control_time - 제어 주기(초)
     * @return 없음
     */
    void Initialize(double min_cutoff, double beta, double dcutoff, double control_time);

    /**
     * @brief 제어주기, 차단주파수, 최초 결과 값 재설정 및 내부 상태 리셋
     * @param min_cutoff - 최소 차단주파수(Hz)
     * @param beta - 반응성 계수
     * @param dcutoff - 변화율 필터의 차단주파수(Hz)
     * @param control_time - 제어 주기(초)
     * @param initial_output - 최초 결과 값
     * @return 없음
     */
    void Initialize(double min_cutoff, double beta, double dcutoff, double control_time, double initial_output);

    /**
     * @brief 입력 신호에 대해 원 유로 필터를 적용한 결과 반환
     * @param val - 필터링을 수행할 값
     * @return 필터가 적용된 출력 값
     */
    double GetFilteredOutput(double val);

  private:
    /**
     * @brief 최소 차단주파수(Hz)
     */
    double min_cutoff_;

    /**
     * @brief 반응성 계수
     */
    double beta_;

    /**
     * @brief 변화율 필터의 차단주파수(Hz)
     */
    double dcutoff_;

    /**
     * @brief 사용할 제어 주기(초)
     */
    double control_time_;

    /**
     * @brief 신호(값) 필터 -> (메인필터)
     */
    LowPassFilter x_filter_;

    /**
     * @brief 변화율(속도) 필터 -> (보조필터)
     */
    LowPassFilter dx_filter_;
  };

}

#endif /* AEIROBOT_TOOLBOX_AEIROBOT_SIGNAL_PROCESSING_H_ */
