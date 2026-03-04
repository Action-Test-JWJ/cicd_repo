/*
 * aeirobot_signal_processing.cpp
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#include "aeirobot_toolbox/aeirobot_signal_processing.h"

using namespace aeirobot;

MovingAverageFilter::MovingAverageFilter()
    : buffer_size(0), buffer(), index(0), count(0), sum(0.0)
{
}

MovingAverageFilter::~MovingAverageFilter() {}

void MovingAverageFilter::Initialize(int input_buffer_size)
{
  buffer_size = (input_buffer_size > 0 ? input_buffer_size : 1);
  buffer.assign(buffer_size, 0.0);
  index = 0;
  count = 0;
  sum = 0.0;
}

void MovingAverageFilter::SetBufferSize(int input_buffer_size)
{
  buffer_size = (input_buffer_size > 0 ? input_buffer_size : buffer_size);
  buffer.assign(buffer_size, 0.0);
  index = 0;
  count = 0;
  sum = 0.0;
}

int MovingAverageFilter::GetBufferSize() const
{
  return buffer_size;
}

double MovingAverageFilter::GetFilteredOutput(double present_raw_value)
{
  if (buffer_size <= 0)
  {
    return present_raw_value;
  }

  // 오래된 값 제거 후 새로운 값 추가
  sum += present_raw_value - buffer[index];
  buffer[index] = present_raw_value;

  // 인덱스 순환
  index = (index + 1) % buffer_size;

  // 유효 샘플 카운트 증가 (워밍업 단계)
  if (count < buffer_size)
  {
    ++count;
  }

  // 평균 계산
  return sum / static_cast<double>(count);
}

double aeirobot::ApplyMovingAverageFilter(
    std::deque<double> &q, double x, size_t max_len)
{
  if (max_len == 0)
    return x;

  if (!std::isfinite(x)) // 1) 입력 검증
    return (q.empty() ? 0.0 : q.back());

  if (q.size() >= max_len)
    q.pop_front();
  q.push_back(x);

  double sum = std::accumulate(q.begin(), q.end(), 0.0);
  double avg = sum / static_cast<double>(q.size());

  // 2) 출력 검증 : NaN, Inf 모두 차단
  if (!std::isfinite(avg))
    avg = (q.size() > 1 ? *(q.rbegin() + 1) : 0.0);

  return avg;
}

LowPassFilter::LowPassFilter()
    : cutoff_frequency(1.0), control_time(aeirobot::GetParameter<double>("time_parameter", "control_time")), previous_output(0.0)
{
  CalculateAlpha();
}

LowPassFilter::LowPassFilter(double input_control_time, double input_cutoff_frequency)
    : cutoff_frequency(input_cutoff_frequency), control_time(input_control_time), previous_output(0.0)
{
  CalculateAlpha();
}

LowPassFilter::LowPassFilter(double input_control_time, double input_cutoff_frequency, double initial_output)
    : cutoff_frequency(input_cutoff_frequency), control_time(input_control_time), previous_output(initial_output)
{
  CalculateAlpha();
}

LowPassFilter::~LowPassFilter() {}

void LowPassFilter::initialize(double input_control_time, double input_cutoff_frequency)
{
  cutoff_frequency = input_cutoff_frequency;
  control_time = input_control_time;
  previous_output = 0.0;
  CalculateAlpha();
}

void LowPassFilter::initialize(double input_control_time, double input_cutoff_frequency, double initial_output)
{
  cutoff_frequency = input_cutoff_frequency;
  control_time = input_control_time;
  previous_output = initial_output;
  CalculateAlpha();
}

void LowPassFilter::SetCutOffFrequency(double input_cutoff_frequency)
{
  cutoff_frequency = input_cutoff_frequency;
  CalculateAlpha();
}

double LowPassFilter::GetCutOffFrequency() const
{
  return cutoff_frequency;
}

double LowPassFilter::GetPreviousOutput() const
{
  return previous_output;
}

double LowPassFilter::GetFilteredOutput(double present_raw_value)
{
  previous_output = alpha * present_raw_value + (1.0 - alpha) * previous_output;
  return previous_output;
}

void LowPassFilter::CalculateAlpha()
{
  if (cutoff_frequency > 0)
    alpha = (2.0 * M_PI * cutoff_frequency * control_time) / (1.0 + 2.0 * M_PI * cutoff_frequency * control_time);
  else
    alpha = 1; // 안전한 기본값
}

OneEuroFilter::OneEuroFilter()
    : min_cutoff_(1.0), beta_(0.0), dcutoff_(1.0), control_time_(aeirobot::GetParameter<double>("time_parameter", "control_time")),
      x_filter_(aeirobot::GetParameter<double>("time_parameter", "control_time"), 1.0),
      dx_filter_(aeirobot::GetParameter<double>("time_parameter", "control_time"), 1.0)
{
  
}

OneEuroFilter::OneEuroFilter(double min_cutoff, double beta, double dcutoff, double control_time)
    : min_cutoff_(min_cutoff), beta_(beta), dcutoff_(dcutoff), control_time_(control_time),
      x_filter_(control_time, 1.0),      // x_filter_의 초기 차단주파수는 의미 없음, GetFilteredOutput() 함수에서 control_time을 기반으로 계산
      dx_filter_(control_time, dcutoff_) // dx_filter_의 차단주파수를 dcutoff로 고정
{
  
}

OneEuroFilter::OneEuroFilter(double min_cutoff, double beta, double dcutoff, double control_time, double initial_output)
    : min_cutoff_(min_cutoff), beta_(beta), dcutoff_(dcutoff), control_time_(control_time),
      x_filter_(control_time, 1.0, initial_output),      // x_filter_의 초기 차단주파수는 의미 없음, GetFilteredOutput() 함수에서 control_time을 기반으로 계산
      dx_filter_(control_time, dcutoff_) // dx_filter_의 차단주파수를 dcutoff로 고정
{
  
}

OneEuroFilter::~OneEuroFilter() {}

void OneEuroFilter::Initialize(double min_cutoff, double beta, double dcutoff, double control_time)
{
  min_cutoff_ = min_cutoff;
  beta_ = beta;
  dcutoff_ = dcutoff;
  control_time_ = control_time;

  x_filter_.initialize(control_time_, 1.0);
  dx_filter_.initialize(control_time_, dcutoff_);
}

void OneEuroFilter::Initialize(double min_cutoff, double beta, double dcutoff, double control_time, double initial_output)
{
  min_cutoff_ = min_cutoff;
  beta_ = beta;
  dcutoff_ = dcutoff;
  control_time_ = control_time;

  x_filter_.initialize(control_time_, 1.0, initial_output);
  dx_filter_.initialize(control_time_, dcutoff_);
}

double OneEuroFilter::GetFilteredOutput(double val)
{ 
  // 1) dx 필터링 (보조필터링)
  double dx_val = (val - x_filter_.GetPreviousOutput()) / control_time_;
  double dx_hat = dx_filter_.GetFilteredOutput(dx_val);

  // 2) x_filter_의 파라미터 업데이트 
  double cutoff = min_cutoff_ + beta_ * std::fabs(dx_hat);
  x_filter_.SetCutOffFrequency(cutoff);
  
  // 3) x 필터링 (메인필터링)
  double x_hat = x_filter_.GetFilteredOutput(val);

  return x_hat;
}