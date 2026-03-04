#include "edie_imu/one_euro_filter.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

OneEuroFilter::OneEuroFilter()
{
}

OneEuroFilter::~OneEuroFilter()
{
}

void OneEuroFilter::OEFInitialize(double freq, double mincutoff, double beta, double dcutoff)
{
    // OneEuroFilter 파라미터 초기화
    SetFrequency(freq);
    SetMinCutoff(mincutoff);
    SetBeta(beta);
    SetDerivateCutoff(dcutoff);
    // LowPassFilter 인스턴스 초기화
    x_.LPFInitialize(GetAdaptiveAlpha(mincutoff));
    dx_.LPFInitialize(GetAdaptiveAlpha(dcutoff));
    // 마지막 시간 초기화
    lasttime_ = UndefinedTime;
}

double OneEuroFilter::GetAdaptiveAlpha(double cutoff) {
  double te = 1.0 / freq_;
  double tau = 1.0 / (2*M_PI*cutoff);
  return 1.0 / (1.0 + tau/te);
}

void OneEuroFilter::SetFrequency(double f) 
{
  if (f <= 0)
#ifdef __EXCEPTIONS
    throw std::range_error("freq should be >0") ;
#else
    f = 400;  // set to 400Hz default
#endif
  freq_ = f;
}

void OneEuroFilter::SetMinCutoff(double mc) 
{
  if (mc <= 0)
#ifdef __EXCEPTIONS
    throw std::range_error("mincutoff should be >0") ;
#else
    mc = 1.0;
#endif
  mincutoff_ = mc;
}

void OneEuroFilter::SetBeta(double b) 
{
  beta_ = b;
}

void OneEuroFilter::SetDerivateCutoff(double dc) 
{
  if (dc <= 0)
#ifdef __EXCEPTIONS
    throw std::range_error("dcutoff should be >0") ;
#else
    dc = 1.0;
#endif
  dcutoff_ = dc;
}

double OneEuroFilter::ApplyOneEuroFilter(double value, double timestamp)
{
  // update the sampling frequency based on timestamps
  if (lasttime_!=UndefinedTime && timestamp!=UndefinedTime && timestamp>lasttime_)
    freq_ = 1.0 / (timestamp - lasttime_);
  lasttime_ = timestamp;
  
  // estimate the current variation per second
  double dvalue = x_.HasLastRawValue() ? (value - x_.LastFilteredValue()) * freq_ : 0.0; // FIXME: 0.0 or value?
  double edvalue = dx_.ApplyEMAFilterWithAlpha(dvalue, GetAdaptiveAlpha(dcutoff_));
  
  // use it to update the cutoff frequency
  double cutoff = mincutoff_ + beta_ * fabs(edvalue);
  
  // filter the given value
  return x_.ApplyEMAFilterWithAlpha(value, GetAdaptiveAlpha(cutoff));
}