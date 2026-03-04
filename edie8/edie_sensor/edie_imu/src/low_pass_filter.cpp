#include "edie_imu/low_pass_filter.hpp"

LowPassFilter::LowPassFilter()
{
}

LowPassFilter::~LowPassFilter()
{
}

void LowPassFilter::LPFInitialize(double alpha, double initval)
{
    y = s = initval;
    SetAlpha(alpha);
    initialized = false;
}

void LowPassFilter::SetAlpha(double alpha) 
{
  if (alpha<=0.0 || alpha>1.0)
#ifdef __EXCEPTIONS
    throw std::range_error("alpha should be in (0.0., 1.0] and its current value is " + std::to_string(alpha)) ;
#else
    alpha = 0.5;
#endif
  a = alpha;
}

double LowPassFilter::ApplyEMAFilter(double value) 
{
  double result;
  if (initialized)
    result = a * value + (1.0 - a) * s;
  else 
  {
    result = value;
    initialized = true;
  }
  y = value;
  s = result;
  
  return result;
}

double LowPassFilter::ApplyEMAFilterWithAlpha(double value, double alpha) 
{
  SetAlpha(alpha);
  return ApplyEMAFilter(value);
}

bool LowPassFilter::HasLastRawValue(void) 
{
  return initialized ;
}

double LowPassFilter::LastRawValue(void) 
{
  return y ;
}

double LowPassFilter::LastFilteredValue(void) 
{
  return s ;
}