#ifndef ONE_EURO_FILTER_H
#define ONE_EURO_FILTER_H

#include <iostream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <ctime>
#include "edie_imu/low_pass_filter.hpp"


static const double UndefinedTime = -1.0;

class OneEuroFilter 
{
public:
    OneEuroFilter();
    ~OneEuroFilter();
  /**
   * @brief Creates the filter and set its parameters
   * @param freq An estimate of the frequency in Hz of the signal (> 0), if timestamps are not available.
   * @param mincutoff Min cutoff frequency in Hz (> 0). Lower values allow to remove more jitter.
   * @param beta Parameter to reduce latency (> 0).
   * @param dcutoff Used to filter the derivates. 1 Hz by default. Change this parameter if you know what you are doing.
   */
  void OEFInitialize(double freq, double mincutoff=1.0, double beta=0.0, double dcutoff=1.0);

  /**
   * @brief Filter the noisy signal
   * @param value Noisy value to filter
   * @param timestamp (optional) timestamp in seconds
   * @return The filtered value
   */
  double ApplyOneEuroFilter(double value, double timestamp=UndefinedTime);   // filter

  /**
   * @brief Sets the frequency of the signal
   * @param f An estimate of the frequency in Hz of the signal (> 0), if timestamps are not available.
   */
  void SetFrequency(double f);                                                  // setFrequency

  /**
   * @brief Sets the filter min cutoff frequency
   * @param mc Min cutoff frequency in Hz (> 0). Lower values allow to remove more jitter.
   */ 
  void SetMinCutoff(double mc);                                                 // setMinCutoff

  /**
   * @brief Sets the Beta parameter
   * @param b Parameter to reduce latency (> 0).
   */ 
  void SetBeta(double b) ;                                                      // setBeta

  /**
   * @brief Sets the Cutoff frequency for derivates
   * @param dc Used to filter the derivates. 1 Hz by default. Change this parameter if you know what you are doing.
   */ 
  void SetDerivateCutoff(double dc) ;                                          // setDerivateCutoff

  /**
   * @brief Get the adaptive alpha value
   * @param cutoff cutoff frequency in Hz (> 0)
   */ 
  double GetAdaptiveAlpha(double cutoff);                                       // alpha

private:
  double freq_;
  double mincutoff_;
  double beta_;
  double dcutoff_;
  LowPassFilter x_;
  LowPassFilter dx_;
  double lasttime_;
} ;

#endif // ONE_EURO_FILTER_H