#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include <vector>
#include <cmath>
#include <stdexcept>

class LowPassFilter 
{
public:
    LowPassFilter();
    ~LowPassFilter();
    void LPFInitialize(double alpha, double initval=0.0);
    void SetAlpha(double alpha);                                    // void setAlpha(double alpha) ;
    double ApplyEMAFilter(double value);                            // double filter(double value);
    double ApplyEMAFilterWithAlpha(double value, double alpha);     // double filterWithAlpha(double value, double alpha) ;
    bool HasLastRawValue(void);                                     // bool hasLastRawValue(void) ;
    double LastRawValue(void);                                     // double lastRawValue(void) ;
    double LastFilteredValue(void);                               // double lastFilteredValue(void) ;
private:
    double y, a, s;
    bool initialized;
} ;


#endif // LOW_PASS_FILTER_H