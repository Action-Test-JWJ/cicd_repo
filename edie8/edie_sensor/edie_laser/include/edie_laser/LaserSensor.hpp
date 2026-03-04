#ifndef LASER_SENSOR_HPP
#define LASER_SENSOR_HPP

#include "VL53L0X.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <unistd.h>


  class LaserSensor
  {
    public:
      LaserSensor(const std::string dev);
      ~LaserSensor();

      void SensorSetup();
      void SensorCalibration();
      int RangingData();
      
    private:
  
      VL53L0X sensor_;
  };

#endif
