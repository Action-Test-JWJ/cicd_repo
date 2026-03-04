#include "edie_laser/LaserSensor.hpp"

LaserSensor::LaserSensor(const std::string dev) : 
      sensor_(dev)
{
}

LaserSensor::~LaserSensor()
{
}

void LaserSensor::SensorSetup()
{
  try 
  {
    sensor_.initialize();
    sensor_.setTimeout(200);
	}   
  catch (const std::exception & error)
  {
    std::cerr << "Error initializing sensor with reason:" << std::endl << error.what() << std::endl;
	  return;
  }
}

void LaserSensor::SensorCalibration()
{
  try 
  {
    sensor_.setSignalRateLimit(1.0);
	  sensor_.setVcselPulsePeriod(VcselPeriodPreRange, 18);
	  sensor_.setVcselPulsePeriod(VcselPeriodFinalRange, 14);
  } 
  catch (const std::exception & error) 
  {
	  std::cerr << "Error enabling long range mode with reason:" << std::endl << error.what() << std::endl;
	  return;
  }

  // Accuracy
  try 
  {
  	sensor_.setMeasurementTimingBudget(20000);
  }  
  catch (const std::exception & error) 
  {
    std::cerr << "Error enabling high accuracy mode with reason:" << std::endl << error.what() << std::endl;
	  return;
  }
}

int LaserSensor::RangingData()
{
  int distance_ = 0;
  try 
  {
  	distance_ = sensor_.readRangeSingleMillimeters();
	} 
  catch (const std::exception & error) 
  {
		std::cerr << "Error geating measurement with reason:" << std::endl << error.what() << std::endl;
		distance_ = 8096;
  }

	if (sensor_.timeoutOccurred()) 
  {
		std::cout << "\rReading" << " | timeout!" << std::endl;			
    distance_ = 8096;
	} 
  else 
  {
		//std::cout << "\rReading" << " | " << distance << std::endl;
	}

  return distance_;
}