#ifndef OrientationHandler_H
#define OrientationHandler_H

#include <Arduino.h>
#include "IMU.h"

//How important acceleration is in the pitch/roll calculation.
//You want just enough to account for gyro drift. Range: 0.0 - 1.0
#define ACCEL_FACTOR .005 
#define ACCEL_AVG_SCALING 32
#define AVG_ARRAY_SIZE 10

class OrientationHandler {

  private:
  	IMU* _imu;
  	uint32_t _prevTime;
  	float _prevGX, _prevGY, _prevGZ;
  	float _pitch, _roll, _yaw;
    uint32_t _numTimesAveraged;
    int16_t *_avgAXArray, *_avgAYArray, *_avgAZArray;
  	//Average acceleration vector.
  	int16_t _avgAX, _avgAY, _avgAZ;

	  void updateAverageAccel(int16_t newAX, int16_t newAY, int16_t newAZ);

  public:
  	OrientationHandler(IMU* imu);
  	bool initialize();
  	bool calibrate();
  	void calcOrientation(float &pitch, float &roll, float &yaw);
};

#endif
