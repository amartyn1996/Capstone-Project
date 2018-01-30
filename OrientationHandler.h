#ifndef OrientationHandler_H
#define OrientationHandler_H

#include <Arduino.h>
#include "IMU.h"

//How much new values influence the average acceleration vector. Range: 0.0 - 1.0
#define EXP_MOV_AVG_ALPHA .05 
//How important acceleration is in the pitch/roll calculation.
//You want just enough to account for gyro drift. Range: 0.0 - 1.0
#define ACCEL_FACTOR .005 

class OrientationHandler {

  private:
	IMU* _imu;
	uint32_t _prevTime = 0;
	float _prevGX = 0, _prevGY = 0, _prevGZ = 0;
	float _pitch, _roll;
	//Exponential moving average stuff
	float _wSumAX=0, _wSumAY=0, _wSumAZ=0, _wCntAX=0, _wCntAY=0, _wCntAZ=0;
	uint32_t _numElementsAveraged=1;
	//Average acceleration vector.
	float _avgAX=0, _avgAY=0, _avgAZ=0;

	void updateWeightedSum(float newValue, float &weightedSum, uint32_t numElementsAveraged);
	void updateWeightedCount(float &weightedCount, uint32_t numElementsAveraged);
	void updateEMA(float newValue, float &weightedSum, float &weightedCount, float &expMovAvg, uint32_t numElementsAveraged);
	void updateAverageAccel(float newAX, float newAY, float newAZ);
	void normalize(float &aX, float &aY, float &aZ);
	float vecLength(float aX, float aY, float aZ);

  public:
  OrientationHandler(IMU* imu);
	bool initialize();
	bool calibrate();
	void calcOrientation(float &pitch, float &roll);
};

#endif
