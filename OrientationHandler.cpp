#include <Arduino.h>
#include "OrientationHandler.h"

OrientationHandler::OrientationHandler(IMU* imu) {
  _imu = imu;
}

/**
 *  Initializes the IMU.
 */
bool OrientationHandler::initialize() {
  return _imu->initialize();
}

/**
 *  Calibrates the IMU and gets the initial pitch and roll.
 */
bool OrientationHandler::calibrate() {
  bool ret = _imu->calibrate();

  //Get the initial pitch and roll
  float aX, aY, aZ, gX, gY, gZ;
  _imu->getSensorData(aX, aY, aZ, gX, gY, gZ);
  updateAverageAccel(aX, aY, aZ);
  aX = _avgAX;
  aY = _avgAY;
  aZ = _avgAZ;
  normalize(aX, aY, aZ);
  _pitch = 90 * aY;
  _roll = 90 * aX;

  return ret;
}

/**
 *  Calculates the current pitch and roll from IMU data.
 */
void OrientationHandler::calcOrientation(float &pitch, float &roll) {
  
  float aX, aY, aZ, gX, gY, gZ;

  _imu->getSensorData(aX, aY, aZ, gX, gY, gZ);

  //Get the average acceleration vector.
  updateAverageAccel(aX, aY, aZ);
  aX = _avgAX;
  aY = _avgAY;
  aZ = _avgAZ;
  normalize(aX, aY, aZ);
  
  //Calculate the change in gyro angle between last update and now.
  uint32_t curTime = millis();
  float deltaTime = .001 * (float) (curTime - _prevTime);
  float deltaX = (_prevGX * deltaTime);
  float deltaY = (_prevGY * deltaTime);
  float deltaZ = (PI/180) * (_prevGZ * deltaTime);

  //Apply the change in angle to the previous angle. 
  _pitch = _pitch + deltaX;
  _roll = _roll - deltaY;

  //Account for yaw motion.
  float tmp = _roll;
  _roll += _pitch * sin(deltaZ);
  _pitch -= tmp * sin(deltaZ);

  //Use accelerometer data to compensate for gyro drift.
  _pitch = _pitch * (1 - ACCEL_FACTOR) + 90 * aY * ACCEL_FACTOR;
  _roll = _roll * (1 - ACCEL_FACTOR) + 90 * aX * ACCEL_FACTOR; 
  
  pitch = _pitch;
  roll = _roll;
  
  _prevGX = gX;
  _prevGY = gY;
  _prevGZ = gZ;
  _prevTime = curTime;
 
}

/**
 *  Updates the weighted sum of an existing exponential moving average.
 *  
 *  Helper function for updateEMA().
 *  Take a look at updateAverageAccel() for an explaination on EMAs.
 */
void OrientationHandler::updateWeightedSum(float newValue, float &weightedSum, uint32_t numElementsAveraged) {
  weightedSum = newValue + (1 - EXP_MOV_AVG_ALPHA) * weightedSum;
}

/**
 *  Updates the weighted count of an existing exponential moving average.
 *  
 *  Helper function for updateEMA().
 *  Take a look at updateAverageAccel() for an explaination on EMAs.
 */
void OrientationHandler::updateWeightedCount(float &weightedCount, uint32_t numElementsAveraged) {
  weightedCount = ( 1 - pow(1 - EXP_MOV_AVG_ALPHA, numElementsAveraged) ) / ( 1 - (1 - EXP_MOV_AVG_ALPHA) );
}

/**
 *  Updates an existing exponential moving average with a new value.
 *  
 *  Helper function for updateAverageAccel().
 *  Take a look there for an explaination on EMAs.
 */
void OrientationHandler::updateEMA(float newValue, float &weightedSum, float &weightedCount, float &expMovAvg, uint32_t numElementsAveraged) {
  updateWeightedSum(newValue, weightedSum, numElementsAveraged);
  updateWeightedCount(weightedCount, numElementsAveraged);
  expMovAvg = weightedSum / weightedCount;
}

/**
 *  Incorporates a new value into the running average acceleration vector.
 * 
 *  The 'average' value is really an exponential moving average.
 *  Simply, it is a way of getting the average of many values but the most recent 
 *  values change the average more than old values.
 *  The formula for it is as follows:
 *  
 *  ExponentialMovingAverage = WeightedSum / WeightedCount
 *  
 *  WeightedSum = NewValue + (1 - Alpha) * WeightedSum
 *  
 *  WeightedCount = (1 + (1 - Alpha)^numElementsAveraged) / (1 - (1 - Alpha))
 *  
 *  Where NewValue is the latest value we want to incorporate into the average.
 *  And where Alpha is how much new values influence the average. Its range is for 0.0 to 1.0.
 *  And where numElementsAveraged can be thought of as the total number of times the average was calculated + 1.
 *  
 */
void OrientationHandler::updateAverageAccel(float newAX, float newAY, float newAZ) { 
  updateEMA(newAX, _wSumAX, _wCntAX, _avgAX, _numElementsAveraged);
  updateEMA(newAY, _wSumAY, _wCntAY, _avgAY, _numElementsAveraged);
  updateEMA(newAZ, _wSumAZ, _wCntAZ, _avgAZ, _numElementsAveraged);
  _numElementsAveraged++;  
}

/**
 *  Takes in a vector and normalizes it.
 */
void OrientationHandler::normalize(float &aX, float &aY, float &aZ) {
  float len = vecLength(aX, aY, aZ);
  aX /= len;
  aY /= len;
  aZ /= len;
}

/**
 *  Returns the length of the input vector.
 */
float OrientationHandler::vecLength(float aX, float aY, float aZ) {
  return sqrt(aX*aX + aY*aY + aZ*aZ);
}


