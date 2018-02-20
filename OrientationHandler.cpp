#include <Arduino.h>
#include "OrientationHandler.h"
#include "MiscMath.h"

OrientationHandler::OrientationHandler(IMU* imu) {
  _imu = imu;
  _prevTime = 0;
  _prevGX = 0; _prevGY = 0; _prevGZ = 0;
  _avgAX=0; _avgAY=0; _avgAZ=0;
  _avgAXArray = (int16_t*) malloc(AVG_ARRAY_SIZE * sizeof(int16_t));
  _avgAYArray = (int16_t*) malloc(AVG_ARRAY_SIZE * sizeof(int16_t));
  _avgAZArray = (int16_t*) malloc(AVG_ARRAY_SIZE * sizeof(int16_t));
  _numTimesAveraged = 0;
}

/**
 *  Initializes the IMU.
 */
bool OrientationHandler::initialize() {
  return _imu->initialize();
}

/**
 *  Calibrates the IMU and gets the initial pitch, roll, yaw.
 */
bool OrientationHandler::calibrate() {
  bool ret = _imu->calibrate();

  //Get the initial pitch and roll
  float aX, aY, aZ, gX, gY, gZ;
  _imu->updateSensorData();
  _imu->getSensorData(aX, aY, aZ, gX, gY, gZ);
  MiscMath::normalize(aX, aY, aZ);
  _pitch = 90 * aY;
  _roll = 90 * aX;
  _yaw = 0;

  return ret;
}

/**
 *  Calculates the current pitch, roll, and yaw from IMU data.
 */
void OrientationHandler::calcOrientation(float &pitch, float &roll, float &yaw) {
  
  float aX, aY, aZ, gX, gY, gZ;
  
  _imu->getSensorData(aX, aY, aZ, gX, gY, gZ);
  
  int16_t tmpAX, tmpAY, tmpAZ;
  tmpAX = (int16_t) (aX * ACCEL_AVG_SCALING);
  tmpAY = (int16_t) (aY * ACCEL_AVG_SCALING);
  tmpAZ = (int16_t) (aZ * ACCEL_AVG_SCALING);

  //Get the average acceleration vector.
  updateAverageAccel(tmpAX, tmpAY, tmpAZ);

  aX = ((float) _avgAX) / ACCEL_AVG_SCALING;
  aY = ((float) _avgAY) / ACCEL_AVG_SCALING;
  aZ = ((float) _avgAZ) / ACCEL_AVG_SCALING;

  MiscMath::normalize(aX, aY, aZ);
  
  //Calculate the change in gyro angle between last update and now.
  uint32_t curTime = millis();
  float deltaTime = .001 * (float) (curTime - _prevTime);
  float deltaX = _prevGX * deltaTime;
  float deltaY = _prevGY * deltaTime;
  float deltaZ = _prevGZ * deltaTime;

  float tmpPitch = _pitch;
  float tmpRoll = _roll;  

  //Apply the change in angle to the previous angle. 
  float cosTmpPitch = abs(cos(DEGREES_TO_RADIANS * tmpPitch));
  float cosTmpRoll = abs(cos(DEGREES_TO_RADIANS * tmpRoll));
  
  _pitch += deltaX * cosTmpRoll;
  _roll -= deltaY * cosTmpPitch;
  _yaw += deltaZ * cosTmpRoll; 
  
  //Account for yaw motion.  
  float sinDeltaZ = sin(DEGREES_TO_RADIANS * deltaZ);
  
  _pitch -= tmpRoll * sinDeltaZ;
  _roll += tmpPitch * sinDeltaZ;

  //Add yaw if rolled over and pitching
  _yaw += deltaX * sin(DEGREES_TO_RADIANS * _roll);

  //Use accelerometer data to compensate for gyro drift.
  static const float ninetyTimesAccelFactor = 90.0 * ACCEL_FACTOR;
  static const float oneMinusAccelFactor = 1.0 - ACCEL_FACTOR;
  _pitch = _pitch * oneMinusAccelFactor + aY * ninetyTimesAccelFactor;
  _roll = _roll * oneMinusAccelFactor + aX * ninetyTimesAccelFactor; 
  
  pitch = _pitch;
  roll = _roll;
  yaw = _yaw;
  
  _prevGX = gX;
  _prevGY = gY;
  _prevGZ = gZ;
  _prevTime = curTime;
}


/**
 *  Incorporates a new value into the running average acceleration vector.
 */
void OrientationHandler::updateAverageAccel(int16_t newAX, int16_t newAY, int16_t newAZ) { 
  
  //Replace old values in the arrays with the new values.
  uint8_t replaceIndex = _numTimesAveraged % AVG_ARRAY_SIZE;
  _avgAXArray[replaceIndex] = newAX;
  _avgAYArray[replaceIndex] = newAY; 
  _avgAZArray[replaceIndex] = newAZ; 

  int32_t aXSum = 0;
  int32_t aYSum = 0;
  int32_t aZSum = 0;

  for (uint8_t i = 0; i < AVG_ARRAY_SIZE; i++) {
    aXSum += _avgAXArray[i];
    aYSum += _avgAYArray[i];
    aZSum += _avgAZArray[i];
  }
  
  _avgAX = aXSum / AVG_ARRAY_SIZE;
  _avgAY = aYSum / AVG_ARRAY_SIZE;
  _avgAZ = aZSum / AVG_ARRAY_SIZE;
  
  _numTimesAveraged++;
}


