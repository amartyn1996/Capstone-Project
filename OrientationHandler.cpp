#include <Arduino.h>
#include "OrientationHandler.h"

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
 *  Calibrates the IMU and gets the initial pitch and roll.
 */
bool OrientationHandler::calibrate() {
  bool ret = _imu->calibrate();

  //Get the initial pitch and roll
  float aX, aY, aZ, gX, gY, gZ;
  _imu->getSensorData(aX, aY, aZ, gX, gY, gZ);
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
  
  int16_t tmpAX, tmpAY, tmpAZ;
  tmpAX = (int16_t) (aX * ACCEL_AVG_SCALING);
  tmpAY = (int16_t) (aY * ACCEL_AVG_SCALING);
  tmpAZ = (int16_t) (aZ * ACCEL_AVG_SCALING);

  //Get the average acceleration vector.
  updateAverageAccel(tmpAX, tmpAY, tmpAZ);

  aX = ((float) _avgAX) / ACCEL_AVG_SCALING;
  aY = ((float) _avgAY) / ACCEL_AVG_SCALING;
  aZ = ((float) _avgAZ) / ACCEL_AVG_SCALING;

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
 *  Incorporates a new value into the running average acceleration vector.
 */
void OrientationHandler::updateAverageAccel(int16_t newAX, int16_t newAY, int16_t newAZ) { 
  
  //Replace old values in the arrays with the new values.
  int replaceIndex = _numTimesAveraged % AVG_ARRAY_SIZE;
  _avgAXArray[replaceIndex] = newAX;
  _avgAYArray[replaceIndex] = newAY; 
  _avgAZArray[replaceIndex] = newAZ; 

  int32_t aXSum = 0;
  int32_t aYSum = 0;
  int32_t aZSum = 0;

  //Get the average of the arays.
  for (int i = 0; i < AVG_ARRAY_SIZE; i++) {
    aXSum += _avgAXArray[i];
    aYSum += _avgAYArray[i];
    aZSum += _avgAZArray[i];
  }
  
  _avgAX = aXSum / AVG_ARRAY_SIZE;
  _avgAY = aYSum / AVG_ARRAY_SIZE;
  _avgAZ = aZSum / AVG_ARRAY_SIZE;
  
  _numTimesAveraged++;
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


