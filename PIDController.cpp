#include <Arduino.h>
#include "PIDController.h"

PIDController::PIDController() {
  _numTimesAveraged = 0;
	_prevErrorPitch = 0;
	_prevErrorRoll = 0;
	_prevErrorYaw = 0;
	_sumErrorPitch = 0;
	_sumErrorRoll = 0;
	_sumErrorYaw = 0;
  _previousTime = 0;
  _avgDeltaErrorPitch = (float*) malloc(NUM_ELEMENTS_TO_AVERAGE * sizeof(float));
  _avgDeltaErrorRoll  = (float*) malloc(NUM_ELEMENTS_TO_AVERAGE * sizeof(float));;
  _avgDeltaErrorYaw   = (float*) malloc(NUM_ELEMENTS_TO_AVERAGE * sizeof(float));;
}

/*
 * The PID Controller:
 * Takes in the current orientation and the desired orientation, then returns the
 * required pitch, roll, and yaw throttle needed to reduce the error between the two values.
 * 
 * curPitch, curRoll, curYaw, desiredPitch, desiredRoll, and desiredYaw should be floats with values between -1.0 and 1.0.
 * 
 * The returned values demandPitch, demandRoll, and demandYaw are floats between -1.0 and 1.0.
 */
void PIDController::PIDControl(float curPitch, float curRoll, float curYaw, float desiredPitch, float desiredRoll, float desiredYaw, float &demandPitch, float &demandRoll, float &demandYaw) {
  
  uint32_t currentTime = micros();
  uint32_t deltaTime = currentTime - _previousTime;
  _previousTime = currentTime;
  float timeFraction = EXPECTED_DELTA_TIME / ((float) deltaTime);
  
  //Make sure inputs are within a valid range.
  curPitch     = max(-1.0, min(1.0, curPitch));
  curRoll      = max(-1.0, min(1.0, curRoll));
  curYaw       = max(-1.0, min(1.0, curYaw));
  desiredPitch = max(-1.0, min(1.0, desiredPitch));
  desiredRoll  = max(-1.0, min(1.0, desiredRoll));
  desiredYaw   = max(-1.0, min(1.0, desiredYaw));

  //Get the current error. (Proportion)
  float errorPitch = desiredPitch - curPitch;
  float errorRoll  = desiredRoll - curRoll;
  float errorYaw   = desiredYaw - curYaw;

  //Get the accumulated error. (Integral)
  _sumErrorPitch += errorPitch * timeFraction;
  _sumErrorRoll  += errorRoll * timeFraction;
  _sumErrorYaw   += errorYaw * timeFraction;

  //Get the change in error. (Derivative)
  
  //Update the average delta error
  int index = _numTimesAveraged % NUM_ELEMENTS_TO_AVERAGE;
  _numTimesAveraged++;
  _avgDeltaErrorPitch[index] = (errorPitch - _prevErrorPitch) * timeFraction;
  _avgDeltaErrorRoll[index]  = (errorRoll  - _prevErrorRoll) * timeFraction;
  _avgDeltaErrorYaw[index]   = (errorYaw   - _prevErrorYaw) * timeFraction;

  //Get the average delta error
  float deltaErrorPitch, deltaErrorRoll, deltaErrorYaw;
  getAverageDeltaError(deltaErrorPitch, deltaErrorRoll, deltaErrorYaw);

  //Return the demanded control.
  demandPitch = (errorPitch * PID_P_GAIN_P) + (_sumErrorPitch * PID_I_GAIN_P) + (deltaErrorPitch * PID_D_GAIN_P);
  demandRoll  = (errorRoll  * PID_P_GAIN_R) + (_sumErrorRoll  * PID_I_GAIN_R) + (deltaErrorRoll  * PID_D_GAIN_R);
  demandYaw   = (errorYaw   * PID_P_GAIN_Y) + (_sumErrorYaw   * PID_I_GAIN_Y) + (deltaErrorYaw   * PID_D_GAIN_Y);
  
  //Update the previous error.
  _prevErrorPitch = errorPitch;
  _prevErrorRoll  = errorRoll;
  _prevErrorYaw   = errorYaw;

}

void PIDController::getAverageDeltaError(float &deltaErrorPitch, float &deltaErrorRoll, float &deltaErrorYaw) {
  float avgP = 0;
  float avgR = 0;
  float avgY = 0;
  
  for (int i = 0; i < NUM_ELEMENTS_TO_AVERAGE; i++) {
    avgP += _avgDeltaErrorPitch[i];
    avgR += _avgDeltaErrorRoll[i];
    avgY += _avgDeltaErrorYaw[i];
  }

  avgP /= NUM_ELEMENTS_TO_AVERAGE;
  avgR /= NUM_ELEMENTS_TO_AVERAGE;
  avgY /= NUM_ELEMENTS_TO_AVERAGE;

  deltaErrorPitch = avgP;
  deltaErrorRoll  = avgR;
  deltaErrorYaw   = avgY;
}

