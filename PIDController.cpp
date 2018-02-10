#include <Arduino.h>
#include "PIDController.h"

PIDController::PIDController() {
	_prevErrorPitch = 0;
	_prevErrorRoll = 0;
	_prevErrorYaw = 0;
	_sumErrorPitch = 0;
	_sumErrorRoll = 0;
	_sumErrorYaw = 0;
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
  //Make sure inputs are within a valid range.
  curPitch     = max(-1.0, min(1.0, curPitch));
  curRoll      = max(-1.0, min(1.0, curRoll));
  curYaw       = max(-1.0, min(1.0, curYaw));
  desiredPitch = max(-1.0, min(1.0, desiredPitch));
  desiredRoll  = max(-1.0, min(1.0, desiredRoll));
  desiredYaw   = max(-1.0, min(1.0, desiredYaw));

  //Get the current error. (Proportion)
  float errorPitch = desiredPitch - curPitch;
  float errorRoll = desiredRoll - curRoll;
  float errorYaw = desiredYaw - curYaw;

  //Get the accumulated error. (Integral)
  _sumErrorPitch += errorPitch;
  _sumErrorRoll += errorRoll;
  _sumErrorYaw += errorYaw;

  //Get the change in error. (Derivative)
  float deltaErrorPitch = errorPitch - _prevErrorPitch;
  float deltaErrorRoll = errorRoll - _prevErrorRoll;
  float deltaErrorYaw = errorYaw - _prevErrorYaw;

  //Return the demanded control.
  demandPitch = (errorPitch * PID_P_GAIN) + (_sumErrorPitch * PID_I_GAIN) + (deltaErrorPitch * PID_D_GAIN);
  demandRoll  = (errorRoll  * PID_P_GAIN) + (_sumErrorRoll  * PID_I_GAIN) + (deltaErrorRoll  * PID_D_GAIN);
  demandYaw   = (errorYaw   * PID_P_GAIN) + (_sumErrorYaw   * PID_I_GAIN) + (deltaErrorYaw   * PID_D_GAIN);

  //Update the previous error.
  _prevErrorPitch = errorPitch;
  _prevErrorRoll = errorRoll;
  _prevErrorYaw = errorYaw;
}
