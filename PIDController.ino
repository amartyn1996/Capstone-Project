#define PID_P_GAIN 1.0
#define PID_I_GAIN 0.0
#define PID_D_GAIN 0.0

float _prevErrorPitch = 0;
float _prevErrorRoll = 0;
float _prevErrorYaw = 0;

float _sumErrorPitch = 0;
float _sumErrorRoll = 0;
float _sumErrorYaw = 0;

/*
 * The PID Controller:
 * Takes in the current orientation and the desired orientation, then returns the
 * required pitch, roll, and yaw throttle needed to reduce the error between the two values.
 * 
 * curPitch, curRoll, curYaw, desiredPitch, desiredRoll, and desiredYaw should be floats with values between -1.0 and 1.0.
 * 
 * The returned values demandPitch, demandRoll, and demandYaw are floats between -1.0 and 1.0.
 */
void PIDControl(float curPitch, float curRoll, float curYaw, float desiredPitch, float desiredRoll, float desiredYaw, float &demandPitch, float &demandRoll, float &demandYaw) {
  //Make sure inputs are within a valid range.
  curPitch     = min(-1.0, max(1.0, curPitch));
  curRoll      = min(-1.0, max(1.0, curRoll));
  curYaw       = min(-1.0, max(1.0, curYaw));
  desiredPitch = min(-1.0, max(1.0, desiredPitch));
  desiredRoll  = min(-1.0, max(1.0, desiredRoll));
  desiredYaw   = min(-1.0, max(1.0, desiredYaw));

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

