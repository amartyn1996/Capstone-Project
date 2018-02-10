#ifndef PIDController_H
#define PIDController_H

#define PID_P_GAIN 1.0
#define PID_I_GAIN 0.0
#define PID_D_GAIN 0.0

class PIDController {
	
	private:
		float _prevErrorPitch, _prevErrorRoll, _prevErrorYaw;
		float _sumErrorPitch, _sumErrorRoll, _sumErrorYaw;
	
	public:
		PIDController();
		void PIDControl(float curPitch, float curRoll, float curYaw, float desiredPitch, float desiredRoll, float desiredYaw, float &demandPitch, float &demandRoll, float &demandYaw);
	
};

#endif