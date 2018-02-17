#ifndef PIDController_H
#define PIDController_H

#define PID_P_GAIN_P 2.5
#define PID_I_GAIN_P 0.00
#define PID_D_GAIN_P 300.0
#define PID_P_GAIN_R 2.5
#define PID_I_GAIN_R 0.00
#define PID_D_GAIN_R 300.0
#define PID_P_GAIN_Y 2.5
#define PID_I_GAIN_Y 0.00
#define PID_D_GAIN_Y 300.0
#define NUM_ELEMENTS_TO_AVERAGE 5

class PIDController {
	
	private:
    float* _avgDeltaErrorPitch, *_avgDeltaErrorRoll, *_avgDeltaErrorYaw;
    float _prevErrorPitch, _prevErrorRoll, _prevErrorYaw;
	  float _sumErrorPitch, _sumErrorRoll, _sumErrorYaw;
    uint32_t _numTimesAveraged;
    void getAverageDeltaError(float &deltaErrorPitch, float &deltaErrorRoll, float &deltaErrorYaw);
	
	public:
		PIDController();
		void PIDControl(float curPitch, float curRoll, float curYaw, float desiredPitch, float desiredRoll, float desiredYaw, float &demandPitch, float &demandRoll, float &demandYaw);
	
};

#endif
