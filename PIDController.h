#ifndef PIDController_H
#define PIDController_H

#define PID_P_GAIN_P 0.10
#define PID_I_GAIN_P 2.50
#define PID_D_GAIN_P 150.0
#define PID_P_GAIN_R 0.10
#define PID_I_GAIN_R 2.50
#define PID_D_GAIN_R 150.0
#define PID_P_GAIN_Y 0.10
#define PID_I_GAIN_Y 2.50
#define PID_D_GAIN_Y 150.0
#define NUM_ELEMENTS_TO_AVERAGE 9
#define EXPECTED_DELTA_TIME 4000.0
#define MAX_SUM_ERROR 0.02

class PIDController {
	
	private:
    float* _avgDeltaErrorPitch, *_avgDeltaErrorRoll, *_avgDeltaErrorYaw;
    float _prevErrorPitch, _prevErrorRoll, _prevErrorYaw;
	  float _sumErrorPitch, _sumErrorRoll, _sumErrorYaw;
    uint32_t _numTimesAveraged;
    void getAverageDeltaError(float &deltaErrorPitch, float &deltaErrorRoll, float &deltaErrorYaw);
    uint32_t _previousTime;
	
	public:
		PIDController();
		void PIDControl(float curPitch, float curRoll, float curYaw, float desiredPitch, float desiredRoll, float desiredYaw, float &demandPitch, float &demandRoll, float &demandYaw);
	
};

#endif
