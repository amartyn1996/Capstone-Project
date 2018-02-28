#ifndef ESC_h
#define ESC_h

#include <Arduino.h>
#include "IMU.h"

#define MAX_PULSE_TIME 2000 //The pulse length representing maximum throttle. Unit is in microseconds.
#define MIN_PULSE_TIME 1000 //The pulse length representing minimum throttle. Unit is in microseconds.

class ESC {
	
	private:
		int _cycleLength;
    
	
	public:
		ESC(int cycleLength);	
    void demandControl(float pitch, float roll, float yaw, float throttle, uint32_t &lastCycleTime);
    void pulseESCs(float throttle1, float throttle2, float throttle3, float throttle4, uint32_t &lastCycleTime);
};

#endif
