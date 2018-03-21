#include <Arduino.h>
#include "ESC.h"

ESC::ESC(int cycleLength) {
  _cycleLength = cycleLength;
}

/**
 *  Sends a pulse to the ESCs. The length of the pulse is determined by the throttle (0.0 - 1.0).
 *  Also, it ensures that the main loop cycle time stays consistent.
 *  Consistent timed pulses are very important for the proper function of the ESCs.
 */
void ESC::pulseESCs(float throttle1, float throttle2, float throttle3, float throttle4, uint32_t &lastCycleTime) {
  
  //Make sure throttles are between 0.0 and 1.0.
  throttle1 = max(0, min(1, throttle1));
  throttle2 = max(0, min(1, throttle2));
  throttle3 = max(0, min(1, throttle3));
  throttle4 = max(0, min(1, throttle4));

  //Calculate how long to pulse each ESC.
  int pulseRange = MAX_PULSE_TIME - MIN_PULSE_TIME;
  int pulseLen1 = throttle1 * pulseRange + MIN_PULSE_TIME;
  int pulseLen2 = throttle2 * pulseRange + MIN_PULSE_TIME;
  int pulseLen3 = throttle3 * pulseRange + MIN_PULSE_TIME;
  int pulseLen4 = throttle4 * pulseRange + MIN_PULSE_TIME;

  //Wait until cycle time is over,
  while(micros() <  lastCycleTime + _cycleLength);
  //Begin the pulse
  PORTD |= 0b11110000;
  //Start a new cycle.
  lastCycleTime = micros();

  while(PORTD & 0b11110000) {
    //Turn off the pulses when they are done.
    uint32_t curTime = micros();
    PORTD &= (curTime < lastCycleTime + pulseLen1) ? 0xFF : 0b11101111;
    PORTD &= (curTime < lastCycleTime + pulseLen2) ? 0xFF : 0b11011111;
    PORTD &= (curTime < lastCycleTime + pulseLen3) ? 0xFF : 0b10111111;
    PORTD &= (curTime < lastCycleTime + pulseLen4) ? 0xFF : 0b01111111;
  }

}

/**
 *  Sends power to the ESCs depending the throttle and how much pitch and roll are desired.
 *  Pitch, Roll, and Yaw have a range between -1.0 and 1.0.
 *  Throttle has a range between 0.0 and 1.0.
 */
void ESC::demandControl(float pitch, float roll, float yaw, float throttle, uint32_t &lastCycleTime) {

  throttle *= .75;
  
  float t1 = throttle, t2 = throttle, t3 = throttle, t4 = throttle;
  throttle = max(0, min(1, throttle));
  pitch = max(-1, min(1, pitch)) * throttle;
  roll = max(-1, min(1, roll)) * throttle;
  yaw = max(-1, min(1, yaw)) * throttle;

  t1 +=  pitch - roll + yaw; //Front left motor.
  t2 +=  pitch + roll - yaw; //Front right motor.
  t3 += -pitch + roll + yaw; //Back right motor.
  t4 += -pitch - roll - yaw; //Back left motor.

  t1 = min(max(throttle/2, t1), .9);
  t2 = min(max(throttle/2, t2), .9);
  t3 = min(max(throttle/2, t3), .9);
  t4 = min(max(throttle/2, t4), .9);

  
  pulseESCs(t1,t2,t3,t4,lastCycleTime);
}



