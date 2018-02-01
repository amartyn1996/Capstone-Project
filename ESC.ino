#define MAX_PULSE_TIME 2000 //The pulse length representing maximum throttle. Unit is in microseconds.
#define MIN_PULSE_TIME 1000 //The pulse length representing minimum throttle. Unit is in microseconds.

#define TEST_ESC 0

int _cycleLength = 4000;

/**
 *  Tells the ESCs what the maximum and minimum pulse length are. (Max throttle and minimum throttle)
 */
void calibrate() {

  int numIter = (TEST_ESC) ? 9000 : 2000;
  int loopTime = 4000;
  int pLESC1 = 0;
  int pLESC2 = 0;
  int pLESC3 = 0;
  int pLESC4 = 0;
  
  DDRD |= 0b11110000;
  PORTD |= 0b11110000;
  
  uint32_t beginLoopTime = micros();

  //A single run of this loop should represent a single pulse to the ESCs.
  for (int iterations = 0; iterations < numIter; iterations++) { 
    
    if (iterations < 1500) { //Tell the ESCs that 2000us pulse is max throttle.
      pLESC1 = MAX_PULSE_TIME;
      pLESC2 = MAX_PULSE_TIME;
      pLESC3 = MAX_PULSE_TIME;
      pLESC4 = MAX_PULSE_TIME;
    }
    else if (iterations < 2000) { //Tell the ESCs that 1000us pulse is min throttle.
      pLESC1 = MIN_PULSE_TIME;
      pLESC2 = MIN_PULSE_TIME;
      pLESC3 = MIN_PULSE_TIME;
      pLESC4 = MIN_PULSE_TIME;
    } 

    #if TEST_ESC
      else if (iterations < 3000) { //Throttle up ESC1.
        pLESC1 = iterations - 2000 + MIN_PULSE_TIME;
        pLESC2 = MIN_PULSE_TIME;
        pLESC3 = MIN_PULSE_TIME;
        pLESC4 = MIN_PULSE_TIME;
      } else if (iterations < 4000) { //Throttle up ESC2.
        pLESC1 = MIN_PULSE_TIME;
        pLESC2 = iterations - 3000 + MIN_PULSE_TIME;
        pLESC3 = MIN_PULSE_TIME;
        pLESC4 = MIN_PULSE_TIME;
      } else if (iterations < 5000) { //Throttle up ESC3.
        pLESC1 = MIN_PULSE_TIME;
        pLESC2 = MIN_PULSE_TIME;
        pLESC3 = iterations - 4000 + MIN_PULSE_TIME;
        pLESC4 = MIN_PULSE_TIME;
      } else if (iterations < 6000) { //Throttle up ESC4.
        pLESC1 = MIN_PULSE_TIME;
        pLESC2 = MIN_PULSE_TIME;
        pLESC3 = MIN_PULSE_TIME;
        pLESC4 = iterations - 5000 + MIN_PULSE_TIME;
      } else if (iterations < 7000) { //Throttle down all ESCs.
        pLESC1 = MIN_PULSE_TIME;
        pLESC2 = MIN_PULSE_TIME;
        pLESC3 = MIN_PULSE_TIME;
        pLESC4 = MIN_PULSE_TIME;
      } else if (iterations < 8000) { //Max throttle to all ESCs
        pLESC1 = MAX_PULSE_TIME;
        pLESC2 = MAX_PULSE_TIME;
        pLESC3 = MAX_PULSE_TIME;
        pLESC4 = MAX_PULSE_TIME;
      } else { //No throttle to all ESCs
        pLESC1 = MIN_PULSE_TIME;
        pLESC2 = MIN_PULSE_TIME;
        pLESC3 = MIN_PULSE_TIME;
        pLESC4 = MIN_PULSE_TIME;
      }
    #endif
    
    //Wait until the loop time is over. Consistency is very important.
    while(micros() <  beginLoopTime + loopTime);
    //Begin the pulse
    PORTD |= 0b11110000;
    beginLoopTime = micros();

    while(PORTD & 0b11110000) {
      //Turn off the pulses when they are done.
      uint32_t curTime = micros();
      PORTD &= (curTime < beginLoopTime + pLESC1) ? 0xFF : 0b11101111;
      PORTD &= (curTime < beginLoopTime + pLESC2) ? 0xFF : 0b11011111;
      PORTD &= (curTime < beginLoopTime + pLESC3) ? 0xFF : 0b10111111;
      PORTD &= (curTime < beginLoopTime + pLESC4) ? 0xFF : 0b01111111;
    }

  }
}

/**
 *  Sends a pulse to the ESCs. The length of the pulse is determined by the throttle (0.0 - 1.0).
 *  Also, it ensures that the main loop cycle time stays consistent.
 *  Consistent timed pulses are very important for the proper function of the ESCs.
 */
void pulseESCs(float throttle1, float throttle2, float throttle3, float throttle4, uint32_t &lastCycleTime) {
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

