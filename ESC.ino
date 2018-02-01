#define MAX_THROTTLE_TIME 2000 //The pulse length representing maximum throttle. Unit is in microseconds.
#define MIN_THROTTLE_TIME 1000 //The pulse length representing minimum throttle. Unit is in microseconds.

#define TEST_ESC 1

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
      pLESC1 = MAX_THROTTLE_TIME;
      pLESC2 = MAX_THROTTLE_TIME;
      pLESC3 = MAX_THROTTLE_TIME;
      pLESC4 = MAX_THROTTLE_TIME;
    }
    else if (iterations < 2000) { //Tell the ESCs that 1000us pulse is min throttle.
      pLESC1 = MIN_THROTTLE_TIME;
      pLESC2 = MIN_THROTTLE_TIME;
      pLESC3 = MIN_THROTTLE_TIME;
      pLESC4 = MIN_THROTTLE_TIME;
    } 

    #if TEST_ESC
      else if (iterations < 3000) { //Throttle up ESC1.
        pLESC1 = iterations - 2000 + MIN_THROTTLE_TIME;
        pLESC2 = MIN_THROTTLE_TIME;
        pLESC3 = MIN_THROTTLE_TIME;
        pLESC4 = MIN_THROTTLE_TIME;
      } else if (iterations < 4000) { //Throttle up ESC2.
        pLESC1 = MIN_THROTTLE_TIME;
        pLESC2 = iterations - 3000 + MIN_THROTTLE_TIME;
        pLESC3 = MIN_THROTTLE_TIME;
        pLESC4 = MIN_THROTTLE_TIME;
      } else if (iterations < 5000) { //Throttle up ESC3.
        pLESC1 = MIN_THROTTLE_TIME;
        pLESC2 = MIN_THROTTLE_TIME;
        pLESC3 = iterations - 4000 + MIN_THROTTLE_TIME;
        pLESC4 = MIN_THROTTLE_TIME;
      } else if (iterations < 6000) { //Throttle up ESC4.
        pLESC1 = MIN_THROTTLE_TIME;
        pLESC2 = MIN_THROTTLE_TIME;
        pLESC3 = MIN_THROTTLE_TIME;
        pLESC4 = iterations - 5000 + MIN_THROTTLE_TIME;
      } else if (iterations < 7000) { //Throttle down all ESCs.
        pLESC1 = MIN_THROTTLE_TIME;
        pLESC2 = MIN_THROTTLE_TIME;
        pLESC3 = MIN_THROTTLE_TIME;
        pLESC4 = MIN_THROTTLE_TIME;
      } else if (iterations < 8000) { //Max throttle to all ESCs
        pLESC1 = MAX_THROTTLE_TIME;
        pLESC2 = MAX_THROTTLE_TIME;
        pLESC3 = MAX_THROTTLE_TIME;
        pLESC4 = MAX_THROTTLE_TIME;
      } else { //No throttle to all ESCs
        pLESC1 = MIN_THROTTLE_TIME;
        pLESC2 = MIN_THROTTLE_TIME;
        pLESC3 = MIN_THROTTLE_TIME;
        pLESC4 = MIN_THROTTLE_TIME;
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

