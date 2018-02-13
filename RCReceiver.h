#ifndef RCReceiver_H
#define RCReceiver_H

#include <Arduino.h>

#define RC_MAX_PULSE_LEN 2000            //
#define RC_MIN_PULSE_LEN 1000            // Microseconds
#define RC_MAX_TIME_WITHOUT_UPDATE 50000 //

class RCReceiver {

  public:
    RCReceiver();
    void initialize();
    void getRCCommands(float &pitch, float &roll, float &yaw, float &throttle);

};

#endif
