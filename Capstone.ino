#include <Wire.h>
#include "MPU6050.h"
#include "OrientationHandler.h"

#define SAFEMODE 0
#define VISUALIZE 0
#define VIS_IMU 0
#define VIS_ORIENTATION 0

#define PULSE_LENGTH 1500
#define LOOP_TIME 4000


IMU* imu;
OrientationHandler* orHand;
uint32_t beginLoopTime = 0;

void setup() {
  
  while (SAFEMODE)
    delay(1000);

  imu = new MPU6050();
  orHand = new OrientationHandler(imu);
    
  Serial.begin(9600);  
  //orHand->initialize();
  //orHand->calibrate();
  
  #if VISUALIZE
    visualize();
  #endif

  //PORTD |= 0b01000000;
  //DDRD &= 0b00001111;
}

void loop() {

  calESC();  

}

void calESC() {

  int iterations = 0;
  
  int loopTime = 4000;
  int pulseLength = 2000;

  DDRD |= 0b11110000;
  PORTD |= 0b11110000;
  Serial.println("Begin");
  //delay(5000);
  beginLoopTime = micros();
  
  while (1) {

    if (iterations % 400 == 0)
      Serial.println(pulseLength);
    
    if (iterations < 1000)
      pulseLength = 2000;
    else if (iterations < 1500)
      pulseLength = 1000;
    else
      pulseLength = 1000 * abs(sin(iterations * .001)) + 1000;
    
    while(micros() <  beginLoopTime + loopTime);
    PORTD |= 0b11110000;
    beginLoopTime = micros();
  

    while(micros() < beginLoopTime + pulseLength);
    PORTD &= 0b00001111;

    iterations++;
  }
}

/**
 *  
 */
void visualize() {
    while (1) {
      #if VIS_IMU
        float aX, aY, aZ, gX, gY, gZ;
        imu->getSensorData(aX, aY, aZ, gX, gY, gZ);
        Serial.print("Gyro : "); Serial.print(gX); Serial.print(" -- "); Serial.print(gY);  Serial.print(" -- "); Serial.println(gZ);
        Serial.print("Accel: "); Serial.print(aX); Serial.print(" -- "); Serial.print(aY);  Serial.print(" -- "); Serial.println(aZ); 
        delay(250);
      #endif
  
      #if VIS_ORIENTATION
        static int numCycles = 0;
        float pitch, roll;
        orHand->calcOrientation(pitch, roll);
        if (numCycles % 30 == 0) {
          Serial.print("Pitch: ");Serial.print(pitch);Serial.print("  Roll: ");Serial.println(roll);
        }
        numCycles++;
        delay(3);
      #endif
    }
}

