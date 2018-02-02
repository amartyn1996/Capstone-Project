#include <Wire.h>
#include "MPU6050.h"
#include "OrientationHandler.h"
#include "ESC.h"

#define SAFEMODE 0
#define VISUALIZE 0
#define VIS_IMU 0
#define VIS_ORIENTATION 0


IMU* imu;
ESC* esc;
OrientationHandler* orHand;
float throttle1=0, throttle2=0, throttle3=0, throttle4=0;

void setup() {
  
  while (SAFEMODE)
    delay(1000);

  DDRD |= 0b11110000;
  PORTD |= 0b11110000;

  esc = new ESC(4000);
  imu = new MPU6050();
  orHand = new OrientationHandler(imu);
    
  Serial.begin(9600);

  Serial.println("Begin ESC Calibrate");
  esc->calibrate();
  Serial.println("End ESC Calibrate");
    
  orHand->initialize();
  orHand->calibrate();
  
  #if VISUALIZE
    visualize();
  #endif

  
}

void loop() {
  static uint32_t lastCycleTime = micros();
  static int testIter = 0;
  float t = abs(sin(testIter * .001));
  throttle1 = abs(sin(testIter * .001));
  throttle2 = abs(sin((testIter * .001) + PI/4));
  throttle3 = abs(sin((testIter * .001) + PI/2));
  throttle4 = abs(sin((testIter * .001) + 3*PI/4));

  if (testIter % 400 == 0)
    Serial.println(t);
  
  esc->pulseESCs(throttle1, throttle2, throttle3, throttle4, lastCycleTime);

  testIter++;
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

