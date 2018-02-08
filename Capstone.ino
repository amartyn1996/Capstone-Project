#include <Wire.h>
#include "MPU6050.h"
#include "OrientationHandler.h"
#include "ESC.h"
#include "RCReceiver.h"

#define SAFEMODE 0
#define VISUALIZE 0
#define VIS_IMU 0
#define VIS_ORIENTATION 0


IMU* imu;
ESC* esc;
OrientationHandler* orHand;
RCReceiver* rc;


float throttle1=0, throttle2=0, throttle3=0, throttle4=0;

void setup() {
  
  while (SAFEMODE)
    delay(1000);

  DDRD |= 0b11110000;
  PORTD |= 0b11110000;

  esc = new ESC(4000);
  imu = new MPU6050();
  orHand = new OrientationHandler(imu);
  rc = new RCReceiver();
    
  Serial.begin(9600);

  //Serial.println("Begin ESC Calibrate");
  //esc->calibrate();
  //Serial.println("End ESC Calibrate");
    
  //orHand->initialize();
  //orHand->calibrate();

  rc->initialize();
  
  #if VISUALIZE
    visualize();
  #endif

  
}

void loop() {
  //static uint32_t lastCycleTime = micros();
  //static float pitch = 0;
  //static float roll = 0;
  
  //orHand->calcOrientation(pitch, roll);
  
  //esc->demandControl(-pitch/90, -roll/90, .25, lastCycleTime);
  static int iterator = 0;
  if (iterator % 10000 == 0) {
    float p, r, y, t;
    rc->getRCCommands(p,r,y,t);
    
    Serial.print(" Pitch : "); Serial.print(p);
    Serial.print(" Roll : "); Serial.print(r);
    Serial.print(" Yaw : "); Serial.print(y);
    Serial.print(" Throttle : "); Serial.println(t);
  }
  iterator++;
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


