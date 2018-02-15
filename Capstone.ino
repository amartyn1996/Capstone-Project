#include <Wire.h>
#include "MPU6050.h"
#include "OrientationHandler.h"
#include "ESC.h"
#include "RCReceiver.h"
#include "PIDController.h"
#include "MiscMath.h"

#define SAFEMODE 0
#define VISUALIZE 0
#define VIS_IMU 0
#define VIS_RC 0
#define VIS_ORIENTATION 0
#define VIS_PID 0
#define VIS_VIBRATION 0


IMU* imu;
ESC* esc;
OrientationHandler* orHand;
RCReceiver* rc;
PIDController* pid;


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
  pid = new PIDController();
    
  Serial.begin(9600);

  Serial.println("Begin ESC Calibrate");
  esc->calibrate();
  Serial.println("End ESC Calibrate");
    
  orHand->initialize();
  orHand->calibrate();

  rc->initialize();
  
  #if VISUALIZE
    visualize();
  #endif

  
}

void loop() {
  static uint32_t lastCycleTime = micros();
  static float pitch = 0;
  static float roll = 0;
  static float yaw = 0;
  static float RCPitch = 0;
  static float RCRoll = 0;
  static float RCYaw = 0;
  static float RCThrottle = 0;
  static float demandPitch = 0;
  static float demandRoll = 0;
  static float demandYaw = 0;
  
  orHand->calcOrientation(pitch, roll, yaw);

  //Convert orientation from degrees to (half radians?). -1.0 straight down, 1.0 straight up.
  pitch /= 90;
  roll /= 90;
  yaw /= 90;
  
  rc->getRCCommands(RCPitch, RCRoll, RCYaw, RCThrottle);

  //Make RC values fall between -1.0 and 1.0
  RCPitch = 2*RCPitch-1;
  RCRoll = 2*RCRoll-1;
  RCYaw = 2*RCYaw-1;

  //NOTE:
  //Pitch is not working on the remote controller I am using.
  //Therefore, I will be using Yaw on the RC for Roll and Roll on the RC for Pitch.
  pid->PIDControl(pitch, 0, 0, 0, 0, 0, demandPitch, demandRoll, demandYaw);
  
  esc->demandControl( demandPitch, demandRoll, demandYaw, RCThrottle, lastCycleTime);
}

/**
 *  Prints information to the serial monitor.
 */
void visualize() {
  
  while (1) {
    static int numCycles = 0;
    
    #if VIS_IMU
      float aX, aY, aZ, gX, gY, gZ;
      imu->getSensorData(aX, aY, aZ, gX, gY, gZ);
      Serial.print("Gyro : "); Serial.print(gX); Serial.print(" -- "); Serial.print(gY);  Serial.print(" -- "); Serial.println(gZ);
      Serial.print("Accel: "); Serial.print(aX); Serial.print(" -- "); Serial.print(aY);  Serial.print(" -- "); Serial.println(aZ); 
      delay(250);
    #endif

    #if VIS_ORIENTATION       
      float pitch, roll, yaw;
      orHand->calcOrientation(pitch, roll, yaw);
      if (numCycles % 30 == 0) {
        Serial.print("Pitch: ");Serial.print(pitch);Serial.print("  Roll: ");Serial.print(roll);Serial.print("  Yaw: ");Serial.println(yaw);
      }        
    #endif

    #if VIS_RC
      float p, r, y, t;
      rc->getRCCommands(p,r,y,t);
      if (numCycles % 30 == 0) {
        Serial.print(" RCPitch : "); Serial.print(p);
        Serial.print(" RCRoll : "); Serial.print(r);
        Serial.print(" RCYaw : "); Serial.print(y);
        Serial.print(" RCThrottle : "); Serial.println(t);
      }      
    #endif

    #if VIS_PID
      float rcp, rcr, rcy, t;
      float p, r, y;
      float dp, dr, dy;
      orHand->calcOrientation(p, r, y);
      rc->getRCCommands(rcp,rcr,rcy,t);
      pid->PIDControl(p/90, 0, 0, 0, 0, 0, dp, dr, dy);
      if (numCycles % 30 == 0) {
        //Serial.print("Pitch: ");Serial.print(p/90);Serial.print("  Roll: ");Serial.print(r/90);Serial.print("  Yaw: ");Serial.println(y/90);
        Serial.print(" DemandPitch: "); Serial.print(dp);
        Serial.print(" DemandRoll: "); Serial.print(dr);
        Serial.print(" DemandYaw: "); Serial.println(dy);
      }      
    #endif

    #if VIS_VIBRATION
      while(1) {
        static uint32_t lastCycleTime = micros();
        float rcp, rcr, rcy, t;
        float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
        static float magGyro = 0;
        
        imu->getSensorData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
        
        magGyro += MiscMath::vecLength(gyroX, gyroY, gyroZ);
        
        rc->getRCCommands(rcp,rcr,rcy,t);
        t = min(.25, t);

        esc->pulseESCs( 0, 0, 0, t, lastCycleTime);

        if (numCycles % 200 == 0) {
          Serial.println(magGyro/200);
          magGyro = 0;
        }
        
        numCycles++;
      }
    #endif
    
    delay(3);
    numCycles++;
  }  
}


