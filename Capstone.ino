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

volatile uint8_t pinsState = 0;
volatile uint32_t startTimeOnPin8 = 0;
volatile uint32_t startTimeOnPin9 = 0;
volatile uint32_t startTimeOnPin10 = 0;
volatile uint32_t startTimeOnPin11 = 0;
volatile uint32_t timeOnPin8 = 0;
volatile uint32_t timeOnPin9 = 0;
volatile uint32_t timeOnPin10 = 0;
volatile uint32_t timeOnPin11 = 0;

void setup() {
  
  while (SAFEMODE)
    delay(1000);

  DDRD |= 0b11110000;
  PORTD |= 0b11110000;

  DDRB &= 0b11110000; //Set pins 8-11 to input.
  PCICR |= (1<<PCIE0); //Enable pin change interupts on PortB.
  PCMSK0 |= (0xF<<PCINT8); //Enagle pin change interupts on pins 8-11.

  esc = new ESC(4000);
  imu = new MPU6050();
  orHand = new OrientationHandler(imu);
    
  Serial.begin(9600);

  //Serial.println("Begin ESC Calibrate");
  //esc->calibrate();
  //Serial.println("End ESC Calibrate");
    
  //orHand->initialize();
  //orHand->calibrate();
  
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
    Serial.print(" Pin8 : "); Serial.print(timeOnPin8);
    Serial.print(" Pin9 : "); Serial.print(timeOnPin9);
    Serial.print(" PinA : "); Serial.print(timeOnPin10);
    Serial.print(" PinB : "); Serial.println(timeOnPin11);
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

/*
 *  Interrupt Service Routine for pin changes on PORTB.
 *  Measures the time between pulses coming from the RF receiver.
 * 
 *  NOTE:
 *  PIN 8  is roll.
 *  PIN 9  is throttle.
 *  PIN 10 is pitch.
 *  PIN 11 is yaw.
 */
ISR(PCINT0_vect) {
  uint8_t curPinState = PINB;
  uint8_t prevPinState = pinsState;
  uint8_t changedPins = curPinState ^ prevPinState;
  uint8_t onlyChangedPins = curPinState & changedPins;
  uint32_t currentTime = micros();

  //If the pin turned high, set the starting time to the current time.
  startTimeOnPin8  = (onlyChangedPins & 0b00000001) ? currentTime : startTimeOnPin8;
  startTimeOnPin9  = (onlyChangedPins & 0b00000010) ? currentTime : startTimeOnPin9;
  startTimeOnPin10 = (onlyChangedPins & 0b00000100) ? currentTime : startTimeOnPin10;
  startTimeOnPin11 = (onlyChangedPins & 0b00001000) ? currentTime : startTimeOnPin11;

  //If the pin turned low, we now know the length of the pulse.
  timeOnPin8   = ((changedPins & 0x1) && !(curPinState & 0x1)) ? (currentTime - startTimeOnPin8)  : timeOnPin8;
  timeOnPin9   = ((changedPins & 0x2) && !(curPinState & 0x2)) ? (currentTime - startTimeOnPin9)  : timeOnPin9;
  timeOnPin10  = ((changedPins & 0x4) && !(curPinState & 0x4)) ? (currentTime - startTimeOnPin10) : timeOnPin10;
  timeOnPin11  = ((changedPins & 0x8) && !(curPinState & 0x8)) ? (currentTime - startTimeOnPin11) : timeOnPin11;

  pinsState = curPinState;
}
