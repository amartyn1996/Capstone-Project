#include <Arduino.h>
#include "RCReceiver.h"

static volatile uint8_t _pinsState;
static volatile uint32_t _startTimeOnPin8, _startTimeOnPin9, _startTimeOnPin10, _startTimeOnPin11;
static volatile uint32_t _timeOnPin8, _timeOnPin9, _timeOnPin10, _timeOnPin11;
static volatile uint32_t _timeLastUpdate; 

RCReceiver::RCReceiver() {
  _pinsState = 0;
  _startTimeOnPin8 = 0; _startTimeOnPin9 = 0; _startTimeOnPin10 = 0; _startTimeOnPin11 = 0;
  _timeOnPin8 = 0; _timeOnPin9 = 0; _timeOnPin10 = 0; _timeOnPin11 = 0;
  _timeLastUpdate = 0;
}

/**
 * Enables pin change interupts on pins 8-11.
 */
void RCReceiver::initialize() {
  DDRB &= 0b11110000; //Set pins 8-11 to input.
  PCICR |= (1<<PCIE0); //Enable pin change interupts on PortB.
  PCMSK0 |= (0xF<<PCINT8); //Enable pin change interupts on pins 8-11.
}

/**
 * Returns information sent from the remote controller.
 * Range: 0.0 - 1.0
 */
void RCReceiver::getRCCommands(float &pitch, float &roll, float &yaw, float &throttle) {

  uint32_t maxTimeOnPin = max(_timeOnPin8, max(_timeOnPin9, max(_timeOnPin10, _timeOnPin11)));
  
  if (micros() - _timeLastUpdate < RC_MAX_TIME_WITHOUT_UPDATE && maxTimeOnPin < RC_MAX_ALLOWED_TIME_ON_PIN) {
    pitch    = (float) min(RC_MAX_PULSE_LEN, max(RC_MIN_PULSE_LEN, _timeOnPin10)) / RC_MIN_PULSE_LEN - 1.0;
    roll     = (float) min(RC_MAX_PULSE_LEN, max(RC_MIN_PULSE_LEN, _timeOnPin8))  / RC_MIN_PULSE_LEN - 1.0;
    yaw      = (float) min(RC_MAX_PULSE_LEN, max(RC_MIN_PULSE_LEN, _timeOnPin11)) / RC_MIN_PULSE_LEN - 1.0;
    throttle = (float) min(RC_MAX_PULSE_LEN, max(RC_MIN_PULSE_LEN, _timeOnPin9))  / RC_MIN_PULSE_LEN - 1.0;
  } else {
    pitch    = 0.0;
    roll     = 0.0;
    yaw      = 0.0;
    throttle = 0.0;
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
  uint8_t prevPinState = _pinsState;
  uint8_t changedPins = curPinState ^ prevPinState;
  uint8_t onlyChangedPins = curPinState & changedPins;
  uint32_t currentTime = micros();

  //If the pin turned high, set the starting time to the current time.
  _startTimeOnPin8  = (onlyChangedPins & 0b00000001) ? currentTime : _startTimeOnPin8;
  _startTimeOnPin9  = (onlyChangedPins & 0b00000010) ? currentTime : _startTimeOnPin9;
  _startTimeOnPin10 = (onlyChangedPins & 0b00000100) ? currentTime : _startTimeOnPin10;
  _startTimeOnPin11 = (onlyChangedPins & 0b00001000) ? currentTime : _startTimeOnPin11;

  //If the pin turned low, we now know the length of the pulse.
  _timeOnPin8   = ((changedPins & 0x1) && !(curPinState & 0x1)) ? (currentTime - _startTimeOnPin8)  : _timeOnPin8;
  _timeOnPin9   = ((changedPins & 0x2) && !(curPinState & 0x2)) ? (currentTime - _startTimeOnPin9)  : _timeOnPin9;
  _timeOnPin10  = ((changedPins & 0x4) && !(curPinState & 0x4)) ? (currentTime - _startTimeOnPin10) : _timeOnPin10;
  _timeOnPin11  = ((changedPins & 0x8) && !(curPinState & 0x8)) ? (currentTime - _startTimeOnPin11) : _timeOnPin11;

  _pinsState = curPinState;
  _timeLastUpdate = currentTime;
}
