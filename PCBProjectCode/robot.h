// ======== Robot States ===========
typedef enum {
  WAITING,
  EXECUTING,
  UNTETHERED
}robotState;
// =================================

// ======= init vars ===============

int waitingLEDCounter = 0;
int movementLEDCounter = 0;

// =================================

// ======= init pins ===============
const byte functionLED = A0;
const byte leftMovementLED = 4;
const byte rightMovementLED = 3;

const byte encoderA = 2;
const byte encoderB = A1;

const byte sensorTrig = A2;
const byte sensorEcho = A3;

const byte motorLeftA = 10; //PWM
const byte motorLeftB = 9; //PWM

const byte motorRightA = 6; //PWM
const byte motorRightB = 5; //PWM



// =================================