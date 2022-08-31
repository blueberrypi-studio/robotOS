// ======== Robot States ===========
typedef enum {
  WAITING,
  EXECUTING,
  UNTETHERED
}robotState;
// =================================

// ======= init vars ===============

typedef enum {
  STRAIGHT,
  TURNING
} movementType;

typedef enum {
  FORWARDS,
  REVERSE,
  CLOCKWISE,
  ANTICLOCKWISE
} newDirection;

int waitingLEDCounter = 0;

int motorSpeed = 100;

int movement = 0;
unsigned int rpm;
volatile byte pulses;
unsigned long time_old;
unsigned int pulses_per_turn= 20; // Depends on the number of spokes on the encoder wheel

long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

// =================================

// ======= init pins ===============
const byte functionLED = A0;
const byte leftMovementLED = 4;
const byte rightMovementLED = 3;

const byte encoderLeft = 2;
const byte encoderRight = A1;

const byte sensorTrig = A2;
const byte sensorEcho = A3;

const byte motorLeftA = 10; //PWM
const byte motorLeftB = 9; //PWM

const byte motorRightA = 6; //PWM
const byte motorRightB = 5; //PWM



// =================================
