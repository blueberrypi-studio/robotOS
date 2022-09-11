#include <Arduino.h>

// ======== Robot States ===========
typedef enum {
  WAITING,
  EXECUTING,
  UNTETHERED
} robotState;
// =================================

// ======= init vars ===============

typedef enum {
  STRAIGHT,
  TURNING
} movementType;

typedef enum {
  STATIONARY,
  FORWARDS,
  REVERSE,
  CLOCKWISE,
  ANTICLOCKWISE
} newDirection;

typedef enum {
  leftSTATIONARY,
  leftFORWARDS,
  leftREVERSE
} leftMotorDirection;

typedef enum {
  rightSTATIONARY,
  rightFORWARDS,
  rightREVERSE
} rightMotorDirection;

const int motorSpeed = 100;
const float wheelCircumference = 21.3;

const long LEDFlashInterval = 500;  // interval at which to blink (milliseconds)
unsigned long currentMillis;

// =========== init function LED ============

int functionLEDState = LOW;
unsigned long functionLEDPreviousMillis = 0;

// ==========================================

// =========== init leftMotor LED ===========

int leftMotorLEDState = LOW;
unsigned long leftMotorPreviousMillis = 0;

// ==========================================

// =========== init rightMotor LED ===========

int rightMotorLEDState = LOW;
unsigned long rightMotorPreviousMillis = 0;

// ==========================================

// ========== encoderLeft init =============

int encoderLeftCount = 1;
int encoderLeftState;
int encoderLeftStateOld = LOW;

// ==========================================

// ========== encoderRight init =============

int encoderRightCount = 1;
int encoderRightState;
int encoderRightStateOld = LOW;

// ==========================================

int movement = 0;

long duration;  // variable for the duration of sound wave travel
int distance;   // variable for the distance measurement

int distanceCM;
int turn;

// =================================

// ======= init pins ===============
#define functionLED A0      // Status LED
#define leftMovementLED 4   // Left LED
#define rightMovementLED 3  // Right LED

#define encoderLeft 2    // Left Encoder Data pin
#define encoderRight A1  // Right Encoder Data pin

#define sensorTrig A2  // Ultrasonic Sensor Trigger pin
#define sensorEcho A3  // Ultrasonic Sensor Data pin

#define motorLeftA 10  //PWM
#define motorLeftB 9   //PWM

#define motorRightA 6  //PWM
#define motorRightB 5  //PWM \
                       // =================================