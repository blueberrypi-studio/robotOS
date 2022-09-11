// =================================
// Group 24 Arduino code
// Haydn Boul
// Nathan van Slooten
// Arabella Cryer
// =================================

#include "robot.h"

robotState Robot;

movementType robotMovement;
newDirection robotDirection;

leftMotorDirection leftDirection;
rightMotorDirection rightDirection;

// ============= SETUP =============
void setup() {
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");

  pinMode(functionLED, OUTPUT);
  pinMode(leftMovementLED, OUTPUT);
  pinMode(rightMovementLED, OUTPUT);

  pinMode(motorLeftA, OUTPUT);  // Motor Setup
  pinMode(motorLeftB, OUTPUT);  // Motor Setup

  pinMode(motorRightA, OUTPUT);  // Motor Setup
  pinMode(motorRightB, OUTPUT);  // Motor Setup

  pinMode(sensorTrig, OUTPUT);
  pinMode(sensorEcho, INPUT);

  pinMode(encoderLeft, INPUT);   // Setting encoder pin as Input
  pinMode(encoderRight, INPUT);  // Setting encoder pin as Input

  Robot = WAITING;                   // Set initial Robot state
  robotDirection = FORWARDS;         // Set initial Robot movement state
  leftDirection = leftSTATIONARY;    // Set initial left motor state
  rightDirection = rightSTATIONARY;  // Set initial right motor state
}
// =================================

void loop() {

  currentMillis = millis();  // For LED flashing

  switch (Robot) {
    case WAITING:
      //Serial.println("State: Waiting");
      runWaitingLED();
      Serial.println("Would you like to move (M) or turn (T)? ");

      while (Serial.available() == 0) {
      }

      String option = Serial.readString();
      option.trim();

      if (option.equals("T")) {
        Serial.println("Enter degrees (negative for left, normal for right) ");
        while (Serial.available() == 0) {}
        turn = Serial.parseInt();

      } else if (option.equals("M")) {
        Serial.println("Enter distance (negative for left, normal for right) ");
        while (Serial.available() == 0) {}
        distanceCM = Serial.parseInt();
        if (distanceCM > 0) {
          robotDirection = FORWARDS;
          moveRobot(distanceCM);
          
        } else if (distanceCM < 0) {
          robotDirection = REVERSE;
          moveRobot(distanceCM);
          
        } else {
          Serial.println("Invalid Input");
        }
  }  else {
    Serial.println("Invalid Inpu lol");
  }

  break;

  case EXECUTING:
    //  Serial.println("State: Executing");
    //  checkDistance();
    runWaitingLED();
    runLeftMovementLED();
    runRightMovementLED();
    // moveRobot(10);



    break;
}
}

int getNumberOfSteps(int distance) {
  return (distance / wheelCircumference) * 20;  // distance to encoder steps converter
}

void moveRobot(int distance) {
  switch (robotDirection) {
    case STATIONARY:
      runLeftMotor(0);
      runRightMotor(0);
      break;
    case FORWARDS:
      runLeftMotor(getNumberOfSteps(distance));
      runRightMotor(getNumberOfSteps(distance));
  }
}

void runLeftMotor(int leftDistance) {
  if (leftDistance == 0) {
    analogWrite(motorLeftA, 0);  // Motor On, swap for other direction
    analogWrite(motorLeftB, 0);  // Motor On, swap for other direction
    leftDirection = leftSTATIONARY;

    return;
  }

  if (encoderLeftCount <= abs(leftDistance)) {
    if (leftDistance < 0) {
      analogWrite(motorLeftA, 0);           // Motor On, swap for other direction
      analogWrite(motorLeftB, motorSpeed);  // Motor On, swap for other direction
      leftDirection = leftREVERSE;
    }
    if (leftDistance > 0) {
      analogWrite(motorLeftB, 0);           // Motor On, swap for other direction
      analogWrite(motorLeftA, motorSpeed);  // Motor On, swap for other direction
      leftDirection = leftFORWARDS;
    }
    encoderLeftState = digitalRead(encoderLeft);
    if ((encoderLeftState == HIGH) && (encoderLeftStateOld == LOW)) {
      encoderLeftCount++;
    }
    encoderLeftStateOld = encoderLeftState;

  } else {
    analogWrite(motorLeftA, 0);  // Motor On, swap for other direction
    analogWrite(motorLeftB, 0);  // Motor On, swap for other direction
    Robot = WAITING;
    leftDirection = leftSTATIONARY;
    encoderLeftCount = 1;
    return;
  }
}


void runRightMotor(int rightDistance) {
  if (rightDistance == 0) {

    analogWrite(motorRightA, 0);  // Motor On, swap for other direction
    analogWrite(motorRightB, 0);  // Motor On, swap for other direction
    rightDirection = rightSTATIONARY;
    return;
  }
  if (encoderRightCount <= abs(rightDistance)) {
    if (rightDistance < 0) {
      analogWrite(motorRightA, 0);           // Set Right motor A to zero
      analogWrite(motorRightB, motorSpeed);  // Motor On, swap for other direction
      rightDirection = rightREVERSE;
    }
    if (rightDistance > 0) {
      analogWrite(motorRightB, 0);           // Motor On, swap for other direction
      analogWrite(motorRightA, motorSpeed);  // Motor On, swap for other direction
      rightDirection = rightFORWARDS;
    }
    encoderRightState = digitalRead(encoderRight);
    if ((encoderRightState == HIGH) && (encoderRightStateOld == LOW)) {
      encoderRightCount++;
    }
    encoderRightStateOld = encoderRightState;

  } else {
    analogWrite(motorRightA, 0);  // Motor On, swap for other direction
    analogWrite(motorRightB, 0);  // Motor On, swap for other direction
    Robot = WAITING;
    rightDirection = rightSTATIONARY;
    encoderRightCount = 1;
    return;
  }
}

// ================== Distance Sensor Functions ==================

void checkDistance() {
  // Clears the trigPin condition
  digitalWrite(sensorTrig, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(sensorTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorTrig, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(sensorEcho, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;  // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

// ===========================================================

// ================== LED Control Functions ==================

void flashFunctionLED() {
  if (currentMillis - functionLEDPreviousMillis >= LEDFlashInterval) {
    // save the last time you blinked the LED
    functionLEDPreviousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (functionLEDState == LOW) {
      functionLEDState = HIGH;
    } else {
      functionLEDState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(functionLED, functionLEDState);
  }
}

void flashLeftMovementLED() {
  if (currentMillis - leftMotorPreviousMillis >= LEDFlashInterval) {
    // save the last time you blinked the LED
    leftMotorPreviousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (leftMotorLEDState == LOW) {
      leftMotorLEDState = HIGH;
    } else {
      leftMotorLEDState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(leftMovementLED, leftMotorLEDState);
  }
}


void flashRightMovementLED() {
  if (currentMillis - rightMotorPreviousMillis >= LEDFlashInterval) {
    // save the last time you blinked the LED
    rightMotorPreviousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (rightMotorLEDState == LOW) {
      rightMotorLEDState = HIGH;
    } else {
      rightMotorLEDState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(rightMovementLED, rightMotorLEDState);
  }
}

void solidLED(byte pin) {
  digitalWrite(pin, HIGH);
}

void runWaitingLED() {
  switch (Robot) {
    case WAITING:
      solidLED(functionLED);
      break;
    case EXECUTING:
      flashFunctionLED();
      break;
  }
}

void runLeftMovementLED() {
  switch (leftDirection) {
    case leftSTATIONARY:
      digitalWrite(leftMovementLED, LOW);
      break;
    case leftFORWARDS:
      solidLED(leftMovementLED);
      break;
    case leftREVERSE:
      flashLeftMovementLED();
      break;
  }
}

void runRightMovementLED() {
  switch (rightDirection) {
    case rightSTATIONARY:
      digitalWrite(rightMovementLED, LOW);
      break;
    case rightFORWARDS:
      solidLED(rightMovementLED);
      break;
    case rightREVERSE:
      flashRightMovementLED();
      break;
  }
}

// =====================================================