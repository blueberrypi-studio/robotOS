// =================================
// Group 24 Arduino code
// Haydn Boul
// Nathan van Slooten
// Arabella Cryer
// =================================

#include "robot.h"

robotState Robot;

robotState left;
robotState right;

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

  analogWrite(motorLeftB, 0);   // Motor On, swap for other direction
  analogWrite(motorLeftA, 0);   // Motor On, swap for other direction
  analogWrite(motorRightB, 0);  // Motor On, swap for other direction
  analogWrite(motorRightA, 0);  // Motor On, swap for other direction
}
// =================================

void loop() {

  currentMillis = millis();  // timer for LED flashing

  if (Robot == WAITING) {
    encoderLeftCount = 1;
    encoderRightCount = 1;

    distanceCM = 0;
    turn = 0;
    // Serial.println("State: Waiting");
    runWaitingLED();

    Serial.println("How fast would you like to go? 80 --> 200");

    while (Serial.available() == 0) {}  // Delay entire program until user input

    speed = Serial.parseInt();
    rightMotorSpeed = speed;
    leftMotorSpeed = speed*1.25;

    Serial.println("Would you like to move (M) or turn (T)? ");

    while (Serial.available() == 0) {}  // Delay entire program until user input

    String option = Serial.readString();
    option.trim();

    if (option.equals("T")) {
      Serial.println("Enter degrees (negative for left, normal for right) ");
      while (Serial.available() == 0) {}  // Delay entire program until user input
      turn = Serial.parseInt();
      Robot = EXECUTING;

    } else if (option.equals("M")) {
      Serial.println("Enter distance (negative for left, normal for right) ");
      while (Serial.available() == 0) {}  // Delay entire program until user input
      distanceCM = Serial.parseInt();
      Robot = EXECUTING;
    }
  }

  if (Robot == EXECUTING) {
    // Serial.println("State: Executing");

    if (distanceCM > 0) {
      robotDirection = FORWARDS;
      moveRobot(distanceCM);

    } else if (distanceCM < 0) {
      robotDirection = REVERSE;
      moveRobot(distanceCM);

    } else if (turn > 0) {
      robotDirection = CLOCKWISE;
      moveRobot(turn);
    } else if (turn < 0) {
      robotDirection = ANTICLOCKWISE;
      moveRobot(turn);
    }

    if ((leftDirection == leftSTATIONARY) && (rightDirection == rightSTATIONARY)) {
      Robot = WAITING;
      robotDirection = STATIONARY;
      // Serial.println(Robot, robotDirection);
    }


    runWaitingLED();
    runLeftMovementLED();
    runRightMovementLED();
  }
}

int getNumberOfSteps(int distance) {
  // Serial.println((distance / wheelCircumference) * 20);
  return (distance / wheelCircumference) * 20;  // distance to encoder steps converter
}

void moveRobot(int distance) {
  if (robotDirection == STATIONARY) {
    runLeftMotor(0);
    runRightMotor(0);
  }
  if (robotDirection == FORWARDS) {
    runLeftMotor(getNumberOfSteps(distance));
    runRightMotor(getNumberOfSteps(distance));
  }
  if (robotDirection == REVERSE) {
    runLeftMotor(getNumberOfSteps(distance));
    runRightMotor(getNumberOfSteps(distance));
  }

  if (robotDirection == CLOCKWISE) {
    runLeftMotor(distance/9);
    runRightMotor(-distance/9);
  }

  if (robotDirection == ANTICLOCKWISE) {
    runLeftMotor(-distance/9);
    runRightMotor(distance/9);
  }

}

void runLeftMotor(int leftDistance) {
  // Move Forward
  if (leftDistance > 0) {

    if (encoderLeftCount <= abs(leftDistance)) {
      leftDirection = leftFORWARDS;
      // Serial.println(encoderLeftCount);
      analogWrite(motorLeftB, 0);               // Motor On, swap for other direction
      analogWrite(motorLeftA, leftMotorSpeed);  // Motor On, swap for other direction

      encoderLeftState = digitalRead(encoderLeft);
      if ((encoderLeftState == HIGH) && (encoderLeftStateOld == LOW)) {
        encoderLeftCount++;
      }
      encoderLeftStateOld = encoderLeftState;

    } else {
      leftDistance = 0;
      leftDirection = leftSTATIONARY;
      analogWrite(motorLeftB, 0);  // Motor On, swap for other direction
      analogWrite(motorLeftA, 0);  // Motor On, swap for other direction
    }
  }

  // Move Backwards
  if (leftDistance < 0) {

    if (encoderLeftCount <= abs(leftDistance)) {
      leftDirection = leftREVERSE;
      analogWrite(motorLeftA, 0);               // Motor On, swap for other direction
      analogWrite(motorLeftB, leftMotorSpeed);  // Motor On, swap for other direction

      encoderLeftState = digitalRead(encoderLeft);
      if ((encoderLeftState == HIGH) && (encoderLeftStateOld == LOW)) {
        encoderLeftCount++;
      }
      encoderLeftStateOld = encoderLeftState;

    } else {
      leftDistance = 0;
      leftDirection = leftSTATIONARY;

      analogWrite(motorLeftB, 0);  // Motor On, swap for other direction
      analogWrite(motorLeftA, 0);  // Motor On, swap for other direction
    }
  }


  // if (leftDistance = 0) {
  //   left = WAITING;
  //   // analogWrite(motorLeftB, 0);  // Motor On, swap for other direction
  //   // analogWrite(motorLeftA, 0);  // Motor On, swap for other direction
  // }
}


void runRightMotor(int rightDistance) {
  right = EXECUTING;
  // Move Forward
  if (rightDistance > 0) {

    if (encoderRightCount <= abs(rightDistance)) {
      rightDirection = rightFORWARDS;
      analogWrite(motorRightB, 0);                // Motor On, swap for other direction
      analogWrite(motorRightA, rightMotorSpeed);  // Motor On, swap for other direction

      encoderRightState = digitalRead(encoderRight);
      if ((encoderRightState == HIGH) && (encoderRightStateOld == LOW)) {
        encoderRightCount++;
      }
      encoderRightStateOld = encoderRightState;

    } else {
      rightDistance = 0;
      rightDirection = rightSTATIONARY;

      analogWrite(motorRightB, 0);  // Motor On, swap for other direction
      analogWrite(motorRightA, 0);  // Motor On, swap for other direction
    }
  }

  // Move Backwards
  if (rightDistance < 0) {

    if (encoderRightCount <= abs(rightDistance)) {
      rightDirection = rightREVERSE;
      analogWrite(motorRightA, 0);                // Motor On, swap for other direction
      analogWrite(motorRightB, rightMotorSpeed);  // Motor On, swap for other direction

      encoderRightState = digitalRead(encoderRight);
      if ((encoderRightState == HIGH) && (encoderRightStateOld == LOW)) {
        encoderRightCount++;
      }
      encoderRightStateOld = encoderRightState;

    } else {
      rightDistance = 0;
      rightDirection = rightSTATIONARY;

      analogWrite(motorRightB, 0);  // Motor On, swap for other direction
      analogWrite(motorRightA, 0);  // Motor On, swap for other direction
    }
  }


  // if (rightDistance = 0) {
  //   right = WAITING;
  //   // analogWrite(motorRightB, 0);  // Motor On, swap for other direction
  //   // analogWrite(motorRightA, 0);  // Motor On, swap for other direction
  // }
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