// =================================
// Group 24 Arduino code
// Haydn Boul
// Nathan van Slooten
// Arabella Cryer
// =================================

#include "robot.h"

robotState Robot;
char receivedChar, directionChar, distanceChar;
boolean newData, inputMovement = false;
int counter = 0;
unsigned int robotDistance;
movementType robotMovement;
newDirection robotDirection;

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

  Robot = WAITING;
  robotDirection = STATIONARY;
}
// =================================

void loop() {

  currentMillis = millis();  // For LED flashing

  switch (Robot) {
    case WAITING:
      // Serial.println("State: Waiting");
      runWaitingLED();
      // recvOneChar();
      // inputDirection();
      // inputDistance();
      break;

    case EXECUTING:
      //  Serial.println("State: Executing");
      //  checkDistance();
      runWaitingLED();
      runLeftMovementLED();
      runRightMovementLED();
      runLeftMotor(0);
      runRightMotor(20);

      break;
  }
}

void runLeftMotor(int leftDistance) {
  if (leftDistance == 0){
    analogWrite(motorLeftA, 0);  // Motor On, swap for other direction
    analogWrite(motorLeftB, 0);  // Motor On, swap for other direction
    return;
  }


  if (encoderLeftCount <= abs(leftDistance)) {
    if (leftDistance < 0) {
      analogWrite(motorLeftA, 0);           // Motor On, swap for other direction
      analogWrite(motorLeftB, motorSpeed);  // Motor On, swap for other direction
    }
    if (leftDistance > 0) {
      analogWrite(motorLeftB, 0);           // Motor On, swap for other direction
      analogWrite(motorLeftA, motorSpeed);  // Motor On, swap for other direction
    }
    encoderLeftState = digitalRead(encoderLeft);
    if ((encoderLeftState == HIGH) && (encoderLeftStateOld == LOW)) {
      encoderLeftCount++;
    }
    encoderLeftStateOld = encoderLeftState;

  } else {
    analogWrite(motorLeftA, 0);  // Motor On, swap for other direction
    analogWrite(motorLeftB, 0);  // Motor On, swap for other direction
    delay(500);
    encoderLeftCount = 1;
  }
}


void runRightMotor(int rightDistance) {
    if (rightDistance == 0){
    analogWrite(motorRightA, 0);  // Motor On, swap for other direction
    analogWrite(motorRightB, 0);  // Motor On, swap for other direction
    return;
  }
  if (encoderRightCount <= abs(rightDistance)) {
    if (rightDistance < 0) {
      analogWrite(motorRightA, 0);           // Motor On, swap for other direction
      analogWrite(motorRightB, motorSpeed);  // Motor On, swap for other direction
    }
    if (rightDistance > 0) {
      analogWrite(motorRightB, 0);           // Motor On, swap for other direction
      analogWrite(motorRightA, motorSpeed);  // Motor On, swap for other direction
    }
    encoderRightState = digitalRead(encoderRight);
    if ((encoderRightState == HIGH) && (encoderRightStateOld == LOW)) {
      encoderRightCount++;
    }
    encoderRightStateOld = encoderRightState;

  } else {
    analogWrite(motorRightA, 0);  // Motor On, swap for other direction
    analogWrite(motorRightB, 0);  // Motor On, swap for other direction
    delay(500);
    // Robot = WAITING;
    encoderRightCount = 1;
  }
}

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

void recvOneChar() {
  //if (Serial.available() > 0) {
  if ((counter % 3) == 0 and inputMovement == false) {
    Serial.println("Input a movement:\nStraight (S)\nTurn (T)\n");
    receivedChar = Serial.read();
    if (receivedChar = 'S') {
      robotMovement = STRAIGHT;
      counter += 1;
    } else if (receivedChar = 'T') {
      robotMovement = TURNING;
      counter += 1;
    }
    newData = true;
  }
  //}
}

void inputDirection() {
  if (newData == true) {
    if ((counter % 3) == 1) {
      counter += 1;
      if (robotMovement == TURNING) {
        Serial.println("Input direction to turn robot:\nAnticlockwise (A)\nClockwise(C)");
        directionChar = Serial.read();
        if (directionChar == 'A') {
          robotDirection = ANTICLOCKWISE;
        } else {
          robotDirection = CLOCKWISE;
        }
      } else if (robotMovement = STRAIGHT) {
        Serial.println("Input direction to drive robot:\nForwards (F)\nReverse (R)");
        directionChar = Serial.read();
        if (directionChar == 'F') {
          robotDirection = FORWARDS;
        } else {
          robotDirection = REVERSE;
        }
      }
    }
    inputDistance();
  }
}

void inputDistance() {
  if ((counter % 3) == 2) {
    counter += 1;
    if (robotMovement = TURNING) {
      Serial.println("Input the amount you would like to turn the robot in degrees");
      distanceChar = Serial.read();
      robotDistance = (int)distanceChar;
      Robot = EXECUTING;
    } else {
      Serial.println("Input the amount you would like to move the robot in centimetres");
      distanceChar = Serial.read();
      robotDistance = (int)distanceChar;
      Robot = EXECUTING;
    }
  }
}

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
      flashFunctionLED();
      break;
    case EXECUTING:
      solidLED(functionLED);
      break;
  }
}

void runLeftMovementLED() {
  switch (robotDirection) {
    case STATIONARY:
      digitalWrite(leftMovementLED, LOW);
      break;
    case FORWARDS:
      solidLED(leftMovementLED);
      break;
    case REVERSE:
      flashLeftMovementLED();
      break;
  }
}

void runRightMovementLED() {
  switch (robotDirection) {
    case STATIONARY:
      digitalWrite(rightMovementLED, LOW);
      break;
    case FORWARDS:
      solidLED(rightMovementLED);
      break;
    case REVERSE:
      flashRightMovementLED();
      break;
  }
}



