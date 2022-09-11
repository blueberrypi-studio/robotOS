// =================================
// Group 24 Arduino code
// Haydn Boul
// Nathan van Slooten
// Arabella Cryer
// =================================

#include "robot.h"

robotState Robot;
char receivedChar, directionChar, distanceChar, robotDistanceString[5];
boolean newData, inputMovement = false;
char inData[20]; // Allocate some space for the string
char c;
int inChar; // Where to store the character read
byte charIndex = 0; // Index into array; where to store the character
byte stringRead = 0; 
byte counter = 0;
boolean okPrint;
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
      //Serial.println("State: Waiting");
      runWaitingLED();
      while(Serial.available() > 0) { // Don't read unless
                                                 // there you know there is data 
        if(charIndex < 19) // One less than the size of the array
        {
            inChar = Serial.read(); // Read a character
            inData[charIndex] = char(inChar); // Store it
            charIndex++; // Increment where to write next
            inData[charIndex] = '\0'; // Null terminate the string
        }
        okPrint == true;
      }
      
      //Serial.println(String(inData));
      readInput();
      if (robotDirection == FORWARDS) {
        Serial.println("true");
      }
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

void readInput() {
  for (c = inData[stringRead]; stringRead<sizeof(inData); stringRead++) {
    Serial.print(c);
    if (c == 'F') {
      Serial.println("true");
      robotDirection = FORWARDS;
    } else if (c == 'R') {
      robotDirection = REVERSE;
    } else if (c == 'S') {
      robotDirection = STATIONARY;
    } else if (c == 'A') {
      robotDirection = ANTICLOCKWISE;
    } else if (c == 'C') {
      robotDirection = CLOCKWISE;
    } else if (c == 'T') {
      robotMovement = TURNING;
    } else if (c == 'N') {
      robotMovement = NOT_TURNING;
    } else if (isDigit(c)) {
      robotDistanceString[counter] = c;
      counter++;
    }
    stringRead++;
  }
  robotDistance = atoi(robotDistanceString);
}
