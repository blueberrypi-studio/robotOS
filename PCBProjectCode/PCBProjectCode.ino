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
  pinMode(motorLeftA,OUTPUT); // Motor Setup
  pinMode(motorLeftB,OUTPUT); // Motor Setup

  pinMode(motorRightA,OUTPUT); // Motor Setup
  pinMode(motorRightB,OUTPUT); // Motor Setup
  
  pinMode(sensorTrig, OUTPUT);
  pinMode(sensorEcho, INPUT);
  
  Robot = WAITING;

  rpm=0;
  pulses=0;
  time_old=0;

  pinMode(encoderLeft,INPUT); // Setting encoder pin as Input
  pinMode(encoderRight,INPUT); // Setting encoder pin as Input
  
  attachInterrupt(digitalPinToInterrupt(encoderLeft), count, FALLING ); // Triggering count function everytime the encoder pin1 turns from 1 to 0

    
}
// =================================

void loop() {
  switch (Robot){
    case WAITING:
//    Serial.println("State: Waiting");
      runWaitingLED();
      recvOneChar();
      inputDirection();
      inputDistance();
      break;

    case EXECUTING:
//      Serial.println("State: Executing");
//      runMotors(20, 50);
      checkDistance();
      break;
  }

}

void count() // Counting the number of pulses for calculation of rpm
{
  pulses++;
  movement++;  
}

void runMotors(int leftDistance, int rightDistance){
    if (movement <= abs(leftDistance)) {
      if (leftDistance < 0){
        analogWrite(motorLeftA, 0); // Motor On, swap for other direction
        analogWrite(motorLeftB, motorSpeed); // Motor On, swap for other direction
      }
      if (leftDistance >= 0){
        analogWrite(motorLeftA, motorSpeed); // Motor On, swap for other direction
        analogWrite(motorLeftB, 0); // Motor On, swap for other direction
      }
      
    
    if(millis()-time_old >=100){ // Updating every 0.1 seconds
      detachInterrupt(digitalPinToInterrupt(encoderLeft));
//      rpm = (60 * 100 / pulses_per_turn )/ (millis() - time_old)* pulses;
      time_old= millis();
//      pulses=0;
//      Serial.print("RPM= ");
//      Serial.println(rpm);
      attachInterrupt(digitalPinToInterrupt(encoderLeft), count, FALLING ); // Triggering count function everytime the encoder pin1 turns from 1 to 0
    }

  }
  else {
    analogWrite(motorLeftA, 0); // Motor On, swap for other direction
    analogWrite(motorLeftB, 0); // Motor On, swap for other direction
    delay(1000);
    movement = 0;
  }
}

void checkDistance(){
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
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void runWaitingLED(){
// Runs the status LED
  if (waitingLEDCounter != 1000){
    digitalWrite(functionLED, HIGH);
    waitingLEDCounter++;
  } else{
    waitingLEDCounter = 0;
    digitalWrite(functionLED, LOW);
  }
}


void recvOneChar() {
    //if (Serial.available() > 0) {
      if ((counter % 3) == 0 and inputMovement == false) {
      Serial.println("Input a movement:\nStraight (S)\nTurn (T)\n");
        receivedChar = Serial.read();
        if (receivedChar = 'S') {
          robotMovement = STRAIGHT;
          counter +=1;
        } else if (receivedChar = 'T') {
          robotMovement = TURNING;
          counter +=1;
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
        Serial.println("Input direction to turn robot:\nForwards (F)\nReverse (R)");
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
    robotDistance = (int) distanceChar;
    Robot = EXECUTING;
  } else {
    Serial.println("Input the amount you would like to move the robot in centimetres");
    distanceChar = Serial.read();
    robotDistance = (int) distanceChar;
    Robot = EXECUTING;
  }
  }
  
}
