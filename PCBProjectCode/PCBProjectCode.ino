// =================================
// Group 24 Arduino code
// Haydn Boul
// Nathan van Slooten
// Arabella Cryer
// =================================

#include "robot.h"

robotState Robot;
char receivedChar;
boolean newData = false;

// ============= SETUP =============
void setup() {
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");
  pinMode(functionLED, OUTPUT);
  
  Robot = WAITING;
    
}
// =================================

void loop() {
  switch (Robot){
    case WAITING:
    Serial.println("State: Waiting");
      runWaitingLED();
      recvOneChar();
      showNewData();
      break;

    case EXECUTING:
      Serial.println("State: Executing");
      
      break;
  }


  delay(1000);
}


void runWaitingLED(){
// Runs the status LED
  if (waitingLEDCounter != 1){
    digitalWrite(functionLED, HIGH);
    waitingLEDCounter++;
  } else{
    waitingLEDCounter = 0;
    digitalWrite(functionLED, LOW);
  }
}


void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        newData = true;
        Robot = EXECUTING;
    }
}

void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChar);
        newData = false;
    }
}
