// =================================
// Group 24 Arduino code
// =================================

char receivedChar;
boolean newData, receivedNums = false;
const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = LOW;

void setup() {
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}

void loop() {
    recvOneChar();
    digitalWrite(ledPin, state);
}

void recvOneChar() {
  Serial.print("Input Direction:\nForward (F), Reverse (R), Turn (T)")
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        newData = true;
        inputNums();
    }
}

void inputNums() {
  if (newData == true) {
    if (receivedChar == "T") {
      Serial.print("Input Angle... ");
      distanceChar = Serial.read();
      receivedNums = true;
    } else {
      Serial.print("Input Distance... ");
      distanceChar = Serial.read();
      receivedNums = true;
    }
    moveMotor();
  }
}

void moveMotor() {
  
}
