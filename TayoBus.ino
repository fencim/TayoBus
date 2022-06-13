#include <SoftwareSerial.h>
#include <AFMotor.h>
#include <Servo.h>
enum Action {
  none = 0,
  goForward = 1,
  goBackward = 2,
  goRight = 4,
  goLeft = 8,
  goStraight = 16,
  goTurnFirst = 32,
  goTurnBackFirst = 64,
  goMoveFirst = 128,
  goRotate = 256,
  goStepOne = 512,
  goStepTwo = 1024,
  goStepThree = 2048,
  goBreak = goForward | goBackward,
  goForwardRight = goRight | goTurnFirst,
  goForwardRightForward = goForward | goRight| goStepTwo,
  goStraightThenForward = goStraight | goTurnFirst | goForward,
  goForwardLeft = goLeft| goTurnFirst| goStepOne,
  goForwardLeftForward = goForward | goLeft| goStepTwo,
  goBackwardRight =  goRight | goTurnFirst| goStepOne,
  goBackwardRightBackward = goBackward | goRight | goStepTwo,
  goBackwardLeft = goLeft | goTurnBackFirst | goStepOne,
  goBackwardLeftBackward = goBackward | goLeft | goStepTwo,
  goTurnRight = goBackward | goRight | goTurnBackFirst,
  goTurnLeft = goBackward | goLeft | goTurnBackFirst,
  goRotateLeft = goRotate | goLeft | goTurnBackFirst,
  goRotateLeftBackward = goRotate | goLeft | goStepTwo,
  goRotateLeftForward = goRotate | goLeft | goStepThree,
  goRotateRight = goRotate | goRight | goTurnBackFirst,
  goRotateRightBackward = goRotateRight | goStepTwo,
  goRotateRightForward = goRotateRight | goStepThree
};


const int SERVO_PIN = 9;
const int RX_PIN = 10;
const int TX_PIN = 13;
const int TRIG_PIN = A5;
const int ECHO_PIN = A4;
const int MAX_SPEED = 350;

SoftwareSerial esp8266(RX_PIN, TX_PIN); // RX, TX
#define DEBUG true

AF_DCMotor motorFRight(1);
AF_DCMotor motorFLeft(2);
AF_DCMotor motorRRight(3);
AF_DCMotor motorRLeft(4);

Servo servoWheel;
int whellPos = 0;
int busSpeed = 0;
int busLastSpeed = 1;
int backingCounter = 0;
int lastDistance = 0;
Action currentAction = none, lastAction = none;
int trapFixTryCount = 0;

#define MAX_SPEED 255
#define TURN_SPEED 200
#define TURN_DELAY 1000
#define STRAIGHT_POS 90
#define TURN_ANGLE 45
#define BACKING_COUNT_LIMIT 13
#define LOOP_INTERVAL 100
#define STOP_THRESHOLD 100
#define SERIAL_BAUD 115200
#define SERIAL_ES8266 115200

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(ECHO_PIN, HIGH);
  digitalWrite(TRIG_PIN, HIGH);
  esp8266.begin(SERIAL_ES8266);
  Serial.begin(SERIAL_BAUD);
  while (!esp8266) {
    ; // wait for serial port to connect. Needed for native USB port only
  }           
  InitWifiModule();
  printLine("Initiating Tayo Little Bus!");
  servoWheel.attach(SERVO_PIN);
  motorFLeft.setSpeed(MAX_SPEED);
  motorFRight.setSpeed(MAX_SPEED);
  motorRRight.setSpeed(MAX_SPEED);
  motorRLeft.setSpeed(MAX_SPEED);
  whellPos = servoWheel.read();  
  motorRelease();
  turnStraigth();
  turnRight();
  turnLeft();
  turnStraigth();
  forward(); delay(BACKING_COUNT_LIMIT * LOOP_INTERVAL);
  backward(); delay(BACKING_COUNT_LIMIT * LOOP_INTERVAL);
  currentAction = goForward;
}

void loop() {
  long duration, cm, inches;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  cm = (duration/2) / 29.1;
  inches = (duration/2) / 74;
  printData(" Speeed: ");
  printData(busSpeed);
  printData(" distance ");
  printLine(cm);
  lastAction = currentAction;
  const bool isNotMoving = abs(lastDistance  - cm)< BACKING_COUNT_LIMIT;
  const bool isTrapped = (trapFixTryCount > 0);
  const bool isGoingForward = (currentAction & goForward) == goForward;
  const bool isAlmostColliding = cm < STOP_THRESHOLD;
  const bool isAboutToCollide = cm < max(busSpeed, STOP_THRESHOLD);
  const bool isSlow = busSpeed < MAX_SPEED || isNotMoving;
  if ((isGoingForward || (isTrapped && isNotMoving)) && (isAlmostColliding || isAboutToCollide)) {
    trapFixTryCount = isNotMoving ? (trapFixTryCount + 1) : 0;
    breakOn();
    delay(LOOP_INTERVAL);
    if (trapFixTryCount > 0) {
      printData("Failed ");
      printData(trapFixTryCount);
      printLine(" times");
    }
    lastDistance  = cm;
    backingCounter = 0;
    if (cm < 10) {
      currentAction = goRotateLeft;
      printLine("emergency turn left back");
    } else {
      switch(trapFixTryCount % 6) {
        case 0:
          currentAction = goRotateRight;
          printLine("goRotateRight");
          break;
        case 1:
          currentAction = goRotateLeft;
          printLine("goRotateLeft");
          break;
        case 2:
          currentAction = goBackwardLeft;   
          printLine("goBackwardLeft");
          break;
        case 3:
          currentAction = goBackwardRight; 
          printLine("goBackwardRight");  
          break;
        case 5:
          currentAction = goForwardRight;   
          printLine("goForwardRight");
          break;
        case 4:
          currentAction = goForwardLeft;   
          printLine("goForwardLeft");
          break;
        
      }
    }
  }
  switch(currentAction) {
    case goBreak:
      breakOn();
      currentAction = lastAction;
    case goForward:
      busLastSpeed = busSpeed;
      busSpeed = busLastSpeed + 2;
      forward();
      if (busSpeed > MAX_SPEED) {
        currentAction = goBreak;
        busSpeed = busLastSpeed - 2;
      }
      break;
    case goForwardLeft:
      turnLeft();
      currentAction = goForwardLeftForward;
      break;
    case goForwardRight:
      turnRight();
      currentAction = goForwardRightForward;
      break;
    case goForwardRightForward:
    case goForwardLeftForward:
      forward();
      backingCounter++;
      if (backingCounter > BACKING_COUNT_LIMIT) {
          currentAction = goStraightThenForward;
          backingCounter = 0;
      }
      break;
    case goBackwardLeftBackward: case goBackwardRightBackward:
    case goBackward:
     backward();
     backingCounter++;
     if (backingCounter > BACKING_COUNT_LIMIT) {
          currentAction = goStraightThenForward;
          backingCounter = 0;
     }
     break;
    case goBackwardRight:
      turnLeft();
      currentAction = goBackwardRightBackward;
      break;
    case goBackwardLeft:
      turnRight();
      currentAction = goBackwardLeftBackward;
      break;
    case goStraight:
      turnStraigth();
      break;  
    case goTurnRight:
      backward();
      backingCounter++;
      if (backingCounter > BACKING_COUNT_LIMIT / 2) {
          currentAction = goForwardRight;
          backingCounter = 0;
      }
      break;
    case goTurnLeft:
      backward();
      backingCounter++;
      if (backingCounter > BACKING_COUNT_LIMIT / 2) {
          currentAction = goForwardLeft;
          backingCounter = 0;
      }
      break;
    case goStraightThenForward:
      turnStraigth();
      busSpeed= 1;
      currentAction = goForward;
      break;
    case goRotateLeft:
      motorRelease(); delay(1000);
      turnRight();
      currentAction = goRotateLeftBackward;
    case goRotateLeftBackward:
      backward();   
      backingCounter++;
      if (backingCounter > BACKING_COUNT_LIMIT) {
          currentAction = goRotateLeftForward;
          turnLeft();
          backingCounter = 0;
      }
      break;
    case goRotateRight:
      motorRelease(); delay(1000);
      turnLeft();
      currentAction = goRotateRightBackward;
      backingCounter = 0;
    case goRotateRightBackward:
      backward();   
      backingCounter++;
      if (backingCounter > BACKING_COUNT_LIMIT) {
          turnRight();
          currentAction = goRotateRightForward;
          backingCounter = 0;
      } else break;
    case goRotateRightForward:
    case goRotateLeftForward:
      forward(); 
      backingCounter++;
      if (backingCounter > BACKING_COUNT_LIMIT) {
        backingCounter = 0;
        currentAction = goStraightThenForward;
      } 
      break;
  }
  delay(LOOP_INTERVAL);
  String IncomingString="";
  boolean StringReady = false;
  while (esp8266.available()){
   IncomingString=esp8266.readString();
   StringReady= true;
  } 
  if (StringReady){
    printLine("Received String: " + IncomingString);
  }
}

void printLine(String str) {
  if (!DEBUG) return;
  Serial.println(str);
}
void printLine(int n) {
  if (!DEBUG) return;
   Serial.println(n);
}
void printData(String str) {
  if (!DEBUG) return;
  Serial.print(str);
}
void printData(int n) {
  if (!DEBUG) return;
  Serial.print(n);
}
void motorRelease() {
  motorFLeft.setSpeed(whellPos == STRAIGHT_POS ? MAX_SPEED : TURN_SPEED);
  motorFRight.setSpeed(whellPos == STRAIGHT_POS ? MAX_SPEED : TURN_SPEED);
  motorFLeft.run(RELEASE);
  motorFRight.run(RELEASE);
  motorRLeft.run(RELEASE); 
  motorRRight.run(RELEASE); 
  busSpeed = 0;
  backingCounter = 0;
  delay(300);
}
void forward() {
  motorFLeft.run(FORWARD); 
  motorFRight.run(FORWARD); 

  motorRLeft.run(FORWARD); 
  motorRRight.run(FORWARD); 
}
void backward() {
  motorFLeft.run(BACKWARD); 
  motorFRight.run(BACKWARD);

  motorRLeft.run(BACKWARD); 
  motorRRight.run(BACKWARD);
   
}
void turnWheelTo(int targetPos) {
   while(whellPos != targetPos ) {
    if (whellPos < targetPos) {
      whellPos ++;
    } else {
      whellPos --;
    }
    servoWheel.write(whellPos);  
    delay(20);
  }
}

void turnLeft() {
  printLine("Turn Left!");
  motorRelease();
  motorFLeft.setSpeed(TURN_SPEED);
  motorFRight.setSpeed(TURN_SPEED);
  motorFLeft.run(FORWARD);
  motorFRight.run(BACKWARD);
  delay(TURN_DELAY);
  turnWheelTo(STRAIGHT_POS - TURN_ANGLE);
  motorRelease();
}

void turnRight() {
  printLine("TurnRight!");
  motorFLeft.setSpeed(TURN_SPEED);
  motorFRight.setSpeed(TURN_SPEED);
  motorFRight.run(FORWARD);
  motorFLeft.run(BACKWARD);
  delay(TURN_DELAY);
  turnWheelTo(STRAIGHT_POS + TURN_ANGLE);
  motorRelease();
}

void breakOn() {
  const bool isGoingForward = (currentAction & goForward) == goForward;
  if (isGoingForward) {
    backward(); 
  } else {
    forward();
  }
}
void turnStraigth() {
  printLine("Straight!");
  motorFLeft.setSpeed(TURN_SPEED);
  motorFRight.setSpeed(TURN_SPEED);

  if (whellPos < 90) {
      motorFLeft.run(BACKWARD);
      motorFRight.run(FORWARD);
  } else if (whellPos > 90) {
      motorFRight.run(BACKWARD);
      motorFLeft.run(FORWARD);
  } else {
    motorRelease();
  }
  delay(TURN_DELAY);
  turnWheelTo(90);
  motorRelease();
}

boolean echoFind(String keyword, const int timeout){
 byte current_char = 0;
 byte keyword_length = keyword.length();
 long deadline = millis() + timeout;
 while(millis() < deadline){
  if (esp8266.available()){
    char ch = esp8266.read();
    Serial.write(ch);
    if (ch == keyword[current_char])
      if (++current_char == keyword_length){
       printLine(keyword);
       return true;
    }
   }
  }
 return false; // Timed out
}
boolean SendCommand(String cmd, String ack, const int timeout){
  esp8266.println(cmd); // Send "AT+" command to module
  return echoFind(ack, timeout);
}
void InitWifiModule() {
  printLine("Connecting to Wifi....");
  if (!SendCommand("AT+RST", "Ready", 8000)) {
    printLine("Failed to Reset");
    return;
  }
  if (!SendCommand("AT+CWMODE=1","OK", 1500)) {
    printLine("Failed to Set Mode");
    return;
  }
  if (!SendCommand("AT+CIFSR", "OK", 1500)) return;
  if (!SendCommand("AT+CIPMUX=1","OK", 1500)) return;
  if (!SendCommand("AT+CIPSERVER=1,80","OK", 1500)) return;
  printLine("Now connected");
}
