#include <SoftwareSerial.h>
#include <AFMotor.h>
#include <Servo.h>

enum Action
{
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
  goForwardRightForward = goForward | goRight | goStepTwo,
  goStraightThenForward = goStraight | goTurnFirst,
  goForwardThenStraight =  goForward | goStraight,
  goForwardLeft = goLeft | goTurnFirst | goStepOne,
  goForwardLeftStepTwo = goLeft | goForward | goStepTwo,
  goBackwardRight = goRight | goTurnFirst | goStepOne,
  goBackwardRightStepTwo = goRight | goBackward | goStepTwo,
  goBackwardLeft = goLeft | goTurnBackFirst | goStepOne,
  goBackwardLeftStepTwo = goLeft | goBackward | goStepTwo,
  goTurnRight = goRight | goBackward | goTurnBackFirst,
  goTurnLeft = goLeft | goBackward | goTurnBackFirst,
  goRotateLeft = goRotate | goLeft | goTurnBackFirst,
  goRotateLeftStepTwo = goRotate | goLeft | goBackward | goStepTwo,
  goRotateLeftStepThree = goRotate | goLeft | goForward | goStepThree,
  goRotateRight = goRotate | goRight | goTurnBackFirst,
  goRotateRightStepTwo = goBackward | goRotateRight | goStepTwo,
  goRotateRightStepThree = goForward | goRotateRight | goStepThree
};

#define DEBUG true

//PIN CONFIGURATIONS
#define SERVO_PIN 9
#define RX_PIN 10
#define TX_PIN 13
#define TRIG_PIN A5
#define ECHO_PIN A4
#define MAX_SPEED 350
#define REAR_PIN A0

#define MOTOR_1 1
#define MOTOR_2 2
#define MOTOR_3 3
#define MOTOR_4 4

#define MAX_QUEUE_SIZE 10

SoftwareSerial esp8266(RX_PIN, TX_PIN); // RX, TX
AF_DCMotor motorFRight(MOTOR_1);
AF_DCMotor motorFLeft(MOTOR_2);
AF_DCMotor motorRRight(MOTOR_3);
AF_DCMotor motorRLeft(MOTOR_4);
Servo servoWheel;

//control variables
int whellPos = 0;
int busSpeed = 0;
int busLastSpeed = 1;
int backingCounter = 0;
int lastDistance = 0;
Action currentAction = none, lastAction = none;
int trapFixTryCount = 0;
Action actionQueue[MAX_QUEUE_SIZE] = {goRight, goLeft, goBackward, goForward};
int queueHead = 0, queueTail = 3;

//control constants
#define MAX_SPEED 255
#define TURN_SPEED 200
#define TURN_DELAY 1000
#define STRAIGHT_POS 90
#define TURN_ANGLE 45
#define BACKING_COUNT_LIMIT 50
#define LOOP_INTERVAL 20
#define STOP_THRESHOLD 10
#define SERIAL_BAUD 115200
#define SERIAL_ES8266 115200

void setup()
{
  pinMode(REAR_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(ECHO_PIN, HIGH);
  digitalWrite(TRIG_PIN, HIGH);
  digitalWrite(REAR_PIN, HIGH);
  Serial.begin(SERIAL_BAUD);

  // InitWifiModule();
  printLine("Initiating Tayo Little Bus!");
  servoWheel.attach(SERVO_PIN);
  whellPos = servoWheel.read();
  
  motorFLeft.setSpeed(MAX_SPEED);
  motorFRight.setSpeed(MAX_SPEED);
  motorRRight.setSpeed(MAX_SPEED);
  motorRLeft.setSpeed(MAX_SPEED);
  digitalWrite(REAR_PIN, HIGH);
  delay(1000);
  //initial test
  enqueue(goBackward);
  enqueue(goStraight);
  enqueue(goRight);
  enqueue(goLeft);
  enqueue(goStraight);
  enqueue(goBackward);
  enqueue(goRight);
  enqueue(goForwardThenStraight);
}

void loop()
{
  const bool isOnBreak = currentAction == goBreak;
  const bool isGoingForward = !isOnBreak && (currentAction & goForward);
  const bool isGoingBackward = !isOnBreak && (currentAction & goBackward);

  lastAction = currentAction;
  
  // read distance
  long duration, cm, inches;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  cm = (duration / 2) / 29.1;
  inches = (duration / 2) / 74;

  // compute collision variables
  const bool cantMove = (isGoingForward || isGoingBackward) && abs(lastDistance - cm) < STOP_THRESHOLD;
  const bool isTrapped = (trapFixTryCount > 0);
  const bool isAlmostColliding = cm < STOP_THRESHOLD;
  const bool isAboutToCollide = cm < max(busSpeed, STOP_THRESHOLD) * (busSpeed / 150);
  const bool isSlow = busSpeed < MAX_SPEED || cantMove;
  digitalWrite(REAR_PIN, (isGoingBackward || isAboutToCollide) ? HIGH : LOW);
  // print distance
  printData("S: ");
  printData(busSpeed);
  printData(" D: ");
  printData(cm);
  printData(" M: ");
  printData(cantMove);
  printData(" W: ");
  printData(whellPos);
  printData(" BC: ");
  printData(backingCounter);
  printData(" A: ");
  printData(currentAction);
  printData(" F: ");
  printData(currentAction & goForward);
  printData(" B: ");
  printData(currentAction & goBackward);
  printData(" L: ");
  printData(currentAction & goLeft);
  printData(" R: ");
  printData(currentAction & goRight);
  printData(" C: ");
  printData(isAboutToCollide);
  printData(" T: ");
  printLine(trapFixTryCount);

  if (currentAction == none) currentAction = dequeue();
  if (0 == (currentAction & (goStraight | goForward | goRotate)) && !isAboutToCollide && backingCounter == 0)
  {
    if (isGoingBackward) {
      breakOn();
      delay(LOOP_INTERVAL);
      
    } 
    currentAction = goStraightThenForward;  
    trapFixTryCount = 0;
  } else if ((currentAction == none) || 
    ((isGoingForward || (isTrapped && cantMove)) && (isAlmostColliding || isAboutToCollide)))
  {
    trapFixTryCount = cantMove ? (trapFixTryCount + 1) : 0;
    lastDistance = cm;
    backingCounter = 0;
    avoidObstacle(cm);
  }
  doAction();
  delay(LOOP_INTERVAL);
}
void checkCommIncommingMsg()
{
  String IncomingString = "";
  boolean StringReady = false;
  while (esp8266.available())
  {
    IncomingString = esp8266.readString();
    StringReady = true;
  }
  if (StringReady)
  {
    printLine("Received String: " + IncomingString);
  }
}
void doAction()
{
  // main input: currentAction
  switch (currentAction)
  {
  case goBreak:
    breakOn();
    currentAction = lastAction == goBreak ? goForward : lastAction;
    busSpeed = max(busLastSpeed - (MAX_SPEED / 2), 0);
    break;
  case goForward:
    busLastSpeed = busSpeed;
    busSpeed = busLastSpeed + 2;
    if (busSpeed > MAX_SPEED) // slow down if too fast
    {
      currentAction = goBreak;
    }
    else forward();
    break;
  case goForwardLeft:
    if (turnLeft()) currentAction = goForwardLeftStepTwo;
    break;
  case goForwardRight:
    if (turnRight()) currentAction = goForwardRightForward;
    break;
  case goForwardRightForward:
  case goForwardLeftStepTwo:
    forward();
    handleBacking(goStraightThenForward);
    break;
  case goBackwardRight:
    if (turnLeft()) currentAction = goBackwardRightStepTwo;
    break;
  case goBackwardLeft:
    if (turnRight()) currentAction = goBackwardLeftStepTwo;
    break;
  case goBackwardLeftStepTwo:
  case goBackwardRightStepTwo:
  case goBackward:
    backward();
    handleBacking(goStraightThenForward);
    break;
  case goStraight:
    if (turnStraigth()) currentAction = none;
    break;
  case goLeft:
    if (turnLeft()) currentAction = none;
    break;
  case goRight:
    if (turnRight()) currentAction = none;
    break;
  case goTurnRight:
    backward();
    handleBacking(goForwardRight, BACKING_COUNT_LIMIT / 2);
    break;
  case goTurnLeft:
    backward();
    handleBacking(goForwardLeft, BACKING_COUNT_LIMIT / 2);
    break;
  case goStraightThenForward:
    if (turnStraigth())
    {
      busSpeed = 1;
      currentAction = goForward;
    }
    break;
  case goRotateLeft:
    if (turnRight()) {
      currentAction = goRotateLeftStepTwo;
      motorRelease();//auto adjust speed
    } 
    break;
  case goRotateLeftStepTwo:
    backward();
    handleBacking(goRotateLeftStepThree);
    break;
  case goRotateLeftStepThree:
    if (turnLeft()) {
      currentAction = goForwardThenStraight;
    }
    break;
  case goRotateRight:
    if (turnLeft()) {
      currentAction = goRotateRightStepTwo;
      motorRelease();//auto adjust speed
    }
    break;
  case goRotateRightStepTwo:
    backward();
    handleBacking(goRotateRightStepThree);
    break;
  case goRotateRightStepThree:
    if (turnRight()) currentAction = goForwardThenStraight;
    break;
  case goForwardThenStraight:
    forward();
    handleBacking(goStraightThenForward);
    break;
  }
}
void handleBacking(Action actionAfter)
{
  handleBacking(actionAfter, BACKING_COUNT_LIMIT);
}
void handleBacking(Action actionAfter, int limit)
{
  backingCounter++;
  if (backingCounter > limit)
  {
    backingCounter = 0;
    currentAction = actionAfter;
  }
}
void avoidObstacle(int cm)
{
  breakOn();
  delay(LOOP_INTERVAL);
  
  if (cm < 10 && currentAction& goForward == goForward )
  {
    currentAction = goRotateLeft;
    printLine("emergency turn left back");
  }
  else
  {
    switch (trapFixTryCount % 8)
    {
    case 0: currentAction = goTurnRight; break;
    case 1: currentAction = goTurnLeft; break;
    case 2: currentAction = goRotateLeft; break;
    case 3: currentAction = goRotateRight; break;
    case 4: currentAction = goBackwardRight; break;
    case 5: currentAction = goBackwardLeft; break;
    case 6: currentAction = goForwardLeft; break;
    case 7: currentAction = goForwardRight; break;
    }
  }
}
Action dequeue(){
  Action ret = none;
  if (queueTail > 0) {
    ret = actionQueue[queueHead];
    actionQueue[queueHead] = none;
    queueHead = (queueHead + 1) % (sizeof(actionQueue) / sizeof(ret));
  }
  return ret;
}
void enqueue(Action action) {
  actionQueue[queueTail] = action;
  queueTail = (queueTail + 1) % (sizeof(actionQueue) / sizeof(action));
}
void printLine(String str)
{
  if (!DEBUG)
    return;
  Serial.println(str);
}
void printLine(int n)
{
  if (!DEBUG)
    return;
  Serial.println(n);
}
void printData(String str)
{
  if (!DEBUG)
    return;
  Serial.print(str);
}
void printData(int n)
{
  if (!DEBUG)
    return;
  Serial.print(n);
}
void motorRelease()
{
  motorFLeft.setSpeed((whellPos == STRAIGHT_POS || currentAction & (goBackward)) ? MAX_SPEED : TURN_SPEED);
  motorFRight.setSpeed((whellPos == STRAIGHT_POS || currentAction & (goBackward)) ? MAX_SPEED : TURN_SPEED);
  motorFLeft.run(RELEASE);
  motorFRight.run(RELEASE);
  motorRLeft.run(RELEASE);
  motorRRight.run(RELEASE);
  busSpeed = 0;
  backingCounter = 0;
  delay(300);
}
void forward()
{
  motorFLeft.run(FORWARD);
  motorFRight.run(FORWARD);

  motorRLeft.run(FORWARD);
  motorRRight.run(FORWARD);
}
void backward()
{
  motorFLeft.run(BACKWARD);
  motorFRight.run(BACKWARD);

  motorRLeft.run(BACKWARD);
  motorRRight.run(BACKWARD);
}
bool turnWheelTo(int targetPos)
{
  if (whellPos != targetPos)
  {
    if (whellPos < targetPos)
    {
      whellPos+= 1;
    }
    else
    {
      whellPos-=1;
    }
    servoWheel.write(whellPos);
  }
  return whellPos == targetPos;
}

bool turnLeft()
{
  if (whellPos != (STRAIGHT_POS - TURN_ANGLE))
  {
    motorFLeft.setSpeed(TURN_SPEED);
    motorFRight.setSpeed(TURN_SPEED);
    motorFLeft.run(FORWARD);
    motorFRight.run(BACKWARD);
    turnWheelTo(STRAIGHT_POS - TURN_ANGLE);
    return false;
  }
  else
  {
    motorRelease();
    delay(TURN_DELAY);
    return true;
  }
}

bool turnRight()
{
  if (whellPos != STRAIGHT_POS + TURN_ANGLE)
  {
    motorFLeft.setSpeed(TURN_SPEED);
    motorFRight.setSpeed(TURN_SPEED);
    motorFRight.run(FORWARD);
    motorFLeft.run(BACKWARD);
    turnWheelTo(STRAIGHT_POS + TURN_ANGLE);
    return false;
  }
  else
  {
    motorRelease();
    delay(TURN_DELAY);
    return true;
  }
}

void breakOn()
{
  const bool isGoingForward = (currentAction & goForward) == goForward;
  if (isGoingForward)
  {
    backward();
  }
  else
  {
    forward();
  }
}
bool turnStraigth()
{
  motorFLeft.setSpeed(TURN_SPEED);
  motorFRight.setSpeed(TURN_SPEED);
  if (whellPos < STRAIGHT_POS)
  {
    motorFLeft.run(BACKWARD);
    motorFRight.run(FORWARD);
  }
  else if (whellPos > STRAIGHT_POS)
  {
    motorFRight.run(BACKWARD);
    motorFLeft.run(FORWARD);
  }
  else
  {
    motorRelease();
    delay(TURN_DELAY);
    return true;
  }
  turnWheelTo(STRAIGHT_POS);
  return false;
}
// WIFI Module Functions
boolean echoFind(String keyword, const int timeout)
{
  byte current_char = 0;
  byte keyword_length = keyword.length();
  long deadline = millis() + timeout;
  while (millis() < deadline)
  {
    if (esp8266.available())
    {
      char ch = esp8266.read();
      Serial.write(ch);
      if (ch == keyword[current_char])
        if (++current_char == keyword_length)
        {
          printLine(keyword);
          return true;
        }
    }
  }
  return false; // Timed out
}
boolean SendCommand(String cmd, String ack, const int timeout)
{
  esp8266.println(cmd); // Send "AT+" command to module
  return echoFind(ack, timeout);
}
void InitWifiModule()
{
  printLine("Connecting to Wifi....");
  esp8266.begin(SERIAL_ES8266);
  while (!esp8266)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  if (!SendCommand("AT+RST", "Ready", 8000))
  {
    printLine("Failed to Reset");
    return;
  }
  if (!SendCommand("AT+CWMODE=1", "OK", 1500))
  {
    printLine("Failed to Set Mode");
    return;
  }
  if (!SendCommand("AT+CIFSR", "OK", 1500))
    return;
  if (!SendCommand("AT+CIPMUX=1", "OK", 1500))
    return;
  if (!SendCommand("AT+CIPSERVER=1,80", "OK", 1500))
    return;
  printLine("Now connected");
}
