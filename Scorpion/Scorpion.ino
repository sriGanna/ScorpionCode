

//libraries
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>

//Tail
Servo servo_appendage1;  // create servo object to control a servo
Servo servo_base;// twelve servo objects can be created on most boards
Servo servo_pistonExtend;
Servo servo_piston;


//Scoripion Drive Varaibles
int leftSpeed, rightSpeed;
Servo servo_RightMotor;
Servo servo_LeftMotor;

//survey function
Servo servo_RightClawHorizontal, servo_LeftClawHorizontal;
long sgSurveyPrevTime;
int sgAngleIncrement = 1; // how much we want to increment the angle by
int sgSurveyAngleMax = 180; // to be determined by testing
int sgSurveyAngleMin = 0; // to be determined by testing
int sgPos;

// grip function variables
Servo sgMyServo;
int ecClawGripOpenL = 180;
int ecClawGripOpenR = 0;
int sgClawGripCloseL = 0;
int sgClawGripCloseR = 130;

//hall effect function variables
int sgMagnetDetectionValueTHigh = 495; // with piston retracted
int sgMagnetDetectionValueTLow = 485; // with piston retracted
int sgMagnetDetectionValueRHigh = 510;
int sgMagnetDetectionValueRLow = 505;
int sgMagnetDetectionValueLHigh = 515;
int sgMagnetDetectionValueLLow = 513;
int count;
int magnetRead[20];
int aveRead;
//debugging
int found;

//PickUpPassBack function variables
Servo VerticalClawServoL, VerticalClawServoR;

//ping function variables
long ul_Echo_Time;

//DetectionPlace Function Variables
int ecdSearch = 0;
int ecTessPlaceL = 0;
int ecTessPlaceR = 0;

//General Code Variabls
int ecRoadMap = 1;
int walkForwardPrevTime;
int walkBackPrevTime;
int ResetPrevTime;
int ecHomeClawR;
int ecHomeClawL;

int HomeNum = 0;

//navigation function variables
int turnNumber = 0;
int width = 0;
int turnCounter = 0;
bool navigate = true;
int prevTurnCount;
bool goHome = false;

//Line Tracker
int ksDark;

//Port pin constants Microcontroller 1
const int ci_Right_Motor = 11;
const int ci_Left_Motor = 2;
const int ci_Ultrasonic_Ping_Center = 3;   //input plug
const int ci_Ultrasonic_Data_Center = 13;   //output plug
const int ci_Ultrasonic_Left = 4;
const int ci_Ultrasonic_Right = 12;
const int ci_RightClawHorizontal = 8;
const int ci_LeftClawHorizontal = 7;
const int ci_RightClawGrip = 10;
const int ci_LeftClawGrip = 5;
const int ci_LeftClawVertical = 6;
const int ci_RightClawVertical = 9;
const int hallLeftClaw = A5;
const int hallRightClaw = A0;
const int bottomLineTracker = 1;

//Microcontroller 2
const int ci_TailRot = 11;
const int ci_TailBase = 9;
const int ci_TailAppend = 10;
const int ci_Piston = 5;
const int ci_PistonExtend = 3;
const int ci_TailHall = A5;
const int topLineTracker = 3;

int mode2LDistance;
int mode2RDistance;
int noWhere;
bool foundSide;
long prevTime;
long distTime = 0;

//motor speed variables
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
unsigned int ui_Motors_Speed = 1800;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;

//functions
void ScorpionDrive(int leftSpeed, int rightSpeed);
void clawGripOpen(int clawHorizontalPin);
void clawGripClose(int clawHorizontalPin);
void Survey (long surveyInterval);
void PickUpPassBack(int clawVerticalPin, int clawGripPin);
void placement();
void tailExtend(int degreeOfExtension, int degreeOfTurn);
void modeTwoPickUp();
void modeTwoPlacement();
void Ping(int Input, int Output);
bool magnet(int hallEffectPin);
void DetectionPlace();
void navigation();
void findWidth();
void findHome();
void DropMagnet();
void PickUpMagnet();
bool readLineTracker(int lineTrackerPin);
void allignWithBase(); //NEED TO BE WRITTEN
void GoBackToTrack(); //NEED TO BE WRITTEN
void turn90(char direction);

void setup() {

  //piston magnet device
  pinMode(Piston, OUTPUT);
  servo_Piston.attach(Piston);
  pinMode(PistonExtend, OUTPUT);
  servo_PistonExtend.attach(PistonExtend);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  //set up Horizontal Claw servos
  pinMode(ci_RightClawHorizontal, OUTPUT);
  servo_RightClawHorizontal.attach(ci_RightClawHorizontal);
  pinMode(ci_LeftClawHorizontal, OUTPUT);
  servo_LeftClawHorizontal.attach(ci_LeftClawHorizontal);

  //set up Grip Claw servos
  pinMode(ci_LeftClawGrip, OUTPUT);
  pinMode(ci_RightClawGrip, OUTPUT);

  //Set up Vertical Claw Servos
  pinMode(ci_LeftClawVertical, OUTPUT);
  pinMode(ci_RightClawVertical, OUTPUT);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping_Center, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Center, INPUT);

  //set up hall sensors
  pinMode(hallLeftClaw, INPUT);
  pinMode(hallRightClaw, INPUT);

  Serial.begin(9600);

}

void loop() {
  
if (Serial.available()) {
      {
        ecRoadMap = Serial.read();
      }

mode = digitalRead(ci_SwitchPin);
if (!mode)
//mode 1
 {
   if(ecRoadMap == 0)
   {
     //detach everything which can be detached
   servo_RightMotor.detach();
  servo_LeftMotor.detach();
  servo_RightClawHorizontal.detach();
  servo_LeftClawHorizontal.detach();
  VerticalClawServoL.detach();
  VerticalClawServoR.detach();
   }
   if (ecRoadMap == 1) {
    navigation();
  }
  else if (ecRoadMap == 2) {
    if (ecdSearch == 0) {
      ScorpionDrive(0, 0); //stops bot
      Survey(50);
      DetectionPlace();
    }
    else if (ecdSearch == 1) {
      if ((millis() - walkForwardPrevTime) <= 10) { // how far it moves forward NEED TO TEST VALUE
        walkForwardPrevTime = millis();
        ScorpionDrive(100, 100);
      }
      ScorpionDrive(0, 0); //stops bot
      Survey(50);
      DetectionPlace();
    }
    else if (ecdSearch == 2) {
      if ((millis() - walkBackPrevTime) <= 20) { // how far it moves forward NEED TO TEST VALUE
        walkBackPrevTime = millis();
        ScorpionDrive(-100, -100);
      }
      ScorpionDrive(0, 0); //stops bot
      Survey(50);
      DetectionPlace();
    }
    else if (ecdSearch > 2) {
      if ((millis() - ResetPrevTime) <= 30) { // how far it moves forward NEED TO TEST VALUE
        ResetPrevTime = millis();
        ScorpionDrive(-100, -100);
      }
      
      ecRoadMap++;
        Serial.write(ecRoadMap);
        ecRoadMap = 0;
    }

  }
  else if (ecRoadMap == 3) {
    if ((ecTessPlaceL > 0) && (ecTessPlaceR == 0)) {
      servo_LeftClawHorizontal.write(ecHomeClawL);
    }
    else if ((ecTessPlaceR > 0) && (ecTessPlaceL == 0)) {
      servo_RightClawHorizontal.write(ecHomeClawR);
    }
    ecRoadMap++;
        Serial.write(ecRoadMap);
        ecRoadMap = 0;
  }

  else if (ecRoadMap == 4) {
    findHome();
    HomeNum++;
    if (HomeNum > 3) {
      HomeNum = 0;
    }
    ecRoadMap++;
        Serial.write(ecRoadMap);
        ecRoadMap = 0;
  }
  else if (ecRoadMap == 5) {
    int ScanNum = 0;
    ecRoadMap++;
        Serial.write(ecRoadMap);
        ecRoadMap = 0;
    }
  }
  else if (ecRoadMap == 6) {
    GoBackToTrack();
  }
}

else if (mode)
//mode 2
if (Mode2RoadMap == 2)
{
  passUnderGate1();
  mode2RoadMap++;
  Serial.write(mode2RoadMap);
}
else if(Mode2RoadMap == 5)
{
  passUnderGate2();
  mode2RoadMap++;
  Serial.write(mode2RoadMap);
}
}

void ScorpionDrive(int left, int right)
{
  ui_Left_Motor_Speed = constrain(1500 + leftSpeed, 1000, 2100);
  ui_Right_Motor_Speed = constrain(1500 + rightSpeed, 1000, 2100);
  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
}

void clawGripOpen(int clawGripPin)
{
  sgMyServo.attach(clawGripPin);
  if (clawGripPin == ci_RightClawGrip)
    sgMyServo.write(ecClawGripOpenR);
  else
    sgMyServo.write(ecClawGripOpenL);
  delay(1000);
  sgMyServo.detach();
}

void clawGripClose(int clawGripPin)
{
  sgMyServo.attach(clawGripPin);
  if (clawGripPin == ci_RightClawGrip)
    sgMyServo.write(sgClawGripCloseR);
  else
    sgMyServo.write(sgClawGripCloseL);
  delay(1000);
  sgMyServo.detach();
}

void PickUpPassBack(int clawVerticalPin, int clawGripPin)
{
  if (clawGripPin == ci_RightClawGrip) {
    VerticalClawServoR.attach(clawVerticalPin);
    for (int pos = 180; pos >= 145; pos -= 1) {
      VerticalClawServoR.write(pos);
      delay(20);
    }
    clawGripOpen(clawGripPin);
    clawGripClose(clawGripPin);
    for (int pos = 140; pos <= 180; pos += 1) {
      VerticalClawServoR.write(pos);
      delay(20);
    }
  }
  else {
    VerticalClawServoL.attach(clawVerticalPin);
    for (int pos = 180; pos >= 145; pos -= 1) {
      VerticalClawServoL.write(pos);
      delay(20);
    }
    clawGripOpen(clawGripPin);
    clawGripClose(clawGripPin);
    for (int pos = 140; pos <= 180; pos += 1) {
      VerticalClawServoL.write(pos);
      delay(20);
    }
  }
}
void Survey(long surveyInterval)
{
  if ((millis() - sgSurveyPrevTime) >= surveyInterval) // how fast it sweeps back and forth
  {
    sgSurveyPrevTime = millis();
    sgPos += sgAngleIncrement;
    servo_RightClawHorizontal.write(sgPos);
    servo_LeftClawHorizontal.write(sgPos);
    if (magnet(hallLeftClaw)) {
      ecTessPlaceL = sgPos;
      ecTessPlaceR = 0;
      ecRoadMap = 3;
    }
    else if (magnet(hallRightClaw)) {
      ecTessPlaceR = sgPos;
      ecTessPlaceL = 0;
      ecRoadMap = 3;
    }
    Serial.println(sgPos); // for debugging purposes
    if ((sgPos >= sgSurveyAngleMax) || (sgPos <= sgSurveyAngleMin))//end of sweep
    {
      // reverse direction
      sgAngleIncrement = -sgAngleIncrement;
    }
  }
}

bool magnet(int hallEffectPin)
{
  magnetRead[count] = analogRead(hallEffectPin);
  count++;
  if (count == 20)
  {
    count = 0;
    aveRead = 0;
    for (int i = 0; i < 21; i++) {
      aveRead += magnetRead[i];
    }
    aveRead /= 20;
    //Serial.println(aveRead); //debugging
    int sgMagnetDetectionValueHigh, sgMagnetDetectionValueLow;
    // this series of if statements indicate which values to use for magnet detection
    if (hallEffectPin == hallRightClaw)
    {
      sgMagnetDetectionValueHigh = sgMagnetDetectionValueRHigh;
      sgMagnetDetectionValueLow = sgMagnetDetectionValueRLow;
    }
    else if (hallEffectPin == hallLeftClaw)
    {
      sgMagnetDetectionValueHigh = sgMagnetDetectionValueLHigh;
      sgMagnetDetectionValueLow = sgMagnetDetectionValueLLow;
    }
    else
    {
      sgMagnetDetectionValueHigh = sgMagnetDetectionValueTHigh;
      sgMagnetDetectionValueLow = sgMagnetDetectionValueTLow;
    }
    // return true if magnet is found, else returns false
    if ((aveRead >= sgMagnetDetectionValueHigh) || (aveRead <= sgMagnetDetectionValueLow))
    {
      found = 1;// debugging
      return true;
    }
    else
    {
      found = 0;//debugging
      return false;
    }
  }
}

long sidePing(int sideUltrasonicPin)
{
  pinMode(sideUltrasonicPin, OUTPUT);
  digitalWrite(sideUltrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sideUltrasonicPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sideUltrasonicPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(sideUltrasonicPin, INPUT);
  duration = pulseIn(sideUltrasonicPin, HIGH);
  delay (5);
  duration /= 58;
  return duration; //returns the distance in mm
}

void centerPing()
{
  for (int i = 0; i < 50; i++) {
    pinMode(ci_UltrasonicCenterInput, OUTPUT);
    pinMode(ci_UltrasonicCenterOutput, INPUT);
    //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
    digitalWrite(ci_UltrasonicCenterInput, HIGH);
    delayMicroseconds(50);  //The 10 microsecond pause where the pulse in "high"
    digitalWrite(ci_UltrasonicCenterInput, LOW);
    //use command pulseIn to listen to Ultrasonic_Data pin to record the
    //time that it takes from when the Pin goes HIGH until it goes LOW
    int dist = pulseIn(ci_UltrasonicCenterOutput, HIGH, 10000) / 58;
    if (dist)
      ksFrontDistance += dist; //sets ksFrontDistance in mm
    else
      i--;

  }
  if (ksFrontDistance)
    ksFrontDistance = ksFrontDistance / 50 ;
}

void findWidth()
{
  // to get average wdth
  int i;
  for ( i = 0; i < 6; i++)
  {
    if (ksFrontDistance == 0)
    {
      i--
    }
    centerPing();
    width += ksFrontDistance;
  }

  width = width / (i * 2); // dividing by two becasue we want half the cage as width
  width = width - (15);

}

void navigation()
{
  if (turnCounter % 2)
  {
    findWidth();
    centerPing();
    if (ksFrontDistance > width) || readLineTracker()) // the right side of the condition is the width of the subtract the free zone
    {
      sideDistance = sidePing(ci_UltrasonicLeft);
      ScorpionDrive(200, 200);// constant driving speed
    }
    else {
      //turn Left for a bit and then use side ultrasonics to get into proper position
      turn90(l); // turn
      turn90(l);
      delay(100);
      while (sidePing(ci_UltrasonicsLeft(left) > width - 8) // random value
             turnCounter++;
    }
}
else if (!turnCounter % 2)
  {
    centerPing();
    if (ksFrontDistance  > 1) // in cm, need better value
    {
      sideDistance = sidePing(ci_UltrasonicsRight);
      ScorpionDrive(200, 200);
    }
    else {
      //turn right  and then use side ultrasonics to get into porper postion
      turn90(r); // turn(might not be correct)
      turn90(r);
      turnCounter++;
    }
  }

}
else if (!turnCounter % 2)
  {
    centerPing();
    if (ksFrontDistance  > 1) // in cm, need better value
    {
      sideDistance = sidePing(ci_UltrasonicsRight);
      ScorpionDrive(200, 200);
    }
    else {
      //turn right  and then use side ultrasonics to get into proper postion
      turn90(r); // turn(might not be correct)
      turn90(r);
      turnCounter++;
    }
  }
}

void findHome()
{
  if (turnCounter % 2)
  {
    //<<<<<<< HEAD
    centerPing();
    while (ksFrontDistance > 1)
    {
      centerPing();
      ScorpionDrive(200, 200);
    }
    //turn left
    centerPing();
    while (ksFrontDistance  > 5)
    {
      centerPing();
      ScorpionDrive(200, 200);
    }
    turnCounter = 0;
  }
  else if (turnCounter % 2)
    //=======
    centerPing();
  while (ksFrontDistance > width || readLineTracker(5))
    //>>>>>>> origin/master
  {
    centerPing();
    ScorpionDrive(200, 200);
  }
  turn90(l);
  turn90(l);
}

while (ksFrontDistance > 1)
{
  centerPing():

    ScorpionDrive(200, 200);
  }
  turn90(l);
  while (ksFrontDistance > 5)
{
  ScorpionDrive(200, 200);
}
turn90(l);
turnCounter = 0;

}
// going back to navigation, skips part that is has already scanned
void GoBackToTrack()
{
  turn90(l);
  centerPing();
  while (ksFrontDistance > sideDistance) //walk until sideDistance
  {
    ScorpionDrive(200, 200);
  }
  turnCounter = 0;
}

}
//  void ModeTwoPickUp() {
//    Ping();
//    if (( ul_Echo_Time <= 10 /**mm**/) && ( ul_Echo_Time >= 5)) {
//      ScorpionDrive (1500, 1500);
//      tailExtend(degreeOfExtension, degreeOfTurn); //guess need to know actual value (extends to 45 and looks at position
//      if (magnet(hallEffectPin) == true) { //NEED PIN NUMBER***********************************************************************
//        magnet(); //picks up cube
//        tailTuck(); //tucks the tail
//      }
//      else {
//        if (degreeOfTurn <= 90) {
//          degreeOfTurn = degreeOfTurn + 90;
//          ModeTwoPickUp();
//        }
//        else {
//          degreeOfTurn = 0; ///MIGHT NEED TO WATCH OUT FOR BOT PLACING CUBE******************************************************
//          ModeTwoPickUp();
//          >>> >>> > origin / master
//        }
//        ScorpionDrive(200, 200);
//        // inlcude break statement if light sensor detected
//      }
//      ScorpionDrive(0, 200); // turn
//      turnCounter++;
//    }
//    else if (turnCounter % 2)
//    {
//
//    }
//
//  }

void DetectionPlace() {
  if ((ecTessPlaceL == 0) && (ecTessPlaceR == 0)) {
    ecdSearch++;
  }
  else if ((ecTessPlaceL > 0) && (ecTessPlaceR == 0)) {
    servo_LeftClawHorizontal.write(ecTessPlaceL);
    PickUpPassBack(ci_LeftClawVertical, ci_LeftClawGrip);
    ecRoadMap = 3;
  }
  else if ((ecTessPlaceR > 0) && (ecTessPlaceL == 0)) {
    servo_RightClawHorizontal.write(ecTessPlaceR);
    PickUpPassBack(ci_RightClawVertical, ci_RightClawGrip);
    ecRoadMap = 3;
  }
}

void DropMagnet() {
  servo_Piston.write(0);
}
void PickUpMagnet() {
  servo_Piston.write(180);
}
void angleMagnet(int angleMag) {
  servo_PistonExtend.write(angleMag);
}

bool readLineTracker(int lineTrackerPin)
{
  if (lineTrackerPin == bottomLineTracker)
    ksDark = 995;
  else if (lineTrackerPin == topLineTracker)
    ksDark = 1005;
  if (analogRead(lineTrackerPin) > ksDark)
    return true;
  else
    return false;
}

void allignWithBase() {}

void turn90(char direction)
{
  long prevTime = millis();
  int left = 350, right = 350;
  if (direction == 'l')
    left = -left;
  else if (direction == 'r')
    right = -right;

  while (millis() - prevTime < turnInterval)
  {
    ScorpionDrive(left, right);
  }

  ScorpionDrive(0, 0);

}

//functions needed
void ScorpionDrive(int leftSpeed, int rightSpeed);
int sidePing(int pin);

//functions made
void passUnderGate1();
void passUnderGate2();

//function definitions:

void passUnderGate1()
{
  prevTime = millis();
  ScorpionDrive(300, 300);
  delay(1000);
  tailCollapse();
  ScorpionDrive(-300, -300);
  delay(1000);
  if (sidePing(ci_Ultrasonic_Left) < noWhere || sidePing(ci_Ultrasonic_Right) < noWhere)
  {
    foundSide = true;
  }
  mode2LDistance = sidePing(ci_Ultrasonic_Left);
  mode2RDistance = sidePing(ci_Ultrasonic_Right);
  while (sidePing(ci_Ultrasonic_Left) <= mode2LDistance && sidePing(ci_Ultrasonic_Right) <= mode2RDistance)
  {
    ScorpionDrive(-300, -300);
  }
  distTime = millis() - prevTime;
  ScorpionDrive(-300, -300);
  delay(1000);
  ScorpionDrive(0,0);

}

void passUnderGate2()
{
  prevTime = millis();
  ScorpionDrive(300, 300);
  delay(1000);
  tailCollapse();
  while (sidePing(ci_Ultrasonic_Left) >= noWhere || sidePing(ci_Ultrasonic_Right) >= noWhere)
  {
    ScorpionDrive(-300, -300);
  }
  foundSide = true;
  distTime = millis() - prevTime;
  ScorpionDrive(-300, -300);
  delay(1000);// determine delay by testing :P 
  ScorpionDrive(0, 0);
}



