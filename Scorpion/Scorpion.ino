

//libraries
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>

//Tail
Servo krtailA;
Servo krtailB;
Servo krtailC;
Servo krtailRot;

int krtailSpeed = 20;
int krtailTimer1 = 0;

double krtailAPos = 20;
double krtailBPos = 20;
double krtailCPos = 20;
double krtailRotPos = 20;
double krtailAPosTarget = 20;
double krtailBPosTarget = 20;
double krtailCPosTarget = 20;
double krtailRotPosTarget = 20;
double krtailAPosTarget2 = 20;
double krtailBPosTarget2 = 20;
double krtailCPosTarget2 = 20;
double krtailRotPosTarget2 = 20;

double krtailARate = 0;
double krtailBRate = 0;
double krtailCRate = 0;
double krtailRotRate = 0;

//Piston Magnet Device
Servo servo_Piston;
Servo servo_PistonExtend;

//Scoripion Drive Varaibles
int leftSpeed, rightSpeed;
Servo servo_RightMotor;
Servo servo_LeftMotor;

//servey function
Servo servo_RightClawHorizontal, servo_LeftClawHorizontal;
long sgSurveyPrevTime;
int sgAngleIncrement = 1; // how much we want to increment the angle by
int sgSurveyAngleMax = 180; // to be determined by testing
int sgSurveyAngleMin = 0; // to be determined by testing
int sgPos;

// grip function variable
Servo sgMyServo;
int ecClawGripOpenL = 180;
int ecClawGripOpenR = 0;
int sgClawGripCloseL = 0;
int sgClawGripCloseR = 130;

//hall effect function varaibles
int sgMagnetDetectionValueTHigh = 495; // with piston up
int sgMagnetDetectionValueTLow = 485; // with piston up
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
//home tail variables are for the tess pick up
int ecHomeTailRot;
int ecHomeTailX;
int ecHomeTailY;
int ecHomeTailZ;
//base tail variables are for the basic position of the tail
int ecBaseTailRot;
int ecBaseTailX;
int ecBaseTailY;
int ecBaseTailZ;
//return tail variables are for tessaract placement at endge of arena
int ecReturnTailRot;
int ecReturnTailX;
int ecReturnTailY;
int ecReturnTailZ;

int HomeNum = 0;

//tail constants
int degreeOfExtension = 45;
int degreeOfTurn = 0;

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
const int bottomLineTracker = A1;


//Microcontroller 2
const int TailRot = 11;
const int TailBase = 9;
const int TailAppend = 10;
const int Piston = 5;
const int PistonExtend = 3;
const int TailHall = A5;
const int topLineTracker = A3;

//motor speed vairables
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
unsigned int ui_Motors_Speed = 1800;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;

//functions
void tailWrite(double Rot, double A, double B, double C);
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
void angleMagnet(int angleMag); //NEED TO BE WRITTEN
bool readLineTracker(int lineTrackerPin);//NEEDS TO BE WRITTEN
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
        ScorpionDrive(300, 300);
      }
      ScorpionDrive(0, 0); //stops bot
      Survey(50);
      DetectionPlace();
    }
    else if (ecdSearch == 2) {
      if ((millis() - walkBackPrevTime) <= 20) { // how far it moves forward NEED TO TEST VALUE
        walkBackPrevTime = millis();
        ScorpionDrive(-300, -300);
      }
      ScorpionDrive(0, 0); //stops bot
      Survey(50);
      DetectionPlace();
    }
    else if (ecdSearch > 2) {
      if ((millis() - ResetPrevTime) <= 30) { // how far it moves forward NEED TO TEST VALUE
        ResetPrevTime = millis();
        ScorpionDrive(-300, -300);
      }
    }

  }
  else if (ecRoadMap == 3) {
    if ((ecTessPlaceL > 0) && (ecTessPlaceR == 0)) {
      servo_LeftClawHorizontal.write(ecHomeClawL);
    }
    else if ((ecTessPlaceR > 0) && (ecTessPlaceL == 0)) {
      servo_RightClawHorizontal.write(ecHomeClawR);
    }
    tailWrite(ecHomeTailRot, ecHomeTailX, ecHomeTailY, ecHomeTailZ);
    //consider putting a delay here so that we reach the position before extending the magnetg
    angleMagnet(90);
    PickUpMagnet();
    angleMagnet(0);
    tailWrite(ecBaseTailRot, ecBaseTailX, ecBaseTailY, ecBaseTailZ);
  }

  else if (ecRoadMap == 4) {
    findHome();
    HomeNum++;
    if (HomeNum > 3) {
      HomeNum = 0;
    }
  }
  else if (ecRoadMap == 5) {
    int ScanNum = 0;
    tailWrite(ecReturnTailRot, ecReturnTailX, ecReturnTailY, ecReturnTailZ);
    for (int tailRot = ecReturnTailRot; tailRot <= 180; tailRot++) {     //NEED TO CHANGE 180 DEGREES --EC
      if (readLineTracker(topLineTracker)) {
        ScanNum++;
      }
      if (HomeNum == ScanNum) {
        break;
      }
      krtailRot.write(tailRot);
      delay(15);
    }
    if (HomeNum == ScanNum) {
      angleMagnet(90);
      DropMagnet();
      angleMagnet(0);
    }
  }
  else if (ecRoadMap == 6) {
    GoBackToTrack();
  }
}

void ScorpionDrive(int left, int right)
{
  ui_Left_Motor_Speed = constrain(1500 + leftSpeed, 1600, 2100);
  ui_Right_Motor_Speed = constrain(1500 + rightSpeed, 1600, 2100);
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
  //<<<<<<< HEAD
  //
  //  if (turnCounter % 2)
  //  {
  //    findWidth();
  //    Ping(ci_Ultrasonic_Ping_Center, ci_Ultrasonic_Data_Center); // which ultrasonic?
  //    if (ul_Echo_Time / 58 > ((width) - (15 + 4 * turnCounter)) || ReadLineTracker) // the right side of the condition is the width of the subtract the free zone
  //    {
  //      Survey (50);
  //      if ((magnet(hallLeftClaw)) || (magnet(hallRightClaw))) {
  //        ecRoadMap = 2;
  //        break;
  //        ScorpionDrive(200, 200);
  //        // inlcude break statement if light sensor detected
  //      }
  //      else {
  //        ScorpionDrive(0, 200); // turn
  //        turnCounter++;
  //      }
  //    }
  //    else if (!turnCounter % 2)
  //    {
  //      Ping(ci_Ultrasonic_Ping_Center, ci_Ultrasonic_Data_Center);
  //      if (ul_Echo_Time / 58 > 1) // in cm, need better value
  //      {
  //        ScoripionDrive(200, 200);
  //
  //      }
  //      else {
  //        ScorpionDrive(200, 0); // turn(might not be correct)
  //        turnCounter++;
  //      }
  //    }


  //=======
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
  //>>>>>>> origin/master
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

void tailWrite(double Rot, double A, double B, double C) {
  //Base Servo
  krtailAPosTarget = A;
  //Appendage Servo
  krtailBPosTarget = B;
  //Piston Extend Servo
  krtailCPosTarget = C;
  //Rot servo
  krtailRotPosTarget = Rot;

  double maxTarget;
  if (krtailAPosTarget != krtailAPosTarget2 || krtailBPosTarget != krtailBPosTarget2 || krtailCPosTarget != krtailCPosTarget2 || krtailRotPosTarget != krtailRotPosTarget2)
  {
    if (krtailAPosTarget >= krtailBPosTarget && krtailAPosTarget >= krtailCPosTarget && krtailAPosTarget >= krtailRotPosTarget)
      maxTarget = krtailAPosTarget;

    else if (krtailBPosTarget >= krtailAPosTarget && krtailBPosTarget >= krtailCPosTarget && krtailBPosTarget >= krtailRotPosTarget)
      maxTarget = krtailBPosTarget;

    else if (krtailCPosTarget >= krtailAPosTarget && krtailCPosTarget >= krtailBPosTarget && krtailCPosTarget >= krtailRotPosTarget)
      maxTarget = krtailCPosTarget;

    else
      maxTarget = krtailRotPosTarget;

    krtailARate = krtailAPosTarget / maxTarget;
    krtailBRate = krtailBPosTarget / maxTarget;
    krtailCRate = krtailCPosTarget / maxTarget;
    krtailRotRate = krtailRotPosTarget / maxTarget;

    krtailAPosTarget2 = krtailAPosTarget;
    krtailBPosTarget2 = krtailBPosTarget;
    krtailCPosTarget2 = krtailCPosTarget;
    krtailRotPosTarget2 = krtailRotPosTarget;
  }
  if ((millis() - krtailTimer1) > krtailSpeed) {
    if (sqrt(pow((krtailAPos - krtailAPosTarget), 2)) <= sqrt(pow(krtailARate, 2)))
      krtailAPos = krtailAPosTarget;
    else
      krtailAPos = krtailAPos + krtailARate;
    krtailA.write(krtailAPos);

    if (sqrt(pow((krtailBPos - krtailBPosTarget), 2)) <= sqrt(pow(krtailBRate, 2)))
      krtailBPos = krtailBPosTarget;
    else
      krtailBPos = krtailBPos + krtailBRate;
    krtailB.write(krtailBPos);

    if (sqrt(pow((krtailCPos - krtailCPosTarget), 2)) <= sqrt(pow(krtailCRate, 2)))
      krtailCPos = krtailCPosTarget;
    else
      krtailCPos = krtailCPos + krtailCRate;
    krtailC.write(krtailCPos);

    if (sqrt(pow((krtailRotPos - krtailRotPosTarget), 2)) <= sqrt(pow(krtailRotRate, 2)))
      krtailRotPos = krtailRotPosTarget;
    else
      krtailRotPos = krtailRotPos + krtailRotRate;
    krtailRot.write(krtailRotPos);

    krtailTimer1 = millis();
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



