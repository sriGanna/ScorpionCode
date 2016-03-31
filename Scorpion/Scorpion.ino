
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
int sgClawGripClose; // to be determined by testing
int ecClawGripOpen; //to be determined by testing

//hall effect function varaibles
int sgMagnetDetectionValue; // to be determined by testing
int count;
int magnetRead[20];
bool foundMagnet;

//PickUpPassBack function variables
int sgPassBackValue; // to be determied by testing
int ecPickUpValue; //to be determined by testing

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

//Port pin constants
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Ultrasonic_Ping_Center = 2;   //input plug
const int ci_Ultrasonic_Data_Center = 3;   //output plug
const int ci_Ultrasonic_Left = 5;
const int ci_Ultrasonic_Right = 6;
const int ci_RightClawHorizontal = 6;
const int ci_LeftClawHorizontal = 7;
const int ci_RightClawGrip = 0;
const int ci_LeftClawGrip = 0;
const int ci_LeftClawVertical = 0;
const int ci_RightClawVertical = 0;
const int hallLeftClaw = 4;
const int hallRightClaw = 2;
const int lineTracker = 0;
const int Piston = 5;
const int PistonExtend = 3;

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
void PickUpPassBack(int clawVerticalPin, int clawHorizontalPin);
void placement();
void tailTuck();
void tailExtend(int degreeOfExtension, int degreeOfTurn);
bool magnet();
void modeTwoPickUp();
void modeTwoPlacement();
void Ping(int Input, int Output);
bool magnet(int hallEffectPin);
void DetectionPlace();
void navigation();
void turn90L();
void turn90R();
void findWidth();
void findLength();
void findHome();
void DropMagnet(); //NEED TO BE WRITTEN
void PickUpMagnet(); //NEED TO BE WRITTEN
void angleMagnet(int angleMag); //NEED TO BE WRITTEN
bool readLineTracker(int lineTrackerPin);
void allignWithBase(); //NEED TO BE WRITTEN
void GoBackToTrack(); //NEED TO BE WRITTEN
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

  //set up claw motors
  pinMode(ci_RightClawHorizontal, OUTPUT);
  servo_RightMotor.attach(ci_RightClawHorizontal);
  pinMode(ci_LeftClawHorizontal, OUTPUT);
  servo_LeftMotor.attach(ci_LeftClawHorizontal);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping_Center, OUTPUT);
  pinMode(ci_Ultrasonic_Data_Center, INPUT);
  pinMode(ci_Ultrasonic_Left, OUTPUT);
  pinMode(ci_Ultrasonic_Left, INPUT);
  pinMode(ci_Ultrasonic_Right, OUTPUT);
  pinMode(ci_Ultrasonic_Right, INPUT);

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
      if (readLineTracker(lineTracker)) {
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
    allignWithBase();
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

void clawGripOpen(int clawHorizontalPin)
{
  sgMyServo.attach(clawHorizontalPin);
  sgMyServo.write(ecClawGripOpen);
  delay(1000);
  sgMyServo.detach();
}

void clawGripClose(int clawHorizontalPin)
{
  sgMyServo.attach(clawHorizontalPin);
  sgMyServo.write(sgClawGripClose);
  delay(1000);
  sgMyServo.detach();
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
    int aveRead = 0;
    for (int i = 0; i < 21; i++) {
      aveRead += magnetRead[i];
    }
    aveRead /= 20;
    Serial.println(aveRead);
    if (aveRead >= sgMagnetDetectionValue)
    {
      navigate = false;
      foundMagnet = true;
      return true;
    }
    else
      return false;
  }
}

void Ping(int input, int output)
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(input, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(input, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time = pulseIn(output, HIGH, 10000);
}

void PickUpPassBack(int clawVerticalPin, int clawHorizontalPin)
{
  sgMyServo.attach(clawVerticalPin);
  sgMyServo.write(ecPickUpValue);
  delay(1000);
  clawGripOpen(clawHorizontalPin);
  clawGripClose(clawHorizontalPin);
  delay(1000);
}
void findWidth()
{

  for (int i = 0; i < 6; i++)
  {
    Ping(ci_Ultrasonic_Ping_Center, ci_Ultrasonic_Data_Center);
    if (ul_Echo_Time == 0)
    {
      i--;
    }
    width += ul_Echo_Time / 58;
  }

  width = width / 10;

}

void navigation()
{

  i  if (returnToNavigation)
  {

    ScorpionDrive(0, 200);
    delay(1000);
    Ping(ci_Ultrasonic_Ping_Center, ci_Ultrasonic_Data_Center);
    while (ul_Echo_Time / 58 > sideDistance) //walk until sideDistance
    {
      ScorpionDrive(200, 200);
    }
    turnCounter = 0;

  }

  if (turnCounter % 2)
  {
    findWidth();
    Ping(ci_Ultrasonic_Ping_Center, ci_Ultrasonic_Data_Center); // which ultrasonic?
    if (ul_Echo_Time / 58 > ((width) - (15 + 4 * turnCounter)) || ReadLineTracker()) // the right side of the condition is the width of the subtract the free zone
    {
      //sideDistance = Ping(left)
      ScorpionDrive(200, 200);
      // inlcude break statement if light sensor detected
    }
    else {
      ScorpionDrive(0, 200); // turn
      turnCounter++;
    }
  }
  else if (!turnCounter % 2)
  {
    Ping(ci_Ultrasonic_Ping_Center, ci_Ultrasonic_Data_Center);
    if (ul_Echo_Time / 58 > 1) // in cm, need better value
    {
      //sideDistance = ping(right);
      ScorpionDrive(200, 200);

    }
    else {
      ScorpionDrive(200, 0); // turn(might not be correct)
      turnCounter++;
    }
  }
  }
  void findHome()
  {
    if (turnCounter % 2)
    {
    Ping(ci_Ultrasonic_Ping_Center, ci_Ultrasonic_Data_Center);
    while (ul_Echo_Time / 58 > ((width) - (15 + 4 * turnCounter)) || ReadLineTracker())
    {
      Ping(ci_Ultrasonic_Ping_Center, ci_Ultrasonic_Data_Center);
      ScorpionDrive(200, 200);
    }
    ScorpionDrive(0, 200);
    delay(1000);
    ScorpionDrive(0, 200);
    delay(1000);
  }

  while (ul_Echo_Time / 58 > 1)
  { Ping(ci_Ultrasonic_Ping_Center, ci_Ultrasonic_Data_Center);

    ScorpionDrive(200, 200);
  }
  ScorpionDrive(0, 200);
  delay(1000);
  while (ul_Echo_Time / 58 > 5)
  {
    ScorpionDrive(200, 200);
  }
  ScorpionDrive(0, 200);
  delay(1000);
  turnCounter = 0;
  returnToNavigation = true;
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
      PickUpPassBack(ci_LeftClawVertical, ci_LeftClawHorizontal);
      ecRoadMap = 3;
    }
    else if ((ecTessPlaceR > 0) && (ecTessPlaceL == 0)) {
      servo_RightClawHorizontal.write(ecTessPlaceR);
      PickUpPassBack(ci_RightClawVertical, ci_RightClawHorizontal);
      ecRoadMap = 3;
    }
  }

  void tailWrite(double Rot, double A, double B, double C) {
    krtailAPosTarget = A;
    krtailBPosTarget = B;
    krtailCPosTarget = C;
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


  //void ModeTwoPickUp(){
  //  Ping();
  //  if(( ul_Echo_Time <= 10 /**mm**/)&&( ul_Echo_Time>=5)){
  //    ScorpionDrive (1500, 1500);
  //    tailExtend(degreeOfExtension, degreeOfTurn); //guess need to know actual value (extends to 45 and looks at position
  //    if(magnet(hallEffectPin)==true){ //NEED PIN NUMBER***********************************************************************
  //    magnet(); //picks up cube
  //      tailTuck(); //tucks the tail
  //    }
  //    else{
  //      if (degreeOfTurn <=90){
  //      degreeOfTurn = degreeOfTurn + 90;
  //      ModeTwoPickUp();
  //      }
  //      else{
  //      degreeOfTurn = 0; ///MIGHT NEED TO WATCH OUT FOR BOT PLACING CUBE******************************************************
  //      ModeTwoPickUp();
  //      }
  //    }
  //  }
  //  else {
  //    ScorpionDrive(1800, 1800);
  //  }
  //}
  //
  //void modeTwoPlacement(){
  // Ping();
  //  if(ul_Echo_Time >= 10 /**mm**/) //NEED ACTUAL DISTANCE AWAY*************************************************************
  //  {
  //    turn90R();
  //    ScorpionDrive(1800, 1800);
  //    if (//Time)
  //  }
  //}




