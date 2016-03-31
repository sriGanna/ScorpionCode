
//libraries
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>

//Tail
Servo krtailA;
Servo krtailB;
Servo krtailC;
Servo krtailRot;

boolean krtailSleep = 0;

const double krtailLength1 = 20;
const double krtailLength2 = 25;
const double krtailLength3 = 5;

const double pi = 3.141592654;

double krtailAPos = 20;
double krtailBPos = 20;
double krtailCPos = 20;
double krtailRotPos = 20;
double krtailAPosTarget = 20;
double krtailBPosTarget = 20;
double krtailCPosTarget = 20;
double krtailRotPosTarget = 20;

const double krtailAd = 0;
const double krtailAz = 5;

int krtailSpeed = 20;
int krtailTimer1 = 0;

double krtailx;
double krtaily;
double krtailz;
char krtailDirection;
double krtailx2;
double krtaily2;
double krtailz2;
char krtailDirection2;

double krtailARate = 0;
double krtailBRate = 0;
double krtailCRate = 0;
double krtailRotRate = 0;

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
int ecHomeTailRotor;
int ecHomeTailBase;
int ecHomeTailAppendage;

//<<<<<<< HEAD
//tail constants
int degreeOfExtension = 45;
int degreeOfTurn = 0;
//=======
//navigation function variables
int turnNumber = 0;
int width = 0;
int turnCounter = 0;
bool navigate = true;
//>>>>>>> origin/master

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
const int tailRotor;
const int tailBase;
const int tailAppendage;

//motor speed vairables
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
//<<<<<<< HEAD
void turn90L();
void turn90R();
//=======
void navigation();
void findWidth();
void findLength();
//>>>>>>> origin/master

void setup() {

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

  //<<<<<<< HEAD
  //clawGrip(8);
  //
  //=======
  //// phases, to help with communicatiom
  ////change in phase indicates a need for communication
  //
  //  //ScorpionDrive(0,0);
  //  if(!magnet(hallLeftClaw)||!magnet(hallRightClaw)) // only searches if no magnet has been found
  //  {
  //  Survey(50);
  //  }
  //  else // will stop and determine which claw needs to capture the tesseract
  //  {
  //    if(magnet(hallLeftClaw))
  //      clawGrip(hallLeftClaw);
  //   else if(magnet(hallRightClaw))
  //      clawGrip(hallRightClaw);
  //  }
  //  //a navigation clause that allows us to enter of exit navigation mode
  //  if (navigate == true)
  //  {
  //    navigation();
  //  }
  //
  //>>>>>>> origin/master

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
    tail(homeTailX, homeTailY, homeTailZ, FOR


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
  sgMyServo.write(sgPassBackValue);
  delay(1000); // I think we can use delay here becasue we wouldn't be navigating
  sgMyServo.detach();

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
void navigation() {
  // to get average wdth
  if (!(turnCounter % 2))
  {
    findWidth();
    Ping(ci_Ultrasonic_Ping_Center, ci_Ultrasonic_Data_Center);
    while (ul_Echo_Time / 58 > ((width) - (15 + 4 * turnCounter))) // the right side of the condition is the width of the subtract the free zone
    {
      Survey (50);
      if ((magnet(hallLeftClaw)) || (magnet(hallRightClaw))) {
        ecRoadMap = 2;
        break;
      }
      ScorpionDrive(200, 200);
      // inlcude break statement if light sensor detected
    }
    ScorpionDrive(0, 200); // turn
    turnCounter++;
  }
  else if (turnCounter % 2)
  {

  }

}

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
void tail (double x, double y, double z, char Ddirection) {
  if (!krtailSleep) {
    krtailx = x;
    krtaily = y;
    krtailz = z;
    if (krtailx == 0)
      krtailx = 0.001;
    if (krtaily == 0)
      krtaily = 0.001;
    double taild = sqrt(pow(krtailx, 2) + pow(krtaily, 2));
    krtailDirection = Ddirection;
    if ((krtailx != krtailx2) || (krtaily != krtaily2) || (krtailz != krtailz2) || (krtailDirection != krtailDirection2)) {
      double tailBd;
      double tailBz;
      double tailCd;
      double tailCz;
      double tailDd;
      double tailDz;
      tailDd = taild;
      tailDz = krtailz;
      if (krtailDirection == 'd') {
        tailCd = tailDd;
        tailCz = tailDz + krtailLength3;
      }
      if (krtailDirection == 'o') {
        tailCd = tailDd + krtailLength3;
        tailCz = tailDz;
      }
      if (krtailDirection == 'u') {
        tailCd = tailDd;
        tailCz = tailDz - krtailLength3;
      }
      double p;
      double q;
      double m;
      double tailADif;
      double tailBDif;
      double tailCDif;
      double tailRotDif;
      p = sqrt(pow((tailCd - krtailAd), 2) + pow((tailCz - krtailAz), 2));
      q = (pow(krtailLength1, 2) - pow(krtailLength2, 2) + pow(p, 2)) / (2 * p);
      m = sqrt(pow(krtailLength1, 2) - pow(q, 2));
      tailBd = ((q / p) * (tailCd - krtailAd)) - ((m / p) * (tailCz - krtailAz)) + krtailAd;
      tailBz = ((q / p) * (tailCz - krtailAz)) + ((m / p) * (tailCd - krtailAd)) + krtailAz;
      krtailAPosTarget = (((atan((tailBd - krtailAd) / (tailBz - krtailAz))) / (2 * pi)) * (-360)) + 90;
      krtailBPosTarget = (((atan((tailCz - tailBz) / (tailCd - tailBd))) / (2 * pi)) * 360) + (180 - krtailAPosTarget);
      if (krtailDirection == 'd')
        krtailCPosTarget = 90 - (atan((tailCz - tailBz) / (tailCd - tailBd))) / (2 * pi) * 360;
      else if (krtailDirection == 'o')
        krtailCPosTarget = 180 - (atan((tailCz - tailBz) / (tailCd - tailBd))) / (2 * pi) * 360;
      else
        krtailCPosTarget = 180;
      if (krtaily > 0)
        krtailRotPosTarget = ((atan(krtailx / krtaily)) / (2 * pi) * 360);
      else
        krtailRotPosTarget = ((atan(krtailx / krtaily)) / (2 * pi) * 360) + 180;
      tailADif = krtailAPosTarget - krtailAPos;
      tailBDif = krtailBPosTarget - krtailBPos;
      tailCDif = krtailCPosTarget - krtailCPos;
      tailRotDif = krtailRotPosTarget - krtailRotPos;
      if (tailADif < 0)
        tailADif = -tailADif;
      if (tailBDif < 0)
        tailBDif = -tailBDif;
      if (tailADif < 0)
        tailCDif = -tailCDif;
      if (tailADif < 0)
        tailRotDif = -tailRotDif;
      double maxDist;
      if (tailADif >= tailBDif && tailADif >= tailCDif && tailADif >= tailRotDif)
        maxDist = tailADif;
      else if (tailBDif >= tailADif && tailBDif >= tailCDif && tailBDif >= tailRotDif)
        maxDist = tailBDif;
      else if (tailCDif >= tailADif && tailCDif >= tailBDif && tailCDif >= tailRotDif)
        maxDist = tailCDif;
      else
        maxDist = tailRotDif;
      krtailARate = (krtailAPosTarget - krtailAPos) / maxDist;
      krtailBRate = (krtailBPosTarget - krtailBPos) / maxDist;
      krtailCRate = (krtailCPosTarget - krtailCPos) / maxDist;
      krtailRotRate = (krtailRotPosTarget - krtailRotPos) / maxDist;
      krtailx2 = krtailx;
      krtaily2 = krtaily;
      krtailz2 = krtailz;
      krtailDirection2 = krtailDirection;
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




