
//libraries
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>

//servo variables
Servo servo_RightMotor;
Servo servo_LeftMotor;

//global variables
int leftSpeed, rightSpeed;
int sgClawGripClosed;
Servo sgMyServo;
Servo rightClawSurvey,leftClawSurvey;
long sgPrev;
int sgSurveyIncrement; 
int sgMagnetDetectionValue;

//Port pin constants
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int rightClawSurveyPin=0;
const int leftClawSurveyPin=0;
const int rightClawGripPin=0;
const int leftClawGripPin=0;
const int leftClawVerticalPin=0;
const int rightClawVerticalPin=0;

//motor speed vairables 
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
unsigned int ui_Motors_Speed = 1800;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;

//functions
void ScorpionDrive(int leftSpeed, int rightSpeed);
void clawGrip(int clawPin);
void clawSurvey (long surveyInterval);
void passBack ();
void placement();
void tailTuck();
bool magnet();
void modeTwoPickUp();
void modeTwoPlacement();
void Ping();
bool magnet(int hallEffectPin);

void setup() {
 
  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);
}

void loop() {


  ScorpionDrive(0,0);

}

void ScorpionDrive(int left, int right)
{
   ui_Left_Motor_Speed = constrain(1500 + left, 1600, 2100);
   ui_Right_Motor_Speed = constrain(1500 + right, 1600, 2100);
   servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
   servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
}

void clawGrip(int claw)
{
  sgMyServo.attach(claw);
  sgMyServo.write(sgClawGripClosed);
}
void clawSurvey(long surveyInterval)
{
 
  sgPrev = millis();
  if(millis() - sgPrev >= surveyInterval)
  {
    rightClawSurvey.write(sgSurveyIncrement);
    leftClawSurvey.write(sgSurveyIncrement);
  }
   
}
bool magnet(int hallEffectPin)
{
  if (analogRead(hallEffectPin)>= sgMagnetDetectionValue)
    return true;
  else
    return false;
}


