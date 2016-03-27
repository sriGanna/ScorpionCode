
//libraries
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>


//Scoripion Drive Varaibles
int leftSpeed, rightSpeed;
Servo servo_RightMotor;
Servo servo_LeftMotor;

//servey function
Servo servo_RightClawHorizontal,servo_LeftClawHoriztonal;
long kssurveyNewTime,kssurveyPrevTime;
int ksAngle;
bool ksScanLeft;


int sgSurveyIncrement; 

// grip function variable
Servo sgMyServo;
int sgClawGripClosed;

//hall effect function varaibles
int sgMagnetDetectionValue;

//passBack function variables
int sgPassBackValue;

//ping function variables
long ul_Echo_Time;

//Port pin constants
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Ultrasonic_Ping = 2;   //input plug
const int ci_Ultrasonic_Data = 3;   //output plug
const int ci_RightClawHorizontal = 6;
const int ci_LeftClawHorizontal =7;
const int ci_RightClawGrip=0;
const int ci_LeftClawGrip=0;
const int ci_LeftClawVertical=0;
const int ci_RightClawVertical=0;

//motor speed vairables 
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
unsigned int ui_Motors_Speed = 1800;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;

//functions
void ScorpionDrive(int leftSpeed, int rightSpeed);
void clawGrip(int clawHorizontalPin);
void Survey (long surveyInterval);
void passBack (int calwVerticalPin);
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
    
    //set up claw motors
   pinMode(ci_RightClawHorizontal, OUTPUT);
  servo_RightMotor.attach(ci_RightClawHorizontal);
  pinMode(ci_LeftClawHorizontal, OUTPUT);
  servo_LeftMotor.attach(ci_LeftClawHorizontal);

   // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

}

void loop() {


  ScorpionDrive(0,0);

}

void ScorpionDrive(int left, int right)
{
   ui_Left_Motor_Speed = constrain(1500 + leftSpeed, 1600, 2100);
   ui_Right_Motor_Speed = constrain(1500 + rightSpeed, 1600, 2100);
   servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
   servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
}

void clawGrip(int clawHorizontalPin)
{
  sgMyServo.attach(clawHorizontalPin);
  sgMyServo.write(sgClawGripClosed);
  delay(1000);
  sgMyServo.detach();
}
void Survey(long surveyInterval)
{
  kssurveyNewTime = millis();
 if(ksAngle>=180)
 ksScanLeft = false;
 if(ksScanLeft == true)
 ksAngle += 1;
 else if (ksScanLeft == false)
 ksAngle = ksAngle - 1; 
 
 if(kssurveyNewTime - kssurveyPrevTime >= surveyInterval)
  {
    servo_RightClawHorizontal.write(ksAngle); 
    kssurveyPrevTime = kssurveyNewTime;
  }
   
}
bool magnet(int hallEffectPin)
{
  
  if (abs(analogRead(hallEffectPin))>= sgMagnetDetectionValue)
    return true;
  else
    return false;
}

void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);
}

void passBack(int clawVerticalPin)
{
  sgMyServo.attach(clawVerticalPin);
  sgMyServo.write(sgPassBackValue);
  delay(1000);
  sgMyServo.detach();
 
}




