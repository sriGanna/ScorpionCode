
//libraries
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>


//Scoripion Drive Varaibles
int leftSpeed, rightSpeed;
Servo servo_RightMotor;
Servo servo_LeftMotor;

//servey function
Servo servo_RightClawHorizontal,servo_LeftClawHorizontal;
long sgSurveyPrevTime;
int sgAngleIncrement=1; // how much we want to increment the angle by
int sgSurveyAngleMax = 180; // to be determined by testing
int sgSurveyAngleMin= 0; // to be determined by testing 
int sgPos;

// grip function variable
Servo sgMyServo;
int sgClawGripClosed; // to be determined by testing

//hall effect function varaibles
int sgMagnetDetectionValue; // to be determined by testing
int count;
int magnetRead[20];

//passBack function variables
int sgPassBackValue; // to be determied by testing

//ping function variables
long ul_Echo_Time;

//navigation function variables
int turnNumber=0;
int width=0;
int turnCounter=0;
bool navigate=true;

//Port pin constants
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Ultrasonic_Ping_Center = 2;   //input plug
const int ci_Ultrasonic_Data_Center = 3;   //output plug
const int ci_Ultrasonic_Left = 5;
const int ci_Ultrasonic_Right = 6;
const int ci_RightClawHorizontal = 6;
const int ci_LeftClawHorizontal =7;
const int ci_RightClawGrip=0;
const int ci_LeftClawGrip=0;
const int ci_LeftClawVertical=0;
const int ci_RightClawVertical=0;
const int hallLeftClaw=4;
const int hallRightClaw=2;

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
void Ping(int Input, int Output);
bool magnet(int hallEffectPin);
void navigation();
void findWidth();
void findLength();

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

// phases, to help with communicatiom
//change in phase indicates a need for communication

  //ScorpionDrive(0,0);
  if(!magnet(hallLeftClaw)||!magnet(hallRightClaw)) // only searches if no magnet has been found 
  {
  Survey(50);
  }
  else // will stop and determine which claw needs to capture the tesseract
  {
    if(magnet(hallLeftClaw))
      clawGrip(hallLeftClaw);
   else if(magnet(hallRightClaw))
      clawGrip(hallRightClaw);
  }
  //a navigation clause that allows us to enter of exit navigation mode
  if (navigate == true)
  {
    navigation();
  }
   
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
  if((millis() - sgSurveyPrevTime) >= surveyInterval)  // how fast it sweeps back and forth
    {
      
      sgSurveyPrevTime = millis();
      sgPos += sgAngleIncrement;
      servo_RightClawHorizontal.write(sgPos);
      servo_LeftClawHorizontal.write(sgPos);
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
  if(count==20)
  {
    count=0;
    int aveRead=0;
    for(int i=0; i<21; i++){
      aveRead+=magnetRead[i]; 
    }
    aveRead/=20;
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

void passBack(int clawVerticalPin)
{
  sgMyServo.attach(clawVerticalPin);
  sgMyServo.write(sgPassBackValue);
  delay(1000); // I think we can use delay here becasue we wouldn't be navigating
  sgMyServo.detach();
 
}
void findWidth()
{
  
 for (int i=0; i<6;i++)
 {
   Ping(ci_Ultrasonic_Ping_Center,ci_Ultrasonic_Data_Center);
   if (ul_Echo_Time == 0)
   {
    i--;
   }
   width += ul_Echo_Time/58;
   }
   
 width = width/10;
    
}
void navigation() 
{
 // to get average wdth 
 if(!(turnCounter%2))
 {
   findWidth();
   Ping(ci_Ultrasonic_Ping_Center,ci_Ultrasonic_Data_Center);
   while(ul_Echo_Time/58 > ((width)-(15 + 4*turnCounter)))// the right side of the condition is the width of the subtract the free zone
   {
   ScorpionDrive(200,200);
   // inlcude break statement if light sensor detected
   }
   ScorpionDrive(0,200); // turn 
   turnCounter++;
 }
 else if (turnCounter%2)
 {
  
 }
 
}




