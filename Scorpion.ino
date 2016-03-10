
//libraries
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>

//servo variables
Servo servo_RightMotor;
Servo servo_LeftMotor;

//global variables
int left, right;

//Port pin constants
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;


//motor speed vairables 
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
unsigned int ui_Motors_Speed = 1800;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;

//functions
void ScorpionDrive(int left, int right);

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
