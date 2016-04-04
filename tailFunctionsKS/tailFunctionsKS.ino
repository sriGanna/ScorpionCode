
#include <Servo.h>

Servo servo_appendage1;  // create servo object to control a servo
Servo servo_base;// twelve servo objects can be created on most boards
Servo servo_pistonExtend, servo_baseRot, servo_piston;

double posbase = 30;    // variable to store the servo position
double posappendage1 = 0;
double rear = 5;
double pos = 0;
double front = 180;
int numberOfTesseractsCollected;
int ksPos;
int increment;
int pistonExtenderExtended, pistonExtenderRetracted;
long duration;

const int ci_TailRot = 11;
const int ci_TailBase = 9;
const int ci_TailAppend = 10;
const int ci_Piston = 5;
const int ci_PistonExtend = 3;
const int ci_TailHall = A5;
const int topLineTracker = 3;

bool magnet(int pin);
bool readLineTracker(int pin);

void setup() {
  servo_base.attach(ci_TailBase);  // attaches the servo on pin 9 to the servo object
  servo_appendage1.attach(ci_TailAppend);
  servo_pistonExtend.attach(ci_PistonExtend);
  servo_baseRot.attach(ci_TailRot);
  servo_piston.attach(ci_Piston);

}

void loop() {
  // the code to get the bot to pick up tesseract at beginning of mode 2 
  servo_baseRot.write(0);
  delay(1000);
  tailExtend();
  delay(5000);
  servo_piston.write(50);
  servo_pistonExtend.write(60);
  delay(5000);
  servo_base.write(170);
  delay(200);
  servo_appendage1.write(120);
  delay(5000);
  servo_baseRot.write(75);
  delay(5000);
  servo_appendage1.write(140);
  delay(2000);
  servo_baseRot.write(50);
  delay(100);
  servo_appendage1.write(110);
  
  delay(100000000000);

// the code to get tail to drop off position 
  /*
  servo_piston.attach(ci_Piston);
  while(sidePing(4) <= 6)
  {
    servo_piston.write(0);
    delay(1000);
    servo_piston.write(160);
  }
  delay(500);
  servo_piston.write(0);
  delay(3000);
  depositMagnetMode2();
  delay(100000000);
  */
}
void tailExtend()
{
  servo_appendage1.write(160);
  delay(100);
  servo_base.write(170);
  delay(2000);
  servo_appendage1.write(180);
  delay(300);
  servo_appendage1.write(100);
  delay(500);
  servo_appendage1.write(30);
  delay(400);
  servo_appendage1.write(0);
  //// delay(10000000);
  servo_base.write(180);

  // servo_base.detach();
  //servo_appendage1.detach();
}

void depositMagnetMode2()
{
  servo_pistonExtend.attach(ci_PistonExtend);
  servo_pistonExtend.write(0);
  delay(2000);

  for (double pos = 0; pos <= 170; pos += .01) { // goes from 180 degrees to 0 degrees
    servo_piston.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15ms for the servo to reach the position
  }
  servo_pistonExtend.write(125);
}

void tailCollapse()
{
  servo_base.attach(ci_TailBase);
  servo_baseRot.attach(ci_TailRot);
  servo_appendage1.attach(ci_TailAppend);
  servo_baseRot.write(rear);
  servo_pistonExtend.attach(ci_PistonExtend);
  servo_pistonExtend.write(0);

  servo_base.write(180);
  servo_appendage1.write(120);
  delay(200);
  servo_base.write(170);
  delay(100);
  servo_base.write(165);
  delay(100);
  servo_base.write(160);
  delay(100);
  servo_base.write(150);
  delay(50);
  servo_base.write(145);
  delay(20);
  servo_appendage1.write(110);
  delay(500);
  servo_base.write(140);
  delay(50);
  servo_appendage1.write(80);
  delay(100);
  servo_base.write(130);
  delay(50);
  servo_appendage1.write(0);
  delay(500);
  servo_base.write(120);
  delay(1000);
  servo_base.write(100);

}
void scanForTesseractsMode2()
{
  tailExtend();
  servo_appendage1.write(5); //test to find angles
  servo_pistonExtend.write(5);
  servo_piston.write(180);
  double pos;
  while (!(magnet(ci_TailHall)))
  {
    ksPos += increment ;
    servo_baseRot.write(ksPos);
    if (magnet(ci_TailHall))
      break;
    if (ksPos >= 180 || ksPos <= 0)
    {
      increment = -increment;
      delay(5000);
    }
  }
}

void tailPickUpMode2()
{
  servo_pistonExtend.write(pistonExtenderExtended);
  servo_piston.write(0);
  servo_pistonExtend.write(pistonExtenderRetracted);
  servo_appendage1.write(0);
  servo_baseRot.write(rear);
  delay(500);
}

void DepositTesseractsMode1()
{
  tailExtend();
  servo_appendage1.write(4); //test to find angles
  servo_pistonExtend.write(4);
  double ksPos;
  increment = 3;
  bool Begin, End;
  int line = 0;
  if (numberOfTesseractsCollected == 0)
  {
    while (line < 3)
    {
      ksPos += increment ;
      servo_baseRot.write(ksPos);

      if (readLineTracker(topLineTracker))
        Begin = true;
      if ((!readLineTracker(topLineTracker)) && Begin)
        End = true;
      if (Begin == true && End == true)
      {
        line ++;
        Begin = false;
        End = false;
      }
    }
  }

  else if (numberOfTesseractsCollected == 1)
  {
    while (line < 2)
    {
      ksPos += increment ;
      servo_baseRot.write(ksPos);

      if (readLineTracker(topLineTracker))
      {
        Begin = true;
      }
      if ((!readLineTracker(topLineTracker)) && Begin)
        End = true;
      if (Begin == true && End == true)
      {
        line ++;
        Begin = false;
        End = false;
      }
    }

  }

  else if (numberOfTesseractsCollected == 2)
  {
    while (line < 1)
    {
      ksPos += increment ;
      servo_baseRot.write(ksPos);

      if (readLineTracker(topLineTracker))
      {
        Begin = true;
      }
      if ((!readLineTracker(topLineTracker)) && Begin)
        End = true;
      if (Begin == true && End == true)
      {
        line ++;
        Begin = false;
        End = false;
      }
    }

    line = 0;
    ksPos += increment ;
    servo_baseRot.write(ksPos);
    numberOfTesseractsCollected++;
    if (numberOfTesseractsCollected > 3)
      numberOfTesseractsCollected = 0;
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
  delay (10);
  duration /= 58;
  return duration; //returns the distance in mm
}

