/* Sweep
  by BARRAGAN <http://barraganstudio.com>
  This example code is in the public domain.

  modified 8 Nov 2013
  by Scott Fitzgerald
  http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo appendage1;  // create servo object to control a servo
Servo base;// twelve servo objects can be created on most boards
Servo pistonExtender, baseRot, piston;

double posbase = 30;    // variable to store the servo position
double posappendage1 = 0;
double rear, pos;
int numberOfTesseractsCollected;

const int ci_TailRot = 11;
const int ci_TailBase = 9;
const int ci_TailAppend = 10;
const int ci_Piston = 5;
const int ci_PistonExtend = 3;
const int ci_TailHall = A5;
const int topLineTracker = 3;

void setup() {
  base.attach(ci_TailBase);  // attaches the servo on pin 9 to the servo object
  appendage1.attach(ci_TailAppend);
  pistonExtender.attach(ci_PistonExtend);
  baseRot.attach(ci_TailRot);

}

void loop() {

}
void tailExtend()
{
  baseRot.write(rear);
  delay(700);
  baseRot.detach(ci_TailRot);
  pistonExtender.write(0);
  delay(600);

  appendage1.write(160);
  delay(100);
  base.write(170);
  delay(2000);
  appendage1.write(180);
  delay(300);
  appendage1.write(100);
  delay(500);
  appendage1.write(30);
  delay(400);
  appendage1.write(0);
  delay(500);
  base.detach();
  appendage1.detach();
}

void depositMagnetMode2()
{
  pistonExtender.attach(ci_PistonExtend);
  pistonExtender.write(120);

  piston.attach(ci_Piston);

  for (double pos = 180; pos >= 0; pos -= .5) { // goes from 180 degrees to 0 degrees
    piston.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }

  pistonExtender.write(0);

}

void tailCollapse()
{
  base.attach(ci_TailBase);
  baseRot.attach(ci_TailRot);
  baseRot.write(rear);
  for (double pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    base.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  void scanForTesseractsMode2()
  {
    tailExtend();
    appendage1.write(); //test to find angles
    pistonExtender.write();
    piston.write(180);
    double pos;
    while (!(magnet(TailHall))
  {
    ksPos += increment ;
    baseRot.write(ksPos);
      if (magnet(TailHall))
        break;
      if (ksPos >= 180 || ksPos <= 0)
      {
        increment = -increment;
        delay(5000);
      }
    }
  }

  void tailPickUp()
  {
    pistonExtender.write(pistonExtenderExtended);
    piston.write(0);
    pistonExtender.write(pistonExtenderRetracted);
    appendage1.write(0);
    baseRot.write(rear);
    delay(500);
  }

  void DepositTesseractsMode1()
  {
    tailExtend();
    appendage1.write(); //test to find angles
    pistonExtender.write();
    double ksPos;
    increment = 3;
    bool Begin, End;
    int line = 0;
    if (numberOfTesseractsCollected == 0)
    {
      while (line < 3)
      {
        ksPos += increment ;
        baseRot.write(ksPos);

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
        baseRot.write(ksPos);

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
        baseRot.write(ksPos);

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
      baseRot.write(ksPos);
      numberOfTesseractsCollected++;
      if (numberOfTesseractsCollected > 3)
        numberOfTesseractsCollected = 0;
    }
