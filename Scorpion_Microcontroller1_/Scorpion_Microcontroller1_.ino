
#include<Servo.h>

//tail Servos
Servo servo_appendage1;  // create servo object to control a servo
Servo servo_base;// twelve servo objects can be created on most boards

Servo servo_pistonExtend, servo_baseRot, servo_piston;

// variable values/angles that need to be tested
int initiateSearchTurn;//tbd by testing
double posbase = 30;    // variable to store the servo position
double posappendage1 = 0;
double rear, pos;
int numberOfTesseractsCollected;
int pistonExtenderExtended;
int pistonExtenderRetracted;

//hall effect function varaibles
int sgMagnetDetectionValueTLow = 485;// with  piston Retracted
int sgMagnetDetectionValueTHigh = 495;
int count;
int magnetRead[20];
int aveRead;
//debugging
int found;

//line tracker function variables
int ksDark;

//function variables
int ksPos;
int increment;


//Microcontroller 2
const int ci_SwitchPin = 2;
const int ci_TailRot = 11;
const int ci_TailBase = 9;
const int ci_TailAppend = 10;
const int ci_Piston = 5;
const int ci_PistonExtend = 3;
const int ci_TailHall = A5;
const int topLineTracker = A3;

//General Code/Master
int ecRoadMap = 0; // for mode 1
int mode2RoadMap = 1; // mode 2
int mode = 0;
//list of functions:
void tailExtend();// extends the tail up
void depositMagnetMode2();//write angles for tail to deposit tesseract on the platform in mode 2, when positioned properly
void tailCollapse(); // writes angles to collapse tail
void scanForTesseractMode2(); //tail sweeps back and forth in search of tesseracts in mode 2
void tailPickUpMode2();//extends piston to pick up magnet in mode 2
void depositTesseractMode1(); // searchs for lines and drops tesseract in the correct postion:mode 1
bool magnet(int pin);
bool readLineTracker(int pin);
//still NEED:
void transportTesseract();

void setup() {
  // put your setup code here, to run once:
  servo_base.attach(ci_TailBase);  // attaches the servo on pin 9 to the servo object
  servo_appendage1.attach(ci_TailAppend);
  servo_pistonExtend.attach(ci_PistonExtend);
  servo_baseRot.attach(ci_TailRot);
  pinMode(ci_SwitchPin, INPUT); //initialize the switch
}

void loop() {
  // put your main code here, to run repeatedly:
  mode = digitalRead(ci_SwitchPin);

  if (!mode) // not sure which way
  { //mode 1:
    if (Serial.available()) {
      {
        ecRoadMap = Serial.read();
      }
      if (ecRoadMap == 0)
      {
        //resting mode, detach anything that can be detached.
      }
      if (ecRoadMap == 4)
      {
        tailExtend();
        //pick up tesseract from claw
        //hold on tesseract
        ecRoadMap++;
        Serial.write(ecRoadMap);
        ecRoadMap = 0;
      }
      if (ecRoadMap = 6)
      {
        DepositTesseractsMode1();
        ecRoadMap++;
        Serial.write(ecRoadMap);
        ecRoadMap = 0;
      }

    }
    else
    { //mode 2:
      if (Serial.available()) {
        {
          mode2RoadMap = Serial.read();
        }


        if (mode2RoadMap == 0)
        {
          // turn all useless things off
        }
        if (mode2RoadMap == 1)
        {
          tailExtend();
          servo_baseRot.write(initiateSearchTurn);
          scanForTesseractsMode2();
          tailPickUpMode2();
          tailCollapse();
          // communication and moving on to next phase
          mode2RoadMap++;
          Serial.write(mode2RoadMap);
          mode2RoadMap = 0;
        }
        if (mode2RoadMap == 3)
        {
          tailExtend();
          depositMagnetMode2();
          tailCollapse();
          // communication and moving on to next phase
          mode2RoadMap++;
          Serial.write(mode2RoadMap);
          mode2RoadMap = 0;
        }
      }
    }

    if (mode2RoadMap == 0)
    {
      // turn all useless things off
    }
    if (mode2RoadMap == 1)
    {
      tailExtend();
      servo_baseRot.write(initiateSearchTurn);
      scanForTesseractsMode2();
      tailPickUpMode2();
      tailCollapse();
      // communication and moving on to next phase
      mode2RoadMap++;
      Serial.write(mode2RoadMap);
      mode2RoadMap = 0;
    }
    if (mode2RoadMap == 3)
    {
      tailExtend();
      depositMagnetMode2();
      tailCollapse();
      // communication and moving on to next phase
      mode2RoadMap++;
      Serial.write(mode2RoadMap);
      mode2RoadMap = 0;
    }
  }
}
void tailExtend()
{
  servo_baseRot.write(rear);
  delay(700);
  servo_baseRot.detach();
  servo_pistonExtend.write(0);
  delay(600);

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
  delay(500);
  servo_base.detach();
  servo_appendage1.detach();

}
void depositMagnetMode2()

{
  servo_pistonExtend.attach(ci_PistonExtend);
  servo_pistonExtend.write(120);
  servo_piston.attach(ci_Piston);
  for (double pos = 180; pos >= 0; pos -= .5) { // goes from 180 degrees to 0 degrees
    servo_piston.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  servo_pistonExtend.write(0);
}

void tailCollapse()
{
  servo_base.attach(ci_TailBase);
  servo_baseRot.attach(ci_TailRot);
  servo_baseRot.write(rear);
  for (double pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo_base.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
void scanForTesseractsMode2()
{
  tailExtend();
  servo_appendage1.write(2); //test to find angles
  servo_pistonExtend.write(2);
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
  servo_appendage1.write(5); //test to find angles
  servo_pistonExtend.write(5);
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
  }

  line = 0;
  ksPos += increment ;
  servo_baseRot.write(ksPos);
  servo_piston.write(180);
  numberOfTesseractsCollected++;
  if (numberOfTesseractsCollected > 3)
    numberOfTesseractsCollected = 0;


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

    sgMagnetDetectionValueHigh = sgMagnetDetectionValueTHigh;
    sgMagnetDetectionValueLow = sgMagnetDetectionValueTLow;
    // MIGHT NEED TO FIND VALUES WITH PISTON EXTENDED!
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

bool readLineTracker(int lineTrackerPin)
{
  if (lineTrackerPin == topLineTracker)
    ksDark = 1005;
  if (analogRead(lineTrackerPin) > ksDark)
    return true;
  else
    return false;
}



