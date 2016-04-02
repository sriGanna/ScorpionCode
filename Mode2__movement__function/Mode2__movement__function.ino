//functions needed
void ScorpionDrive(int leftSpeed, int rightSpeed);
void tailCollapse();
int sidePing(int pin);

//variables 
int mode2LDistance;
int mode2RDistance;
int noWhere;
bool foundSide;
long prevTime;
long distTime = 0;
const int ci_Ultrasonic_Ping_Center = 3;   //input plug
const int ci_Ultrasonic_Data_Center = 13;   //output plug
const int ci_Ultrasonic_Left = 4;
const int ci_Ultrasonic_Right = 12;

//pin constants 


//functions made
void passUnderGate1();
void passUnderGate2();


void setup()
{}
void loop()
{}

//function definitions:

void passUnderGate1()
{
  prevTime = millis();
  ScorpionDrive(300, 300);
  delay(1000);
  tailCollapse();
  ScorpionDrive(-300, -300);
  delay(1000);
  if (sidePing(ci_Ultrasonic_Left) < noWhere || sidePing(ci_Ultrasonic_Right) < noWhere)
  {
    foundSide = true;
  }
  mode2LDistance = sidePing(ci_Ultrasonic_Left);
  mode2RDistance = sidePing(ci_Ultrasonic_Right);
  while (sidePing(ci_Ultrasonic_Left) <= mode2LDistance && sidePing(ci_Ultrasonic_Right) <= mode2RDistance)
  {
    ScorpionDrive(-300, -300);
  }
  distTime = millis() - prevTime;
  ScorpionDrive(-300, -300);
  delay(1000);
  ScorpionDrive(0,0);

}

void passUnderGate2()
{
  prevTime = millis();
  ScorpionDrive(300, 300);
  delay(1000);
  tailCollapse();
  while (sidePing(ci_Ultrasonic_Left) >= noWhere || sidePing(ci_Ultrasonic_Right) >= noWhere)
  {
    ScorpionDrive(-300, -300);
  }
  foundSide = true;
  distTime = millis() - prevTime;
  ScorpionDrive(-300, -300);
  delay(1000);// determine delay by testing :P 
  ScorpionDrive(0, 0);
}

