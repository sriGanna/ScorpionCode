#include <Servo.h> 

Servo krtailA;
Servo krtailB;
Servo krtailC;
Servo krtailRot;

int krtailSpeed=20;
int krtailTimer1=0;

double krtailAPos=20;
double krtailBPos=20;
double krtailCPos=20;
double krtailRotPos=20;
double krtailAPosTarget=20;
double krtailBPosTarget=20;
double krtailCPosTarget=20;
double krtailRotPosTarget=20;
double krtailAPosTarget2=20;
double krtailBPosTarget2=20;
double krtailCPosTarget2=20;
double krtailRotPosTarget2=20;

double krtailARate=0;
double krtailBRate=0;
double krtailCRate=0;
double krtailRotRate=0;

void tailWrite(double Rot, double A, double B, double C){
krtailAPosTarget=A;
krtailBPosTarget=B;
krtailCPosTarget=C;
krtailRotPosTarget=Rot;
double maxTarget;
if (krtailAPosTarget!=krtailAPosTarget2||krtailBPosTarget!=krtailBPosTarget2||krtailCPosTarget!=krtailCPosTarget2||krtailRotPosTarget!=krtailRotPosTarget2)
{
if (krtailAPosTarget>=krtailBPosTarget&&krtailAPosTarget>=krtailCPosTarget&&krtailAPosTarget>=krtailRotPosTarget)
maxTarget=krtailAPosTarget;

else if (krtailBPosTarget>=krtailAPosTarget&&krtailBPosTarget>=krtailCPosTarget&&krtailBPosTarget>=krtailRotPosTarget)
maxTarget=krtailBPosTarget;

else if (krtailCPosTarget>=krtailAPosTarget&&krtailCPosTarget>=krtailBPosTarget&&krtailCPosTarget>=krtailRotPosTarget)
maxTarget=krtailCPosTarget;

else
maxTarget=krtailRotPosTarget;

krtailARate=krtailAPosTarget/maxTarget;
krtailBRate=krtailBPosTarget/maxTarget;
krtailCRate=krtailCPosTarget/maxTarget;
krtailRotRate=krtailRotPosTarget/maxTarget;

krtailAPosTarget2=krtailAPosTarget;
krtailBPosTarget2=krtailBPosTarget;
krtailCPosTarget2=krtailCPosTarget;
krtailRotPosTarget2=krtailRotPosTarget;
}
 if ((millis()-krtailTimer1)>krtailSpeed){
 if(sqrt(pow((krtailAPos-krtailAPosTarget),2))<=sqrt(pow(krtailARate,2)))
   krtailAPos=krtailAPosTarget;
   else
   krtailAPos=krtailAPos+krtailARate;
   krtailA.write(krtailAPos);
   
  if(sqrt(pow((krtailBPos-krtailBPosTarget),2))<=sqrt(pow(krtailBRate,2)))
   krtailBPos=krtailBPosTarget;
   else
   krtailBPos=krtailBPos+krtailBRate;
   krtailB.write(krtailBPos);
   
  if(sqrt(pow((krtailCPos-krtailCPosTarget),2))<=sqrt(pow(krtailCRate,2)))
   krtailCPos=krtailCPosTarget;
   else
   krtailCPos=krtailCPos+krtailCRate;
   krtailC.write(krtailCPos);
   
  if(sqrt(pow((krtailRotPos-krtailRotPosTarget),2))<=sqrt(pow(krtailRotRate,2)))
   krtailRotPos=krtailRotPosTarget;
   else
   krtailRotPos=krtailRotPos+krtailRotRate;
   krtailRot.write(krtailRotPos);
   
   krtailTimer1=millis();
 }
}
