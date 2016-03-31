#include <math.h>
#include <Servo.h> 

Servo krtailA;
Servo krtailB;
Servo krtailC;
Servo krtailRot;

boolean krtailSleep=0;

const double krtailLength1=20;
const double krtailLength2=25;
const double krtailLength3=5;

const double pi=3.141592654;

double krtailAPos=20;
double krtailBPos=20;
double krtailCPos=20;
double krtailRotPos=20;
double krtailAPosTarget=20;
double krtailBPosTarget=20;
double krtailCPosTarget=20;
double krtailRotPosTarget=20;

const double krtailAd=0;
const double krtailAz=5;

int krtailSpeed=20;
int krtailTimer1=0;

double krtailx;
double krtaily;
double krtailz;
char krtailDirection;
double krtailx2;
double krtaily2;
double krtailz2;
char krtailDirection2;

double krtailARate=0;
double krtailBRate=0;
double krtailCRate=0;
double krtailRotRate=0;


void tail (double x, double y, double z, char Ddirection){
  if(!krtailSleep){
  krtailx=x;
  krtaily=y;
  krtailz=z;
  if (krtailx==0)
  krtailx=0.001;
  if (krtaily==0)
  krtaily=0.001;
  double taild=sqrt(pow(krtailx,2)+pow(krtaily,2));
  krtailDirection=Ddirection;
 if ((krtailx!=krtailx2)||(krtaily!=krtaily2)||(krtailz!=krtailz2)||(krtailDirection!=krtailDirection2)){
  double tailBd;
  double tailBz;
  double tailCd;
  double tailCz;
  double tailDd;
  double tailDz;
  tailDd=taild;
  tailDz=krtailz;
  if (krtailDirection=='d'){
  tailCd=tailDd;
  tailCz=tailDz+krtailLength3;}
  if (krtailDirection=='o'){
  tailCd=tailDd+krtailLength3;
  tailCz=tailDz;}
  if (krtailDirection=='u'){
  tailCd=tailDd;
  tailCz=tailDz-krtailLength3;}
  double p;
  double q;
  double m;
  double tailADif;
  double tailBDif;
  double tailCDif;
  double tailRotDif;
  p=sqrt(pow((tailCd-krtailAd),2)+pow((tailCz-krtailAz),2));
  q=(pow(krtailLength1,2)-pow(krtailLength2,2)+pow(p,2))/(2*p);
  m=sqrt(pow(krtailLength1,2)-pow(q,2));
  tailBd=((q/p)*(tailCd-krtailAd))-((m/p)*(tailCz-krtailAz))+krtailAd;
  tailBz=((q/p)*(tailCz-krtailAz))+((m/p)*(tailCd-krtailAd))+krtailAz;
  krtailAPosTarget=(((atan((tailBd-krtailAd)/(tailBz-krtailAz)))/(2*pi))*(-360))+90;
  krtailBPosTarget=(((atan((tailCz-tailBz)/(tailCd-tailBd)))/(2*pi))*360)+(180-krtailAPosTarget);
  if (krtailDirection=='d')
  krtailCPosTarget=90-(atan((tailCz-tailBz)/(tailCd-tailBd)))/(2*pi)*360;
  else if (krtailDirection=='o')
  krtailCPosTarget=180-(atan((tailCz-tailBz)/(tailCd-tailBd)))/(2*pi)*360;
  else
  krtailCPosTarget=180;
  if (krtaily>0)
  krtailRotPosTarget=((atan(krtailx/krtaily))/(2*pi)*360);
  else 
  krtailRotPosTarget=((atan(krtailx/krtaily))/(2*pi)*360)+180;
  tailADif=krtailAPosTarget-krtailAPos;
  tailBDif=krtailBPosTarget-krtailBPos;
  tailCDif=krtailCPosTarget-krtailCPos;
  tailRotDif=krtailRotPosTarget-krtailRotPos;
  if (tailADif<0)
  tailADif=-tailADif;
  if (tailBDif<0)
  tailBDif=-tailBDif;
  if (tailADif<0)
  tailCDif=-tailCDif;
  if (tailADif<0)
  tailRotDif=-tailRotDif;
  double maxDist;
  if (tailADif>=tailBDif&&tailADif>=tailCDif&&tailADif>=tailRotDif)
  maxDist=tailADif;
  else if (tailBDif>=tailADif&&tailBDif>=tailCDif&&tailBDif>=tailRotDif)
  maxDist=tailBDif;
  else if (tailCDif>=tailADif&&tailCDif>=tailBDif&&tailCDif>=tailRotDif)
  maxDist=tailCDif;
  else
  maxDist=tailRotDif;
  krtailARate=(krtailAPosTarget-krtailAPos)/maxDist;
  krtailBRate=(krtailBPosTarget-krtailBPos)/maxDist;
  krtailCRate=(krtailCPosTarget-krtailCPos)/maxDist;
  krtailRotRate=(krtailRotPosTarget-krtailRotPos)/maxDist;
  krtailx2=krtailx;
  krtaily2=krtaily;
  krtailz2=krtailz;
  krtailDirection2=krtailDirection;
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
  
}}
