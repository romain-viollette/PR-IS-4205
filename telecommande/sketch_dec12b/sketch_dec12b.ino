#include <math.h>

#define XMOD          7
#define YMOD          -15

#define DEADZONE      2

const int Xmod = 512 - 519; // alter the second number to match the idle reading of your joystick's X axis
const int Ymod = 512 - 497; // alter the second number to match the idle reading of your joystick's Y axis

#define FORWARD       0
#define BACKWARD      1
#define STOP          2


long x;
byte xdir;
long  y;
byte ydir;

float L;
float R;

float angle;
float amplitude;

void setup()
{
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  Serial.begin(115200); 
  delay(10);
}

void loop()
{
  x = 512-analogRead(A0); //from -512 to +511
  y = 512-analogRead(A1);

  x += XMOD;
  y += YMOD;

  angle = atan2(y,x);
  amplitude = sqrt(x*x+y*y);

  if(amplitude < DEADZONE)
  {
    xdir = STOP;
    ydir = STOP; 
  }
  else if(angle < -(5.0/6.0) * M_PI)
  {
    xdir = BACKWARD;
    ydir = FORWARD;
    L = map2(amplitude,DEADZONE,360.0,0.0,255.0);
    R = L * map2(angle, - M_PI, -(5.0/6.0)*M_PI, 1.0, 0.0);
  }
  else if(angle < -(1.0/2.0) * M_PI)
  {
    xdir = BACKWARD;
    ydir = BACKWARD;
    L = map2(amplitude,DEADZONE,360.0,0.0,255.0);
    R = L * map2(angle, -(5.0/6.0)*M_PI, -(1.0/2.0)*M_PI, 0.0, 1.0);
  }
  else if(angle < -(1.0/6.0) * M_PI)
  {
    xdir = BACKWARD;
    ydir = BACKWARD;
    R = map2(amplitude,DEADZONE,360.0,0.0,255.0);
    L = R * map2(angle, -(1.0/2.0)*M_PI, -(1.0/6.0)*M_PI, 1.0, 0.0);
  }
  else if(angle < 0)
  {
    xdir = FORWARD;
    ydir = BACKWARD;
    R = map2(amplitude,DEADZONE,360.0,0.0,255.0);
    L = R * map2(angle, -(1.0/6.0)*M_PI, 0.0, 0.0, 1.0);
  }
  else if(angle < (1.0/6.0) * M_PI)
  {
    xdir = FORWARD;
    ydir = BACKWARD;
    L = map2(amplitude,DEADZONE,360.0,0.0,255.0);
    R = L * map2(angle, 0.0, (1.0/6.0)*M_PI, 1.0, 0.0);
  }
  else if(angle < (1.0/2.0) * M_PI)
  {
    xdir = FORWARD;
    ydir = FORWARD;
    L = map2(amplitude,DEADZONE,360.0,0.0,255.0);
    R = L * map2(angle, (1.0/6.0)*M_PI, (1.0/2.0)*M_PI, 0.0, 1.0);
  }
  else if(angle < (5.0/6.0) * M_PI)
  {
    xdir = FORWARD;
    ydir = FORWARD;
    R = map2(amplitude,DEADZONE,360.0,0.0,255.0);
    L = R * map2(angle, (1.0/2.0)*M_PI, (5.0/6.0)*M_PI, 1.0, 0.0);
  }
  else /*if(angle <= M_PI)*/
  {
    xdir = BACKWARD;
    ydir = FORWARD;
    R = map2(amplitude,DEADZONE,360.0,0.0,255.0);
    L = R * map2(angle, (5.0/6.0)*M_PI, M_PI, 0.0, 1.0);
  }


  L = constrain(L,0.0,255.0);
  R = constrain(R,0.0,255.0);

//  L/=7.0;
//  R/=7.0;

  if(xdir == STOP)
  {
    Serial.write('o');
    Serial.write('r');
    Serial.println();
  }
  else
  {
    if(xdir == FORWARD)
      Serial.write('l');
    else
      Serial.write('L'); 
    Serial.write((byte)L);
    Serial.println();
  }
  delay(100);

  if(ydir == STOP)
  {
    Serial.write('o');
    Serial.write('l');
    Serial.println();
  }
  else
  {
    if(ydir == FORWARD)
      Serial.write('r');
    else
      Serial.write('R');
    Serial.write((byte)R);
    Serial.println();
  }

  delay(100);

  //Serial.println(L,DEC);
  //Serial.println(R,DEC);
  //
  ////  Serial.print("Ang ");
  //  Serial.println(angle,DEC);
  //  
  //  Serial.print("Amp ");
  //  Serial.println(map2(amplitude,DEADZONE,360.0,0.0,255.0)/2.0,DEC);
  //  
  //  Serial.print("X ");
  //  Serial.println(x,DEC);
  //  
  //  Serial.print("Y ");
  //  Serial.println(y,DEC);
  //  
  //  Serial.println();
}

float map2(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}







