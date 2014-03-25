/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>


#define InA1            8                      // INA motor pin
#define InB1            10                      // INB motor pin 
#define PWM1            9                       // PWM motor pin
#define encodPinA1      2                       // encoder A pin
#define encodPinB1      4                       // encoder B pin
#define RPS1            272                     // max rotates per seconds of the motor at 12v (PWM 100%)

#define InA2            13                      // INA motor pin
#define InB2            12                      // INB motor pin 
#define PWM2            11                       // PWM motor pin
#define encodPinA2      3                       // encoder A pin
#define encodPinB2      5                       // encoder B pin
#define RPS2            285                     // max rotates per seconds of the motor at 12v (PWM 100%)


struct Wheel{
  int stepCount;
  double Setpoint, Input, Output;
};

Wheel lWheel;
Wheel rWheel;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID lPID(&(lWheel.Input), &(lWheel.Output), &(lWheel.Setpoint),2,10,0, DIRECT);
PID rPID(&(rWheel.Input), &(rWheel.Output), &(rWheel.Setpoint),2,5,1, DIRECT);

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("Bonjour!");

  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(encodPinA1, INPUT); 
  pinMode(encodPinB1, INPUT); 
  pinMode(encodPinA2, INPUT); 
  pinMode(encodPinB2, INPUT); 
  attachInterrupt(0, rencoder1, FALLING);
  attachInterrupt(1, rencoder2, FALLING);

  digitalWrite(InA2, HIGH);
  digitalWrite(InB2, LOW);

  //initialize the variables we're linked to
  lWheel.stepCount = 0;
  lWheel.Input = 0;
  lWheel.Setpoint = 0;

  rWheel.stepCount = 0;
  rWheel.Input = 0;
  rWheel.Setpoint = 0;

  lPID.SetOutputLimits(-256, 255);
  rPID.SetOutputLimits(-256, 255);

  //turn the PID on
  lPID.SetMode(AUTOMATIC);
  rPID.SetMode(AUTOMATIC);
}

void loop()
{
  lWheel.Input = lWheel.stepCount;
  rWheel.Input = rWheel.stepCount;

  lPID.Compute();
  rPID.Compute();

  if(lWheel.Output > 0)
  {
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, LOW);
    analogWrite(PWM1, lWheel.Output);
  }
  else if(lWheel.Output == 0)
  {
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, LOW);
    analogWrite(PWM1, 0);
  }
  else
  {
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);
    analogWrite(PWM1, -lWheel.Output);
  }

  if(rWheel.Output > 0)
  {
    digitalWrite(InA2, HIGH);
    digitalWrite(InB2, LOW);
    analogWrite(PWM2, rWheel.Output);
  }
  else if(rWheel.Output == 0)
  {
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, LOW);
    analogWrite(PWM2, 0);
  }
  else
  {
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, HIGH);
    analogWrite(PWM2, -rWheel.Output);
  }
  getParam();
}

void rencoder1()  {
  if (PIND & (1<<4))    lWheel.stepCount--;    
  else                  lWheel.stepCount++;
}

void rencoder2()  { 
  if (PIND & (1<<5))    rWheel.stepCount--;
  else                  rWheel.stepCount++;
}
void getParam()  {
  char param, cmd1=0, cmd2=0;

  if(!Serial.available())   
    return;                
  param = Serial.read();   

  switch (param) {
  case 's':
    delay(1); 
    if(!Serial.available())
      return;
    cmd1 = Serial.read();
    delay(1); 
    if(!Serial.available())
      return;
    cmd2 = Serial.read();

    Serial.print("Setpoint: ");
    Serial.print(lWheel.Setpoint, DEC);
    lWheel.stepCount = 0;
    rWheel.stepCount = 0;
    lWheel.Setpoint = cmd1<<8 | cmd2;
    rWheel.Setpoint = lWheel.Setpoint;
    Serial.print(" -> ");
    Serial.println(lWheel.Setpoint, DEC);
    break;

  case 'o':
    Serial.print("Setpoint: ");
    Serial.println(lWheel.Setpoint);
    Serial.print("input (capteur L): ");
    Serial.println(lWheel.Input);
    Serial.print("input (capteur R): ");
    Serial.println(rWheel.Input);
    Serial.print("Output (PWM L): ");
    Serial.println(lWheel.Output);
    Serial.print("Output (PWM R): ");
    Serial.println(rWheel.Output);
    break;

  case 'p':
  delay(1); 
    if(!Serial.available())
      return;
    cmd1 = Serial.read();
    delay(1); 
    if(!Serial.available())
      return;
    cmd2 = Serial.read();
    if(cmd1 == '1')
    {
      lPID.SetTunings(cmd2, lPID.GetKi(), lPID.GetKd());
    }
    else if(cmd1 == '2')
    {
      rPID.SetTunings(cmd2, rPID.GetKi(), rPID.GetKd());
    }
    affichePID();
    break;

  case 'i':
    if(cmd1 == '1')
    {
      lPID.SetTunings(lPID.GetKp(), cmd2, lPID.GetKd());
    }
    else if(cmd1 == '2')
    {
      rPID.SetTunings(lPID.GetKp(), cmd2, rPID.GetKd());
    }
    affichePID();
    break;

  case 'd':
    if(cmd1 == '1')
    {
      lPID.SetTunings(rPID.GetKp(), lPID.GetKi(), cmd2);
    }
    else if(cmd1 == '2')
    {
      rPID.SetTunings(rPID.GetKp(), rPID.GetKi(), cmd2);
    }
    affichePID();
    break;

  case 'a':
    affichePID();
    break;
  }
  Serial.flush();

  return;
}


void affichePID()
{
  Serial.print("L: ");
  Serial.print("p = ");
  Serial.print(lPID.GetKp());
  Serial.print("   i = ");
  Serial.print(lPID.GetKi());
  Serial.print("   d = ");
  Serial.println(lPID.GetKd()); 

  Serial.print("R: ");
  Serial.print("p = ");
  Serial.print(rPID.GetKp());
  Serial.print("   i = ");
  Serial.print(rPID.GetKi());
  Serial.print("   d = ");
  Serial.println(rPID.GetKd()); 
}











