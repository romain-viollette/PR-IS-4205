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

int count1=0;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("Bonjour!");
  
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(encodPinA1, INPUT); 
  pinMode(encodPinB1, INPUT); 
  attachInterrupt(0, rencoder1, FALLING);

  digitalWrite(InA1, HIGH);
  digitalWrite(InB1, LOW);

  //initialize the variables we're linked to
  Input = 0;
  Setpoint = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = count1;
  
  myPID.Compute();
  analogWrite(PWM1, Output);
  getParam();
}

void rencoder1()  {                                    // pulse and direction, direct port reading to save cycles
  if (PIND & (1<<4))    count1--;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                  count1++;                // if (digitalRead(encodPinB1)==LOW)   count --;
}

int getParam()  {
  char param, cmd;
  if(!Serial.available())    return 0;
  delay(1);                  
  param = Serial.read();                              // get parameter byte
  if(!Serial.available())    return 1;
  cmd = Serial.read();                                // get command byte
  Serial.flush();
  switch (param) {
    case 's':
      Serial.print("Setpoint: ");
      Serial.print(Setpoint, DEC);
      count1 = 0;
      Setpoint = cmd;
      Serial.print(" -> ");
      Serial.println(Setpoint, DEC);
      break;
    case 'p':
      Serial.print("input (capteur): ");
      Serial.println(Input);
      Serial.print("Setpoint: ");
      Serial.println(Setpoint);
      Serial.print("Output (PWM): ");
      Serial.println(Output);
      break;
  }
  
  return 2;
}


