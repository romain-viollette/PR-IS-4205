#include <EEPROM.h>
#include <SimpleTimer.h>   // http://arduino.cc/playground/Code/SimpleTimer

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

#define EEPROM_L_KP    0      
#define EEPROM_L_KI    EEPROM_L_KP+4  
#define EEPROM_L_KD    EEPROM_L_KI+4 
#define EEPROM_R_KP    EEPROM_L_KD+4      
#define EEPROM_R_KI    EEPROM_R_KP+4  
#define EEPROM_R_KD    EEPROM_R_KI+4 

#define SAMPLING_PERIOD  20                      //PID sampling period (ms)
#define SAMPLING_FREQ  (1000/SAMPLING_PERIOD)    //PID sampling frequency (Hz)

#define TICK_BY_MOTOR_SHAFT  3
#define GEAR_RATIO      30

struct Wheel{
  int stepCount, speed;
  double Setpoint, Input, Output;
};

Wheel lWheel;
Wheel rWheel;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID lPID(&(lWheel.Input), &(lWheel.Output), &(lWheel.Setpoint),2,0,0, DIRECT);
PID rPID(&(rWheel.Input), &(rWheel.Output), &(rWheel.Setpoint),0,0,0, DIRECT);

// Max command LENGTH
#define COMMAND_LENGTH  100
// Buffer for the incoming data
char inData[COMMAND_LENGTH];
// Buffer for the parsed data chunks
char *inParse[COMMAND_LENGTH];
// Storage for data as string
String inString = "";
// Incoming data id
int index = 0;
// Read state of incoming data
boolean stringComplete = false;

SimpleTimer timer;

void setup()
{
  Serial.begin(19200);
  delay(100);
  Serial.println("\nBonjour!");

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
  lWheel.speed = 0;
  lWheel.Input = 0;
  lWheel.Setpoint = 0;

  rWheel.stepCount = 0;
  rWheel.speed = 0;
  rWheel.Input = 0;
  rWheel.Setpoint = 0;

  lPID.SetTunings(
  eepromReadDouble(EEPROM_L_KP), 
  eepromReadDouble(EEPROM_L_KI), 
  eepromReadDouble(EEPROM_L_KD)
    );

  rPID.SetTunings(
  eepromReadDouble(EEPROM_R_KP),
  eepromReadDouble(EEPROM_R_KI),
  eepromReadDouble(EEPROM_R_KD)     
    );

  lPID.SetOutputLimits(-255, 255);
  rPID.SetOutputLimits(-255, 255);

  lPID.SetSampleTime(50);
  rPID.SetSampleTime(50);

  //turn the PID on
  lPID.SetMode(AUTOMATIC);
  rPID.SetMode(AUTOMATIC);
  
  
}

void loop()
{
  //lWheel.Input = lWheel.stepCount;
  //rWheel.Input = rWheel.stepCount;

  int frequence_codeuse = SAMPLING_FREQ*lWheel.stepCount;
  lWheel.stepCount=0;
  lWheel.Input = (double)frequence_codeuse/(double)TICK_BY_MOTOR_SHAFT/(double)GEAR_RATIO; //number of wheel rotatation per second
  
  processPIDs();

  if (stringComplete)
  {
    // Parse the recieved data
    ParseSerialData();
    // Reset inString to empty
    inString = "";
    for(int i=0; i< COMMAND_LENGTH; i++)
    {
      inData[i]=0;
    } 
    // Reset the system for further
    // input of data
    stringComplete = false;
  }

}

void rencoder1()  {
  /*if (PIND & (1<<4))    lWheel.stepCount--;    
  else*/                  lWheel.stepCount++;
}

void rencoder2()  { 
  /*if (PIND & (1<<5))    rWheel.stepCount--;
  else */                 rWheel.stepCount++;
}

void processPIDs()
{ 
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

void serialEvent()
{
  // Read while we have data
  while (Serial.available() && stringComplete == false)
  {
    if(index>=COMMAND_LENGTH)
    {
      Serial.flush();
      stringComplete =true;
      index = 0;
      return;
    }
    // Read a character
    char inChar = Serial.read();

    // Check for termination character
    if (inChar == '\n' || inChar == '\r')
    {
      // Reset the index
      index = 0;
      // Set completion of read to true
      stringComplete = true; 
      return;
    }

    // Store it in char array
    inData[index] = inChar;
    // Increment where to write next
    index++;
    // Also add it to string storage just
    // in case, not used yet!
    inString += inChar;
  }
}

void ParseSerialData()
{
  // The data to be parsed
  char *p = inData;
  // Temp store for each data chunk
  char *str;
  // Id ref for each chunk
  int count = 0;
  // Loop through the data and seperate it into
  // chunks at each " " delimeter
  while ((str = strtok_r(p, " ", &p)) != NULL)
  {
    // Add chunk to array
    inParse[count] = str;
    // Increment data count
    count++;
  }

  if(count == 1)
  {
    if(strcmp(inParse[0],"stop")==0)
    {
      Serial.print("Stoppin' ");

      lPID.SetMode(MANUAL);
      rPID.SetMode(MANUAL);
      do
      {
        do
        {
          digitalWrite(InA1, LOW);
          digitalWrite(InB1, LOW);

          digitalWrite(InA2, LOW);
          digitalWrite(InB2, LOW);

          lWheel.Input = lWheel.stepCount = lWheel.Setpoint=0;
          rWheel.Input = rWheel.stepCount = rWheel.Setpoint=0;

          Serial.print('.');  
          delay(10);
        }
        while(lWheel.stepCount || rWheel.stepCount);
        delay(100);
      }
      while(lWheel.stepCount || rWheel.stepCount);

      lWheel.Output = 0;
      rWheel.Output = 0;
      processPIDs();

      lPID.SetMode(AUTOMATIC);
      rPID.SetMode(AUTOMATIC);
      Serial.println(" ok");
    }
    else
    {
      Serial.print("?? : ");
      Serial.print(inParse[0]);
    }
  }

  else if(count == 2)
  {
    if(strcmp(inParse[0],"print")==0)
    {
      if(strcmp(inParse[1],"pid")==0)
      { //"print pid"
        affichePID();
      }
      else if(strcmp(inParse[1],"debug")==0)
      { //"print debug"
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
      }
      else
      {
        Serial.print("?? : ");
        Serial.print(inParse[0]); 
        Serial.print(" ");
        Serial.println(inParse[1]); 
      }

    }
    else if(strcmp(inParse[0],"step")==0)
    { //"step 123"
      int cmd = atoi(inParse[1]);

      Serial.print("Setpoint: ");
      Serial.print(lWheel.Setpoint);
      Serial.print(" -> ");
      Serial.println(cmd);

      Serial.print(lWheel.Setpoint);
      Serial.print(" -> ");
      Serial.println(cmd);

      lWheel.stepCount = 0;
      rWheel.stepCount = 0;
      lWheel.Setpoint = cmd;
      rWheel.Setpoint = cmd;

    }

    else
    {
      Serial.print("?? : ");
      Serial.print(inParse[0]); 
      Serial.print(" ");
      Serial.println(inParse[1]); 
    }

  }

  else if(count == 3)
  {
    if(strcmp(inParse[0],"pid")==0)
    {
      char cmd1 = inParse[1][0];    // 'p' ou 'i' ou 'd'
      double cmd2 = atof(inParse[2]);     //valeur 12.345 

      if(cmd1 == 'p')
      { //pid p 5"
        eepromUpdateDouble(EEPROM_L_KP,cmd2);
        lPID.SetTunings(cmd2, lPID.GetKi(), lPID.GetKd());
        eepromUpdateDouble(EEPROM_R_KP,cmd2);
        rPID.SetTunings(cmd2, rPID.GetKi(), rPID.GetKd());
      }
      else if(cmd1 == 'i')
      {  //pid i 5"
        eepromUpdateDouble(EEPROM_L_KI,cmd2);
        lPID.SetTunings(lPID.GetKp(), cmd2, lPID.GetKd());
        eepromUpdateDouble(EEPROM_R_KI,cmd2);
        rPID.SetTunings(rPID.GetKp(), cmd2, rPID.GetKd());
      }
      else if(cmd1 == 'd')
      { //pid d 5"
        eepromUpdateDouble(EEPROM_L_KD,cmd2);
        lPID.SetTunings(lPID.GetKp(), lPID.GetKi(), cmd2);
        eepromUpdateDouble(EEPROM_R_KD,cmd2);
        rPID.SetTunings(rPID.GetKp(), rPID.GetKi(), cmd2);
      }
      affichePID();
    }
    else
    {
      Serial.print("?? : ");
      Serial.print(inParse[0]); 
      Serial.print(" ");
      Serial.print(inParse[1]); 
      Serial.print(" ");
      Serial.println(inParse[2]); 
    }
  }
  else if(count == 4)
  {
    if(strcmp(inParse[0],"pid")==0)
    {
      char cmd1 = inParse[1][0];     //'l' ou 'r'
      char cmd2 = inParse[2][0];     // 'p' ou 'i' ou 'd'
      double cmd3 = atof(inParse[3]);  //valeur 12.345

      Serial.println(cmd3);


      if(cmd1 == 'l')
      {
        if(cmd2 == 'p')
        { //pid l p 5"
          eepromUpdateDouble(EEPROM_L_KP,cmd3);
          lPID.SetTunings(cmd3, lPID.GetKi(), lPID.GetKd());
        }
        else if(cmd2 == 'i')
        {  //pid l i 5"
          eepromUpdateDouble(EEPROM_L_KI,cmd3);
          lPID.SetTunings(lPID.GetKp(), cmd3, lPID.GetKd());
        }
        else if(cmd2 == 'd')
        { //pid l d 5"
          eepromUpdateDouble(EEPROM_L_KD,cmd3);
          lPID.SetTunings(lPID.GetKp(), lPID.GetKi(), cmd3);
        }
      }
      else if(cmd1 == 'r')
      {
        if(cmd2 == 'p')
        { //pid r p 5"
          eepromUpdateDouble(EEPROM_R_KP,cmd3);
          rPID.SetTunings(cmd3, rPID.GetKi(), rPID.GetKd());
        }
        else if(cmd2 == 'i')
        { //pid r i 5"
          eepromUpdateDouble(EEPROM_R_KI,cmd3);
          rPID.SetTunings(rPID.GetKp(), cmd3, rPID.GetKd());
        }
        else if(cmd2 == 'd')
        { //pid r d 5"
          eepromUpdateDouble(EEPROM_R_KD,cmd3);
          rPID.SetTunings(rPID.GetKp(), rPID.GetKi(), cmd3);
        }
      }
      affichePID();
    }
    else
    {
      Serial.print("?? : ");
      Serial.print(inParse[0]); 
      Serial.print(" ");
      Serial.print(inParse[1]); 
      Serial.print(" ");
      Serial.print(inParse[2]); 
      Serial.print(" ");
      Serial.println(inParse[3]); 
    }
  }
}

double eepromReadDouble(int address)
{
  union u_tag {
    byte b[4];
    double dval;
  } 
  u;   
  u.b[0] = EEPROM.read(address);
  u.b[1] = EEPROM.read(address+1);
  u.b[2] = EEPROM.read(address+2);
  u.b[3] = EEPROM.read(address+3);
  return u.dval;
}

void eepromWriteDouble(int address, double value)
{
  union u_tag {
    byte b[4];
    double dval;
  } 
  u;
  u.dval=value;

  EEPROM.write(address  , u.b[0]);
  EEPROM.write(address+1, u.b[1]);
  EEPROM.write(address+2, u.b[2]);
  EEPROM.write(address+3, u.b[3]);
}

void eepromUpdateDouble(int address, double value)
{
  if(eepromReadDouble(address) != value)
    eepromWriteDouble(address, value);
}















































