// Test MD03a / Pololu motor with encoder
// speed control (PI), V & I display
// Credits:
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923


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

#define Vpin            0                       // battery monitoring analog pin
#define Apin            1                       // motor current monitoring analog pin

#define CURRENT_LIMIT   1000                     // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        20                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average


unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing

int readings1[NUMREADINGS];
long speed_req1 = 0;                            // speed (Set Point)
long speed_act1 = 0;                              // speed (actual value)
int PWM_val1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
//int voltage = 0;                                // in mV
//int current = 0;                                // in mA
volatile long count1 = 0;                        // rev counter
float Kp1 =   1;                                // PID proportional control Gain
float Kd1 =   1;                                // PID Derivitave control gain

int readings2[NUMREADINGS];
long speed_req2 = 0;                            // speed (Set Point)
long speed_act2 = 0;                              // speed (actual value)
int PWM_val2 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
volatile long count2 = 0;                        // rev counter
float Kp2 =   1;                                // PID proportional control Gain
float Kd2 =   1;                                // PID Derivitave control gain


void setup() {
  //  analogReference(EXTERNAL);                            // Current external ref is 3.3V
  Serial.begin(115200);
  delay(100);
  Serial.println("Bonjour!");

  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(encodPinA1, INPUT); 
  pinMode(encodPinB1, INPUT); 
  attachInterrupt(0, rencoder1, FALLING);

  analogWrite(PWM1, PWM_val1);
  digitalWrite(InA1, LOW);
  digitalWrite(InB1, LOW);

  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(encodPinA2, INPUT); 
  pinMode(encodPinB2, INPUT); 
  attachInterrupt(1, rencoder2, FALLING);

  analogWrite(PWM2, PWM_val2);
  digitalWrite(InA2, LOW);
  digitalWrite(InB2, LOW);
}

void loop() {
  static int last_error1=0;
  static int last_error2=0;

  getParam();                                                                 // check keyboard
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
    lastMilli = millis();
    getMotorData();                                                           // calculate speed, volts and Amps
    PWM_val1= updatePid(PWM_val1, speed_req1, speed_act1, &last_error1, Kp1, Kd1);                        // compute PWM value
    analogWrite(PWM1, PWM_val1);                                              // send PWM to motor

    PWM_val2= updatePid(PWM_val2, speed_req2, speed_act2, &last_error2, Kp2, Kd2);                        // compute PWM value
    analogWrite(PWM2, PWM_val2);                                              // send PWM to motor
  }

  static int count = 0;
  if(count ==10000)
  {
    Serial.println(speed_req1,DEC);
    Serial.println(speed_act1,DEC);
    Serial.println(PWM_val1,DEC);
    Serial.println(speed_req2,DEC);
    Serial.println(speed_act2,DEC); 
    Serial.println(PWM_val2,DEC);
    Serial.println();  
    count =0;
  } 
  count++;

}

void getMotorData()  {                                                        // calculate speed, volts and Amps
  static long countAnt1 = 0;                                                   // last count
  static long countAnt2 = 0;                                                   // last count

  speed_act1 = map(count1 - countAnt1, -RPS1, RPS1, -256, 255);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
  countAnt1 = count1;                  

  speed_act2 = 4*((count2 - countAnt2)*(60*(1000/LOOPTIME)))/(16*30);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
  countAnt2 = count2;                  
}

int updatePid(int command, int targetValue, int currentValue,int *last_error,int Kp, int Kd)   {             // compute PWM value
  float pidTerm = 0;                                                            // PID correction
  int error=0;
  static int Ti=0;  
  error = abs(targetValue) - abs(currentValue);
  pidTerm = (Kp * error) + (Kd * (error - *last_error));          //correction, positive ou négative                  
  *last_error = error;
  return constrain(command + int(pidTerm), 0, 255);
}

void rencoder1()  {                                    // pulse and direction, direct port reading to save cycles
  if (PIND & (1<<4))    count1++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                  count1--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}

void rencoder2()  {                                    // pulse and direction, direct port reading to save cycles
  if (PIND & (1<<5))    count2++;                // if(digitalRead(encodPinB2)==HIGH)   count ++;
  else                  count2--;                // if (digitalRead(encodPinB2)==LOW)   count --;
}

int getParam()  {
  char param, cmd;
  if(!Serial.available())    return 0;
  delay(1);                  
  param = Serial.read();                              // get parameter byte
  if(!Serial.available())    return 0;
  cmd = Serial.read();                                // get command byte
  Serial.flush();
  switch (param) {
    //  case 'v':                                         // adjust speed
    //    if(cmd=='+')  {
    //      speed_req1 += 5;
    //      if(speed_req1>255)   speed_req1=255;
    //    }
    //    if(cmd=='-')    {
    //      speed_req1 -= 5;
    //      if(speed_req1<0)   speed_req1=0;
    //    }
    //    break;
    //  case 's':                                        // adjust direction
    //    if(cmd=='+'){
    //      digitalWrite(InA1, LOW);
    //      digitalWrite(InB1, HIGH);
    //    }
    //    if(cmd=='-')   {
    //      digitalWrite(InA1, HIGH);
    //      digitalWrite(InB1, LOW);
    //    }
    //    break;
    //  case 'V':                                         // adjust speed
    //    if(cmd=='+')  {
    //      speed_req2 += 5;
    //      if(speed_req2>255)   speed_req2=255;
    //    }
    //    if(cmd=='-')    {
    //      speed_req2 -= 5;
    //      if(speed_req2<0)   speed_req2=0;
    //    }
    //    break;
    //  case 'S':                                        // adjust direction
    //    if(cmd=='+'){
    //      digitalWrite(InA2, HIGH);
    //      digitalWrite(InB2, LOW);
    //    }
    //    if(cmd=='-')   {
    //      digitalWrite(InA2, LOW);
    //      digitalWrite(InB2, HIGH);
    //    }
    //    break;
    //  case 'b':                                        // adjust direction
    //    if(cmd=='+'){
    //      digitalWrite(InA2, HIGH);
    //      digitalWrite(InB2, LOW);
    //      digitalWrite(InA1, HIGH);
    //      digitalWrite(InB1, LOW);
    //    }
    //    if(cmd=='-')   {
    //      digitalWrite(InA2, LOW);
    //      digitalWrite(InB2, HIGH);
    //      digitalWrite(InA1, LOW);
    //      digitalWrite(InB1, HIGH);
    //    }
    //    break;
    //  case 'B':                                        // adjust direction
    //    if(cmd=='+'){
    //      speed_req1 += 5;
    //      if(speed_req1>255)   speed_req1=255;
    //      speed_req2 += 5;
    //      if(speed_req2>255)   speed_req2=255;
    //    }
    //    if(cmd=='-')   {
    //      speed_req1 -= 5;
    //      if(speed_req1<0)   speed_req1=0;
    //      speed_req2 -= 5;
    //      if(speed_req2<0)   speed_req2=0;
    //    }
    //    break;
    //  case 'o':                                        // user should type "oo"
    //    digitalWrite(InA1, LOW);
    //    digitalWrite(InB1, LOW);
    //    speed_req1 = 0;
    //
    //    digitalWrite(InA2, LOW);
    //    digitalWrite(InB2, LOW);
    //    speed_req2 = 0;
    //
    //    break;
    //  case 'p':
    //    /*Serial.print("speed_req1 : ");*/    Serial.println(speed_req1,DEC);
    //    /*Serial.print("speed_act1 : ");*/    Serial.println(speed_act1,DEC);
    //    /*Serial.print("speed_req2 : ");*/    Serial.println(speed_req2,DEC);
    //    /*Serial.print("speed_act2 : ");*/    Serial.println(speed_act2,DEC);
    //    break;

  case 'l':
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, LOW);
    speed_req1 = cmd;
    break; 

  case 'L':
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);
    speed_req1 = cmd;
    break;

  case 'r':
    digitalWrite(InA2, HIGH);
    digitalWrite(InB2, LOW);
    speed_req2 = cmd;
    break;

  case 'R':
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, HIGH);
    speed_req2 = cmd;
    break;

  case 'o':
    if(cmd == 'l')
    {
      digitalWrite(InA1, LOW);
      digitalWrite(InB1, LOW);
      speed_req1 = 0;
      
    }
    else
      if(cmd == 'r')
      {
        digitalWrite(InA2, LOW);
        digitalWrite(InB2, LOW);
        speed_req2 = 0;
      }
    break;

    //  default: 
    //    Serial.println("???");
  }
}

int digital_smooth(int value, int *data_array)  {    // remove signal noise
  static int ndx=0;                                                         
  static int count=0;                          
  static int total=0;                          
  total -= data_array[ndx];               
  data_array[ndx] = value;                
  total += data_array[ndx];               
  ndx = (ndx+1) % NUMREADINGS;                                
  if(count < NUMREADINGS)      count++;
  return total/count;
}






