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

#define InA2            13                      // INA motor pin
#define InB2            12                      // INB motor pin 
#define PWM2            11                       // PWM motor pin
#define encodPinA2      3                       // encoder A pin
#define encodPinB2      5                       // encoder B pin

#define LOOPTIME        1000                    // PID loop time


unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing

int PWM_val1 = 255;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
volatile long count1 = 0;                        // rev counter

int PWM_val2 = 255;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
volatile long count2 = 0;                        // rev counter

long countAnt1 = 0; 
long countAnt2 = 0;
long speed_act1 = 0;
long speed_act2 = 0;



void setup() {
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
  digitalWrite(InA1, HIGH);
  digitalWrite(InB1, LOW);

  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(encodPinA2, INPUT); 
  pinMode(encodPinB2, INPUT); 
  attachInterrupt(1, rencoder2, FALLING);

  analogWrite(PWM2, PWM_val2);
  digitalWrite(InA2, HIGH);
  digitalWrite(InB2, LOW);
}

void loop() {
  if((millis()-lastMilli) >= LOOPTIME)   { 
  lastMilli = millis();  
       

  speed_act1 = count1 - countAnt1;
  countAnt1 = count1;                  

  speed_act2 = count2 - countAnt2;       
  countAnt2 = count2;             

    Serial.println(lastMilli,DEC);
    Serial.println(speed_act1,DEC);
    Serial.println(speed_act2,DEC);

  }
}



void rencoder1()  {                                    // pulse and direction, direct port reading to save cycles
  if (PIND & (1<<4))    count1++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                  count1--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}

void rencoder2()  {                                    // pulse and direction, direct port reading to save cycles
  if (PIND & (1<<5))    count2++;                // if(digitalRead(encodPinB2)==HIGH)   count ++;
  else                  count2--;                // if (digitalRead(encodPinB2)==LOW)   count --;
}






