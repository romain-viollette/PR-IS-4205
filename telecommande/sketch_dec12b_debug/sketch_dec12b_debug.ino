#include <math.h>

int Joy1RawX = 0;      
int Joy1RawY = 0;
int Joy1X = 0;
int Joy1Y = 0;
float Bearing1 = 0;
float Bearing2 = 0;
int Bearing3 = 0;
boolean Token = false;
const int Xmod = 512 - 519; // alter the second number to match the idle reading of your joystick's X axis
const int Ymod = 512 - 497; // alter the second number to match the idle reading of your joystick's Y axis

void setup() {

  Serial.begin(115200);
}

void loop() {
  delay(50);
  Token = true;
  Joy1RawX = analogRead(0);
  delay(1);
  Joy1RawY = analogRead(1);
  delay(1);
    Joy1X = (Xmod + Joy1RawX) - 512;
    Joy1Y = (Ymod + Joy1RawY) - 512;
    Bearing1 = RAD_TO_DEG * atan2(Joy1Y, Joy1X);

  Serial.print("JOY1:  Raw X: ");
  Serial.print(Joy1RawX);
  Serial.print(" | Raw Y: ");
  Serial.print(Joy1RawY);
  Serial.print(" | Mod X: ");
  Serial.print(Joy1X);
  Serial.print(" | Mod Y: ");
  Serial.print(Joy1Y);
  Serial.print(" | Atan2: ");
  Serial.println(Bearing1);
  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

    if ((Joy1X >= -2 && Joy1X <= 2) && (Joy1Y >= -2 && Joy1Y <= 2)) {
      Token = false;
      Serial.println(" Joystick Dectected in DeadZone");
      // Don't change prevous heading
    }
    if (Token) {
      if (Bearing1 >= 0.0 && Bearing1 <= 90.0) {
        Serial.print("1st: ");
        Serial.print(fabs(Bearing1 - 90.0));
        Bearing2 = fabs(Bearing1 - 90.0);
        Bearing3 = Bearing2;
      }
      if (Bearing1 > 90.0 && Bearing1 <=180.0) {
        Serial.print("4th: ");
        Serial.print(fabs(Bearing1 - 180.0) + 270.0);
        Bearing2 = fabs(Bearing1 - 180.0) + 270.0;
        Bearing3 = Bearing2;
      }
      if (Bearing1 < 0.0 && Bearing1 >= - 90.0) {
        Serial.print("2nd: ");
        Serial.print(fabs(Bearing1) + 90.0);
        Bearing2 = fabs(Bearing1) + 90.0;   
        Bearing3 = Bearing2;
      }
      if (Bearing1 < -90.0 && Bearing1 >= -180.0) {
        Serial.print("3rd: ");
        Serial.print(fabs(Bearing1 + 90.0) + 180.0);
        Bearing2 = fabs(Bearing1 + 90.0) + 180.0;
        Bearing3 = Bearing2;
      }  
    Serial.print(" | Float: ");
    Serial.print(Bearing2);
    Serial.print(" | Bearing: ");
    Serial.println(Bearing3);
    }   
      

  delay(50);
}
