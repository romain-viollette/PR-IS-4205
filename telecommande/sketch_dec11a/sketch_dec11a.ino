int x;
int y;

void setup()
{
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
   Serial.begin(115200); 
}

void loop()
{
  x = 1014-analogRead(A0);
  x = map(x,2,1024,-255,255);
  x = x +9;
  x = constrain(x,-255,255);
  
  y = 1014-analogRead(A1);
  y = map(y,3,1020,-255,255);
  y = y +2;
  y = constrain(y,-255,255);
  
  Serial.print("x");
  Serial.print(x);
  Serial.print(" y");
  Serial.print(y);
  Serial.print('\n');
  delay(50);
  
}
