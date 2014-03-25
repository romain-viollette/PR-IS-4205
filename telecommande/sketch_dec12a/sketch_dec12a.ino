#define XMIN      2
#define XMED      505
#define XMAX      1024
#define XTOLERANCE 10

#define YMIN      3
#define YMED      520
#define YMAX      1020
#define YTOLERANCE 10

#define FORWARD   1
#define BACKWARD  0


int x;
char xdir;
int y;
char ydir;

void setup()
{
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  Serial.begin(115200); 
  delay(10);
}

void loop()
{
  x = 1014-analogRead(A0);
  //  x = map(x,2,1024,-255,255);
  //  x = x +9;
  //  x = constrain(x,-255,255);

  y = 1014-analogRead(A1);
//  y = map(y,3,1020,-255,255);
//  y = y +2;
//  y = constrain(y,-255,255);


  if(x < XMED-XTOLERANCE)
  {
    x = map(x,XMIN,XMED-XTOLERANCE-1,255,0);
    x = constrain(x,0,255);
    xdir = BACKWARD;
  }
  else if(x >= XMED-XTOLERANCE && x <= XMED+XTOLERANCE)
  {
    x = 0;
    xdir = FORWARD;
  }
  else if(x > XMED+XTOLERANCE)
  {
    x = map(x, XMED+XTOLERANCE+1,XMAX,0,255);
    x = constrain(x,0,255);
    xdir = FORWARD;
  }
  
    if(y < YMED-YTOLERANCE)
  {
    y = map(y,YMIN,YMED-YTOLERANCE-1,255,0);
    y = constrain(y,0,255);
    ydir = BACKWARD;
  }
  else if(y >= YMED-YTOLERANCE && y <= YMED+YTOLERANCE)
  {
    y = 0;
    ydir = FORWARD;
  }
  else if(y > YMED+YTOLERANCE)
  {
    y = map(y, YMED+YTOLERANCE+1,YMAX,0,255);
    y = constrain(y,0,255);
    ydir = FORWARD;
  }

  if(xdir == FORWARD)
    Serial.write('l');
  else
    Serial.write('L'); 
  Serial.write(x-y);
  
  delay(100);
  
  if(xdir == FORWARD)
    Serial.write('r');
  else
    Serial.write('R');
  Serial.write(x+y);

  delay(100);
  
  
  

}

