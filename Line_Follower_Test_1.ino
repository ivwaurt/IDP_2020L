/*------ Arduino Line Follower Code----- */
/*-------definning Inputs------*/

//change motor voltage 
#define LS 1      // left sensor
#define RS 2      // right sensor

/*-------definning Outputs------*/
#define LM1 3       // left motor //(only 2 motors)
#define RM1 4       // right motor

void setup()
{
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(LM1, OUTPUT);
  pinMode(RM1, OUTPUT);
}

void loop()
{
  if(digitalRead(LS) && !(digitalRead(RS)))     // Move Forward
  {
    digitalWrite(LM1, HIGH);
    digitalWrite(RM1, HIGH);
  }
  
  if((digitalRead(LS) && digitalRead(RS)) or (!(digitalRead(LS)) && digitalRead(RS)))     // AGV going left, needs to turn right.
  {
    digitalWrite(LM1, HIGH);
    digitalWrite(RM1, LOW);
  }
  
  if(!(digitalRead(LS)) && !(digitalRead(RS)))     // AGV going right, needs to turn left.
  {
    digitalWrite(LM1, LOW);
    digitalWrite(RM1, HIGH);
  }
}
