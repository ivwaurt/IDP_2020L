/*------ Arduino Line Follower Code----- */
/*-------definning Inputs------*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);
//change motor voltage 
/*-------definning Outputs------*/
#define LM1 3       // left motor //(only 2 motors)
#define RM1 4       // right motor
double val_1;
double val_2;

void setup()
{
  Serial.begin(9600);
  pinMode(A0, INPUT); //left sensor as input
  pinMode(A1, INPUT); //right sensor as input
  AFMS.begin();
}

void loop()
{
  uint8_t i=255;
  val_1 = analogRead(A0); //left sensor analogue value
  val_2 = analogRead(A1); //right sensor analogue value
  Serial.println(val_1);
  Serial.println(val_2);
  myMotor->run(FORWARD);
  myOtherMotor->run(FORWARD);
  if(val_1>=4.00 && val_2<=2.00)     // Move Forward
  {
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);
    delay(500);
  }
  
  if((val_1>=4.00 && val_2>=4.00) or (val_1<=2.00 && val_2>=4.00))     // AGV going left, needs to turn right.
  {
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i/2);
    delay(500);
  }
  
  if(val_1<=2.00 && val_2<=2.00)     // AGV going right, needs to turn left.
  {
    myMotor->setSpeed(i/2);
    myOtherMotor->setSpeed(i);
    delay(500);
  }
}