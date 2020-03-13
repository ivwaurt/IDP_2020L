/*------ Arduino Line Follower Code----- */
/*-------definning Inputs------*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);

//change motor voltage 
/*-------definning Outputs------*/
#define LM1 3       // left motor //(only 2 motors)
#define RM1 4       // right motor


//Variables
bool sensor_l;      // 1 = white, 0 = black
bool sensor_r;
bool sensor_s;      // side sensors (3rd one, at the right of AGV)
bool speed_l;       // 1 = faster, 0 = slower
bool speed_r;


//Parameters
int tol_l = 700;  //Boundary between black/white
uint8_t v = 100;      //Motor speed

void setup()
{
  Serial.begin(9600);
  pinMode(A0, INPUT); //left sensor as input
  pinMode(A1, INPUT); //right sensor as input
  pinMode(A2, INPUT);
  AFMS.begin();
}

void loop()
{
  //Print readings from sensors
  Serial.print("Left sensor: ");
  Serial.println(analogRead(A0));
  Serial.print("Right sensor: ");
  Serial.println(analogRead(A1));
  Serial.print("Side sensor: ");
  Serial.println(analogRead(A2));
  
  //?
  motorL->run(FORWARD);
  motorR->run(FORWARD);
  
  //Sensor reading to bool
  //sensor_l = (analogRead(A0)>tol_l) ? 1 : 0;
  //sensor_r = (analogRead(A0)>tol_r) ? 1 : 0;
  if (analogRead(A0)>tol_l){
    sensor_l = 1;
  }
  else{
    sensor_l = 0;
  }
  if (analogRead(A1)>tol_l){
    sensor_r = 1;
  }
  else{
    sensor_r = 0;
  }
  if (analogRead(A2)>tol_l){
    sensor_s = 1;
  }
  else{
    sensor_s = 0;
  }
  Serial.print(sensor_l);
  Serial.print(sensor_r);
  Serial.println(sensor_s);
  Serial.println("----");
  //sensor_s = (analogRead(A0)>tol_r) ? 1 : 0;
  
  //Determine motor speeds using boolean logic
  speed_l = sensor_l || sensor_r;
  speed_r = ! sensor_r;
  
  //Set left motor speed
  if(speed_l){
      motorL->setSpeed(v);
  } else {
      motorL->setSpeed(0);
  }
  
  //Set right motor speed
  if(speed_r){
      motorR->setSpeed(v);
  } else {
      motorR->setSpeed(0);
  }

  if(sensor_s){
    motorL->setSpeed(0);
    motorR->setSpeed(0);
  }
  //Delay till next loop
  delay(100);
}
