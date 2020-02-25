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
bool sensor_s;      // side sensors (3rd one)
bool speed_l;       // 1 = faster, 0 = slower
bool speed_r;

//Parameters
double tol_l = 3.00;  //Boundary between black/white
double tol_r = 3.00;
double tol_s = 3.00;  
uint8_t v = 255;      //Motor speed
int i=0;

void setup()
{
  Serial.begin(9600);
  pinMode(A0, INPUT); //left sensor as input
  pinMode(A1, INPUT); //right sensor as input
  AFMS.begin();
}

void loop()
{
  //Print readings from sensors
  Serial.println(analogRead(A0));
  Serial.println(analogRead(A1));
  
  //?
  motorL->run(FORWARD);
  motorR->run(FORWARD);
  
  //Sensor reading to bool
  sensor_l = (analogRead(A0)>tol_l) ? 1 : 0;
  sensor_r = (analogRead(A0)>tol_r) ? 1 : 0;
  sensor_s = (analogRead(A0)>tol_r) ? 1 : 0;
  
  //Determine motor speeds using boolean logic
  speed_l = sensor_l || sensor_r;
  speed_r = ! sensor_r;

  //Set to continue during first white line, stop during second white line
  if(sensor_s){
    if (i==0){
      motorL->setSpeed(v);
      motorR->setSpeed(v);
    }
    else{
      motorL->setSpeed(0);
      motorR->setSpeed(0);
    }
    i++;
  }
  
  //Set left motor speed
  if(speed_l){
      motorL->setSpeed(v);
  } else {
      motorL->setSpeed(v/2);
  }
  
  //Set right motor speed
  if(speed_r){
      motorR->setSpeed(v);
  } else {
      motorR->setSpeed(v/2);
  }

  //Delay till next loop
  delay(500);
}
